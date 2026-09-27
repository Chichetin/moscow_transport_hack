"""Read-only core experiments. Paths are explicit; no ROS execution or hidden parameter tuning."""
from __future__ import annotations
import argparse, ast, dataclasses, importlib.util, json, math, os, re, statistics, subprocess, sys, time, traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pyproj

HERE=Path(__file__).resolve().parent
REPO=Path(os.environ.get('BENCH_REPO',str(HERE.parents[1])))
DATA=Path(os.environ.get('TRAM_DATA_DIR',str(REPO/'dataset/data')))
SOURCES=Path(os.environ.get('PUBLIC_REPO_ROOT','/tmp'))
LCM=Path(os.environ.get('LCM_REPO',str(SOURCES/'msk-competitors-lcm')))
sys.path[:0]=[str(REPO/'tools/eval'),str(REPO/'src/tram_odometry_core'),str(LCM/'ros2_ws/src/tram_odometry'),str(SOURCES/'msk-competitors-ilushenssss/tram_nav')]
from tram_eval import bag
from tram_eval.metrics import Estimates, bag_metrics, match_nearest, speed_modes
from tram_eval.reference import build_reference,geodetic_to_enu
from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.slip import SlipDetector
from tram_odometry_core.types import SlipState

TRAIN6=['30618_0652866c','30618_0e41eac3','30618_1cc230fa','30639_3b3d9eb8','30639_44226bde','30639_c31df386']
assert set(TRAIN6)<=set(bag.load_splits()['train'])

class FrozenDetector:
    """Experiment inspired by LCM. No active default or params.yaml change."""
    def __init__(self, original):
        self.original=original; self.last={}; self.count={};self.drift={}
    def update(self,front,rear,accel_model,est):
        st=self.original.update(front,rear,accel_model,est)
        bad=[]
        for name,s in [('front',front),('rear',rear)]:
            if s is None:continue
            old=self.last.get(name)
            if old is not None and s.t>old.t:
                if abs(s.speed-old.speed)<1e-9:
                    self.count[name]=self.count.get(name,0)+1
                    self.drift[name]=self.drift.get(name,0)+accel_model*(s.t-old.t)
                else:self.count[name]=0;self.drift[name]=0
            if old is None or s.t>old.t:self.last[name]=s
            if s.speed>0.3 and self.count.get(name,0)>=4 and abs(self.drift.get(name,0))>0.3:bad.append(name)
        changes={}
        for name in bad:changes[name+'_trust']=0.0;changes['slip_'+name]=True
        return dataclasses.replace(st,**changes)

def ours(variant):
    odo=bag.default_odometry()
    if variant=='ours_trapezoid':
        # Change only the one path-integration expression; source and symbol checks fail closed.
        src=(REPO/'src/tram_odometry_core/tram_odometry_core/pipeline.py').read_text()
        assert src.count('        ds = self._v * dt')==1
        # eeca835: `self._v, _, _ = ...`, later main: `self._v, _, accel = ...`; exactly one line either way.
        src,n=re.subn(r'^(        )(self\._v, _, \w+ = self\._filter\.state\(\))$',r'\1previous_speed = self._v\n\1\2',src,flags=re.M)
        assert n==1
        src=src.replace('        ds = self._v * dt','        ds = 0.5 * (previous_speed + self._v) * dt')
        scope={'__name__':'tram_odometry_core.trapezoid_experiment','__package__':'tram_odometry_core'}
        exec(compile(src,'trapezoid_experiment','exec'),scope)
        odo=scope['Odometry'](odo.params,odo.route)
    # LCM-inspired experiments are opt-in and do not modify the runtime pipeline.
    if variant=='ours_adapt':
        src=(REPO/'src/tram_odometry_core/tram_odometry_core/estimator/__init__.py').read_text()
        anchor='        self._scale_delta = 0.0'
        assert src.count(anchor)==1
        src=src.replace(anchor,anchor+'\n        self._innovation_window = []')
        # The bare `projected = self._cov @ observation` also sits (deeper indented) in the paired-zero
        # branch, so anchor on the innovation line + projected line: exactly one site, fail closed otherwise.
        old='        innovation = measurement - observation @ self._x\n        projected = self._cov @ observation'
        new='''        innovation = measurement - observation @ self._x
        self._innovation_window.append(abs(innovation))
        self._innovation_window = self._innovation_window[-30:]
        if len(self._innovation_window) >= 10:
            robust_std = sorted(self._innovation_window)[len(self._innovation_window)//2] * 1.4826
            if robust_std > 3.0 * self._p.r_wheel ** 0.5:
                measurement_var = max(measurement_var, min(1.0, robust_std)**2 / (trust * scale**2) + self._p.q_accel * age)
        projected = self._cov @ observation'''
        assert src.count(old)==1
        src=src.replace(old,new)
        scope={'__name__':'tram_odometry_core.estimator.adapt_experiment','__package__':'tram_odometry_core.estimator'}
        exec(compile(src,'adapt_experiment','exec'),scope)
        odo._filter=scope['SpeedFilter'](odo.params)
    if variant=='ours_frozen':odo._slip=FrozenDetector(odo._slip)
    return odo

def speed_result(ref,t,v,seconds):
    t=np.asarray(t,float);v=np.asarray(v,float)
    good=np.isfinite(t)&np.isfinite(v);t=t[good];v=v[good]
    ri,ei=match_nearest(ref.vel_t,t);d=v[ei]-ref.speed[ri]
    out={'speed_rmse':float(np.sqrt(np.mean(d*d))) if len(d) else None,'speed_mae':float(np.mean(abs(d))) if len(d) else None,'speed_matched':len(ri),'speed_coverage':len(ri)/max(1,len(ref.vel_t)),'outputs':len(t),'elapsed_s':seconds,'crashed':False,'_matched_ref':ri,'_matched_error':d}
    modes=speed_modes(ref.vel_t,ref.speed)
    for m in ['accel','brake','cruise','stop']:
        x=d[modes[ri]==m];out['speed_bias_'+m]=float(np.mean(x)) if len(x) else None
    return out

class FakeNode:
    def __init__(self,*a):self.parameters={}
    def declare_parameter(self,k,v):self.parameters[k]=v
    def get_parameter(self,k):return NS(value=self.parameters[k])
    def create_subscription(self,*a,**kw):pass
    def create_publisher(self,*a,**kw):return NS(publish=lambda x:None)
    def create_timer(self,*a,**kw):pass
    def get_logger(self):return NS(info=lambda *a:None,warning=lambda *a:None)

def node_class(path,name):
    tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
    ns={'Node':FakeNode,'np':np,'pyproj':pyproj,'os':os,'time':time,'VelocitySensor':object,'DriverControllerCommand':object,'Odometry':object,'NavSatFix':object,'stamp_to_sec':bag.stamp}
    # stamp_to_sec in mttex receives stamp, not message.
    ns['stamp_to_sec']=lambda s:s.sec+s.nanosec*1e-9
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),ns)
    return ns[name]

def foreign(variant,msgs,ref,window,name=""):
    begin=time.perf_counter();tt=[];vv=[];positions=[];headings=[];mis=0
    if variant.startswith('lcm'):
        from tram_odometry.estimator import Estimator,Params
        from tram_odometry.traction import TractionModel
        from tram_odometry.trackmap import TrackMap
        assets=LCM/'ros2_ws/src/tram_odometry/assets'
        p=Params(default_s0=10342.0,stuck_min_pred_change=0.4,adapt_r=variant=='lcm_adapt')
        e=Estimator(TrackMap(assets/'track_ring.json'),TractionModel(assets/('calib_'+(name[:5] if variant=='lcm_pertram' else 'default')+'.yaml')),p)
        proj=pyproj.Transformer.from_crs(4326,32637,always_xy=True)
        for topic,m in msgs:
            t=bag.stamp(m);out=None
            if topic==bag.CMD:e.on_notch(t,m.position);out=e.predict_to(t)
            elif topic in (bag.FRONT,bag.REAR):out=e.on_wheel(t,'front' if topic==bag.FRONT else 'rear',m.velocity)
            elif topic in (bag.MASTER_FIX,bag.ROVER_FIX) and t<=window:
                if e.first_input_t is None or t-e.first_input_t>5.0 or e.initialized:continue
                if not (math.isfinite(m.latitude) and math.isfinite(m.longitude)) or m.latitude==0:continue
                x,y=proj.transform(m.longitude,m.latitude)
                e.on_gnss(t,'master' if topic==bag.MASTER_FIX else 'rover',m.latitude,m.longitude,x-e.map.x0,y-e.map.y0)
            if out:
                mis+=abs(out['t']-t)>1e-6
                # Actual callbacks publish the input stamp, including for a delayed wheel.
                tt.append(t);vv.append(out['v']);positions.append((out['x']+e.map.x0,out['y']+e.map.y0,out['z']));headings.append(out['heading'])
        result=speed_result(ref,tt,vv,time.perf_counter()-begin)
        if positions and ref.origin:
            xyz=np.asarray(positions);heading=np.asarray(headings)
            inv=pyproj.Transformer.from_crs(32637,4326,always_xy=True)
            # Both raw base_link and master-antenna convention; fixed lever arm only, no fit.
            for label,offset in [('base_link',0.0),('master',-9.873)]:
                lon,lat=inv.transform(xyz[:,0]+offset*np.cos(heading),xyz[:,1]+offset*np.sin(heading))
                pos=geodetic_to_enu(lat,lon,xyz[:,2],ref.origin)
                metrics=bag_metrics(ref,Estimates(np.array(tt),np.array(vv),pos,np.zeros(len(tt),bool)))
                for k in ['along_rmse','cross_rmse','pos3d_rmse','drift_pct']:result[label+'_'+k]=metrics[k]
        result['internal_state_stamp_mismatch']=mis
        result['calibration']=('per-tram from bag name' if variant=='lcm_pertram' else 'default')+'; published calibration overlaps evaluation data'
        return result
    if variant=='mttex':
        C=node_class(SOURCES/'msk-competitors-mttex/src/tram_reserve_odometry/tram_reserve_odometry/reserve_odometry_node.py','ReserveOdometryNode');e=C()
        for topic,m in msgs:
            if topic==bag.CMD:e.on_controller(m)
            elif topic==bag.REAR:e.on_rear(m)
            elif topic==bag.FRONT:
                e.on_front(m);tt.append(bag.stamp(m));vv.append(e.latest_velocity)
    elif variant in ('ktoyart','ktoyart_si'):
        C=node_class(SOURCES/'msk-competitors-ktoyart/tram_odometry_pkg/odometry_node.py','AdaptiveTramOdometry')
        # Map does not enter speed equations: use supported straight-line fallback; never unpickle external asset.
        C.load_map=lambda self:setattr(self,'offline_paths',[])
        C.publish_results=lambda self,s:(tt.append(s.sec+s.nanosec*1e-9),vv.append(float(self.x[1,0])))
        e=C()
        for topic,m in msgs:
            if topic==bag.MASTER_FIX and bag.stamp(m)<=window:e.gnss_fix_cb(m)
            elif topic==bag.CMD:e.cmd_cb(m)
            elif topic in (bag.FRONT,bag.REAR):
                if variant=='ktoyart_si':m=NS(header=m.header,velocity=m.velocity/3.6)
                (e.v_front_cb if topic==bag.FRONT else e.v_rear_cb)(m)
    elif variant=='ilush':
        from tram_nav.core.estimator import TramNavigator
        e=TramNavigator(n_sensors=2,sensor_powered=[True,True],track=None)
        for topic,m in msgs:
            if topic not in bag.INPUTS:continue
            t=bag.stamp(m)
            if topic==bag.CMD:e.set_handle(m.position/15)
            else:e.set_wheel_speed(0 if topic==bag.FRONT else 1,t,m.velocity/3.6/e.p.wheel_radius)
            st=e.step(t);tt.append(t);vv.append(st.v)
    elif variant=='ange':
        rows=[];prev=None;u=0;wheel={}
        for topic,m in msgs:
            if topic not in bag.INPUTS:continue
            t=bag.stamp(m)
            if topic==bag.CMD:u=max(-1,min(1,m.position/15))
            else:wheel[topic]=(t,m.velocity/3.6)
            dt=0 if prev is None else max(0,t-prev);prev=t if prev is None else max(t,prev)
            live=[v for s,v in wheel.values() if 0<=t-s<=0.25]
            valid=topic!=bag.CMD and bool(live)
            rows.append(f'{t:.9f} {dt:.9f} {u} {sum(live)/len(live) if live else 0} {int(valid)}\n')
        process=subprocess.run([os.environ.get('ANGE_RUNNER',str(REPO/'out/public-comparison/ange_runner'))],input=''.join(rows),text=True,capture_output=True,check=True)
        for row in process.stdout.splitlines():
            t,v,_=map(float,row.split());tt.append(t);vv.append(v)
    else:raise ValueError(variant)
    return speed_result(ref,tt,vv,time.perf_counter()-begin)

def run(task):
    name,variants=task;msgs=bag.read_bag(DATA/name);window=bag.gnss_window_end(msgs,5)
    ref=build_reference(*bag.reference_inputs(msgs),window);result={}
    for v in variants:
        started=time.perf_counter()
        try:
            if v.startswith('ours'):
                est,crash,mis=bag.run_pipeline(msgs,ours(v),window)
                result[v]=bag_metrics(ref,est)|speed_result(ref,est.t,est.speed,time.perf_counter()-started)|{'crashed':crash is not None,'stamp_mismatch':mis}
            else:result[v]=foreign(v,msgs,ref,window,name)
        except Exception:result[v]={'crashed':True,'error':traceback.format_exc()}
    sets=[r['_matched_ref'] for r in result.values() if '_matched_ref' in r]
    common=sets[0] if sets else np.array([],int)
    for rr in sets[1:]:common=np.intersect1d(common,rr)
    for r in result.values():
        if '_matched_ref' not in r:continue
        ri=r.pop('_matched_ref');d=r.pop('_matched_error')
        errors=d[np.isin(ri,common)]
        r['common_speed_rmse']=float(np.sqrt(np.mean(errors**2))) if len(errors) else None
        r['common_speed_coverage']=len(common)/max(1,len(ref.vel_t))
    return name,result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--split',default='train6');ap.add_argument('--variants',nargs='+',default=['ours','lcm','mttex','ktoyart','ktoyart_si','ilush','ange']);ap.add_argument('--jobs',type=int,default=3);ap.add_argument('--out',required=True);args=ap.parse_args()
    names=TRAIN6 if args.split=='train6' else bag.load_splits()[args.split]
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);rows={}
    with ProcessPoolExecutor(args.jobs) as pool:
        for future in as_completed([pool.submit(run,(n,args.variants)) for n in names]):
            name,r=future.result();rows[name]=r;(out/'bags.json').write_text(json.dumps(rows,indent=2,allow_nan=False))
            print(name, {k:round(v.get('speed_rmse') or 0,5) if not v.get('crashed') else v.get('error','crash')[-300:] for k,v in r.items()},flush=True)
    summary={}
    for v in args.variants:
        keys=set().union(*(r[v].keys() for r in rows.values()))
        summary[v]={'bags':len(rows),'crashes':sum(r[v].get('crashed',False) for r in rows.values())}
        for k in keys:
            values=[r[v].get(k) for r in rows.values()];values=[x for x in values if isinstance(x,(float,int)) and not isinstance(x,bool) and math.isfinite(x)]
            if values:summary[v][k]=statistics.median(values)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2),flush=True)
if __name__=='__main__':main()
