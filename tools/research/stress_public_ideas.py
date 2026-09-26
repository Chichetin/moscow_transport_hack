from compare_public import *
from tram_eval.stress import perturb, _errors
import copy

def frozen_event(msgs):
    first=bag.stamp(msgs[0][1]);w={};command=0;start=None;frozen={}
    for topic,m in msgs:
        t=bag.stamp(m)
        if topic==bag.CMD:command=m.position
        elif topic in (bag.FRONT,bag.REAR):w[topic]=m.velocity
        if t>first+60 and command>=6 and len(w)==2 and min(w.values())>10.8 and max(w.values())<43.2 and abs(w[bag.FRONT]-w[bag.REAR])<1.0:
            start=t;frozen=dict(w);break
    if start is None:return None
    end=start+12;out=[]
    for topic,m in msgs:
        if topic in frozen and start<=bag.stamp(m)<end:
            m=copy.deepcopy(m);m.velocity=frozen[topic]
        out.append((topic,m))
    return out,start,end

def run_stress(name):
    msgs=bag.read_bag(DATA/name);window=bag.gnss_window_end(msgs,5);ref=build_reference(*bag.reference_inputs(msgs),window)
    rows={}
    for variant in ['ours','ours_frozen','ours_adapt']:
        clean,cc,_=bag.run_pipeline(msgs,ours(variant),window)
        for scenario in ['freeze_both','noise','gap_70','spike']:
            change=frozen_event(msgs) if scenario=='freeze_both' else perturb(msgs,scenario,5)
            if change is None:continue
            dirty,start,end=change
            est,crash,mis=bag.run_pipeline(dirty,ours(variant),window)
            p,cp,ex,rec,n,during=_errors(ref.vel_t,ref.speed,clean,est,'speed',start,end,0.2)
            pp,cpp,pex,prec,_,_=_errors(ref.pos_t,ref.pos,clean,est,'pos',start,end,2)
            rows[variant+'__'+scenario]={'speed_peak':p,'speed_excess':ex,'speed_recovery':rec,'position_peak':pp,'position_excess':pex,'position_recovery':prec,'crashed':bool(cc or crash),'stamp_mismatch':mis,'event_start':start,'event_end':end}
    return name,rows
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--split',default='train6');parser.add_argument('--out',required=True);args=parser.parse_args()
    names=TRAIN6 if args.split=='train6' else bag.load_splits()[args.split]
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);rows={}
    with ProcessPoolExecutor(3) as pool:
        for f in as_completed([pool.submit(run_stress,n) for n in names]):
            n,r=f.result();rows[n]=r;(out/'bags.json').write_text(json.dumps(rows,indent=2));print(n,r,flush=True)
