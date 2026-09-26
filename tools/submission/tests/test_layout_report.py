"""ROS storage logs must not corrupt the machine-readable jury report."""
import layout_report


def test_multiline_ros_log_remains_one_run_in_report(tmp_path):
    path = tmp_path / 'run.tsv'
    note = 'нода жива: yes; /result/velocity: 10;\n[INFO] bag opened\n\tfail: stamp 12.000000001'
    path.write_text(layout_report.run_row('bag1', 'fail', note), encoding='utf-8')
    assert layout_report.read_runs(str(path)) == [{
        'bag': 'bag1', 'status': 'fail',
        'note': 'нода жива: yes; /result/velocity: 10; [INFO] bag opened fail: stamp 12.000000001',
    }]
