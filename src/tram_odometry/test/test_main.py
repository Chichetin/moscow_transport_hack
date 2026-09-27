"""ROS entry-point shutdown: normal stops succeed; unexpected failures stay visible."""
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.executors import ExternalShutdownException  # noqa: E402
from tram_odometry import odometry_node as on  # noqa: E402

PARAMS_FILE = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'


@pytest.fixture
def destroyed_nodes(monkeypatch):
    node_class = on.OdometryNode
    destroyed = []

    def create_node():
        node = node_class(params_file=str(PARAMS_FILE))
        destroy = node.destroy_node

        def destroy_node():
            destroy()
            destroyed.append(node)

        monkeypatch.setattr(node, 'destroy_node', destroy_node)
        return node

    monkeypatch.setattr(on, 'OdometryNode', create_node)
    yield destroyed
    rclpy.try_shutdown()


@pytest.mark.parametrize('stop_error', [None, KeyboardInterrupt, ExternalShutdownException])
def test_main_cleans_up_after_normal_stop(monkeypatch, destroyed_nodes, stop_error):
    def stop(node):
        if stop_error is ExternalShutdownException:
            # Humble's signal handler already shut down the context before spin raises.
            rclpy.try_shutdown()
        if stop_error is not None:
            raise stop_error()

    monkeypatch.setattr(rclpy, 'spin', stop)
    on.main(args=[])
    assert len(destroyed_nodes) == 1
    assert not rclpy.ok()


def test_main_cleans_up_and_preserves_unexpected_error(monkeypatch, destroyed_nodes):
    error = RuntimeError('unexpected executor failure')

    def fail(node):
        raise error

    monkeypatch.setattr(rclpy, 'spin', fail)
    with pytest.raises(RuntimeError) as caught:
        on.main(args=[])
    assert caught.value is error
    assert len(destroyed_nodes) == 1
    assert not rclpy.ok()
