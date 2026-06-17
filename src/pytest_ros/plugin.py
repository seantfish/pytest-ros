import pytest

from pytest_ros import RCLPY_AVAILABLE
if RCLPY_AVAILABLE:
    import rclpy

from .process import ROSProcessManager
from .monitor import EpisodeMonitor
 


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Session-scoped fixture to manage ROS context initialization and shutdown."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()

@pytest.fixture
def ros_process_manager():
    """Factory fixture to manage ROS launch processes."""
    manager = ROSProcessManager()
    yield manager
    manager.terminate_all()  # Ensure all ROS processes are terminated at the end of the session

@pytest.fixture
def ros_monitor():
    """Fixture to provide an EpisodeMonitor instance for monitoring test conditions."""  
    monitor = EpisodeMonitor(name="episode_monitor")
    yield monitor
    monitor.shutdown()
