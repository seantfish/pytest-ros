

import signal
import subprocess
import uuid
import os
import psutil
import pytest
import time

from pytest_ros import RCLPY_AVAILABLE
if RCLPY_AVAILABLE:
    import rclpy
    from rclpy.node import Node

from contextlib import contextmanager
    
@pytest.fixture(scope='session')
def ros_launch(request):

    rclpy.init()
    node = Node('pytest_ros_launch_node')

    # Here you would typically launch your ROS nodes or processes
    # using the provided launch_args, and yield control back to the test.

    yield node

    # After tests are done, clean up
    node.destroy_node()
    rclpy.shutdown()



class MultiProcessOrchestrator:
    """
    A utility class to manage multiple processes in a pytest session, 
    ensuring that all processes are properly terminated at the end of the session.
    """
    def __init__(self, tag="PYTEST_MULTIPROC_ID"):
        self.tag = tag
        self.session_id = str(uuid.uuid4())

    @contextmanager
    def runner(self, cmd):
        # Inject the tag into the environment
        env = os.environ.copy()
        env[self.tag] = self.session_id
        
        # Start the process group
        proc = subprocess.Popen(
            cmd, 
            env=env, 
            preexec_fn=os.setsid
        )
        
        try:
            yield proc
        finally:
            # Kill process group
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            # Kill rogue descendants
            self.terminate_all()

    def terminate_all(self):
        """Kills everything tagged with this session ID."""
        for proc in psutil.process_iter(['environ']):
            try:
                if (env := proc.info.get('environ')) and env.get(self.tag) == self.session_id:
                    proc.send_signal(signal.SIGKILL)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue


class ROSOrchestrator(MultiProcessOrchestrator):
    """
    A specialized orchestrator for managing ROS processes, ensuring that all ROS nodes
    launched during the pytest session are properly terminated at the end of the session.
    
    Manages ROS 2 launch files and provides health checks for nodes/topics.
    """
    def __init__(self, node=None):
        super().__init__(tag="ROS_SESSION_ID")
        # Reuse an existing node if provided, otherwise you'll need to init rclpy
        self.node = node

    def launch(self, package_name, launch_file, arguments=None):
        """
        Launches a ROS 2 launch file.
        Example: orch.launch('my_robot_bringup', 'simulation.launch.py')
        """
        cmd = ["ros2", "launch", package_name, launch_file]
        if arguments:
            cmd.extend(arguments)
        
        # This uses the 'runner' context manager from the base class
        return self.runner(cmd)

    def wait_until_topic(
            self, 
            topic_name, 
            msg_type, 
            timeout=10.0,
            value_check=None
    ):
        """
        Blocks until a message is received on a specific topic.

        Args:
            topic_name (str): The name of the topic to subscribe to.
            msg_type: The ROS message type to expect (e.g., std_msgs.msg.String).
            timeout (float): Maximum time to wait in seconds.
            value_check (callable, optional): A function that takes a message and returns True if 
                it meets the desired condition. If None, any message will suffice.
        """
        received = []
        
        # Create a temporary subscription
        sub = self.node.create_subscription(
            msg_type, 
            topic_name, 
            lambda msg: received.insert(0, msg), 
            10
        )
        
        start_time = time.time()
        try:
            while time.time() - start_time < timeout:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if received:
                    if value_check is None or value_check(received[0]):
                        return True
            return False
        finally:
            self.node.destroy_subscription(sub)
