

import signal
import subprocess
import uuid
import os
import psutil
import pytest
import time
import threading

from pytest_ros import RCLPY_AVAILABLE
if RCLPY_AVAILABLE:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor

from contextlib import contextmanager
    


class MultiProcessManager:
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
            preexec_fn=os.setsid,
            shell=True
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


class ROSProcessManager(MultiProcessManager):
    """
    A specialized context manager for managing ROS processes, ensuring that all ROS nodes
    launched during the pytest session are properly terminated at the end of the session.
    
    Manages ROS 2 launch files.

    Example:
    with orch.launch(package_name='my_package', launch_file='my_launch.py', arguments=['arg1:=value1']):
        # Run tests that depend on the launched ROS nodes
    """
    def __init__(self, ):
        super().__init__(tag="ROS_SESSION_ID")

    def launch(self, package_name, launch_file, arguments=None):
        """
        Launches a ROS 2 launch file.
        
        Returns a context manager that can be used to manage the lifecycle of the launched processes.

        Example: orch.launch(arguments=['arg1:=value1', 'arg2:=value2'])
        """
        cmd = ["ros2", "launch", package_name, launch_file]
        if arguments:
            cmd.extend(arguments)

        cmd = ' '.join(cmd)  # Join the command list into a single string for better logging

        print(f"Launching ROS with command: {cmd}")  # Debug statement to verify the command being launched
        
        # This uses the 'runner' context manager from the base class
        return self.runner(cmd)
    

    def run(self, package_name, run_process, arguments=None):
        """        
        Returns a context manager that can be used to manage the lifecycle of the run process.

        Example: orch.run(package_name='my_package', run_process='my_run.py', arguments=['arg1:=value1', 'arg2:=value2'])
        """
        cmd = ["ros2", "run", package_name, run_process]
        if arguments:
            cmd.extend(arguments)
        
        # This uses the 'runner' context manager from the base class
        return self.runner(cmd)


class EpisodeMonitor:
    """
    A class to monitor test episodes, enabling waiting for start conditions and monitoring for 
    termination conditions, both success and failure.

    The conditions are defined as ROS topics with either a requirement for a message to be received
    or a specific value to be met.
    """

    def __init__(
        self,
        node_name: str,
    ):
        
        self._conditions = {} # dict of condition name to its 'value' (True/False/None) and 'interface' (e.g. subscriber or service client)
        self._statuses = {}
        
        # Initialize the internal ROS 2 node
        self.node = Node(node_name)

        # Setup threading and execution infrastructure
        self._stop_event = threading.Event()
        self._is_running = threading.Event()
        
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        
        self._spin_thread = threading.Thread(target=self._run, daemon=True)
        self._spin_thread.start()
        
        if not self._is_running.wait(timeout=2.0):
            raise RuntimeError("ROS 2 background executor failed to start.")
        
        # Create default statuses 
        self.register_status(status_name='timeout', conditions=[], trigger='ANY') # Evaluates to True if any timeouts occur

    def register_condition(self, condition_name, topic_name, msg_type, value_check=None):
        """
        Registers a condition based on a ROS topic. 
        Conditions can be True, False, or None (not yet received).

        Parameters:
            condition_name (str): The name of the condition.
            topic_name (str): The name of the ROS topic to subscribe to.
            msg_type: The ROS message type to expect (e.g., std_msgs.msg.String).
            value_check (callable, optional): A function that takes a message and returns True if it meets the desired condition. If None, any message will suffice.
        """
        self._conditions[condition_name] = {'value': None, 'interface': None}  # Initialize condition value as None
        self._conditions[condition_name]['interface'] = self._setup_subscription(
            condition_name=condition_name,
            topic_name=topic_name, 
            msg_type=msg_type, 
            value_check=value_check
        )

    
    def get_condition(self, condition_name):
        """
        Retrieves the current value of a condition.

        Parameters:
            condition_name (str): The name of the condition.

        Returns:
            bool: The current value of the condition, or None if the condition is not registered.
        """
        condition_info = self._conditions.get(condition_name)
        if condition_info:
            return condition_info['value']
        return None
    
    def register_service_condition(self, 
        condition_name, 
        service_name, 
        srv_type,
        srv_req,
        value_check=None,
        period: float = 1.0,
        start_on_status: str = None,
        timeout: float = None
    ):
        """
        Registers a condition based on a ROS service.

        Parameters:
            condition_name (str): The name of the condition.
            service_name (str): The name of the ROS service to call.
            srv_type: The ROS service type to expect (e.g., std_srvs.srv.Trigger).
            srv_req (srv_type.Request or callable): The request object or a function that returns the request object for the service call.
            value_check (callable, optional): A function that takes a service response and returns True if it meets the desired condition. If None, any response will suffice.
            period (float): How often to call the service in seconds.
            start_on_status (str, optional): If provided, the service calls will only start once this status is triggered.
            timeout (float, optional): Maximum time to keep trying the service call before giving up. If None, it will keep trying indefinitely.
        """
        self._conditions[condition_name] = {'value': None, 'interface': None}  # Initialize condition value as None
        if timeout is not None:
            self._conditions[f'{condition_name}_timeout'] = {'value': False, 'interface': None} 
            self.add_condition_to_status('timeout', f'{condition_name}_timeout')
        self._conditions[condition_name]['interface'] = self._setup_client(
            condition_name=condition_name,
            service_name=service_name,
            srv_type=srv_type,
            srv_req=srv_req,
            value_check=value_check,
            period=period,
            start_on_status=start_on_status,
            timeout=timeout
        )
    
    def register_status(self, status_name, conditions, trigger='ALL'):
        """
        Registers a status to monitor.

        Parameters:
            status_name (str): The name of the status.
            conditions (list of str): A list of condition names that contribute to this status.
            trigger (str): 'ANY' or 'ALL' - determines how multiple conditions are evaluated.
        """
        # Prevent overwriting existing statuses
        if status_name in self._statuses:
            self.node.get_logger().warning(f"Status '{status_name}' is already registered. Skipping registration.")
            return
        self._statuses[status_name] = {
            'trigger': trigger,
            'conditions': conditions
        }
        self._conditions.setdefault(status_name, {'value': None, 'interface': 'status'})  # Ensure the status itself is also a condition for potential nesting

    def add_condition_to_status(self, status_name, condition_name):
        """
        Dynamically adds a condition to an existing status.

        Parameters:
            status_name (str): The name of the status to update.
            condition_name (str): The name of the condition to add.
        """
        if status_name in self._statuses:
            self._statuses[status_name]['conditions'].append(condition_name)
        else:
            self.node.get_logger().warning(f"Status '{status_name}' not found. Cannot add condition '{condition_name}'.")

    def get_status(self, status_name):
        """
        Checks the status by dynamically resolving its underlying condition values.

        Returns:
            bool: True if the logical trigger condition is met, False otherwise.
        """
        status = self._statuses.get(status_name)
        if not status:
            self.node.get_logger().warning(f"Status '{status_name}' is not registered.")
            return False

        # If a condition is actually another status, we need to evaluate it recursively by calling get_status on it.
        # TODO: This could be optimized with memoization if we expect deep nesting, but for now we assume simple cases.
        for cond in status['conditions']:
            if self._conditions[cond]['interface'] == 'status':
                # If the condition is actually another status, we need to evaluate it recursively
                self._conditions[cond]['value'] = self.get_status(cond)
        
        # Map the list of condition strings to their current real-time boolean states
        # Using .get('value', False) handles uninitialized or pending states safely.
        resolved_values = [
            self._conditions.get(cond, {}).get('value', False) 
            for cond in status['conditions']
        ]

        # 2. Evaluate the values against the gate logic
        if status['trigger'] == 'ANY':
            return any(resolved_values)
        elif status['trigger'] == 'ALL':
            return all(resolved_values)
            
        return False
    

    def _update_condition(self, condition_name, value):
        """
        Updates the value of a condition and checks if the associated status is triggered.
        """
        self._conditions[condition_name]['value'] = value

    def _setup_subscription(self, condition_name, topic_name, msg_type, value_check=None):
        """
        Sets up a subscription to a topic and returns a function that checks for the desired condition.
        """
        sub = self.node.create_subscription(
            msg_type, 
            topic_name, 
            lambda msg: self._update_condition(condition_name, value_check(msg) if value_check else True),
            10
        )

        return sub

    def _setup_client(self,
            condition_name, 
            service_name, 
            srv_type,
            srv_req,
            value_check=None,
            period: float = 1.0,
            start_on_status: str = None,
            timeout: float = None
        ):
        """
        Sets up a service client that periodically calls the service and updates the condition based on the response.
        """
        client = self.node.create_client(
            srv_type, 
            service_name
        )
        
        def service_poll_worker():
            """Background thread that periodically calls the service and updates the condition status."""
            
            worker_start_time = None

            def check_timeout_breach():
                if timeout is not None and (time.time() - worker_start_time) > timeout:
                    self.node.get_logger().error(f"[{condition_name}] Master timeout of {timeout}s exceeded.")
                    # Explicitly fail the condition so the test state knows it timed out
                    self._update_condition(condition_name, False)
                    self._update_condition(f'{condition_name}_timeout', True)  # Set the timeout flag
                    return True
                return False
            
            if start_on_status:
                # Wait until the specified status is triggered before starting the service calls
                while rclpy.ok() and not self._stop_event.is_set():
                    if self.get_status(start_on_status):
                        worker_start_time = time.time()
                        break
                    time.sleep(0.1)
            else:
                worker_start_time = time.time()
            
            # Wait for the service to become available
            while rclpy.ok() and not self._stop_event.is_set():
                if check_timeout_breach():
                    return
                if client.wait_for_service(timeout_sec=1.0):
                    break

            # Periodically call the service and check the response
            while rclpy.ok() and not self._stop_event.is_set():
                if check_timeout_breach():
                    return
                start_time = time.time()
                # Before making the service call, check if the condition has already been met to avoid unnecessary calls
                if self._conditions[condition_name]['value']:
                    break

                req = srv_req() if callable(srv_req) else srv_req

                future = client.call_async(req)
                
                timeout_limit = 5.0
                while rclpy.ok() and not self._stop_event.is_set() and not future.done():
                    if time.time() - start_time > timeout_limit:
                        break
                    time.sleep(0.01) # Cooperatively yield execution slice

                if future.done():
                    try:
                        response = future.result()
                        if value_check is None or value_check(response):
                            self._update_condition(condition_name, True)
                            break
                    except Exception as e:
                        self.node.get_logger().error(f"Service call failed: {e}")
                else:
                    self.node.get_logger().error("Service call timed out")

                elapsed = time.time() - start_time
                time_to_sleep = max(0, period - elapsed)
                time.sleep(time_to_sleep)    
        
        poll_thread = threading.Thread(target=service_poll_worker, daemon=True)
        poll_thread.start()
    

    def _run(self):
        """Internal background loop running the executor."""
        self._is_running.set()
        
        while rclpy.ok() and not self._stop_event.is_set():
            # Safely process pending callbacks on the internal node
            self._executor.spin_once(timeout_sec=0.0)
            
        

    def shutdown(self):
        """
        Shuts down the internal ROS node and cleans up resources.
        """
        self._stop_event.set()
        
        if hasattr(self, '_executor'):
            self._executor.shutdown()
            
        if hasattr(self, '_spin_thread') and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
            
        # Clean up the node itself
        if hasattr(self, 'node'):
            self.node.destroy_node()


    
@pytest.fixture
def ros_launcher():
    """Factory fixture to manage ROS launch processes."""

    def _launch(package_name, launch_file, arguments=None):
        orch = ROSProcessManager()
        return orch.launch(package_name, launch_file, arguments)
    
    yield _launch

@pytest.fixture
def ros_monitor():
    """Fixture to provide an EpisodeMonitor instance for monitoring test conditions."""
    rclpy.init()
    
    monitor = EpisodeMonitor(node_name="episode_monitor")

    yield monitor

    monitor.shutdown()
    rclpy.shutdown()