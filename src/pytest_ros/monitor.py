import time
import threading

from pytest_ros import RCLPY_AVAILABLE
if RCLPY_AVAILABLE:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor


import logging


class EpisodeMonitor:
    """
    A class to monitor test episodes, enabling waiting for start conditions and monitoring for 
    termination conditions, both success and failure.

    The conditions are defined as ROS topics with either a requirement for a message to be received
    or a specific value to be met.
    """

    def __init__(
        self,
        name: str,
        timeout: float | None = None
    ):
        
        self.name = name
        self._logger = logging.getLogger(f'EpisodeMonitor.{self.name}')

        self._conditions = {} # dict of condition name to its 'value' (True/False/None) and 'interface' (e.g. subscriber or service client)
        self._statuses = {}
        
        # Monitor start time
        self._start_time = time.time()
        self._timeout = timeout

        # Initialize the internal ROS 2 node
        self.node = Node(name)

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
        self.register_condition(condition_name='episode_timeout')
        self.register_status(status_name='timeout', conditions=['episode_timeout'], trigger='ANY') # Evaluates to True if any timeouts occur
        self.register_status(status_name='complete', conditions=['timeout'], trigger='ANY') # A special status that can be triggered by any condition or externally to indicate episode completion
        
    def set_timeout(self, timeout: float):
        """Sets a new timeout for the episode. If the timeout is breached, the 'timeout' status will be triggered."""
        self._start_time = time.time()  # Reset start time whenever timeout is set
        self._timeout = timeout
        self._logger.info(f"Episode timeout set to {timeout} seconds from time {self._start_time}.")


    def register_condition(self, condition_name):
        """
        Registers a simple condition that can be manually set to True or False.

        Parameters:
            condition_name (str): The name of the condition.
        """
        self._conditions[condition_name] = {'value': None, 'interface': 'manual'}  # Initialize condition value as None

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

    def register_topic_condition(self, condition_name, topic_name, msg_type, value_check=None):
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

    

    def register_service_condition(self, 
        condition_name, 
        service_name, 
        srv_type,
        srv_req,
        value_check=None,
        period: float = 1.0,
        start_on_status: str = None,
        timeout: float = None,
        timeout_is_fatal: bool = False
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
            timeout_is_fatal (bool, optional): If True, the timeout will be reported as an episode timeout.
        """
        self._conditions[condition_name] = {'value': None, 'interface': None}  # Initialize condition value as None
        if timeout is not None:
            self._conditions[f'{condition_name}_timeout'] = {'value': False, 'interface': None} 
            if timeout_is_fatal:
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

        if len(status['conditions']) == 0:
            return False  # A status with no conditions is considered always False

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

    def wait_for_status(self, status_name: str, poll_interval: float = 0.1) -> bool:
        """
        Blocks synchronously until the status becomes True, 'complete' triggers,
        or the configured timeout duration is exceeded.
        
        Returns:
            bool: True if the status was successfully reached, False otherwise.
        """
        start_time = time.time()
        
        while not self.get_status(status_name):
            # 1. Check if an external mechanism or internal logic set 'complete'
            if self.get_status("complete"):
                break
                
            # 2. Check if the time elapsed exceeds our timeout limit
            if (time.time() - start_time) > self._timeout:
                self._statuses["timeout"] = True
                return False
                
            # 3. Yield thread execution to allow rclpy executor thread 
            # or topic callbacks to flush and update data structures
            time.sleep(poll_interval)
            
        # Double check we didn't exit the loop due to a break condition failure
        return self.get_status(status_name)

    def _update_condition(self, condition_name, value):
        """
        Updates the value of a condition and checks if the associated status is triggered.
        """
        self._conditions[condition_name]['value'] = value
        self._logger.info(f"Condition '{condition_name}' updated: {value}.")

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

            # Check for episode timeout
            if self._timeout is not None and (time.time() - self._start_time) > self._timeout:
                self.node.get_logger().error(f"Episode timeout of {self._timeout}s exceeded.")
                self._update_condition('episode_timeout', True)

                break
                

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
