import signal
import subprocess
import uuid
import os
import psutil


class MultiProcessManager:
    """
    A utility class to manage multiple processes in a pytest session, 
    ensuring that all processes are properly terminated at the end of the session.
    """
    def __init__(self, tag="PYTEST_MULTIPROC_ID"):
        self.tag = tag
        self.session_id = str(uuid.uuid4())
        self.processes = []

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
        
        self.processes.append(proc)

        return proc


    def stop_process(self, proc):
        """Kill a process group."""
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

        if proc in self.processes:
            self.processes.remove(proc)

    def terminate_all(self):
        """Kills everything tagged with this session ID."""
        for proc in list(self.processes):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

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
    