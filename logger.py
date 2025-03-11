import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
from geometry_msgs.msg import Twist
import psutil  # Import psutil for CPU & memory tracking
import time
import os
import threading
from datetime import datetime

LOG_FOLDER = "/home/agilex/logfolder/"  # Log directory

class NavMetricsLogger(Node):
    def __init__(self):
        super().__init__('nav_metrics_logger')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to Behavior Tree Log
        self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.bt_log_callback,
            qos_profile
        )

        # Subscribe to cmd_vel to track velocity
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            qos_profile
        )

        # Navigation session tracking
        self.nav_active = False
        self.start_time = None
        self.velocities = []

        # Tracking variables for CPU & system usage
        self.cpu_logs = []
        self.process = psutil.Process(os.getpid())  # Get process info
        self.previous_velocity = None
        self.previous_angular_velocity = None
        self.previous_timestamp = None
        self.max_angular_velocity = 0.0
        self.max_acceleration = 0.0
        self.max_angular_acceleration = 0.0

        # Recovery action tracking
        self.recovery_active = False
        self.recovery_count = 0
        self.recovery_actions = {'Spin': 0, 'BackUp': 0, 'Wait': 0}

        # Ensure log directory exists
        os.makedirs(LOG_FOLDER, exist_ok=True)

        self.get_logger().info("📡 Monitoring Navigation & CPU Metrics...")

    def get_next_trial_number(self):
        """ Determines the next available trial number. """
        existing_logs = [f for f in os.listdir(LOG_FOLDER) if f.startswith("trial")]
        trial_numbers = sorted([
            int(f.split("_")[0].replace("trial", "")) for f in existing_logs if f.split("_")[0].replace("trial", "").isdigit()
        ])
        return trial_numbers[-1] + 1 if trial_numbers else 1

    def write_log_file(self, log_data):
        """ Writes the collected trial data to a log file. """
        trial_number = self.get_next_trial_number()
        timestamp = datetime.now().strftime("%m-%d-%y_%I-%M%p").replace("AMPM", "PM")
        filename = f"trial{trial_number}_{timestamp}.log"
        filepath = os.path.join(LOG_FOLDER, filename)

        with open(filepath, "w") as log_file:
            log_file.write(log_data)

        self.get_logger().info(f"📄 Log saved: {filepath}")

    def bt_log_callback(self, msg):
        """ Callback for BehaviorTreeLog messages. Detects start/end of navigation and recovery actions. """
        for event in msg.event_log:
            node = event.node_name
            prev_status = event.previous_status
            curr_status = event.current_status

            # Detect start of navigation
            if node in ['ComputePathToPose', 'NavigateWithReplanning'] and curr_status == 'RUNNING' and not self.nav_active:
                self.start_navigation()

            # Detect end of navigation
            if node in ['FollowPath', 'NavigateRecovery'] and curr_status in ['SUCCESS', 'IDLE'] and self.nav_active:
                self.end_navigation()

    def velocity_callback(self, msg):
        """ Callback for cmd_vel messages to track velocity and compute new metrics. """
        current_time = time.time()

        # Compute linear speed magnitude
        speed = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
        self.velocities.append(speed)

        # Track maximum angular velocity
        self.max_angular_velocity = max(self.max_angular_velocity, abs(msg.angular.z))

        # Compute acceleration if previous data exists
        if self.previous_velocity is not None and self.previous_timestamp is not None:
            dt = current_time - self.previous_timestamp
            if dt > 0:
                linear_acceleration = (speed - self.previous_velocity) / dt
                angular_acceleration = (msg.angular.z - self.previous_angular_velocity) / dt

                self.max_acceleration = max(self.max_acceleration, abs(linear_acceleration))
                self.max_angular_acceleration = max(self.max_angular_acceleration, abs(angular_acceleration))

        # Store current values for next calculation
        self.previous_velocity = speed
        self.previous_angular_velocity = msg.angular.z
        self.previous_timestamp = current_time

        # Track CPU Usage
        self.record_cpu_usage()

    def start_cpu_monitoring(self):
        """ Starts continuous CPU monitoring in a separate thread. """
        self.cpu_logs = []  # Reset logs at trial start
        self.monitoring_active = True

        def monitor():
            while self.monitoring_active:
                self.record_cpu_usage()
                time.sleep(0.5)  # Sample CPU data every 0.5 seconds

        self.cpu_thread = threading.Thread(target=monitor)
        self.cpu_thread.start()

    def stop_cpu_monitoring(self):
        """ Stops CPU monitoring and calculates averages. """
        self.monitoring_active = False
        self.cpu_thread.join()  # Wait for monitoring thread to finish

    def record_cpu_usage(self):
        """ Continuously samples CPU usage while trial is active. """
    
        total_cpu = psutil.cpu_percent(interval=0.1)
        process_cpu = self.process.cpu_percent(interval=0.1)
        cpu_usage_adjusted = max(0, total_cpu - process_cpu)  # Deduct script's CPU usage

        ram_usage = psutil.virtual_memory().percent
        swap_usage = psutil.swap_memory().percent
        system_load = os.getloadavg()

        timestamp = time.time() - self.start_time if self.start_time is not None else 0.0

        self.cpu_logs.append((timestamp, total_cpu, process_cpu, cpu_usage_adjusted, ram_usage, swap_usage, system_load))
        
    def start_recovery(self, action):
        """ Marks the start of a recovery action. """
        self.recovery_active = True
        self.recovery_count += 1
        if action in self.recovery_actions:
            self.recovery_actions[action] += 1
        self.get_logger().info(f"⚠️ Recovery triggered: {action}")

    def end_recovery(self, action):
        """ Marks the end of a recovery action. """
        self.recovery_active = False
        self.get_logger().info(f"✅ Recovery completed: {action}")
    
    def start_navigation(self):
        """ Marks the start of a navigation goal. """
        self.nav_active = True
        self.start_time = time.time()
        self.velocities.clear()
        self.cpu_logs.clear()
        self.recovery_count = 0
        self.max_angular_velocity = 0.0
        self.max_acceleration = 0.0
        self.max_angular_acceleration = 0.0
        self.recovery_actions = {'Spin': 0, 'BackUp': 0, 'Wait': 0}  # Reset
        self.start_cpu_monitoring()
        self.get_logger().info("🚦 Navigation started.")
        
    def end_navigation(self):
        """ Marks the end of navigation and calculates metrics. """
        if not self.nav_active:
            return
        self.stop_cpu_monitoring()  # Stop the CPU monitoring loop
        end_time = time.time()
        total_time = end_time - self.start_time
        avg_velocity = sum(self.velocities) / len(self.velocities) if self.velocities else 0.0
        max_velocity = max(self.velocities, default=0.0)

        if self.cpu_logs:
            time_intervals = [self.cpu_logs[i][0] - self.cpu_logs[i-1][0] for i in range(1, len(self.cpu_logs))]
            weighted_cpu_usage = sum(self.cpu_logs[i][3] * time_intervals[i-1] for i in range(1, len(self.cpu_logs)))
            avg_cpu = weighted_cpu_usage / sum(time_intervals) if sum(time_intervals) > 0 else 0.0
        else:
            avg_cpu = 0.0

        max_cpu = max(log[3] for log in self.cpu_logs) if self.cpu_logs else 0.0

        # Create log data
        log_data = (
            f"Trial Number: {self.get_next_trial_number()}\n"
            f"Date: {datetime.now().strftime('%m-%d-%y')}\n"
            f"Time: {datetime.now().strftime('%I-%M%p')}\n"
            f"Time Taken: {total_time:.2f} sec\n"
            f"Average Velocity: {avg_velocity:.3f} m/s\n"
            f"Max Velocity: {max_velocity:.3f} m/s\n"
            f"Max Angular Velocity: {self.max_angular_velocity:.3f} rad/s\n"
            f"Max Acceleration: {self.max_acceleration:.3f} m/s²\n"
            f"Max Angular Acceleration: {self.max_angular_acceleration:.3f} rad/s²\n"
            f"Max CPU Usage: {max_cpu:.2f}%\n"
            f"Average CPU Usage: {avg_cpu:.2f}%\n"
            f"Total Recoveries: {self.recovery_count}\n"
            f"Recovery Breakdown: {self.recovery_actions}\n"
        )

        self.write_log_file(log_data)

        # Reset tracking
        self.nav_active = False
        self.start_time = None
        self.velocities.clear()

def main(args=None):
    rclpy.init(args=args)
    node = NavMetricsLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

