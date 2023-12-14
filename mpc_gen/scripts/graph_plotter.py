#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import pandas as pd
import matplotlib.pyplot as plt
import threading
import atexit
import signal

class OdomPlotter:
    def __init__(self, save_path):
        rospy.init_node('odom_plotter', anonymous=True)
        rospy.Subscriber("/gem/base_footprint/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cte", Float64, self.cte_callback)
        self.odom_data = {'x': [], 'y': []}
        self.cte_data = []
        self.lock = threading.Lock()
        self.save_path = save_path
        signal.signal(signal.SIGINT, self.signal_handler)

    def odom_callback(self, msg):
        with self.lock:
            # Extract odometry data
            self.odom_data['x'].append(msg.pose.pose.position.x)
            self.odom_data['y'].append(msg.pose.pose.position.y)
            
    def cte_callback(self, msg):
        with self.lock:
            self.cte_data.append(msg.data)

    def plot_and_save(self, path_csv):
        path_data = pd.read_csv(path_csv)
        plt.figure(figsize=(12, 8))

        # Subplot for Odometry and Path Trajectory
        plt.subplot(2, 1, 1)
        plt.plot(self.odom_data['x'], self.odom_data['y'], label='Odometry', color='blue', linewidth=2)
        plt.plot(path_data['path_x'], path_data['path_y'], label='Path Trajectory', color='red', linestyle='--', linewidth=2)
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Odometry and Path Trajectory')
        plt.legend()
        plt.grid(True)

        # Subplot for CTE
        plt.subplot(2, 1, 2)
        plt.plot(self.cte_data, label='CTE', color='green', linewidth=2)
        plt.xlabel('Time Step')
        plt.ylabel('CTE')
        plt.title('Cross-Track Error over Time')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig(self.save_path, dpi=300, format='png')
        print(f"Plot saved to {self.save_path}")
        plt.show()

    def signal_handler(self, signum, frame):
        self.plot_and_save('/home/oem/gem2_ws/src/mpc_ros/mpc_gen/src/wps_plot.csv')
        exit(0)

if __name__ == '__main__':
    save_path = '/home/oem/gem2_ws/src/mpc_ros/mpc_gen/odom_trajectory_plot.png'
    plotter = OdomPlotter(save_path)
    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
    finally:
        plotter.plot_and_save('/home/oem/gem2_ws/src/mpc_ros/mpc_gen/src/wps_plot.csv') 

