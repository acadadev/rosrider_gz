#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import signal
import sys
import threading

# Using instance variables instead of global lists now

class OdometryPlotter(Node):
    """
    A ROS 2 node that subscribes to two Odometry topics and plots their X/Y positions
    on the same graph in real-time.
    """
    def __init__(self):
        """
        Initializes the node, subscribers, plot, and data storage for two topics.
        """
        super().__init__('dual_odometry_plotter_node')
        self.get_logger().info('Dual Odometry Plotter Node started.')

        # --- Data Storage ---
        # Lists to store trajectory data for each topic
        self.x_data_1, self.y_data_1 = [], []
        self.x_data_2, self.y_data_2 = [], []

        # --- Parameters ---
        # Declare and get parameters for both topic names
        self.declare_parameter('topic_name_1', '/odometry/filtered')
        self.declare_parameter('topic_name_2', '/next/odometry')
        topic_name_1 = self.get_parameter('topic_name_1').get_parameter_value().string_value
        topic_name_2 = self.get_parameter('topic_name_2').get_parameter_value().string_value
        self.get_logger().info(f"Subscribing to Topic 1: {topic_name_1} (Red)")
        self.get_logger().info(f"Subscribing to Topic 2: {topic_name_2} (Green)")

        # --- Subscribers ---
        # Create subscribers for both Odometry topics
        self.subscription_1 = self.create_subscription(
            Odometry,
            topic_name_1,
            self.odometry_callback_1, # Link to the first callback
            10)  # QoS profile depth

        self.subscription_2 = self.create_subscription(
            Odometry,
            topic_name_2,
            self.odometry_callback_2, # Link to the second callback
            10)  # QoS profile depth

        # --- Plotting Setup ---
        plt.ion()  # Turn on interactive mode for plotting
        self.fig, self.ax = plt.subplots()

        # Initial plot setup for both lines
        # label parameter is used for the legend
        self.line_1, = self.ax.plot(self.x_data_1, self.y_data_1, 'r-', label=topic_name_1)
        self.line_2, = self.ax.plot(self.x_data_2, self.y_data_2, 'g-', label=topic_name_2)

        # Configure plot appearance
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Real-time Dual Odometry Plot (X vs Y)')
        self.ax.grid(True)
        self.ax.axis('equal') # Ensure aspect ratio is equal
        self.ax.legend() # Add a legend to distinguish the lines

        # --- Plot Update Lock ---
        # Use a lock to prevent race conditions when updating the plot from different callbacks
        self.plot_lock = threading.Lock()

        # --- Initial Plot Update ---
        # Draw the initial empty plot
        self.fig.canvas.draw()
        plt.show(block=False) # Display the plot without blocking

    def _update_plot(self):
        """
        Helper function to update the plot axes and redraw the canvas.
        This should be called after updating line data within a lock.
        """
        # --- Rescale Plot Axes ---
        # Adjust plot limits dynamically to fit all data
        self.ax.relim()  # Recalculate limits based on all lines
        self.ax.autoscale_view(True, True, True) # Autoscale axes

        # --- Redraw Plot ---
        # Redraw the canvas
        self.fig.canvas.draw()
        # Pause briefly to allow the plot to update and process GUI events
        # Needs to be done outside the lock if possible, but here is safer for simplicity
        plt.pause(0.001) # Use a very small pause

    def odometry_callback_1(self, msg: Odometry):
        """
        Callback for the first odometry topic. Updates the red line.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        with self.plot_lock: # Acquire lock before modifying shared plot resources
            # Append new data points for the first trajectory
            self.x_data_1.append(x)
            self.y_data_1.append(y)

            # Update plot data for the first line
            self.line_1.set_xdata(self.x_data_1)
            self.line_1.set_ydata(self.y_data_1)

            # Update the plot (rescale and redraw)
            self._update_plot()

    def odometry_callback_2(self, msg: Odometry):
        """
        Callback for the second odometry topic. Updates the green line.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        with self.plot_lock: # Acquire lock before modifying shared plot resources
            # Append new data points for the second trajectory
            self.x_data_2.append(x)
            self.y_data_2.append(y)

            # Update plot data for the second line
            self.line_2.set_xdata(self.x_data_2)
            self.line_2.set_ydata(self.y_data_2)

            # Update the plot (rescale and redraw)
            self._update_plot()

    def shutdown_plot(self):
        """
        Closes the matplotlib plot window.
        """
        self.get_logger().info('Closing plot window...')
        # Check if figure exists and is managed by pyplot before closing
        if self.fig and plt.fignum_exists(self.fig.number):
             plt.close(self.fig)

def main(args=None):
    """
    Main function to initialize ROS, create the node, spin, and handle shutdown.
    """
    rclpy.init(args=args)
    odometry_plotter_node = None
    try:
        odometry_plotter_node = OdometryPlotter()

        # Define a signal handler for graceful shutdown (Ctrl+C)
        def signal_handler(sig, frame):
            print('\nCtrl+C detected. Shutting down node...')
            # Shutdown sequence is important: close plot, destroy node, shutdown rclpy
            if odometry_plotter_node:
                odometry_plotter_node.shutdown_plot() # Close plot first
                odometry_plotter_node.destroy_node()
            if rclpy.ok():
                 rclpy.shutdown()
            sys.exit(0) # Exit the script

        signal.signal(signal.SIGINT, signal_handler)

        # Keep the node running until shutdown is requested
        rclpy.spin(odometry_plotter_node)

    except KeyboardInterrupt:
        # This handles the case where spin is interrupted before the signal handler
        print("KeyboardInterrupt caught, initiating shutdown...")
    except Exception as e:
        if odometry_plotter_node:
            odometry_plotter_node.get_logger().error(f"An error occurred: {e}", exc_info=True)
        else:
            print(f"An error occurred during initialization: {e}")
    finally:
        # Ensure resources are released even if errors occur or spin exits normally
        print("Executing final cleanup...")
        if odometry_plotter_node:
            # Check if node is still valid before destroying
            try:
                 # Attempt to destroy the node if it hasn't been already
                 if rclpy.ok() and odometry_plotter_node.context.ok():
                     odometry_plotter_node.shutdown_plot() # Ensure plot is closed
                     odometry_plotter_node.destroy_node()
            except Exception as destroy_e:
                 print(f"Error during node destruction: {destroy_e}")
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS Cleanup complete.")


if __name__ == '__main__':
    main()
