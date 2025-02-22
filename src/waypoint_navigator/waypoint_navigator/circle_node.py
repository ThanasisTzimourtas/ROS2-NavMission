#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn, LifecycleNode
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import time

class CircleNode(LifecycleNode):
    def __init__(self):
        super().__init__('circle_node', namespace='a200_0000')
        self._service = None
        self._cmd_vel_pub = None
        self._pose_sub = None
        self.executing_circle = False
        self.current_pose = None
        
        # Movement parameters
        self.ANGULAR_VEL = 0.3  # rad/s
        self.PUBLISH_RATE = 0.05  # 20Hz
        self.TARGET_ROTATION = 2 * math.pi  # Full 360 degrees

    def on_configure(self, state):
        """Configure node and create service."""
        self.get_logger().info('Configuring circle node...')
        
        self._service = self.create_service(
            Trigger,
            'execute_circle',
            self.circle_callback)
        
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10)
            
        return TransitionCallbackReturn.SUCCESS

    def pose_callback(self, msg):
        """Store current pose."""
        self.current_pose = msg

    def on_activate(self, state):
        """Activate node."""
        self.get_logger().info('Activating circle node...')
        
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        # Ensure robot starts from stop
        time.sleep(0.5)
        self.publish_zero_velocity()
        
        return TransitionCallbackReturn.SUCCESS
        
    def on_deactivate(self, state):
        """Deactivate node."""
        self.get_logger().info('Deactivating circle node...')
        
        # Stop execution if running
        self.executing_circle = False
        
        # Stop the robot
        if hasattr(self, '_cmd_vel_pub') and self._cmd_vel_pub:
            try:
                self.publish_zero_velocity()
            except Exception as e:
                self.get_logger().warn(f'Error stopping robot: {e}')
        
        return TransitionCallbackReturn.SUCCESS

    def circle_callback(self, request, response):
        """Execute rotation movement."""
        if self.executing_circle:
            response.success = False
            response.message = "Already executing movement"
            return response

        try:
            self.executing_circle = True
            
            if not self.wait_for_pose():
                raise RuntimeError("Failed to get initial pose")
            
            initial_yaw = self.get_yaw_from_pose(self.current_pose)
            self.get_logger().info(f'Starting rotation from yaw: {math.degrees(initial_yaw):.1f}°')
            
            success = self.execute_rotation(initial_yaw)
            
            response.success = success
            response.message = "Rotation completed successfully" if success else "Rotation failed"
            
        except Exception as e:
            self.get_logger().error(f'Rotation failed: {str(e)}')
            response.success = False
            response.message = str(e)
            
        finally:
            self.executing_circle = False
            self.publish_zero_velocity()
        
        return response

    def wait_for_pose(self, timeout=5.0):
        """Wait for a valid pose estimate."""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.current_pose is not None:
                return True
            time.sleep(0.1)
        return False

    def execute_rotation(self, initial_yaw):
        """Execute pure Z-axis rotation."""
        if not self._cmd_vel_pub:
            return False

        # Add extra time to ensure full rotation
        duration = (self.TARGET_ROTATION / self.ANGULAR_VEL) * 1.8
        cmd_vel = Twist()
        cmd_vel.angular.z = self.ANGULAR_VEL

        start_time = time.time()
        last_log_time = start_time
        rotation_complete = False

        try:
            while rclpy.ok() and self.executing_circle and not rotation_complete:
                current_time = time.time()
                elapsed = current_time - start_time
                
                # Check if we've completed the rotation
                if self.current_pose:
                    current_yaw = self.get_yaw_from_pose(self.current_pose)
                    rotation_diff = abs(current_yaw - initial_yaw)
                    if elapsed >= duration:
                        rotation_complete = True
                
                # Publish velocity command
                if self._cmd_vel_pub and not rotation_complete:
                    self._cmd_vel_pub.publish(cmd_vel)

                # Log progress
                if current_time - last_log_time >= 1.0:
                    progress = min((elapsed / duration) * 100, 100)
                    self.get_logger().info(
                        f'Rotation progress: {progress:.1f}% ({elapsed:.1f}/{duration:.1f}s)'
                    )
                    if self.current_pose:
                        current_yaw = self.get_yaw_from_pose(self.current_pose)
                        self.get_logger().info(
                            f'Current yaw: {math.degrees(current_yaw):.1f}°'
                        )
                    last_log_time = current_time

                time.sleep(self.PUBLISH_RATE)

            # Stop motion
            self.publish_zero_velocity()
            
            return rotation_complete

        except Exception as e:
            self.get_logger().error(f'Error during rotation: {e}')
            return False

    def get_yaw_from_pose(self, pose_msg):
        """Extract yaw from pose quaternion."""
        q = pose_msg.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_zero_velocity(self):
        """Publish zero velocity."""
        if hasattr(self, '_cmd_vel_pub') and self._cmd_vel_pub:
            try:
                zero_cmd = Twist()
                for _ in range(5):
                    self._cmd_vel_pub.publish(zero_cmd)
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().warn(f'Error publishing zero velocity: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = CircleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()