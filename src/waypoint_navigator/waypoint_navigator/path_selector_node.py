#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from lifecycle_msgs.msg import State
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import math
import random

class PathSelectorNode(LifecycleNode):
    def __init__(self):
        super().__init__('path_selector', namespace='a200_0000')
        
        # Initialize variables
        self.current_pose = None
        self._nav_client = None
        self._pose_sub = None
        self._random_nav_service = None
        self._execute_nav_service = None
        self.random_nav_count = 0
        self.max_random_events = 2
        self.interrupted_waypoints = set()
        
        # Track current state
        self._current_state = State.PRIMARY_STATE_UNCONFIGURED
        
        # Alternative poses that can be visited
        self.alternative_poses = [
            self.create_pose_stamped(11.175, 21.708, 1.582),    # Option 1
            self.create_pose_stamped(-12.745, -8.339, 1.605)   # Option 2
        ]
        
        self.get_logger().info('Path selector node initialized')

    def on_configure(self, state) -> TransitionCallbackReturn:
        """Configure node with subscribers and action client."""
        try:
            self.get_logger().info('Configuring path selector node...')
            self._current_state = State.PRIMARY_STATE_INACTIVE
            
            callback_group = ReentrantCallbackGroup()
            
            # Create action client for navigation
            self._nav_client = ActionClient(
                self,
                NavigateToPose,
                'navigate_to_pose',
                callback_group=callback_group
            )
            
            # Create pose subscriber
            self._pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                'amcl_pose',
                self.pose_callback,
                10)
                
            # Create services
            self._random_nav_service = self.create_service(
                Trigger,
                'check_random_nav',
                self.check_random_nav_callback
            )
            
            self._execute_nav_service = self.create_service(
                Trigger,
                'execute_random_nav',
                self.execute_random_nav_callback
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Error configuring node: {str(e)}')
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state) -> TransitionCallbackReturn:
        """Activate the node."""
        self.get_logger().info('Activating path selector node...')
        self._current_state = State.PRIMARY_STATE_ACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info('Deactivating path selector node...')
        self._current_state = State.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.get_logger().info('Cleaning up path selector node...')
        self._current_state = State.PRIMARY_STATE_UNCONFIGURED
        if self._nav_client:
            self._nav_client.destroy()
            self._nav_client = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info('Shutting down path selector node...')
        self._current_state = State.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS

    def is_active(self) -> bool:
        """Check if node is in active state."""
        return self._current_state == State.PRIMARY_STATE_ACTIVE

    def pose_callback(self, msg):
        """Store current pose."""
        self.current_pose = msg

    def create_pose_stamped(self, x, y, theta):
        """Create PoseStamped message with quaternion orientation."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta/2)
        pose.pose.orientation.w = math.cos(theta/2)
        return pose

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses."""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def should_trigger_random_event(self):
        """Determine if we should trigger a random navigation event."""
        if not self.is_active():
            return False
            
        if self.random_nav_count >= self.max_random_events:
            return False
            
        # 30% chance of triggering
        should_trigger = random.random() < 0.3
        
        if should_trigger:
            self.get_logger().info('Random navigation event triggered')
        
        return should_trigger

    def select_nearest_pose(self):
        """Select the nearest pose from available options."""
        if not self.current_pose:
            return None

        distances = []
        for pose in self.alternative_poses:
            dist = self.calculate_distance(
                self.current_pose.pose.pose,
                pose.pose
            )
            distances.append(dist)

        nearest_index = distances.index(min(distances))
        selected_pose = self.alternative_poses[nearest_index]
        
        self.get_logger().info(
            f'Selected nearest pose: ({selected_pose.pose.position.x}, {selected_pose.pose.position.y})'
        )
        return selected_pose

    async def check_random_nav_callback(self, request, response):
        """Service callback to check if random navigation should occur."""
        if not self.is_active():
            response.success = False
            response.message = "Node not active"
            return response

        should_navigate = self.should_trigger_random_event()
        response.success = should_navigate
        response.message = "Random navigation triggered" if should_navigate else "No random navigation"
        return response

    async def execute_random_nav_callback(self, request, response):
        """Service callback to execute random navigation."""
        if not self.is_active():
            response.success = False
            response.message = "Node not active"
            return response

        if not self.current_pose:
            response.success = False
            response.message = "No current pose available"
            return response

        try:
            selected_pose = self.select_nearest_pose()
            if not selected_pose:
                response.success = False
                response.message = "Failed to select nearest pose"
                return response

            # Create and send goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = selected_pose

            self.get_logger().info('Sending random navigation goal...')
            
            # Wait for action server
            if not self._nav_client.wait_for_server(timeout_sec=5.0):
                response.success = False
                response.message = "Navigation action server not available"
                return response

            send_goal_future = await self._nav_client.send_goal_async(goal_msg)
            if not send_goal_future.accepted:
                response.success = False
                response.message = "Goal rejected"
                return response

            result = await send_goal_future.get_result_async()
            success = bool(result.result.result)

            if success:
                self.random_nav_count += 1
                self.get_logger().info(f'Random navigation completed successfully. Count: {self.random_nav_count}')
                response.success = True
                response.message = "Random navigation completed"
            else:
                response.success = False
                response.message = "Navigation failed"

            return response

        except Exception as e:
            self.get_logger().error(f'Error during navigation: {str(e)}')
            response.success = False
            response.message = str(e)
            return response

def main(args=None):
    rclpy.init(args=args)
    
    path_selector = PathSelectorNode()
    
    try:
        rclpy.spin(path_selector)
    except KeyboardInterrupt:
        pass
    finally:
        path_selector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()