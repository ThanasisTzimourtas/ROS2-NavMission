#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn, LifecycleNode
from rclpy.action import ActionServer, GoalResponse
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math
from std_msgs.msg import Empty
import uuid

class WaypointNavigator(LifecycleNode):
    def __init__(self):
        super().__init__('waypoint_navigator', namespace='a200_0000')
        self._action_server = None
        self._tf_broadcaster = None
        self._tf_timer = None
        self._active_goals = {}  # Track active goals

    def on_configure(self, state):
        """Configure node and create action server."""
        self.get_logger().info('Configuring waypoint navigator node...')
        
        # Create transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        
        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback)
        
        self.get_logger().info('Waypoint navigator configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def goal_callback(self, goal_request):
        """Accept or reject a new goal."""
        # Generate a unique identifier for this goal
        goal_id = str(uuid.uuid4())
        self._active_goals[goal_id] = None
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Handle a newly accepted goal."""
        # Store the goal handle
        goal_id = str(uuid.uuid4())
        self._active_goals[goal_id] = goal_handle
        return goal_handle

    async def execute_callback(self, goal_handle):
        """Execute goal callback for navigation action."""
        try:
            self.get_logger().info('Executing navigation goal...')
            
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.distance_remaining = 0.0
            goal_handle.publish_feedback(feedback_msg)
            
            result = NavigateToPose.Result()
            result.result = Empty()
            
            # Remove this goal from active goals
            goal_id = next((gid for gid, gh in self._active_goals.items() if gh == goal_handle), None)
            if goal_id:
                del self._active_goals[goal_id]
            
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f'Error in execute callback: {str(e)}')
            raise

    def on_activate(self, state):
        """Activate node."""
        self.get_logger().info('Activating waypoint navigator node...')
        # Start publishing transforms
        self._tf_timer = self.create_timer(0.1, self.publish_tf)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Deactivate node."""
        self.get_logger().info('Deactivating waypoint navigator node...')
        if self._tf_timer:
            self._tf_timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Cleanup node."""
        self.get_logger().info('Cleaning up waypoint navigator node...')
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        if self._tf_timer:
            self._tf_timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def publish_tf(self):
        """Publish required transforms."""
        try:
            now = self.get_clock().now().to_msg()
            
            # Map -> Odom transform
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(t)

            # Odom -> Base_link transform
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error publishing transforms: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()