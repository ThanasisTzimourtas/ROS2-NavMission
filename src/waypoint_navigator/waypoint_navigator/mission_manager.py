#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import asyncio
import threading
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition, State
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
import yaml
import os
from std_msgs.msg import Int32

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Node state tracking
        self._node_states = {
            'path_selector': State.PRIMARY_STATE_UNCONFIGURED,
            'circle_node': State.PRIMARY_STATE_UNCONFIGURED,
            'waypoint_navigator': State.PRIMARY_STATE_UNCONFIGURED
        }
        
        # Use ReentrantCallbackGroup to allow parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/a200_0000/navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Create service client for circle execution
        self.circle_client = self.create_client(
            Trigger,
            '/a200_0000/execute_circle',
            callback_group=self.callback_group
        )
        
        # Create initial pose publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/a200_0000/initialpose',
            10)
        
        # Create pose subscriber
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/a200_0000/amcl_pose',
            self.pose_callback,
            10)
        
        # Lifecycle control services
        self.nav_lifecycle = self.create_client(
            ChangeState, 
            '/a200_0000/waypoint_navigator/change_state',
            callback_group=self.callback_group
        )
        self.circle_lifecycle = self.create_client(
            ChangeState, 
            '/a200_0000/circle_node/change_state',
            callback_group=self.callback_group
        )
        self.path_selector_lifecycle = self.create_client(
            ChangeState,
            '/a200_0000/path_selector/change_state',
            callback_group=self.callback_group
        )
        
        # Path selector service clients
        self.check_random_nav_client = self.create_client(
            Trigger,
            '/a200_0000/check_random_nav',
            callback_group=self.callback_group
        )
        self.execute_random_nav_client = self.create_client(
            Trigger,
            '/a200_0000/execute_random_nav',
            callback_group=self.callback_group
        )
        
        # Set up pose file path
        self.pose_file = os.path.expanduser('~/clearpath_ws/src/waypoint_navigator/config/last_robot_pose.yaml')
        self.status_pub = self.create_publisher(Int32, '/robot_status', 10)
        
        # Initialize waypoints
        self.waypoints = [
            self.create_pose_stamped(0.056, 0.016, 0.031),  # Default first waypoint
            self.create_pose_stamped(10.097, 6.909, 0.003),
            self.create_pose_stamped(13.753, 7.269, 1.720),
            self.create_pose_stamped(6.974, 13.261, -3.062),
            self.create_pose_stamped(-6.105, 13.146, -3.075),
            self.create_pose_stamped(-8.896, 8.616, -1.559),
            self.create_pose_stamped(-5.682, -4.963, -1.569),
            self.create_pose_stamped(-4.337, -3.876, 1.562),
            self.create_pose_stamped(-0.090, 0.994, 0.042)
        ]
        self.current_waypoint = 0
        self.current_action = None

    async def update_node_state(self, name, new_state):
        """Update tracked state for a node."""
        self._node_states[name] = new_state
        self.get_logger().debug(f'Updated {name} state to {new_state}')

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

    def save_current_pose(self):
        """Save current robot pose to YAML file."""
        if self.current_pose is None:
            self.get_logger().warn('No pose available to save')
            return False

        try:
            pose = self.current_pose.pose.pose
            pose_data = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            }
            
            with open(self.pose_file, 'w') as f:
                yaml.dump(pose_data, f)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to save pose: {str(e)}')
            return False

    def publish_initial_pose(self):
        """Publish initial pose for AMCL."""
        try:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose = self.waypoints[0].pose
            msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06]
            
            self.initial_pose_pub.publish(msg)
            time.sleep(2.0)  # Wait for AMCL to process the pose
            self.get_logger().info('Published initial pose')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing initial pose: {str(e)}')
            raise

    async def start_mission(self):
        """Start the mission execution."""
        try:
            self.get_logger().info('Starting mission...')
            
            # Configure nodes
            nodes_to_configure = [
                (self.nav_lifecycle, 'waypoint_navigator'),
                (self.circle_lifecycle, 'circle_node'),
                (self.path_selector_lifecycle, 'path_selector')
            ]
            
            for client, name in nodes_to_configure:
                await self.configure_node(client, name)
                await asyncio.sleep(2.0)
            
            # Publish initial pose and wait for AMCL
            self.publish_initial_pose()
            await asyncio.sleep(5.0)
            
            # Activate required nodes
            await self.activate_node(self.nav_lifecycle, 'waypoint_navigator')
            await self.activate_node(self.path_selector_lifecycle, 'path_selector')
            await asyncio.sleep(2.0)
            
            # Wait for navigation stack to be ready
            if not await self.wait_for_navigation_ready():
                self.get_logger().error('Navigation stack failed to initialize')
                return
            
            # Start navigation
            await self.navigate_to_next_waypoint()
            
            # Save final pose after mission completion
            if self.save_current_pose():
                self.get_logger().info('Mission completed and final pose saved')
            else:
                self.get_logger().warn('Mission completed but failed to save final pose')
                
        except Exception as e:
            self.get_logger().error(f'Error during mission: {str(e)}')
            raise

    async def check_random_navigation(self):
        """Check if random navigation should occur."""
        try:
            if not await self.wait_for_service(self.check_random_nav_client):
                return False

            # Ensure path selector node is active
            if not await self.ensure_path_selector_active():
                return False

            response = await self.check_random_nav_client.call_async(Trigger.Request())
            return response.success

        except Exception as e:
            self.get_logger().error(f'Error checking random navigation: {str(e)}')
            return False

    async def execute_random_navigation(self):
        """Execute random navigation sequence."""
        try:
            self.get_logger().info('Starting random navigation sequence...')
            
            # Deactivate navigator temporarily
            await self.deactivate_node(self.nav_lifecycle, 'waypoint_navigator')
            await asyncio.sleep(3.0)
            
            # Execute random navigation
            result = await self.execute_random_nav_client.call_async(Trigger.Request())
            
            if result.success:
                self.get_logger().info('Random navigation completed')
                await asyncio.sleep(2.0)
            else:
                self.get_logger().error(f'Random navigation failed: {result.message}')
            
            # Reactivate navigator
            await self.activate_node(self.nav_lifecycle, 'waypoint_navigator')
            await asyncio.sleep(3.0)
            
            return result.success
                
        except Exception as e:
            self.get_logger().error(f'Error during random navigation: {str(e)}')
            try:
                await self.activate_node(self.nav_lifecycle, 'waypoint_navigator')
            except:
                pass
            raise

    async def ensure_path_selector_active(self):
        """Ensure path selector node is active."""
        try:
            if self._node_states['path_selector'] != State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info('Path selector not active, activating...')
                await self.activate_node(self.path_selector_lifecycle, 'path_selector')
                await asyncio.sleep(2.0)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to ensure path selector active: {e}')
            return False

    async def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the sequence."""
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Mission complete!')
            status = Int32(data=0)
            self.status_pub.publish(status)
            return

        try:
            status = Int32(data=1)
            self.status_pub.publish(status)
            
            # Ensure navigation is ready
            if not await self.wait_for_navigation_ready():
                self.get_logger().error('Navigation system not ready, aborting mission')
                return
            
            # Check for random navigation
            if await self.check_random_navigation():
                self.get_logger().info('Random navigation triggered')
                if await self.execute_random_navigation():
                    self.get_logger().info('Random navigation completed, continuing to waypoint')
                else:
                    self.get_logger().warn('Random navigation failed, continuing to waypoint')
            
            # Continue with normal waypoint navigation
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint + 1}')
            
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.waypoints[self.current_waypoint]
            
            # Send goal with retry logic
            max_attempts = 3
            for attempt in range(max_attempts):
                try:
                    self.current_action = await self.nav_client.send_goal_async(
                        goal_msg,
                        feedback_callback=self.navigation_feedback_callback
                    )
                    
                    if self.current_action.accepted:
                        self.get_logger().info('Navigation goal accepted')
                        break
                    else:
                        if attempt < max_attempts - 1:
                            self.get_logger().warn(f'Goal rejected, attempt {attempt + 1} of {max_attempts}')
                            await asyncio.sleep(2.0)
                        else:
                            self.get_logger().error('Goal rejected after all attempts!')
                            return
                            
                except Exception as e:
                    if attempt < max_attempts - 1:
                        self.get_logger().warn(f'Error sending goal, attempt {attempt + 1}: {str(e)}')
                        await asyncio.sleep(2.0)
                    else:
                        raise

            result = await self.current_action.get_result_async()
            
            if result.result.result:
                self.get_logger().info('Navigation goal succeeded!')
                
                # Execute circle at specific waypoints
                if self.current_waypoint in [2, len(self.waypoints)-1]:
                    await self.execute_circle()
                
                self.current_waypoint += 1
                await asyncio.sleep(2.0)
                await self.navigate_to_next_waypoint()
            else:
                self.get_logger().error('Navigation goal failed!')
                
        except Exception as e:
            self.get_logger().error(f'Error during navigation: {str(e)}')
            raise

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        pass
    
    async def wait_for_navigation_ready(self):
        """Wait for navigation stack to be ready."""
        self.get_logger().info('Waiting for navigation stack to be ready...')
        
        # Check for nav2 core nodes
        required_nodes = [
            '/a200_0000/behavior_server',
            '/a200_0000/controller_server',
            '/a200_0000/planner_server',
            '/a200_0000/bt_navigator'
        ]
        
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                # Wait for action server first
                if not await self.wait_for_action_server():
                    raise RuntimeError('Navigation action server not available')
                    
                # Try a test goal
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = self.waypoints[0]  # Use first waypoint as test
                
                # Send goal
                self.get_logger().info('Sending test navigation goal...')
                goal_handle = await self.nav_client.send_goal_async(goal_msg)
                
                if goal_handle.accepted:
                    self.get_logger().info('Test goal accepted, canceling...')
                    # Cancel the goal since this was just a test
                    await goal_handle.cancel_goal_async()
                    return True
                else:
                    self.get_logger().warn('Test goal rejected, retrying...')
                    
            except Exception as e:
                self.get_logger().warn(f'Navigation check attempt {attempt + 1} failed: {str(e)}')
                
            # Wait before retry
            await asyncio.sleep(5.0)
        
        return False

    async def wait_for_action_server(self):
        """Wait for the navigation action server to become available."""
        self.get_logger().info('Waiting for navigation action server...')
        for _ in range(60):  # Try for 60 seconds
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Navigation action server available.')
                return True
            await asyncio.sleep(1.0)
        return False

    async def execute_circle(self):
        """Execute circle movement."""
        try:
            status = Int32(data=2)
            self.status_pub.publish(status)
            self.get_logger().info('Executing circle movement...')
            
            # Switch to circle node
            await self.deactivate_node(self.nav_lifecycle, 'waypoint_navigator')
            await asyncio.sleep(3.0)
            
            await self.activate_node(self.circle_lifecycle, 'circle_node')
            await asyncio.sleep(3.0)
            
            # Execute circle
            server_ready = await self.wait_for_service(self.circle_client)
            if not server_ready:
                self.get_logger().error('Circle service not available')
                return
                
            # Call circle service and wait for result
            self.get_logger().info('Calling circle service...')
            result = await self.circle_client.call_async(Trigger.Request())
            
            if result.success:
                self.get_logger().info('Circle movement completed')
            else:
                self.get_logger().error(f'Circle movement failed: {result.message}')
            
            # Switch back to navigation
            await self.deactivate_node(self.circle_lifecycle, 'circle_node')
            await asyncio.sleep(3.0)
            
            await self.activate_node(self.nav_lifecycle, 'waypoint_navigator')
            await asyncio.sleep(3.0)
            
            self.get_logger().info('Circle execution sequence completed')
            
        except Exception as e:
            self.get_logger().error(f'Error during circle execution: {str(e)}')
            # Ensure navigator is reactivated on error
            try:
                await self.activate_node(self.nav_lifecycle, 'waypoint_navigator')
            except:
                pass
            raise

    async def wait_for_service(self, client, timeout_sec=60):
        """Wait for a service to become available."""
        self.get_logger().info(f'Waiting for service {client.srv_name}...')
        for _ in range(timeout_sec):
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {client.srv_name} available.')
                return True
            await asyncio.sleep(1.0)
        self.get_logger().error(f'Service {client.srv_name} not available after {timeout_sec} seconds')
        return False

    async def configure_node(self, client, name):
        """Configure a lifecycle node."""
        try:
            self.get_logger().info(f'Configuring {name}...')
            
            if self._node_states[name] != State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f'Node {name} is not in unconfigured state, skipping configuration')
                return False
            
            server_ready = await self.wait_for_service(client)
            if not server_ready:
                raise RuntimeError(f'Service for {name} not available')
            
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            
            response = await client.call_async(req)
            await asyncio.sleep(2.0)
            
            if not response.success:
                raise RuntimeError(f'Failed to configure {name}: {response.message}')
            
            await self.update_node_state(name, State.PRIMARY_STATE_INACTIVE)
            self.get_logger().info(f'Successfully configured {name}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error configuring {name}: {str(e)}')
            return False

    async def activate_node(self, client, name):
        """Activate a lifecycle node."""
        try:
            self.get_logger().info(f'Activating {name}...')
            
            if self._node_states[name] != State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f'Node {name} is not in inactive state, skipping activation')
                return False
            
            if not await self.wait_for_service(client):
                raise RuntimeError(f'Service for {name} not available')
            
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            
            response = await client.call_async(req)
            await asyncio.sleep(2.0)
            
            if not response.success:
                raise RuntimeError(f'Failed to activate {name}: {response.message}')
            
            await self.update_node_state(name, State.PRIMARY_STATE_ACTIVE)
            self.get_logger().info(f'Successfully activated {name}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error activating {name}: {str(e)}')
            return False

    async def deactivate_node(self, client, name):
        """Deactivate a lifecycle node."""
        try:
            self.get_logger().info(f'Deactivating {name}...')
            
            if self._node_states[name] != State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f'Node {name} is not in active state, skipping deactivation')
                return False
            
            if not await self.wait_for_service(client):
                raise RuntimeError(f'Service for {name} not available')
            
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            
            response = await client.call_async(req)
            await asyncio.sleep(2.0)
            
            if not response.success:
                raise RuntimeError(f'Failed to deactivate {name}: {response.message}')
            
            await self.update_node_state(name, State.PRIMARY_STATE_INACTIVE)
            self.get_logger().info(f'Successfully deactivated {name}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error deactivating {name}: {str(e)}')
            return False

    async def cleanup_node(self, client, name):
        """Cleanup a lifecycle node."""
        try:
            self.get_logger().info(f'Cleaning up {name}...')
            
            if self._node_states[name] != State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f'Node {name} is not in inactive state, skipping cleanup')
                return False
            
            if not await self.wait_for_service(client):
                return False
            
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CLEANUP
            
            response = await client.call_async(req)
            await asyncio.sleep(2.0)
            
            if not response.success:
                self.get_logger().warn(f'Failed to cleanup {name}: {response.message}')
                return False
                
            await self.update_node_state(name, State.PRIMARY_STATE_UNCONFIGURED)
            self.get_logger().info(f'Successfully cleaned up {name}')
            return True
                
        except Exception as e:
            self.get_logger().error(f'Error cleaning up {name}: {str(e)}')
            return False

    async def shutdown(self):
        """Clean shutdown of the mission manager."""
        try:
            # Save final pose
            self.save_current_pose()
            
            # Cleanup nodes in reverse order
            nodes_to_cleanup = [
                (self.path_selector_lifecycle, 'path_selector'),
                (self.circle_lifecycle, 'circle_node'),
                (self.nav_lifecycle, 'waypoint_navigator')
            ]
            
            for client, name in nodes_to_cleanup:
                try:
                    if not await self.wait_for_service(client, timeout_sec=2):
                        self.get_logger().warn(f'Service for {name} not available during shutdown')
                        continue
                    
                    current_state = self._node_states[name]
                    
                    # Only attempt deactivation if node is active
                    if current_state == State.PRIMARY_STATE_ACTIVE:
                        try:
                            await self.deactivate_node(client, name)
                            await asyncio.sleep(2.0)
                        except Exception as deactivate_error:
                            self.get_logger().warn(f'Deactivation of {name} failed: {deactivate_error}')
                    
                    # Only attempt cleanup if node is inactive
                    if self._node_states[name] == State.PRIMARY_STATE_INACTIVE:
                        try:
                            await self.cleanup_node(client, name)
                            await asyncio.sleep(2.0)
                        except Exception as cleanup_error:
                            self.get_logger().warn(f'Cleanup of {name} failed: {cleanup_error}')
                            
                except Exception as e:
                    self.get_logger().warn(f'Error during shutdown sequence for {name}: {str(e)}')
                    
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {str(e)}')
        finally:
            self.get_logger().info('Shutdown sequence completed')

def main(args=None):
    rclpy.init(args=args)
    
    mission_manager = MissionManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mission_manager)
    
    # Create separate thread for executor
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Run the coroutine in the main thread
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(mission_manager.start_mission())
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        try:
            loop.run_until_complete(mission_manager.shutdown())
        except:
            pass
        loop.close()
        executor.shutdown()
        mission_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()