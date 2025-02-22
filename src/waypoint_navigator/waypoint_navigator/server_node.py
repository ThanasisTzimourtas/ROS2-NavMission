#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import socket
import threading
import queue
import signal

class RobotStatusServer(Node):
    def __init__(self):
        super().__init__('robot_status_server')
        
        # Use a queue for thread-safe communication
        self._status_queue = queue.Queue()
        self._socket_connections = set()  # Renamed from _clients to avoid conflict
        self._lock = threading.Lock()
        self._current_status = 0
        self._shutdown_flag = threading.Event()

        # Create the ROS subscription
        self.create_subscription(
            Int32,
            '/robot_status',
            self._status_callback,
            10)
            
        # TCP server configuration
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind(('0.0.0.0', 12345))
        self._server_socket.listen(5)
        self._server_socket.settimeout(1.0)

        # Start the server thread
        self._server_thread = threading.Thread(target=self._run_server)
        self._server_thread.daemon = True
        self._server_thread.start()
        
        self.get_logger().info("Status server initialized")

    def _status_callback(self, msg):
        """Handle incoming status updates"""
        self._current_status = msg.data
        with self._lock:
            disconnected = set()
            for conn in self._socket_connections:
                try:
                    conn.sendall(str(msg.data).encode())
                except Exception as e:
                    self.get_logger().warn(f"Failed to send status: {e}")
                    disconnected.add(conn)
            
            # Clean up disconnected clients
            for conn in disconnected:
                try:
                    conn.close()
                    self._socket_connections.remove(conn)
                except Exception as e:
                    self.get_logger().error(f"Error closing connection: {e}")

    def _run_server(self):
        """Server thread main loop"""
        while not self._shutdown_flag.is_set():
            try:
                try:
                    client_socket, addr = self._server_socket.accept()
                    self.get_logger().info(f"New connection from {addr}")
                    
                    # Send current status to new client
                    try:
                        client_socket.sendall(str(self._current_status).encode())
                        with self._lock:
                            self._socket_connections.add(client_socket)
                    except Exception as e:
                        self.get_logger().error(f"Error sending initial status: {e}")
                        client_socket.close()
                except socket.timeout:
                    continue
                except Exception as e:
                    if not self._shutdown_flag.is_set():
                        self.get_logger().error(f"Accept error: {e}")
                        
            except Exception as e:
                if not self._shutdown_flag.is_set():
                    self.get_logger().error(f"Server error: {e}")

    def cleanup(self):
        """Clean shutdown of the server"""
        self._shutdown_flag.set()
        
        # Close all client connections
        with self._lock:
            for conn in self._socket_connections:
                try:
                    conn.close()
                except:
                    pass
            self._socket_connections.clear()
        
        # Close server socket
        try:
            self._server_socket.close()
        except:
            pass
        
        # Wait for server thread to finish
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    def signal_handler(signum, frame):
        nonlocal node
        if node:
            node.cleanup()
        rclpy.shutdown()
    
    try:
        # Set up signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        node = RobotStatusServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()