#!/usr/bin/env python3

import socket

HOST = '127.0.0.1'  # Server's IP address
PORT = 12345        # Server's port

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
            while True:
                data = s.recv(1024)
                if not data:
                    break
                status = int(data.decode())
                if status == 0:
                    print("Device Off - Mission Complete")
                elif status == 1:
                    print("Device On - Navigating Waypoints")
                elif status == 2:
                    print("Device Pausing - Executing Circle")
                else:
                    print(f"Unknown Status: {status}")
        except ConnectionRefusedError:
            print("Connection refused. Check server status.")
        except KeyboardInterrupt:
            print("\nClient terminated")

if __name__ == '__main__':
    main()
