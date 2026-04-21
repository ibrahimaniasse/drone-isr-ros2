#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OperatorTeleop(Node):
    def __init__(self):
        super().__init__('operator_teleop')
        self.pub = self.create_publisher(String, '/operator_command', 10)
        self.get_logger().info("Operator Teleop Started.")
        self.get_logger().info("Commands available:")
        self.get_logger().info("  i <ID> : Inspect target <ID> (e.g., 'i 0')")
        self.get_logger().info("  r      : Return to loiter (AWAITING_COMMAND)")
        self.get_logger().info("  l      : Land")

    def send_command(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

def read_keyboard(node):
    while rclpy.ok():
        try:
            line = sys.stdin.readline().strip().lower()
            if not line:
                continue
                
            if line.startswith('i '):
                # Convert 'i 0' to 'inspect 0'
                parts = line.split()
                if len(parts) == 2 and parts[1].isdigit():
                    node.send_command(f"inspect {parts[1]}")
                else:
                    print("Invalid inspect format. Use 'i <ID>'")
            elif line == 'r':
                node.send_command("return")
            elif line == 'l':
                node.send_command("land")
            else:
                print("Unknown command.")
        except Exception as e:
            print(f"Input error: {e}")
            break

def main():
    rclpy.init()
    node = OperatorTeleop()
    
    # Thread pour la lecture clavier non-bloquante
    thread = threading.Thread(target=read_keyboard, args=(node,))
    thread.daemon = True
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
