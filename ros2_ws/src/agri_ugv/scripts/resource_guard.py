#!/usr/bin/python3
"""Stop the launch before a SLAM database can consume the last disk reserve."""
import shutil
from pathlib import Path
import rclpy
from rclpy.node import Node
class Guard(Node):
    def __init__(self):
        super().__init__('simulation_resource_guard');self.create_timer(5.,self.check)
    def check(self):
        free=shutil.disk_usage(Path.cwd()).free
        if free<1024**3:
            self.get_logger().fatal('Less than 1 GiB disk reserve remains; stopping simulation before database failure.')
            raise SystemExit(2)
def main():
    rclpy.init();n=Guard()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
