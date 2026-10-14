import rclpy
from rclpy.node import Node
from rov_msgs.msg import ROVAttitude
from numpy import rad2deg

from pymavlink import mavutil
import numpy as np

class ROVAttitudeNode(Node):
    def __init__(self):
        super().__init__('rov_attitude_node')
        
        self.pix_address = '127.0.0.1:14553'
        self.pix_baud = 57600
        
        self.publisher = self.create_publisher(
            ROVAttitude, 
            'rov/sensor/attitude', 
            10
            )
        
        self.get_logger().info(f'PUBLISHING rov/sensor/attitude')
        
        self.master = mavutil.mavlink_connection(
            self.pix_address,
            self.pix_baud
            )
        
        self.master.wait_heartbeat()
        self.get_logger().info('Node ATTITUDE berhasil connect ke mavrouter - Pixhawk 6C')
        
        self.create_timer(
            0.1, 
            self.att_cb
            )
        
    def att_cb(self):
        
        get_data = self.master.recv_match(
            type='ATTITUDE',
            blocking=False
            )
        
        if get_data is None:
            return
        
        msg = ROVAttitude()
        msg.roll = rad2deg(get_data.roll) 
        msg.pitch = rad2deg(get_data.pitch)
        msg.yaw = rad2deg(get_data.yaw)
        
        self.publisher.publish(msg)
        
        # self.get_logger().info(f'[Roll] {msg.roll:.2f} | [Pitch] {msg.pitch:.2f} | [Yaw] {msg.yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    att_node = ROVAttitudeNode()
    try:
        rclpy.spin(att_node)
    except KeyboardInterrupt:
        pass
    finally:
        att_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()