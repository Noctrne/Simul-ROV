import rclpy
from rclpy.node import Node
from rov_msgs.msg import ROVDepth
from pymavlink import mavutil

class ROVDepthNode(Node):
    def __init__(self):
        super().__init__('rov_depth_node')
        
        self.pix_address = '127.0.0.1:14552'
        self.pix_baud = 57600        
        
        self.publisher = self.create_publisher(
            ROVDepth,
            'rov/sensor/depth',
            10
            )
        
        self.get_logger().info(f'PUBLISHING rov/sensor/depth')
        
        
        self.master = mavutil.mavlink_connection(
            self.pix_address,
            self.pix_baud
            )
        
        self.master.wait_heartbeat()
        self.get_logger().info('Node DEPTH SENSOR berhasil connect ke mavrouter - Pixhawk 6C')
        
        self.create_timer(
            0.1,
            self.depth_cb
            )
        
    @staticmethod
    def hPa2depth(hPa):
        return hPa*100/1000/9.8
    
    @staticmethod
    def hPa2Pa(hPa):
        return hPa*100 
        
    def depth_cb(self):
        get_data = self.master.recv_match(
            type='SCALED_PRESSURE2', 
            blocking=True
            )
        
        if get_data is None:
            return
        msg = ROVDepth()
        msg.depth = self.hPa2depth(get_data.press_abs)
        msg.pressure = self.hPa2Pa(get_data.press_abs)
        
        self.publisher.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    depth_node = ROVDepthNode()
    try:
        rclpy.spin(depth_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()