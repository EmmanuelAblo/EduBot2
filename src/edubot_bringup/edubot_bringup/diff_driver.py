#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder , BinaryPayloadBuilder
from pymodbus.constants import Endian
from pymodbus import pymodbus_apply_logging_config
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DiffDriver(Node):

    def __init__(self):
        pymodbus_apply_logging_config("CRITICAL")
        super().__init__('DiffDriver')
        self.tfb = TransformBroadcaster(self)


        self.odom_publisher = self.create_publisher(Odometry, '/odom', 1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        self.odom_msg.pose.covariance[0] = 1
        self.odom_msg.pose.covariance[11] = 1
        self.odom_msg.pose.covariance[35] = 1
        
        self.odom_msg.twist.covariance[0] = 1
        self.odom_msg.twist.covariance[11] = 1
        self.odom_msg.twist.covariance[35] = 1

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            1)
        
        self.subscription  # prevent unused variable warning

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.xdot = [0.0,0.0,0.0]


    def send_modbus(self,addr, data):
        try:
            self.modbus = ModbusTcpClient(host = "127.0.0.1", port = 1502)
            self.modbus.connect()
            self.modbus.write_registers(addr,data)
            print(data)
        except:
            pass

    def read_modbus(self,addr, count):
        try:
            self.modbus = ModbusTcpClient(host = "127.0.0.1", port = 1502)
            self.modbus.connect()
            result = self.modbus.read_input_registers(address=addr,count=count)
            #print(result.registers)
            if result.isError():
                return [0] * count
            return result.registers
        except:
            return [0] * count
            pass
    
    def to_register(self, val):
        upper = (val >> 16) & 0xffff
        lower = val & 0xffff
        return [upper, lower]

    def vel_callback(self, msg : Twist):
        data = []
        data += self.to_register( int(msg.linear.x / 0.0001) )
        data += self.to_register( int(msg.angular.z / 0.0001) )
        self.send_modbus(100, data )


    def get_int32(self,data):
        return data
    
    def to_int32(self,high,low):
        value = (high << 16) |  low
        if(value & 0x80000000):
            value = -0x100000000 + value
        return value
    
    def timer_callback(self):
        count=30
        result = self.read_modbus(0,count)

        robot_status=[]
        for i in range(0, count, 2):  # Read in pairs (16-bit + 16-bit)
            high = result[i]
            low = result[i+1]
            value=self.to_int32(high,low)* 0.001
            robot_status.append(value)
        #print(robot_status[9]* 0.0001, robot_status[10]* 0.0001)
        #print(f"Robot status is V,W,X,Y,TH= {', '.join([f'{robot_status[i]:.2f}' for i in range(9, 14)])}")
        v = robot_status[9]
        w = robot_status[10]
        x = robot_status[11]
        y = robot_status[12]
        th = robot_status[13]
        #print(v,w,x,y,th)


        now  = self.get_clock().now().to_msg()
        self.odom_msg.header.stamp = now

        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, th)

        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]

        self.odom_msg.twist.twist.linear.x = self.xdot[0]
        self.odom_msg.twist.twist.linear.y = self.xdot[1]
        self.odom_msg.twist.twist.angular.z = w

        tfs = TransformStamped()
        tfs.header.stamp = now
        tfs.header.frame_id="odom"
        tfs._child_frame_id = "base_footprint"
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = 0.0  
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]

        self.tfb.sendTransform(tfs)    
        self.odom_publisher.publish( self.odom_msg )

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
