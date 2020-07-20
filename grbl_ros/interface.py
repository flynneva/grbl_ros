#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from grbl_ros.grbl_device import grbl

grbl_node_name = 'cnc_001'

class GRBLInterface(Node):
    def __init__(self):
        super().__init__(grbl_node_name)
        self.pub_pos_     = self.create_publisher(Twist, grbl_node_name + '/position', 10)
        self.pub_status_  = self.create_publisher(String, grbl_node_name + '/status', 10)
        self.sub_cmd_ = self.create_subscription(Twist, grbl_node_name + '/cmd' , self.cmdCallback, 10)
        self.sub_stop_ = self.create_subscription(String, grbl_node_name + '/stop', self.stopCallback, 10)
        self.sub_cmd_ # prevent unused variable warning
        self.sub_stop_ # prevent unused variable warning

        # declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('acceleration', 1000)
        self.declare_parameter('x_max', 50)
        self.declare_parameter('y_max', 50)
        self.declare_parameter('z_max', 50)
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('x_max_speed', 100)
        self.declare_parameter('y_max_speed', 100)
        self.declare_parameter('z_max_speed', 100)
        self.declare_parameter('x_steps_mm', 100)
        self.declare_parameter('y_steps_mm', 100)
        self.declare_parameter('z_steps_mm', 100)

        port           = self.get_parameter('port')
        baud           = self.get_parameter('baudrate')
        acc            = self.get_parameter('acceleration')    # axis acceleration (mm/s^2)
        max_x	       = self.get_parameter('x_max')           # workable travel (mm)
        max_y 	       = self.get_parameter('y_max')           # workable travel (mm)
        max_z 	       = self.get_parameter('x_max')           # workable travel (mm)
        default_speed  = self.get_parameter('default_speed')   # mm/min 
        speed_x        = self.get_parameter('x_max_speed')     # mm/min
        speed_y        = self.get_parameter('y_max_speed')     # mm/min
        speed_z        = self.get_parameter('z_max_speed')     # mm/min
        steps_x        = self.get_parameter('x_steps_mm')      # axis steps per mm
        steps_y        = self.get_parameter('y_steps_mm')      # axis steps per mm
        steps_z        = self.get_parameter('z_steps_mm')      # axis steps per mm

        self.grbl_obj = grbl()
        self.grbl_obj.startup( port.get_parameter_value().string_value,
                               baud.get_parameter_value().integer_value,
                               acc.get_parameter_value().integer_value,
                               max_x.get_parameter_value().integer_value,
                               max_y.get_parameter_value().integer_value,
                               max_z.get_parameter_value().integer_value,
                               default_speed.get_parameter_value().integer_value,
                               speed_x.get_parameter_value().integer_value,
                               speed_y.get_parameter_value().integer_value,
                               speed_z.get_parameter_value().integer_value,
                               steps_x.get_parameter_value().integer_value,
                               steps_y.get_parameter_value().integer_value,
                               steps_z.get_parameter_value().integer_value)

    def cmdCallback(msg):
        self.get_logger().info("received msg: " + str(msg))
        print(msg.linear.x, msg.linear.y, msg.linear.z)
        grbl_obj.moveTo(msg.linear.x, msg.linear.y, msg.linear.z, blockUntilComplete=True)

    def stopCallback(msg):
        #stop steppers
        if   msg.data == 's':
            grbl_obj.disableSteppers()
	    # fire steppers
        elif msg.data == 'f':
            grbl_obj.enableSteppers()

def main():
    rclpy.init()
    interface = GRBLInterface()
    status     = interface.grbl_obj.getStatus()
    pose       = interface.grbl_obj.getTwist()
    print(str(status))
    print(pose)
    ros_status = String()
    ros_status.data = str(status)
    interface.pub_pos_.publish(pose)
    interface.pub_status_.publish(ros_status)
    rclpy.spin(interface)

if __name__ == '__main__':
    main()
