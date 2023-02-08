import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

class SimpleSubscriber(Node):

    def __init__(self):
        # Here we have the class constructor
        # call super() in the constructor in order to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('simple_subscriber')
        # create the subscriber object
        # in this case the subscriptor will be subsribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.subscriber= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) #is the most used to read LaserScan data and some sensor data too.
        # prevent unused variable warning
        self.subscriber
        # define the variable to save the received info
        self.laser_forward = 0
        

    def listener_callback(self, msg):
        # print the log info in the terminal
        
        self.get_logger().info('I receive: "%s"' % str(msg))
        # save the received data
        #self.laser_forward = msg.ranges[0]
        # print the received data
        print(self.laser_forward)
        
        
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_subscriber = SimpleSubscriber()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_subscriber)
    # Explicity destroy the node
    simple_subscriber.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
