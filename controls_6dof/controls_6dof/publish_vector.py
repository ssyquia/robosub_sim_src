import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, String

# Queue size: A required QoS (quality of service) setting that limits the amout of queued messages 
#             if a subscriber is not receiving them fast enough
QUEUE_SIZE = 10

class PublishVector(Node):
    """
    PublishVector class receives the 6 dimensional vector information from 'listenkey' node through 'vector_topic'
    and converts this data into string.

    Attributes:
        publisher_: The node where 'publishvector' node sends the string
        subscription_ : The node where 'publishvector' nodes receives the 6 dimensional vector information
    """
    def __init__(self):
        """
        No Arguments and No Return Value

        Initialize the class
        """
        super().__init__('publishvector')
        self.publisher_ = self.create_publisher(String, 'string_topic', QUEUE_SIZE)

        # Whenever 'publishvector' node receives the 6 dimensional vector data, the subscription calls the publish method
        self.subscription_ = self.create_subscription(
            Int32MultiArray, 
            'vector_topic', 
            self.publish, 
            QUEUE_SIZE
        )
        self.subscription_

    def publish(self, msg):
        """
        Args:
            msg: 6 dimensional vector received from 'listenkey' through 'vector_topic'
        No return value

        Store the movement and rotation data corresponding to each element in 6 dimensional vector in the string variable (strmsg) 
        and send this string data to 'publishstring' node through 'string_topic'
        """
        self.get_logger().info('6D vector: {0}'.format(msg.data))
        
        strmsg = ''
        # Forward / Backward
        if msg.data[0] == 1: strmsg += 'forward / '
        elif msg.data[0] == -1: strmsg += 'backward / '

        # Strafe Left / Right
        if msg.data[1] == 1: strmsg += 'strafe left / '
        elif msg.data[1] == -1: strmsg += 'strafe right / '

        # Surface / Dive
        if msg.data[2] == 1: strmsg += 'surface / '
        elif msg.data[2] == -1: strmsg += 'dive / '

        # Yaw Counterclockwise / Clockwise
        if msg.data[3] == 1: strmsg += 'yaw counterclockwise / '
        elif msg.data[3] == -1: strmsg += 'yaw clockwise / '

        # Pitch Left / Right
        if msg.data[4] == 1: strmsg += 'pitch left / '
        elif msg.data[4] == -1: strmsg += 'pitch right / '

        # Roll Forward / Backward
        if msg.data[5] == 1: strmsg += 'roll forward / '
        elif msg.data[5] == -1: strmsg += 'roll backward / '

        # Check if there is neither movement nor rotation
        if strmsg == "" : strmsg += 'No movement and rotation / '
        newmsg = String()
        newmsg.data = strmsg[:-3]
        self.publisher_.publish(newmsg)

def main(args=None):
    rclpy.init(args=None) # the rclpy library is initialized
    publishvector = PublishVector() # The node publishvector is created
    rclpy.spin(publishvector) # The node publishvector is spinned, meaning its callbacks are called

    publishvector.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
