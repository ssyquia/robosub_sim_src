import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Queue size: A required QoS (quality of service) setting that limits the amout of queued messages 
#             if a subscriber is not receiving them fast enough
QUEUE_SIZE = 10

class PublishString(Node):
    """
    PublishString class receives the string information from 'publishvector' node through 'string_topic'
    and print the movement and rotation status
    """
    def __init__(self):
        """
        No Arguments and No Return Value

        Initialize the class
        """
        super().__init__('publishstring')

        # Whenever 'publishstring' node receives the 6 dimensional vector data, the subscription calls the publish method
        self.subscription_ = self.create_subscription(
            String,
            'string_topic',
            self.publish,
            QUEUE_SIZE
        )
        self.subscription_

    def publish(self, msg):
        """
        Args:
            msg: string data received from 'publishvector' through 'string_topic'
        No return value

        Print the movement and rotation sttatus
        """
        self.get_logger().info('Movement & Rotation: {0}'.format(msg.data))

def main(args=None):
    rclpy.init(args=None) # the rclpy library is initialized
    publishstring = PublishString() # The node publishstring is created
    rclpy.spin(publishstring) # The node publishstring is spinned, meaning its callbacks are called

    publishstring.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
