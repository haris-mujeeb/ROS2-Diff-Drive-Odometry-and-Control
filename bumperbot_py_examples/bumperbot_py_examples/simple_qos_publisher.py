import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class SimpleQoSPublisher(Node):
  
  def __init__(self):
    super().__init__("simple_qos_publisher")

    self.qos_profile_pub = QoSProfile(depth=10)

    self.declare_parameter("reliability", "reliable")
    self.declare_parameter("durability", "volatile")

    reliability = self.get_parameter("reliability").get_parameter_value().string_value
    durability = self.get_parameter("durability").get_parameter_value().string_value

    if reliability == "best_effort":
      self.qos_profile_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
      self.get_logger().info("[Reliability] : Best Effort")
    elif reliability == "reliable":
      self.qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
      self.get_logger().info("[Reliability] : Reliable")
    
    if durability == "volatile":
        self.qos_profile_pub.durability = QoSDurabilityPolicy.VOLATILE
        self.get_logger().info("[Durability] : Volatile")
    elif durability == "transient_local":
        self.qos_profile_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.get_logger().info("[Durability] : Transient Local")


    self.pub = self.create_publisher(String, "character", self.qos_profile_pub)
    self.counter = 0
    self.frequency_ = 1.0
    self.get_logger().info("Publishing at %d Hz " % self.frequency_)

    self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

  def timerCallback(self):
    msg = String()
    msg.data = "Hello World: %d" % self.counter
    self.pub.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.counter += 1

def main():
    rclpy.init()
    node = SimpleQoSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()