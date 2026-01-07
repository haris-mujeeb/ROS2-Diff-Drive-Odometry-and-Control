import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SimpleQoSPublisher(Node):
  
  def __init__(self):
    super().__init__("simple_qos_publisher")

    self.qos_profile_pub = QoSProfile(depth=10)

    self.declare_parameter("reliability", "system_default")
    self.declare_parameter("durability", "system_default")

    reliability = self.get_parameter("reliability").get_parameter_value().string_value
    durability = self.get_parameter("durability").get_parameter_value().string_value

    if reliability == "best_effort":
      self.qos_profile_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
      self.get_logger().info("[Reliability] : Best Effort")
    elif reliability == "reliable":
      self.qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
      self.get_logger().info("[Reliability] : Reliable")


    self.pub = self.create_publisher(String, "character", 10)
    self.counter = 0
    self.frequency_ = 1.0
    self.get_logger().info("Publishing at %d Hz " % self.frequency_)

    self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

  def timeCallback(self):
    # ,msg = String 
    pass