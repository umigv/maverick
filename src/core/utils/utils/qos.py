from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Use for topics that must deliver the last message to late-joining subscribers
# (e.g. ground truth map, robot state). Both publisher and subscriber must use
# this profile. External tools like RViz require their durability policy set to
# "Transient Local" in the topic display options.
LATCHED = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
