from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


class TopicDiagnostic(Node):
    def __init__(self, msg_type, topic: str, timedout: float = 3.0, interval: float = 1.0):
        super().__init__("odom_diagnostic")
        self.topic = topic
        self.timedout = timedout
        self.interval = interval

        self.start_at = monotonic()
        self.last_recived = None
        self.previous_status = None

        self.create_subscription(msg_type, self.topic, self.on_topic, 10)
        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostic", 10)
        self.create_timer(self.interval, self.diagnose)

    def on_topic(self, msg):
        self.last_recived = monotonic()

    def diagnose(self):
        now = monotonic()
        publishers = self.get_publishers_info_by_topic(self.topic)
        publisher_count = len(publishers)

        publisher_names = sorted(
            {f"{publisher.node_namespace.rstrip('/')}/{publisher.node_name}" for publisher in publishers}
        )

        status = DiagnosticStatus()
        status.hardware_id = "maverick"
        status.name = "Navigation: Odometry"

        if self.last_recived is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Publisher exists, but no message recieved"

        elif publisher_count == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No subscriber to {self.topic}"

        elif now - self.start_at < self.timedout:
            status.level = DiagnosticStatus.WARN
            status.message = "Starting, waiting for odometry"

        elif now - self.last_recived > self.last_recived:
            status.level = DiagnosticStatus.ERROR
            status.message = "Odometry data is stale"

        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                f"Receiving odometry data from {publisher_count} publisher(s): {', '.join(publisher_names)}"
            )

        current_status = (status.level, status.message)
        if current_status != self.previous_status:
            logger = self.get_logger()

            if status.level == DiagnosticStatus.OK:
                logger.info(status.message)
            elif status.level == DiagnosticStatus.WARN:
                logger.warning(status.message)
            else:
                logger.error(status.message)

        self.previous_status = current_status

        age = f"{now - self.last_recived:.02}" if self.last_recived is not None else "not recieved"
        status.values = [
            KeyValue(key="topic", value=f"{self.topic}"),
            KeyValue(key="publisher_count", value=str(publisher_count)),
            KeyValue(key="publisher_nodes", value=", ".join(publisher_names)),
            KeyValue(key="seconds_since_last_recieved", value=age),
        ]

        diagnostic = DiagnosticArray()
        diagnostic.status = [status]
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(diagnostic)
