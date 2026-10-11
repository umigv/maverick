If you are recording a rosbag, be very careful about including the history of this node if you do not need it, as it will likely inflate the size of the rosbag massively.

# Usage
To allow your node's main processing to run independent of the zed frame rate, add a callback that updates some sort of ```latest_message``` variable. This variable can be fetched at the beginning of each main processing loop. Your main loop would need to be structured such that it will call ```spin_once``` during each loop or ```spin``` once at startup to properly update ```latest_message```.
``` python
def _image_callback(self, msg: Image) -> None:
    self.latest_message = msg
```
Make sure the subscriber object for this node has a queue size of 1 so it doesn't redundantly process several backed up messages at once.

## Example Usage
``` python
class ImageProcessor(Node):
    def __init__(self) -> None:
        super().__init__("image-processor-node")

        self.latest_message: Image | None = None

        self.image_sub = self.create_subscription(Image, "/zed/left/rgb/image",  self._image_callback, 1)

    def _image_callback(self, msg: Image) -> None:
        self.latest_message = msg

    def run(self) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.latest_message is None:
                continue

            msg = self.latest_message
            self.latest_message = None

            # do some processing here


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImageProcessor()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```
