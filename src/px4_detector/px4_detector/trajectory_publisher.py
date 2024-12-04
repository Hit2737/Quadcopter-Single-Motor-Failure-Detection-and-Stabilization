import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisherNode(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "command/pose", 10)
        self.k = 0.0
        self.timer_ = self.create_timer(0.01, self.publish_command_pose)

    def publish_command_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = (
            "base_link"  # Change this to your desired frame ID
        )

        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = self.k
        pose_stamped.pose.orientation.w = 1.0

        self.publisher_.publish(pose_stamped)

        if self.k < 10:
            self.k += 0.01


"""
You can try difference trajectories by changing the `publish_command_pose` method. For example, you can try the following code to make the drone move in a circle:

class TrajectoryPublisherNode(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'command/pose', 10)
        angle = M_PI
        k = 0.0
        radius = 0.0
        self.timer_ = self.create_timer(0.01, self.publish_command_pose)

    def publish_command_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'base_link'  # Change this to your desired frame ID

        if self.k < 5:
            pose_stamped.pose.position.x = 0.0
            pose_stamped.pose.position.y = 0.0
            pose_stamped.pose.position.z = self.k
            pose_stamped.pose.orientation.w = 1.0
            self.k += 0.005
        elif self.radius < 2:
            pose_stamped.pose.position.x = self.radius * cos(self.angle)
            pose_stamped.pose.position.y = self.radius * sin(self.angle)
            pose_stamped.pose.position.z = self.k
            pose_stamped.pose.orientation.w = 1.0
            self.angle += 0.001
            self.radius += 0.001
        else:
            pose_stamped.pose.position.x = self.radius * cos(self.angle)
            pose_stamped.pose.position.y = self.radius * sin(self.angle)
            pose_stamped.pose.position.z = self.k
            pose_stamped.pose.orientation.w = 1.0
            self.angle += 0.001
            
        self.publisher_.publish(pose_stamped)

"""


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
