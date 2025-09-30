import rclpy
import os
import time
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32


# Coverage parameters
field_start = [2.5, 22.0]
field_size = [10.0, 10.0]
lane_width = 1.7
headland_width = 1.0


class FieldCoverClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('aan_field_cover_client')

        # Init path
        self.path = []


    def start(self, robot_name):

        # Start action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, robot_name+'/navigate_through_poses')

        # Publishers
        self.field_publisher = self.create_publisher(PolygonStamped, '/field_polygon', 10)
        self.headland_publisher = self.create_publisher(PolygonStamped, '/headland_polygon', 10)


    def publish_polygons(self, field_start, field_size, headland_width):

        field_polygon = [[field_start[0], field_start[1]], 
                         [field_start[0], field_start[1] + field_size[1]], 
                         [field_start[0] + field_size[0], field_start[1] + field_size[1]], 
                         [field_start[0] + field_size[0], field_start[1]]]
        
        field_polygon_msg = self.polygon_to_msg(field_polygon)
        self.field_publisher.publish(field_polygon_msg)
        
        headland_polygon = [[field_start[0] - headland_width, field_start[1] - headland_width], 
                            [field_start[0] - headland_width, field_start[1] + field_size[1] + headland_width], 
                            [field_start[0] + field_size[0] + headland_width, field_start[1] + field_size[1] + headland_width], 
                            [field_start[0] + field_size[0] + headland_width, field_start[1] - headland_width]]
        
        headland_polygon_msg = self.polygon_to_msg(headland_polygon)
        self.headland_publisher.publish(headland_polygon_msg)     


    def polygon_to_msg(self, polygon):
        
        msg = PolygonStamped()
        msg.header.frame_id = "map"

        for point in polygon:
            point_msg = Point32()
            point_msg.x = point[0]
            point_msg.y = point[1]
            msg.polygon.points.append(point_msg)

        return msg


    def plan_path(self, field_start, field_size, lane_width, headland_width):

        n_lanes = 0
        self.path = [[field_start[0] + lane_width/2.0, field_start[1] - headland_width]]

        while n_lanes < 6:
            self.path.append([field_start[0] + lane_width/2.0 + n_lanes*lane_width, field_start[1] - headland_width/2.0])
            self.path.append([field_start[0] + lane_width/2.0 + n_lanes*lane_width, field_start[1] + field_size[1] + headland_width/2.0])
            self.path.append([field_start[0] + lane_width/2.0 + (n_lanes+1)*lane_width, field_start[1] + field_size[1] + headland_width/2.0])
            self.path.append([field_start[0] + lane_width/2.0 + (n_lanes+1)*lane_width, field_start[1] - headland_width/2.0])
            n_lanes += 2
    

    def send_start(self):

        # First send starting position to NavigateToPose Navigator
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_to_pose.xml')
        
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.path[0][0]
        goal_msg.pose.pose.position.y = self.path[0][1]
        goal_msg.pose.pose.orientation.z = 0.7
        goal_msg.pose.pose.orientation.w = 0.7

        self.get_logger().info('Waiting for Nav2 NavigateToPose action server to come online...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server available, sending start pose as goal...')
        future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Done callback will send coverage path once start is reached
        future.add_done_callback(self.nav_to_pose_goal_response_callback)


    def nav_to_pose_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_to_pose_get_result_callback)


    def nav_to_pose_get_result_callback(self, future):
        self.get_logger().info(f"Reached start of coverage path")
        self.send_coverage_path()


    def send_coverage_path(self):

        # Send coverage path to NavigateThroughPoses navigator
        goal_msg = NavigateThroughPoses.Goal()
         
        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_exact_path.xml')
        
        for point in self.path:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = point[0]
            pose_msg.pose.position.y = point[1]
            pose_msg.pose.orientation.z = -0.7
            pose_msg.pose.orientation.w = 0.7
            goal_msg.poses.append(pose_msg)

        self.get_logger().info('Waiting for Nav2 NavigateThroughPoses action server to come online...')
        self.nav_through_poses_client.wait_for_server()
        self.get_logger().info('NavigateThroughPoses action server available, sending field coverage path...')
        future = self.nav_through_poses_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.nav_through_poses_goal_response_callback)


    def nav_through_poses_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_through_poses_get_result_callback)


    def nav_through_poses_get_result_callback(self, future):
        self.get_logger().info(f"Done!")
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        # Publish field polygons
        self.publish_polygons(field_start, field_size, headland_width)

        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    field_cover_client = FieldCoverClient()

    # Defaults
    robot_name = ""

    # Handle any command line inputs 
    if len(sys.argv) > 1 :
        robot_name = sys.argv[1]

    # Start client
    field_cover_client.start(robot_name=robot_name)

    # Publish field and headland polygons
    field_cover_client.publish_polygons(field_start, field_size, headland_width)

    # Compute coverage paths 
    field_cover_client.plan_path(field_start, field_size, lane_width, headland_width)
    
    # Send action request to Nav2 
    field_cover_client.send_start
   
    # Spin (so the node won't shut down after the goal is sent)
    rclpy.spin(field_cover_client)


if __name__ == '__main__':
    main()