import rclpy
import os
import time
import sys
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses,Wait,BackUp,DriveOnHeading
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32,Twist,Point,Polygon,Quaternion
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry

# Docking parameters
approx_docking_location = [28.0, 3.0]
# Coverage parameters
field_start = [2.5, 22.0]
field_size = [10.0, 10.0]
lane_width = 1.7
headland_width = 1.0
# Row parameters (x, y, East/West)
row_starts = [[19.0, 23.5, 'E'],
              [33.0, 25.0, 'W'],
              [19.0, 26.5, 'E']
              ] 

class DockingClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('aan_docking_client')

    def start(self, robot_name):

        # Start action client
        self.nav_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')
        

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_docking.xml')

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = approx_docking_location[0]
        pose_msg.pose.position.y = approx_docking_location[1]
        goal_msg.pose = pose_msg

        
        self.get_logger().info('Waiting for Nav2 action server to come online...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available, sending docking goal...')
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Done!")
        self.status = 'done'
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        time.sleep(1)

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
        self.status = 'done'
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        # Publish field polygons
        self.publish_polygons(field_start, field_size, headland_width)

        time.sleep(1)
class RowFollowClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('row_follow_client')

    def start(self, robot_name):

        # Start action client
        self.nav_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_row_following.xml')

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = row_starts[0][0]
        pose_msg.pose.position.y = row_starts[0][1]
        if row_starts[0][2] == 'E':
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
        elif row_starts[0][2] == 'W':
            pose_msg.pose.orientation.z = 1.0
            pose_msg.pose.orientation.w = 0.0
        goal_msg.pose = pose_msg

        self.get_logger().info('Waiting for Nav2 action server to come online...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available, sending row following goals...')
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        if len(row_starts) > 1:
            self.get_logger().info(f"Row completed, moving to next...")
            row_starts.pop(0)
            self.send_goal()
            return

        self.get_logger().info(f"Done!")
        self.status = 'done'
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        time.sleep(1)


'''Plan for Collision Avoidance Using /trajectories
Trajectory Publishing:

Each robot publishes its planned trajectory (a series of waypoints or poses) to a topic, e.g., /trajectory.
Trajectory Monitoring:

Each robot subscribes to the trajectory topic of the other robot.
Compare the robot's trajectory with the other robot's to detect overlaps.
Collision Detection:

Use geometric checks to identify trajectory overlaps, such as proximity of waypoints.
Recovery Behavior:

If a potential collision is detected, the robot can:
Pause execution using a Wait action.
Reroute using updated waypoints.
Adjust its position using DriveOnHeading.'''




class StatusHandler(Node):
    def __init__(self, robot_name):
        super().__init__('status_handler')
        self.robot_name = robot_name
        self.other_robot_name = "diffbot_0"
        robot_name == "diffbot_1" 

        # Subscribe to footprint topics
        self.footprint_sub = self.create_subscription(PolygonStamped, f'/{self.robot_name}/local_costmap/published_footprint', self.footprint_callback, 10)
        self.other_footprint_sub = self.create_subscription(PolygonStamped, f'/{self.other_robot_name}/local_costmap/published_footprint', self.other_footprint_callback, 10)

        # Store footprints
        self.my_footprint = None
        self.other_footprint = None

        # Collision threshold
        self.collision_distance = 2.0  # meters

    def footprint_callback(self, msg):
        """Callback for this robot's footprint."""
        self.my_footprint = self.polygon_from_msg(msg)
        self.check_collision()

    def other_footprint_callback(self, msg):
        """Callback for the other robot's footprint."""
        self.other_footprint = self.polygon_from_msg(msg)
        self.check_collision()

    def polygon_from_msg(self, msg):
        """Convert PolygonStamped to Shapely Polygon."""
        points = [(point.x, point.y) for point in msg.polygon.points]
        return Polygon(points)

    def check_collision(self):
        """Check for collisions between footprints."""
        if self.my_footprint and self.other_footprint:
            # Check for overlap
            if self.my_footprint.intersects(self.other_footprint):
                self.get_logger().warn("Collision detected: Footprints overlap!")
                self.avoid_collision()
                return

            # Check proximity
            min_distance = self.my_footprint.distance(self.other_footprint)
            if min_distance < self.collision_distance:
                self.get_logger().warn(f"Collision detected: Footprints too close (distance: {min_distance:.2f}m)!")
                self.avoid_collision()

    def avoid_collision(self):
        """Take collision avoidance actions."""
        self.get_logger().info("Stopping the robot to avoid collision.")
        # Implement recovery actions such as stopping or re-planning





class RecoveryActions(Node):
    def __init__(self,robot_name):
        super().__init__('recovery_actions')

        self.robot_name = robot_name
        self.wait_client = ActionClient(self, NavigateToPose, robot_name+'/wait')
        self.drive_client = ActionClient(self, NavigateToPose, robot_name+'/drive_on_heading')

    def wait(self, duration_seconds):
        """Pause execution using a valid Duration message."""
        duration_msg = Duration()
        duration_msg.sec = int(duration_seconds)
        duration_msg.nanosec = int((duration_seconds % 1) * 1e9)  # Handle fractional seconds

        self.get_logger().info(f"Pausing for {duration_seconds} seconds...")

        # Create a goal and send it
        goal_msg = Wait.Goal()
        goal_msg.time = duration_msg  # Use the Duration message
        self.wait_client.wait_for_server()
        future = self.wait_client.send_goal_async(goal_msg)
        future.add_done_callback(self.wait_response_callback)

    def wait_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Wait goal rejected!")
            return

        self.get_logger().info("Wait goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Wait completed.")

    def drive_on_heading(self, heading, distance, speed):
        """Perform a drive-on-heading recovery action."""
        goal_msg = DriveOnHeading.Goal()
        goal_msg.target.x = distance * np.cos(heading)
        goal_msg.target.y = distance * np.sin(heading)
        goal_msg.speed = speed
        self.get_logger().info(f"Sending drive goal: heading={heading}, distance={distance}, speed={speed}...")
        self.drive_client.wait_for_server()
        future = self.drive_client.send_goal_async(goal_msg)
        future.add_done_callback(self.drive_response_callback)

    def drive_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Drive goal rejected')
            return
        self.get_logger().info('Drive goal accepted, executing...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Recovery action completed.')


def wait_for_task_to_complete(client):
    """Generic wait function for task completion."""
    while rclpy.ok():
        rclpy.spin_once(client)
        if getattr(client, 'status', None) == 'done':  # Check client status
            print(f"{client.get_name()} task completed")
            break

def main(args=None):
    rclpy.init(args=args)  # Single initialization
    
    try:
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "diffbot_0"
        rclpy.init(args=args)
        status_handler = StatusHandler(robot_name)
        rclpy.init(args=args)
        recovery_actions = RecoveryActions(robot_name)
        

    # --- Field Coverage Task ---
       
        field_cover_client = FieldCoverClient()
        field_cover_client.start(robot_name)
    # Check for collisions before proceeding
        if status_handler.check_collision()== True:
            recovery_actions.wait(2.5)
        field_cover_client.publish_polygons(field_start, field_size, headland_width)
        field_cover_client.plan_path(field_start, field_size, lane_width, headland_width)
        field_cover_client.send_start()
        wait_for_task_to_complete(field_cover_client)
       
        field_cover_client.destroy_node()
        
        # --- Row Following Task
        rclpy.init(args=args) 
        row_follow_client = RowFollowClient()
        row_follow_client.start(robot_name) 
        if status_handler.check_collision()== True:
            recovery_actions.wait(2.5)
            # Start the row-following task
        row_follow_client.send_goal()
        wait_for_task_to_complete(row_follow_client)
        # Publish task completed
        # Destroy the client node after task completion
        row_follow_client.destroy_node

            # --- Docking Task ---
        rclpy.init(args=args)
        docking_client = DockingClient()
        docking_client.start(robot_name)

        if status_handler.check_collision()== True:
            recovery_actions.wait(2.5)
        docking_client.send_goal()
        wait_for_task_to_complete(docking_client)
        docking_client.destroy_node()

        rclpy.spin(status_handler)
        rclpy.spin(recovery_actions)

    except Exception as e:
      print(f"An error occurred: {e}")
    finally:
     if rclpy.ok():
        rclpy.shutdown()



if __name__ == '__main__':
    main()

''' if status_handler.check_collision()== True:
            recovery_actions.wait(1.0)
            recovery_actions.drive_on_heading(heading=np.pi, distance=5.0, speed=0.5)'''