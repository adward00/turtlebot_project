import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pi, cos, sin
import heapq
import numpy as np

class NodeHybridAStar:
    def __init__(self, parent=None, position=None, theta=0.0):
        self.parent = parent
        self.position = position
        self.theta = theta
        self.g = 0
        self.h = 0
        self.f = 0
    def __lt__(self, other):
        return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation_dynamic_replan_local_v4')

        # ===== Parameters =====
        self.lookahead_dist = 0.4
        self.linear_vel = 0.18
        self.min_linear_vel = 0.1  # 최소 linear velocity 강화
        self.turn_speed = 0.6
        self.stop_tolerance = 0.15
        self.front_dist = 0.6
        self.front_angle = 30 * pi / 180
        self.safety_margin = 0.25

        # ===== Replan cooldown =====
        self.last_replan_time = 0.0
        self.replan_interval = 1.0  # 최소 1초 간격

        # ===== Velocity smoothing =====
        self.prev_linear = 0.0
        self.max_delta_v = 0.03

        # ===== Map =====
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0

        # ===== State =====
        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = []
        self.path_index = 0
        self.goal_world = None
        self.dynamic_obstacle_grid = None

        # ===== ROS =====
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("IntegratedNavigation + Local Avoidance v4 READY")

    # =====================
    # Callbacks
    # =====================
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))

    def scan_callback(self, msg):
        if self.current_pose is None:
            return
        self.dynamic_obstacle_grid = np.zeros((self.map_height, self.map_width), dtype=bool)
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        x_robot, y_robot = self.current_pose
        safe_cells = int(self.safety_margin / self.map_resolution)
        for r, a in zip(ranges, angles):
            if abs(a) > self.front_angle:
                continue
            if r < msg.range_max:
                x_obs = x_robot + r*cos(self.current_yaw + a)
                y_obs = y_robot + r*sin(self.current_yaw + a)
                gy, gx = self.world_to_grid([x_obs, y_obs])
                for dy in range(-safe_cells, safe_cells+1):
                    for dx in range(-safe_cells, safe_cells+1):
                        ny, nx = gy+dy, gx+dx
                        if 0<=ny<self.map_height and 0<=nx<self.map_width:
                            self.dynamic_obstacle_grid[ny, nx] = True

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None:
            return
        self.goal_world = [msg.pose.position.x, msg.pose.position.y]
        self.force_plan_path()

    # =====================
    # Force Hybrid A* replan
    # =====================
    def force_plan_path(self):
        if self.current_pose is None or self.goal_world is None:
            return
        start = self.world_to_grid(self.current_pose)
        goal = self.world_to_grid(self.goal_world)
        path = self.run_hybrid_astar(start, goal)
        if path is None:
            self.get_logger().warn("Hybrid A* failed!")
            return
        self.global_path = [self.grid_to_world(p) for p in path]
        self.path_index = 0
        self.publish_path_viz()
        self.last_replan_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Path replanned: {len(self.global_path)} points")

    def run_hybrid_astar(self, start, goal):
        open_list = []
        visited = set()
        heapq.heappush(open_list, NodeHybridAStar(None, start))
        moves = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        safe_cells = int(self.safety_margin / self.map_resolution)
        while open_list:
            current = heapq.heappop(open_list)
            if current.position in visited: continue
            visited.add(current.position)
            if current.position == goal:
                path = []
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]
            for dy, dx in moves:
                ny, nx = current.position[0]+dy, current.position[1]+dx
                if not (0<=ny<self.map_height and 0<=nx<self.map_width): continue
                blocked = False
                for ddy in range(-safe_cells, safe_cells+1):
                    for ddx in range(-safe_cells, safe_cells+1):
                        ny2, nx2 = ny+ddy, nx+ddx
                        if 0<=ny2<self.map_height and 0<=nx2<self.map_width:
                            if self.map_data[ny2, nx2]>50 or (self.dynamic_obstacle_grid[ny2, nx2]):
                                blocked = True
                                break
                    if blocked: break
                if blocked: continue
                node = NodeHybridAStar(current, (ny, nx))
                node.g = current.g + 1
                node.h = sqrt((ny-goal[0])**2 + (nx-goal[1])**2)
                node.f = node.g + node.h
                heapq.heappush(open_list, node)
        return None

    # =====================
    # Front obstacle check
    # =====================
    def check_front_obstacle(self):
        robot_grid = self.world_to_grid(self.current_pose)
        check_cells = int(self.front_dist/self.map_resolution)
        angle_cells = int(np.tan(self.front_angle)*check_cells)
        for dy in range(-angle_cells, angle_cells+1):
            for dx in range(0, check_cells+1):
                ny, nx = robot_grid[0]+dy, robot_grid[1]+dx
                if 0<=ny<self.map_height and 0<=nx<self.map_width:
                    if self.dynamic_obstacle_grid[ny,nx] or self.map_data[ny,nx]>50:
                        return True
        return False

    # =====================
    # Local avoidance
    # =====================
    def local_avoidance(self, target):
        linear_candidates = np.linspace(self.min_linear_vel, self.linear_vel, 5)
        angular_candidates = np.linspace(-self.turn_speed, self.turn_speed, 11)
        best_score = -np.inf
        best_cmd = Twist()
        for v in linear_candidates:
            for w in angular_candidates:
                score = self.evaluate_local(v, w, target)
                if score > best_score:
                    best_score = score
                    best_cmd.linear.x = max(v, self.min_linear_vel)  # 최소 linear 강화
                    best_cmd.angular.z = w
        if best_score == -np.inf:
            best_cmd.linear.x = self.min_linear_vel
            best_cmd.angular.z = self.turn_speed*0.5
        return best_cmd

    def evaluate_local(self, v, w, target):
        dt = 0.2
        x_new = self.current_pose[0] + v*dt*cos(self.current_yaw + w*dt)
        y_new = self.current_pose[1] + v*dt*sin(self.current_yaw + w*dt)
        grid = self.world_to_grid([x_new, y_new])
        for dy in range(-int(self.safety_margin/self.map_resolution), int(self.safety_margin/self.map_resolution)+1):
            for dx in range(-int(self.safety_margin/self.map_resolution), int(self.safety_margin/self.map_resolution)+1):
                ny, nx = grid[0]+dy, grid[1]+dx
                if 0<=ny<self.map_height and 0<=nx<self.map_width:
                    if self.map_data[ny,nx]>50 or self.dynamic_obstacle_grid[ny,nx]:
                        return -np.inf
        dx = target[0]-x_new
        dy = target[1]-y_new
        alpha = atan2(dy, dx) - (self.current_yaw + w*dt)
        while alpha>pi: alpha-=2*pi
        while alpha<-pi: alpha+=2*pi
        return v - abs(alpha)*0.3  # alpha 패널티 완화

    # =====================
    # Control loop
    # =====================
    def control_loop(self):
        if self.current_pose is None or self.goal_world is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        obstacle_close = self.check_front_obstacle()

        # Force Replan with cooldown
        if (self.path_blocked() or obstacle_close) and (now - self.last_replan_time > self.replan_interval):
            self.force_plan_path()

        if not self.global_path:
            return

        target_index = min(self.path_index+2, len(self.global_path)-1)
        target = self.global_path[target_index]

        cmd = self.local_avoidance(target)
        self.pub_cmd.publish(cmd)

        dx = target[0]-self.current_pose[0]
        dy = target[1]-self.current_pose[1]
        dist_target = sqrt(dx**2 + dy**2)
        if dist_target < self.lookahead_dist and self.path_index < len(self.global_path)-1:
            self.path_index += 1

        goal = self.global_path[-1]
        dist_goal = sqrt((goal[0]-self.current_pose[0])**2 + (goal[1]-self.current_pose[1])**2)
        if dist_goal < self.stop_tolerance:
            self.stop_robot()
            self.global_path = []
            self.get_logger().info("Goal reached!")

    def path_blocked(self):
        for i in range(self.path_index, min(self.path_index+10, len(self.global_path))):
            grid = self.world_to_grid(self.global_path[i])
            if self.dynamic_obstacle_grid[grid[0], grid[1]] or self.map_data[grid[0], grid[1]]>50:
                return True
        return False

    # =====================
    # Utils
    # =====================
    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [grid[1]*self.map_resolution+self.map_origin[0],
                grid[0]*self.map_resolution+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

def main():
    rclpy.init()
    node = IntegratedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
