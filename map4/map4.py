from controller import Robot
import math
import heapq

# PARAMETERS
TIME_STEP = 64
MAX_SPEED = 6.28

ARENA_SIZE = 1
GRID_SIZE = 5
CELL_SIZE = ARENA_SIZE / GRID_SIZE

START_GRID = (0, 0)
GOAL_GRID  = (4, 4)

OBSTACLES = [
    (1, 2),
    (2, 0),
    (3, 2),
    (2, 4),
    (4, 3),
]

LOOKAHEAD_DIST = 0.15
KP_ANGULAR = 2.5
BASE_SPEED = MAX_SPEED * 0.6

IR_THRESHOLD = 100


class AStarPlanner:
    """Pathfinding menggunakan algoritma A*"""
    
    def __init__(self, grid_size, obstacles):
        self.N = grid_size
        self.obstacles = set(obstacles)
    
    def heuristic(self, a, b):
        """Menghitung jarak Euclidean antara dua titik"""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def get_neighbors(self, node):
        """Menghasilkan tetangga yang valid dari sebuah node"""
        x, y = node
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        valid_neighbors = []
        
        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            
            # Cek batas arena
            if not (0 <= nx < self.N and 0 <= ny < self.N):
                continue
            
            # Cek obstacle
            if (nx, ny) in self.obstacles:
                continue
            
            # Cek corner cutting untuk diagonal
            is_diagonal = (dx != 0 and dy != 0)
            if is_diagonal:
                horizontal_blocked = (x + dx, y) in self.obstacles
                vertical_blocked = (x, y + dy) in self.obstacles
                if horizontal_blocked or vertical_blocked:
                    continue
            
            # Hitung jarak
            distance = math.sqrt(dx * dx + dy * dy)
            valid_neighbors.append(((nx, ny), distance))
        
        return valid_neighbors
    
    def plan(self, start, goal):
        """Mencari jalur optimal dari start ke goal"""
        frontier = [(0, start)]
        came_from = {start: None}
        cost = {start: 0}
        
        while frontier:
            _, current = heapq.heappop(frontier)
            
            if current == goal:
                break
            
            for nxt, step_cost in self.get_neighbors(current):
                new_cost = cost[current] + step_cost
                
                if nxt not in cost or new_cost < cost[nxt]:
                    cost[nxt] = new_cost
                    priority = new_cost + self.heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        
        # Rekonstruksi jalur
        if goal not in came_from:
            print("[A*] No path found")
            return []
        
        path = []
        current_node = goal
        while current_node is not None:
            path.append(current_node)
            current_node = came_from[current_node]
        
        path.reverse()
        return path


class RobotNavigator:
    """Mengatur navigasi robot menggunakan pure pursuit dan obstacle avoidance"""
    
    def __init__(self, robot):
        self.robot = robot
        self.planner = AStarPlanner(GRID_SIZE, OBSTACLES)
        self._initialize_motors()
        self._initialize_sensors()
        
        self.path = []
        self.index = 0
        self.trajectory = []
        self.last_ir_state = "CLEAR"
    
    def _initialize_motors(self):
        """Inisialisasi motor roda"""
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
    
    def _initialize_sensors(self):
        """Inisialisasi GPS, compass, dan infrared sensors"""
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(TIME_STEP)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(TIME_STEP)
        
        sensor_names = ['ps0', 'ps1', 'ps6', 'ps7']
        self.ir = []
        for name in sensor_names:
            s = self.robot.getDevice(name)
            s.enable(TIME_STEP)
            self.ir.append(s)
    
    def grid_to_world(self, g):
        """Konversi koordinat grid ke koordinat dunia"""
        world_x = (g[0] + 0.5) * CELL_SIZE - ARENA_SIZE / 2
        world_y = (g[1] + 0.5) * CELL_SIZE - ARENA_SIZE / 2
        return world_x, world_y
    
    def get_position(self):
        """Ambil posisi robot dari GPS"""
        p = self.gps.getValues()
        return p[0], p[1]
    
    def get_heading(self):
        """Ambil orientasi robot dari compass"""
        n = self.compass.getValues()
        return math.atan2(n[0], n[1])
    
    def ir_left(self):
        """Deteksi obstacle di sisi kiri"""
        left_reading = max(self.ir[2].getValue(), self.ir[3].getValue())
        return left_reading > IR_THRESHOLD
    
    def ir_right(self):
        """Deteksi obstacle di sisi kanan"""
        right_reading = max(self.ir[0].getValue(), self.ir[1].getValue())
        return right_reading > IR_THRESHOLD
    
    def ir_state(self):
        """Tentukan status obstacle berdasarkan IR sensors"""
        left_detected = self.ir_left()
        right_detected = self.ir_right()
        
        if left_detected and right_detected:
            return "FRONT"
        elif left_detected:
            return "LEFT"
        elif right_detected:
            return "RIGHT"
        else:
            return "CLEAR"
    
    def log_ir_event(self):
        """Log perubahan status IR sensor"""
        state = self.ir_state()
        
        if state == self.last_ir_state:
            return
        
        messages = {
            "LEFT": "[IR] Obstacle LEFT detected",
            "RIGHT": "[IR] Obstacle RIGHT detected",
            "FRONT": "[IR] Obstacle FRONT detected",
            "CLEAR": "[IR] Path clear"
        }
        
        if state in messages:
            print(messages[state])
        
        self.last_ir_state = state
    
    def find_lookahead_point(self, px, py):
        """Cari titik lookahead untuk pure pursuit"""
        for i in range(self.index, len(self.path)):
            wx, wy = self.grid_to_world(self.path[i])
            distance = math.hypot(wx - px, wy - py)
            
            if distance >= LOOKAHEAD_DIST:
                self.index = i
                return wx, wy
        
        return self.grid_to_world(self.path[-1])
    
    def plan_path(self):
        """Rencanakan jalur dari start ke goal"""
        self.path = self.planner.plan(START_GRID, GOAL_GRID)
        
        print(f"[A*] Planning path from {START_GRID} to {GOAL_GRID}")
        print(f"[A*] Path found with {len(self.path)} waypoints")
        
        for i, p in enumerate(self.path):
            print(f"  {i}: {p}")
        
        return len(self.path) > 0
    
    def save_trajectory(self):
        """Simpan trajectory ke file CSV"""
        with open("trajectory_map1.csv", "w") as f:
            f.write("x,y\n")
            for x, y in self.trajectory:
                f.write(f"{x},{y}\n")
    
    def _set_motor_speeds(self, left_speed, right_speed):
        """Set kecepatan motor dengan batasan"""
        left_clamped = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_clamped = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
        
        self.left_motor.setVelocity(left_clamped)
        self.right_motor.setVelocity(right_clamped)
    
    def _handle_obstacle_avoidance(self):
        """Handle obstacle avoidance behavior"""
        left_detected = self.ir_left()
        right_detected = self.ir_right()
        
        if left_detected and not right_detected:
            self._set_motor_speeds(BASE_SPEED * 0.6, BASE_SPEED * 0.2)
            return True
        
        if right_detected and not left_detected:
            self._set_motor_speeds(BASE_SPEED * 0.2, BASE_SPEED * 0.6)
            return True
        
        return False
    
    def _pure_pursuit_control(self, px, py):
        """Implementasi pure pursuit controller"""
        tx, ty = self.find_lookahead_point(px, py)
        target_angle = math.atan2(ty - py, tx - px)
        
        current_heading = self.get_heading()
        angle_error = target_angle - current_heading
        
        # Normalisasi angle error ke [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        
        omega = KP_ANGULAR * angle_error
        left_speed = BASE_SPEED - omega
        right_speed = BASE_SPEED + omega
        
        self._set_motor_speeds(left_speed, right_speed)
    
    def step(self):
        """Step utama untuk kontrol robot"""
        px, py = self.get_position()
        self.trajectory.append((px, py))
        
        self.log_ir_event()
        
        # Cek apakah goal tercapai
        gx, gy = self.grid_to_world(self.path[-1])
        distance_to_goal = math.hypot(gx - px, gy - py)
        
        if distance_to_goal < 0.07:
            self._set_motor_speeds(0, 0)
            self.save_trajectory()
            print("[NAV] Goal reached")
            return True
        
        # Obstacle avoidance
        if self._handle_obstacle_avoidance():
            return False
        
        # Pure pursuit
        self._pure_pursuit_control(px, py)
        return False


def main():
    """Fungsi utama untuk menjalankan navigasi robot"""
    robot = Robot()
    navigator = RobotNavigator(robot)
    
    if not navigator.plan_path():
        return
    
    print("[MAIN] Navigation started")
    
    while robot.step(TIME_STEP) != -1:
        if navigator.step():
            break
    
    print("[MAIN] Navigation finished")


if __name__ == "__main__":
    main()