import numpy as np
import math
from typing import Tuple, List, Optional
from fuzzy_centered import FuzzyInferenceSystem


class Vehicle:
    def __init__(self, x: float, y: float, angle: float, 
                 length: float = 40, width: float = 20):
        self.x = x
        self.y = y
        self.angle = angle
        self.length = length
        self.width = width
        
        self.steering_angle = 0.0
        self.velocity = 0.0
        self.max_steering_angle = 40.0
        self.wheelbase = length * 0.7
        
        self.trajectory = [(x, y)]
        self.max_trajectory_points = 500
        
        self.sensor_front = 0.0
        self.sensor_lateral = 0.0
        self.sensor_angle = 0.0
        self.sensor_depth = 0.0

        self.is_colliding = False
        self.is_parked = False
        self.parking_time = 0.0
        
    def get_corners(self) -> List[Tuple[float, float]]:
        cx, cy = self.x, self.y
        
        theta = math.radians(self.angle)
        
        half_length = self.length / 2
        half_width = self.width / 2
        
        local_corners = [
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width), 
            (-half_length, half_width)
        ]
        
        corners = []
        for lx, ly in local_corners:
            gx = cx + lx * math.cos(theta) - ly * math.sin(theta)
            gy = cy + lx * math.sin(theta) + ly * math.cos(theta)
            corners.append((gx, gy))
        
        return corners
    
    def get_front_center(self) -> Tuple[float, float]:
        theta = math.radians(self.angle)
        dx = (self.length / 2) * math.cos(theta)
        dy = (self.length / 2) * math.sin(theta)
        return (self.x + dx, self.y + dy)
    
    def get_rear_center(self) -> Tuple[float, float]:
        theta = math.radians(self.angle)
        dx = (self.length / 2) * math.cos(theta)
        dy = (self.length / 2) * math.sin(theta)
        return (self.x - dx, self.y - dy)
    
    def update_kinematics(self, dt: float):

        if abs(self.velocity) < 0.01:
            return
        
        theta = math.radians(self.angle)
        delta = math.radians(self.steering_angle)
        
        if abs(delta) > 0.001:
            turning_radius = self.wheelbase / math.tan(delta)
            
            omega = self.velocity / turning_radius
            
            dx = self.velocity * math.cos(theta) * dt
            dy = self.velocity * math.sin(theta) * dt
            dtheta = omega * dt
            
            self.x += dx
            self.y += dy
            self.angle += math.degrees(dtheta)
        else:
            dx = self.velocity * math.cos(theta) * dt
            dy = self.velocity * math.sin(theta) * dt
            
            self.x += dx
            self.y += dy
        
        self.angle = (self.angle + 180) % 360 - 180
        
        self.trajectory.append((self.x, self.y))
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)
    
    def update_sensors(self, parking_spot: 'ParkingSpot', obstacles: List['Obstacle']):
        
        front_pos = self.get_front_center()
        target_x = parking_spot.x
        self.sensor_front = abs(target_x - front_pos[0])
        
        parking_center_y = parking_spot.y + (parking_spot.width / 2)
        vehicle_center_y = self.y
        self.sensor_lateral = vehicle_center_y - parking_center_y
        
        self.sensor_angle = self.angle
        vehicle_center_x = self.x
        if vehicle_center_x >= parking_spot.x:
            self.sensor_depth = min(vehicle_center_x - parking_spot.x, 150)
        else:
            self.sensor_depth = 0
        
        self.is_colliding = False
        vehicle_corners = self.get_corners()
        
        for obstacle in obstacles:
            if obstacle.check_collision(vehicle_corners):
                self.is_colliding = True
                break
        

    
    def check_parked(self, parking_spot: 'ParkingSpot', dt: float) -> bool:

        
        front_pos = self.get_front_center()
        corners = self.get_corners()
        
        inside_parking = parking_spot.is_inside(corners, margin=5)
        
        well_aligned = abs(self.sensor_angle) < 5.0
        
        well_centered = abs(self.sensor_lateral) < 10
        
        good_front_distance = 10 < self.sensor_front < 100
        
        almost_stopped = abs(self.velocity) < 10.0
        
        if (inside_parking and well_aligned and well_centered and 
            good_front_distance and almost_stopped):
            self.parking_time += dt
            if self.parking_time > 1.0:
                self.is_parked = True
                return True
        else:
            self.parking_time = 0.0
        
        return False


class ParkingSpot:
    
    def __init__(self, x: float, y: float, length: float = 100, width: float = 60):
        self.x = x
        self.y = y
        self.length = length
        self.width = width
        
        self.x_min = x
        self.x_max = x + length
        self.y_min = y
        self.y_max = y + width
    
    def is_inside(self, points: List[Tuple[float, float]], margin: float = 0) -> bool:
        for px, py in points:
            if not (self.x_min - margin <= px <= self.x_max + margin and
                    self.y_min - margin <= py <= self.y_max + margin):
                return False
        return True
    
    def get_target_position(self) -> Tuple[float, float, float]:
        target_x = self.x + self.length / 2
        target_y = self.y + self.width / 2
        target_angle = 0.0
        return (target_x, target_y, target_angle)


class Obstacle:
    
    def __init__(self, x: float, y: float, width: float, height: float):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.x + self.width, self.y + self.height)
    
    def check_collision(self, points: List[Tuple[float, float]]) -> bool:
        x_min, y_min, x_max, y_max = self.get_bounds()
        
        for px, py in points:
            if x_min <= px <= x_max and y_min <= py <= y_max:
                return True
        return False


class ParkingSimulation:    
    def __init__(self, fuzzy_system: FuzzyInferenceSystem):
        self.fuzzy_system = fuzzy_system
        
        self.width = 800
        self.height = 600
        
        self.parking_spot = ParkingSpot(x=600, y=250, length=150, width=80)
        
        self.obstacles = [
            Obstacle(600, 170, 150, 10),
            Obstacle(600, 410, 150, 10),
            Obstacle(750, 170, 10, 250),
        ]
        
        self.reset_vehicle()

        self.time_elapsed = 0.0
        self.control_updates = 0
    
    def reset_vehicle(self, random_position: bool = False):
        if random_position:
            x = np.random.uniform(100, 400)
            y = np.random.uniform(200, 400)
            angle = np.random.uniform(-45, 45)
        else:
            x = 250
            y = 350
            angle = -15
        
        self.vehicle = Vehicle(x, y, angle, length=50, width=25)
        self.time_elapsed = 0.0
        self.control_updates = 0
    
    def update(self, dt: float) -> bool:
        self.time_elapsed += dt
        
        self.vehicle.update_sensors(self.parking_spot, self.obstacles)

        if self.vehicle.check_parked(self.parking_spot, dt):
            return False

        if self.vehicle.is_colliding:
            self.vehicle.velocity = 0.0
            return False
        
        fuzzy_inputs = {
            "distancia_frontal": self.vehicle.sensor_front,
            "distancia_lateral": self.vehicle.sensor_lateral,
            "angulo_veiculo": self.vehicle.sensor_angle,
            "profundidade_vaga": self.vehicle.sensor_depth
        }
        
        fuzzy_outputs = self.fuzzy_system.infer(fuzzy_inputs)
        
        if 60 <= self.vehicle.sensor_depth <= 90:
            distance_from_center = abs(self.vehicle.sensor_depth - 75)
            
            if distance_from_center <= 10:
                fuzzy_outputs["velocidade"] = 0.0
                fuzzy_outputs["angulo_direcao"] = 0.0
            else:
                fuzzy_outputs["velocidade"] = min(fuzzy_outputs["velocidade"], 5.0)
        
        self.vehicle.steering_angle = fuzzy_outputs["angulo_direcao"]
        self.vehicle.velocity = fuzzy_outputs["velocidade"]
        
        self.control_updates += 1
        
        self.vehicle.update_kinematics(dt)
        
        if self.time_elapsed > 30.0:
            return False
        
        return True
    
    def get_state(self) -> dict:
        return {
            'vehicle': self.vehicle,
            'parking_spot': self.parking_spot,
            'obstacles': self.obstacles,
            'time_elapsed': self.time_elapsed,
            'control_updates': self.control_updates,
            'fuzzy_inputs': {
                'distancia_frontal': self.vehicle.sensor_front,
                'distancia_lateral': self.vehicle.sensor_lateral,
                'angulo_veiculo': self.vehicle.sensor_angle,
                'profundidade_vaga': self.vehicle.sensor_depth
            },
            'fuzzy_outputs': {
                'angulo_direcao': self.vehicle.steering_angle,
                'velocidade': self.vehicle.velocity
            },
            'is_parked': self.vehicle.is_parked,
            'is_colliding': self.vehicle.is_colliding
        }


if __name__ == "__main__":
    from fuzzy_centered import create_centered_parking_system
    
    fis = create_centered_parking_system()
    sim = ParkingSimulation(fis)
    
    print("\nTestando simulação...")
    print(f"Posição inicial: ({sim.vehicle.x:.1f}, {sim.vehicle.y:.1f}), ângulo: {sim.vehicle.angle:.1f}°")

    dt = 0.1
    for i in range(10):
        sim.update(dt)
        state = sim.get_state()
        print(f"\nPasso {i+1}:")
        print(f"  Posição: ({sim.vehicle.x:.1f}, {sim.vehicle.y:.1f})")
        print(f"  Sensores: Front={state['fuzzy_inputs']['distancia_frontal']:.1f}, " 
              f"Lateral={state['fuzzy_inputs']['distancia_lateral']:.1f}, "
              f"Ângulo={state['fuzzy_inputs']['angulo_veiculo']:.1f}°")
        print(f"  Controle: Direção={state['fuzzy_outputs']['angulo_direcao']:.1f}°, "
              f"Velocidade={state['fuzzy_outputs']['velocidade']:.1f}")
    
    print("\n✓ Simulação funcionando corretamente!")
