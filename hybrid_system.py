import numpy as np
import math
from typing import List, Tuple, Dict, Optional
from genetic_algorithm import GeneticAlgorithm
from fuzzy_centered import FuzzyInferenceSystem
from simulation import Vehicle, ParkingSpot, Obstacle


class TrajectoryTracker:
    def __init__(self, trajectory: List[Tuple[float, float, float]]):
        self.trajectory = trajectory
        self.current_index = 0
        self.lookahead_distance = 60.0
        self.progress = 0
        self.last_velocity = 0.0
        self.last_steering = 0.0
    
    def get_reference_point(self, vehicle_x: float, vehicle_y: float) -> Tuple[float, float, float]:
        if not self.trajectory:
            return (vehicle_x, vehicle_y, 0.0)
        
        min_dist = float('inf')
        closest_idx = self.progress
        search_start = max(0, self.progress - 10)
        search_end = min(len(self.trajectory), self.progress + 50)
        for i in range(search_start, search_end):
            x, y, angle = self.trajectory[i]
            dist = math.sqrt((x - vehicle_x)**2 + (y - vehicle_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        self.progress = closest_idx
        base_lookahead = max(5, int(self.lookahead_distance / 4))
        lookahead_idx = min(closest_idx + base_lookahead, len(self.trajectory) - 1)
        x_ref, y_ref, angle_ref = self.trajectory[lookahead_idx]
        self.current_index = lookahead_idx
        return (x_ref, y_ref, angle_ref)
    
    def calculate_tracking_error(self, vehicle_x: float, vehicle_y: float, 
                                vehicle_angle: float) -> Dict[str, float]:
        x_ref, y_ref, angle_ref = self.get_reference_point(vehicle_x, vehicle_y)
        
        position_error = math.sqrt((x_ref - vehicle_x)**2 + (y_ref - vehicle_y)**2)
        angle_error = angle_ref - vehicle_angle
        while angle_error > 180:
            angle_error -= 360
        while angle_error < -180:
            angle_error += 360
        return {
            'position_error': position_error,
            'angle_error': angle_error,
            'reference_x': x_ref,
            'reference_y': y_ref,
            'reference_angle': angle_ref
        }


class HybridParkingSystem:
    
    
    def __init__(self, 
                 fuzzy_system: FuzzyInferenceSystem,
                 initial_pose: Tuple[float, float, float],
                 final_pose: Tuple[float, float, float],
                 parking_spot: ParkingSpot,
                 obstacles: List[Obstacle],
                 vehicle_params: Dict):
        self.fuzzy_system = fuzzy_system
        self.initial_pose = initial_pose
        self.final_pose = final_pose
        self.parking_spot = parking_spot
        self.obstacles = obstacles
        self.vehicle_params = vehicle_params
        
        self.ga_result = None
        self.trajectory_tracker = None
        self.ga_optimized = False
        
    def optimize_trajectory(self, 
                           population_size: int = 50,
                           generations: int = 100,
                           verbose: bool = True) -> Dict:
        if verbose:
            print("\n[FASE 1] Executando Algoritmo Genético (Offline)...")
        
        ga = GeneticAlgorithm(
            initial_pose=self.initial_pose,
            final_pose=self.final_pose,
            parking_spot=self.parking_spot,
            obstacles=self.obstacles,
            vehicle_params=self.vehicle_params,
            population_size=population_size,
            generations=generations
        )
        
        self.ga_result = ga.run()
        self.trajectory_tracker = TrajectoryTracker(self.ga_result['trajectory'])
        self.ga_optimized = True
        
      if verbose:
        print("[FASE 1] ✓ Trajetória otimizada pelo AG")
        print(f"  Parâmetros: k₀={self.ga_result['k0']:.4f}, "
            f"k₁={self.ga_result['k1']:.4f}, Vₛ={self.ga_result['vs']}")
        print(f"  Desempenho: |S|={self.ga_result['path_length']:.2f}, "
            f"|φ|max={self.ga_result['max_steering']:.2f}°\n")
        
        return self.ga_result
    
    def reoptimize_trajectory(self, new_initial_pose: Tuple[float, float, float],
                             population_size: int = 30,
                             generations: int = 50,
                             verbose: bool = False) -> Dict:
        self.initial_pose = new_initial_pose
        if verbose:
            print(f"\n[AG] Reotimizando trajetória para nova posição: ({new_initial_pose[0]:.1f}, {new_initial_pose[1]:.1f}), {new_initial_pose[2]:.1f}°")
        ga = GeneticAlgorithm(
            initial_pose=self.initial_pose,
            final_pose=self.final_pose,
            parking_spot=self.parking_spot,
            obstacles=self.obstacles,
            vehicle_params=self.vehicle_params,
            population_size=population_size,
            generations=generations
        )
        self.ga_result = ga.run()
        self.trajectory_tracker = TrajectoryTracker(self.ga_result['trajectory'])
        self.ga_optimized = True
        if verbose:
            print(f"[AG] ✓ Nova trajetória otimizada")
            print(f"  k₀={self.ga_result['k0']:.4f}, k₁={self.ga_result['k1']:.4f}, Vₛ={self.ga_result['vs']}")
            print(f"  |S|={self.ga_result['path_length']:.2f}, |φ|max={self.ga_result['max_steering']:.2f}°")
        return self.ga_result
    
    def _calculate_tracking_control(self, vehicle: Vehicle) -> Dict[str, float]:
        tracking_error = self.trajectory_tracker.calculate_tracking_error(
            vehicle.x, vehicle.y, vehicle.angle
        )
        
        position_error = tracking_error['position_error']
        angle_error = tracking_error['angle_error']
        ref_x = tracking_error['reference_x']
        ref_y = tracking_error['reference_y']
        ref_angle = tracking_error['reference_angle']
        dx = ref_x - vehicle.x
        dy = ref_y - vehicle.y
        dist_to_ref = math.sqrt(dx*dx + dy*dy)
        if dist_to_ref > 0.1:
            desired_angle = math.degrees(math.atan2(dy, dx))
        else:
            desired_angle = ref_angle
        direction_error = desired_angle - vehicle.angle
        while direction_error > 180:
            direction_error -= 360
        while direction_error < -180:
            direction_error += 360
        base_velocity = min(25.0, max(8.0, dist_to_ref * 0.5))
        angle_error_abs = abs(angle_error)
        if angle_error_abs > 45:
            base_velocity *= 0.4
        elif angle_error_abs > 25:
            base_velocity *= 0.7
        elif angle_error_abs > 10:
            base_velocity *= 0.9
        if dist_to_ref < 15:
            base_velocity *= 0.6
        kp_direction = 0.5
        kp_angle = 0.3
        steering_angle = direction_error * kp_direction + angle_error * kp_angle
        steering_angle = np.clip(steering_angle, -40, 40)
        alpha = 0.7
        steering_angle = alpha * self.trajectory_tracker.last_steering + (1 - alpha) * steering_angle
        self.trajectory_tracker.last_steering = steering_angle
        vs = self.ga_result['vs']
        if vs == -1:
            base_velocity = -abs(base_velocity)
        alpha_vel = 0.6
        base_velocity = alpha_vel * self.trajectory_tracker.last_velocity + (1 - alpha_vel) * base_velocity
        self.trajectory_tracker.last_velocity = base_velocity
        if vehicle.sensor_front < 25:
            base_velocity *= 0.3
        elif vehicle.sensor_front < 40:
            base_velocity *= 0.6
        return {
            'velocidade': base_velocity,
            'angulo_direcao': steering_angle
        }
    
    def get_fuzzy_control(self, vehicle: Vehicle, use_tracking: bool = True) -> Dict[str, float]:
        if not self.ga_optimized or not use_tracking:
            fuzzy_inputs = {
                "distancia_frontal": vehicle.sensor_front,
                "distancia_lateral": vehicle.sensor_lateral,
                "angulo_veiculo": vehicle.sensor_angle,
                "profundidade_vaga": vehicle.sensor_depth
            }
            fuzzy_outputs = self.fuzzy_system.infer(fuzzy_inputs)
            return fuzzy_outputs
        else:
            tracking_control = self._calculate_tracking_control(vehicle)
            fuzzy_inputs = {
                "distancia_frontal": vehicle.sensor_front,
                "distancia_lateral": vehicle.sensor_lateral,
                "angulo_veiculo": vehicle.sensor_angle,
                "profundidade_vaga": vehicle.sensor_depth
            }
            fuzzy_outputs = self.fuzzy_system.infer(fuzzy_inputs)
            if vehicle.sensor_front < 15:
                final_velocity = fuzzy_outputs['velocidade'] * 0.7 + tracking_control['velocidade'] * 0.3
                final_steering = fuzzy_outputs['angulo_direcao'] * 0.7 + tracking_control['angulo_direcao'] * 0.3
            elif vehicle.sensor_front < 30:
                final_velocity = tracking_control['velocidade'] * 0.75 + fuzzy_outputs['velocidade'] * 0.25
                final_steering = tracking_control['angulo_direcao'] * 0.8 + fuzzy_outputs['angulo_direcao'] * 0.2
            else:
                final_velocity = tracking_control['velocidade'] * 0.95 + fuzzy_outputs['velocidade'] * 0.05
                final_steering = tracking_control['angulo_direcao'] * 0.9 + fuzzy_outputs['angulo_direcao'] * 0.1
            return {
                'velocidade': final_velocity,
                'angulo_direcao': final_steering
            }
    
    def get_ga_trajectory(self) -> Optional[List[Tuple[float, float, float]]]:
        if self.ga_result:
            return self.ga_result['trajectory']
        return None
    
    def get_ga_parameters(self) -> Optional[Dict]:
        if self.ga_result:
            return {
                'k0': self.ga_result['k0'],
                'k1': self.ga_result['k1'],
                'vs': self.ga_result['vs'],
                'path_length': self.ga_result['path_length'],
                'max_steering': self.ga_result['max_steering'],
                'objective_value': self.ga_result['objective_value']
            }
        return None

