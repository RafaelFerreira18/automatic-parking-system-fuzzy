import numpy as np
import math
from typing import List, Tuple, Dict, Optional
from simulation import Vehicle, ParkingSpot, Obstacle


class GeneticAlgorithm:
    def __init__(self, 
                 initial_pose: Tuple[float, float, float],
                 final_pose: Tuple[float, float, float],
                 parking_spot: ParkingSpot,
                 obstacles: List[Obstacle],
                 vehicle_params: Dict,
                 population_size: int = 50,
                 generations: int = 100,
                 crossover_rate: float = 0.6,
                 mutation_rate: float = 0.04,
                 k0_range: Tuple[float, float] = (-2.0, 2.0),
                 k1_range: Tuple[float, float] = (-2.0, 2.0),
                 precision: float = 1e-8):

        self.initial_pose = initial_pose
        self.final_pose = final_pose
        self.parking_spot = parking_spot
        self.obstacles = obstacles
        self.vehicle_params = vehicle_params
        
        self.population_size = population_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.k0_range = k0_range
        self.k1_range = k1_range
        self.precision = precision
        
        k0_bits = self._calculate_bits(k0_range[0], k0_range[1], precision)
        k1_bits = self._calculate_bits(k1_range[0], k1_range[1], precision)
        vs_bits = 1
        
        self.chromosome_length = k0_bits + k1_bits + vs_bits
        self.k0_bits = k0_bits
        self.k1_bits = k1_bits
        self.vs_bits = vs_bits
        
        self.best_fitness_history = []
        self.avg_fitness_history = []
        self.best_individual_history = []
        
    def _calculate_bits(self, min_val: float, max_val: float, precision: float) -> int:
        range_size = max_val - min_val
        num_intervals = range_size / precision
        bits = int(np.ceil(np.log2(num_intervals)))
        return max(bits, 8)
    
    def _encode_parameter(self, value: float, min_val: float, max_val: float, bits: int) -> List[int]:
        normalized = (value - min_val) / (max_val - min_val)
        normalized = np.clip(normalized, 0.0, 1.0)
        
        max_int = (1 << bits) - 1
        int_value = int(normalized * max_int)
        
        binary = [int(b) for b in format(int_value, f'0{bits}b')]
        return binary
    
    def _decode_parameter(self, binary: List[int], min_val: float, max_val: float) -> float:
        int_value = int(''.join(map(str, binary)), 2)
        
        max_int = (1 << len(binary)) - 1
        normalized = int_value / max_int if max_int > 0 else 0.0
        
        value = min_val + normalized * (max_val - min_val)
        return value
    
    def _create_individual(self) -> List[int]:
        k0 = np.random.uniform(self.k0_range[0], self.k0_range[1])
        k1 = np.random.uniform(self.k1_range[0], self.k1_range[1])
        vs = np.random.choice([-1, 1])
        
        k0_binary = self._encode_parameter(k0, self.k0_range[0], self.k0_range[1], self.k0_bits)
        k1_binary = self._encode_parameter(k1, self.k1_range[0], self.k1_range[1], self.k1_bits)
        vs_binary = [1 if vs == 1 else 0]
        
        return k0_binary + k1_binary + vs_binary
    
    def _decode_individual(self, chromosome: List[int]) -> Tuple[float, float, int]:
        k0_binary = chromosome[:self.k0_bits]
        k1_binary = chromosome[self.k0_bits:self.k0_bits + self.k1_bits]
        vs_binary = chromosome[self.k0_bits + self.k1_bits:]
        
        k0 = self._decode_parameter(k0_binary, self.k0_range[0], self.k0_range[1])
        k1 = self._decode_parameter(k1_binary, self.k1_range[0], self.k1_range[1])
        vs = 1 if vs_binary[0] == 1 else -1
        
        return k0, k1, vs
    
    def _generate_trajectory(self, k0: float, k1: float, vs: int, num_points: int = 200) -> Tuple[List[Tuple[float, float, float]], float, float]:
        x0, y0, theta0 = self.initial_pose
        xf, yf, thetaf = self.final_pose
        
        theta0_rad = math.radians(theta0)
        thetaf_rad = math.radians(thetaf)
        
        s_values = np.linspace(0, 1, num_points)
        
        trajectory = []
        path_length = 0.0
        max_steering = 0.0
        
        prev_x, prev_y = x0, y0
        
        for s in s_values:
            t = s

            total_dist = math.sqrt((xf - x0)**2 + (yf - y0)**2)
            control_scale = total_dist * 0.4
            
            control_x1 = x0 + k0 * math.cos(theta0_rad) * control_scale
            control_y1 = y0 + k0 * math.sin(theta0_rad) * control_scale
            control_x2 = xf - k1 * math.cos(thetaf_rad) * control_scale
            control_y2 = yf - k1 * math.sin(thetaf_rad) * control_scale
            
            x = (1-t)**3 * x0 + 3*(1-t)**2*t * control_x1 + 3*(1-t)*t**2 * control_x2 + t**3 * xf
            y = (1-t)**3 * y0 + 3*(1-t)**2*t * control_y1 + 3*(1-t)*t**2 * control_y2 + t**3 * yf
            
            if s > 0:
                dx_dt = -3*(1-t)**2 * x0 + 3*((1-t)**2 - 2*(1-t)*t) * control_x1 + \
                        3*(2*(1-t)*t - t**2) * control_x2 + 3*t**2 * xf
                dy_dt = -3*(1-t)**2 * y0 + 3*((1-t)**2 - 2*(1-t)*t) * control_y1 + \
                        3*(2*(1-t)*t - t**2) * control_y2 + 3*t**2 * yf
                
                theta = math.atan2(dy_dt, dx_dt)
            else:
                theta = theta0_rad
            
            if vs == -1:
                theta = theta + math.pi
                if theta > math.pi:
                    theta -= 2 * math.pi
            
            trajectory.append((x, y, math.degrees(theta)))
            
            if s > 0:
                dx = x - prev_x
                dy = y - prev_y
                path_length += math.sqrt(dx*dx + dy*dy)
            
            prev_x, prev_y = x, y
        
        wheelbase = self.vehicle_params.get('wheelbase', 35.0)
        max_steering = 0.0
        
        for i in range(1, len(trajectory)):
            x1, y1, theta1_deg = trajectory[i-1]
            x2, y2, theta2_deg = trajectory[i]
            
            dx = x2 - x1
            dy = y2 - y1
            ds = math.sqrt(dx*dx + dy*dy)
            
            if ds > 0.01:
                dtheta = math.radians(theta2_deg - theta1_deg)
                
                if abs(ds) > 1e-6:
                    steering_rad = math.atan(wheelbase * dtheta / ds)
                    steering_deg = abs(math.degrees(steering_rad))
                    max_steering = max(max_steering, steering_deg)
        
        return trajectory, path_length, max_steering
    
    def _check_collision(self, trajectory: List[Tuple[float, float, float]]) -> bool:
        vehicle_length = self.vehicle_params.get('length', 50.0)
        vehicle_width = self.vehicle_params.get('width', 25.0)
        detection_radius = max(vehicle_length, vehicle_width) / 2
        
        for x, y, angle in trajectory:
            for obstacle in self.obstacles:
                obs_x_min, obs_y_min, obs_x_max, obs_y_max = obstacle.get_bounds()
                obs_center_x = (obs_x_min + obs_x_max) / 2
                obs_center_y = (obs_y_min + obs_y_max) / 2
                
                dist = math.sqrt((x - obs_center_x)**2 + (y - obs_center_y)**2)
                
                if (obs_x_min - detection_radius <= x <= obs_x_max + detection_radius and
                    obs_y_min - detection_radius <= y <= obs_y_max + detection_radius):
                    return True
        
        return False
    
    def _evaluate_fitness(self, chromosome: List[int]) -> float:
        k0, k1, vs = self._decode_individual(chromosome)
        
        trajectory, path_length, max_steering = self._generate_trajectory(k0, k1, vs)
        
        has_collision = self._check_collision(trajectory)
        
        if has_collision:
            path_length = 100.0 * path_length
        
        fitness = path_length**2 + max_steering**2
        
        return -fitness
    
    def _roulette_wheel_selection(self, population: List[List[int]], fitness: List[float]) -> List[int]:
        min_fitness = min(fitness)
        adjusted_fitness = [f - min_fitness + 1e-10 for f in fitness]
        total_fitness = sum(adjusted_fitness)
        
        if total_fitness < 1e-10:
            return population[np.random.randint(len(population))]
        
        probabilities = [f / total_fitness for f in adjusted_fitness]
        
        selected_idx = np.random.choice(len(population), p=probabilities)
        return population[selected_idx].copy()
    
    def _crossover(self, parent1: List[int], parent2: List[int]) -> Tuple[List[int], List[int]]:
        """Cruzamento de um ponto."""
        if np.random.random() > self.crossover_rate:
            return parent1.copy(), parent2.copy()
        
        crossover_point = np.random.randint(1, len(parent1))
        
        child1 = parent1[:crossover_point] + parent2[crossover_point:]
        child2 = parent2[:crossover_point] + parent1[crossover_point:]
        
        return child1, child2
    
    def _mutate(self, chromosome: List[int]) -> List[int]:
        """Mutação bit-flip."""
        mutated = chromosome.copy()
        
        for i in range(len(mutated)):
            if np.random.random() < self.mutation_rate:
                mutated[i] = 1 - mutated[i]
        
        return mutated
    
    def run(self) -> Dict:
        print("\n" + "="*70)
        print("ALGORITMO GENÉTICO - OTIMIZAÇÃO DE TRAJETÓRIA")
        print("="*70)
        print(f"População: {self.population_size}")
        print(f"Gerações: {self.generations}")
        print(f"Taxa de Cruzamento: {self.crossover_rate*100:.0f}%")
        print(f"Taxa de Mutação: {self.mutation_rate*100:.0f}%")
        print(f"Comprimento do Cromossomo: {self.chromosome_length} bits")
        print("="*70)
        
        population = [self._create_individual() for _ in range(self.population_size)]
        
        best_individual = None
        best_fitness = float('-inf')
        
        for generation in range(self.generations):
            fitness = [self._evaluate_fitness(ind) for ind in population]
            
            max_fitness_idx = np.argmax(fitness)
            if fitness[max_fitness_idx] > best_fitness:
                best_fitness = fitness[max_fitness_idx]
                best_individual = population[max_fitness_idx].copy()
            
            avg_fitness = np.mean(fitness)
            self.best_fitness_history.append(-best_fitness)
            self.avg_fitness_history.append(-avg_fitness)
            self.best_individual_history.append(best_individual.copy())
            
            if (generation + 1) % 10 == 0 or generation == 0:
                k0, k1, vs = self._decode_individual(best_individual)
                _, path_length, max_steering = self._generate_trajectory(k0, k1, vs)
                print(f"Geração {generation+1:3d}/{self.generations} | "
                      f"Melhor Fitness: {-best_fitness:.4f} | "
                      f"|S|: {path_length:.2f} | |φ|max: {max_steering:.2f}° | "
                      f"k₀: {k0:.4f}, k₁: {k1:.4f}, Vₛ: {vs}")
            
            new_population = []
            
            new_population.append(best_individual.copy())
            
            while len(new_population) < self.population_size:
                parent1 = self._roulette_wheel_selection(population, fitness)
                parent2 = self._roulette_wheel_selection(population, fitness)
                
                child1, child2 = self._crossover(parent1, parent2)
                
                child1 = self._mutate(child1)
                child2 = self._mutate(child2)
                
                new_population.extend([child1, child2])
            
            population = new_population[:self.population_size]
        
        k0, k1, vs = self._decode_individual(best_individual)
        trajectory, path_length, max_steering = self._generate_trajectory(k0, k1, vs)
        has_collision = self._check_collision(trajectory)
        
        result = {
            'k0': k0,
            'k1': k1,
            'vs': vs,
            'path_length': path_length,
            'max_steering': max_steering,
            'objective_value': path_length**2 + max_steering**2,
            'has_collision': has_collision,
            'trajectory': trajectory,
            'fitness_history': self.best_fitness_history,
            'avg_fitness_history': self.avg_fitness_history
        }
        
        print("\n" + "="*70)
        print("RESULTADOS FINAIS DO ALGORITMO GENÉTICO")
        print("="*70)
        print(f"k₀: {k0:.6f}")
        print(f"k₁: {k1:.6f}")
        print(f"Vₛ: {vs} ({'Frente' if vs == 1 else 'Ré'})")
        print(f"Comprimento da Trajetória |S|: {path_length:.4f}")
        print(f"Ângulo Máximo de Esterçamento |φ|max: {max_steering:.4f}°")
        print(f"Valor da Função Objetivo fₒ: {result['objective_value']:.4f}")
        print(f"Colisão Detectada: {'Sim' if has_collision else 'Não'}")
        print("="*70 + "\n")
        
        return result

