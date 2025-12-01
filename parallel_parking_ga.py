"""
Sistema de Estacionamento Paralelo (Baliza) usando Algoritmo Genético
Baseado no artigo: Vieira, Argento & Revoredo (IEEE)
DUAS FASES: Aproximação (frente) + Entrada (ré)
"""

import pygame
import numpy as np
import math
from typing import List, Tuple, Dict, Optional

from genetic_algorithm import GeneticAlgorithm
from simulation import Vehicle, ParkingSpot, Obstacle

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (100, 100, 100)
RED = (220, 50, 50)
GREEN = (50, 220, 50)
BLUE = (50, 100, 220)
YELLOW = (255, 220, 0)
ORANGE = (255, 150, 0)
CYAN = (0, 200, 200)


class GeneticAlgorithmParking(GeneticAlgorithm):
            
    def __init__(self, *args, use_two_phase: bool = True, **kwargs):
        super().__init__(*args, **kwargs)
        self.use_two_phase = use_two_phase
        self._original_initial_pose = self.initial_pose
        self._original_final_pose = self.final_pose
    
    def _calculate_intermediate_point(self) -> Tuple[float, float, float]:
        xf, yf, thetaf = self._original_final_pose
        B = self.vehicle_params['width']
        A = self.vehicle_params['length']
        vaga_width = 1.3 * B
        lateral_offset = vaga_width + B/2 + 5  # Reduzido para ~40-50px
        longitudinal_offset = A * 0.5          # Aumentado para ~25px
        angle_rad = math.radians(thetaf)
        xi = xf - lateral_offset * math.sin(angle_rad) + longitudinal_offset * math.cos(angle_rad)
        yi = yf + lateral_offset * math.cos(angle_rad) + longitudinal_offset * math.sin(angle_rad)
        thetai = thetaf
        print(f"\n[DEBUG] Ponto Intermediário:")
        print(f"  Final: ({xf:.2f}, {yf:.2f}, {thetaf:.2f}°)")
        print(f"  Calculado: ({xi:.2f}, {yi:.2f}, {thetai:.2f}°)")
        print(f"  Offset lateral: {lateral_offset:.2f}")
        print(f"  Offset longitudinal: {longitudinal_offset:.2f}")
        print(f"  sin({thetaf}°) = {math.sin(angle_rad):.3f}")
        print(f"  cos({thetaf}°) = {math.cos(angle_rad):.3f}")
        return (xi, yi, thetai)
    
    def _generate_trajectory(self, k0: float, k1: float, vs: int, num_points: int = 200) -> Tuple[List[Tuple[float, float, float]], float, float]:
        """
        Implementação do artigo com parametrização temporal.
        s(t): 5ª ordem, x(s) e y(s): 3ª ordem
        """
        x0, y0, theta0 = self.initial_pose
        xf, yf, thetaf = self.final_pose
        
        dist = math.sqrt((xf - x0)**2 + (yf - y0)**2)
        tf = max(4.0, dist / 0.25)
        
        # s(t) - Polinômio 5ª ordem
        Ts = np.array([
            [0**5, 0**4, 0**3, 0**2, 0, 1],
            [tf**5, tf**4, tf**3, tf**2, tf, 1],
            [0, 0, 0, 0, 1, 0],
            [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*tf**3, 12*tf**2, 6*tf, 2, 0, 0]
        ])
        s_cond = np.array([0, 1, 0, 0, 0, 0])
        cs = np.linalg.solve(Ts, s_cond)
        
        # x(s) e y(s) - Polinômios 3ª ordem
        T3 = np.array([
            [0, 0, 0, 1],
            [1, 1, 1, 1],
            [0, 0, 1, 0],
            [3, 2, 1, 0]
        ])
        
        theta0_rad = math.radians(theta0)
        thetaf_rad = math.radians(thetaf)
        
        x_cond = np.array([x0, xf, vs*k0*math.cos(theta0_rad), vs*k1*math.cos(thetaf_rad)])
        y_cond = np.array([y0, yf, vs*k0*math.sin(theta0_rad), vs*k1*math.sin(thetaf_rad)])
        
        cx = np.linalg.solve(T3, x_cond)
        cy = np.linalg.solve(T3, y_cond)
        
        # Gera trajetória
        trajectory = []
        path_length = 0.0
        max_steering = 0.0
        prev_x, prev_y = x0, y0
        
        for i in range(num_points):
            t = (i / (num_points - 1)) * tf
            
            s = cs[0]*t**5 + cs[1]*t**4 + cs[2]*t**3 + cs[3]*t**2 + cs[4]*t + cs[5]
            s = np.clip(s, 0, 1)
            
            x = cx[0]*s**3 + cx[1]*s**2 + cx[2]*s + cx[3]
            y = cy[0]*s**3 + cy[1]*s**2 + cy[2]*s + cy[3]
            
            ds_dt = 5*cs[0]*t**4 + 4*cs[1]*t**3 + 3*cs[2]*t**2 + 2*cs[3]*t + cs[4]
            dx_ds = 3*cx[0]*s**2 + 2*cx[1]*s + cx[2]
            dy_ds = 3*cy[0]*s**2 + 2*cy[1]*s + cy[2]
            
            dx_dt = dx_ds * ds_dt
            dy_dt = dy_ds * ds_dt
            
            if abs(dx_dt) > 1e-6 or abs(dy_dt) > 1e-6:
                angle = math.degrees(math.atan2(dy_dt, dx_dt))
            else:
                angle = theta0 if i == 0 else trajectory[-1][2]
            
            trajectory.append((x, y, angle))
            
            if i > 0:
                path_length += math.sqrt((x-prev_x)**2 + (y-prev_y)**2)
            prev_x, prev_y = x, y
        
        # Esterçamento
        wheelbase = self.vehicle_params.get('wheelbase', 35.0)
        for i in range(1, len(trajectory)):
            x1, y1, theta1 = trajectory[i-1]
            x2, y2, theta2 = trajectory[i]
            ds = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            
            if ds > 0.01:
                dtheta = math.radians(theta2 - theta1)
                while dtheta > math.pi:
                    dtheta -= 2 * math.pi
                while dtheta < -math.pi:
                    dtheta += 2 * math.pi
                
                steering = abs(math.degrees(math.atan(wheelbase * abs(dtheta) / ds)))
                max_steering = max(max_steering, steering)
        
        return trajectory, path_length, max_steering
    
    def _run_single_phase(self, force_direction: Optional[int] = None) -> Dict:
        """
        Executa uma fase do AG com direção forçada.
        force_direction: 1 (frente), -1 (ré), None (livre)
        """
        # Salva método original de decodificação
        original_decode = self._decode_individual
        
        if force_direction is not None:
            # Cria versão modificada que força o Vs
            def forced_decode(chromosome):
                k0, k1, _ = original_decode(chromosome)
                return k0, k1, force_direction
            
            self._decode_individual = forced_decode
        
        # Executa AG normalmente (chama método da classe pai)
        result = GeneticAlgorithm.run(self)
        
        # Restaura método original
        self._decode_individual = original_decode
        
        return result
    
    def _combine_phases(self, phase1: Dict, phase2: Dict) -> Dict:
        """Combina resultados das duas fases"""
        return {
            'phase1': phase1,
            'phase2': phase2,
            'trajectory': phase1['trajectory'] + phase2['trajectory'],
            'path_length': phase1['path_length'] + phase2['path_length'],
            'max_steering': max(phase1['max_steering'], phase2['max_steering']),
            'k0_phase1': phase1['k0'],
            'k1_phase1': phase1['k1'],
            'k0_phase2': phase2['k0'],
            'k1_phase2': phase2['k1'],
            'intermediate_point': phase1['trajectory'][-1] if phase1['trajectory'] else None,
            'fitness_history': phase1.get('fitness_history', []) + phase2.get('fitness_history', []),
            'has_collision': phase1.get('has_collision', False) or phase2.get('has_collision', False)
        }
    
    def run(self) -> Dict:
        """
        Executa planejamento de trajetória.
        Se use_two_phase=True, executa duas fases conforme artigo.
        """
        if not self.use_two_phase:
            # Modo original (uma trajetória única)
            return self._run_single_phase(force_direction=None)
        
        # MODO DUAS FASES (conforme artigo, Seção V-A)
        print("\n" + "="*70)
        print("BALIZA COM DUAS FASES (Artigo Vieira et al.)")
        print("="*70)
        
        # Calcula ponto intermediário Ir
        intermediate = self._calculate_intermediate_point()
        print(f"\nPonto Intermediário Ir: ({intermediate[0]:.2f}, {intermediate[1]:.2f}, {intermediate[2]:.2f}°)")
        
        # FASE 1: Aproximação (FRENTE)
        print("\n" + "-"*70)
        print("FASE 1: APROXIMAÇÃO (FRENTE - Vs=1)")
        print("-"*70)
        
        self.final_pose = intermediate
        phase1_result = self._run_single_phase(force_direction=1)
        
        # FASE 2: Entrada (RÉ)
        print("\n" + "-"*70)
        print("FASE 2: ENTRADA DE RÉ (RÉ - Vs=-1)")
        print("-"*70)
        
        self.initial_pose = intermediate
        self.final_pose = self._original_final_pose
        phase2_result = self._run_single_phase(force_direction=-1)
        
        # Restaura poses originais
        self.initial_pose = self._original_initial_pose
        self.final_pose = self._original_final_pose
        
        # Combina resultados
        combined = self._combine_phases(phase1_result, phase2_result)
        
        print("\n" + "="*70)
        print("RESULTADOS COMBINADOS")
        print("="*70)
        print(f"Comprimento Total: {combined['path_length']:.2f}")
        print(f"Esterçamento Máximo: {combined['max_steering']:.2f}°")
        print(f"Pontos na Trajetória: {len(combined['trajectory'])}")
        print("="*70 + "\n")
        
        return combined


class ParallelParkingVisualizer:
    def __init__(self, width: int = 1000, height: int = 700):
        pygame.init()
        pygame.font.init()
        
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Baliza - 2 Fases (Artigo)")
        
        self.clock = pygame.time.Clock()
        self.fps = 60
        
        self.font_large = pygame.font.SysFont('Arial', 24, bold=True)
        self.font_medium = pygame.font.SysFont('Arial', 18)
        self.font_small = pygame.font.SysFont('Arial', 14)
        
        self.parking_spot = ParkingSpot(x=700, y=300, length=60, width=120)
        
        self.obstacles = [
            Obstacle(700, 170, 60, 120),
            Obstacle(700, 430, 60, 120),
            Obstacle(770, 0, 10, 700),
            Obstacle(0, 0, 10, 700),
        ]
        
        self.initial_pose = (650, 100, 90)
        
        target_x = self.parking_spot.x + self.parking_spot.length / 2
        target_y = self.parking_spot.y + self.parking_spot.width / 2
        self.final_pose = (target_x, target_y, 90)
        
        vehicle_params = {'length': 50, 'width': 25, 'wheelbase': 35}
        
        self.ga = GeneticAlgorithmParking(
            initial_pose=self.initial_pose,
            final_pose=self.final_pose,
            parking_spot=self.parking_spot,
            obstacles=self.obstacles,
            vehicle_params=vehicle_params,
            population_size=50,
            generations=50,  # 50 por fase = 100 total
            use_two_phase=True
        )
        
        print("Executando AG com Duas Fases...")
        self.result = self.ga.run()
        self.trajectory = self.result['trajectory']
        
        # Identifica transição entre fases
        if 'phase1' in self.result:
            self.phase1_end = len(self.result['phase1']['trajectory'])
        else:
            self.phase1_end = 0
        
        self.current_step = 0
        self.playing = False
        self.speed = 1
        
    def draw_parking_spot(self):
        rect = pygame.Rect(self.parking_spot.x, self.parking_spot.y,
                          self.parking_spot.length, self.parking_spot.width)
        pygame.draw.rect(self.screen, (50, 100, 50), rect, 0)
        pygame.draw.rect(self.screen, WHITE, rect, 3)
        
        for i in range(0, int(self.parking_spot.width), 20):
            pygame.draw.line(self.screen, WHITE,
                           (self.parking_spot.x, self.parking_spot.y + i),
                           (self.parking_spot.x, self.parking_spot.y + i + 10), 2)
            pygame.draw.line(self.screen, WHITE,
                           (self.parking_spot.x + self.parking_spot.length, self.parking_spot.y + i),
                           (self.parking_spot.x + self.parking_spot.length, self.parking_spot.y + i + 10), 2)
    
    def draw_obstacles(self):
        for obs in self.obstacles:
            rect = pygame.Rect(obs.x, obs.y, obs.width, obs.height)
            color = DARK_GRAY if (obs.width == 10 or obs.height == 10) else (120, 120, 120)
            pygame.draw.rect(self.screen, color, rect, 0)
            pygame.draw.rect(self.screen, BLACK, rect, 2)
    
    def draw_vehicle(self, x: float, y: float, angle: float, color=BLUE):
        vehicle = Vehicle(x, y, angle)
        corners = vehicle.get_corners()
        pygame.draw.polygon(self.screen, color, corners, 0)
        pygame.draw.polygon(self.screen, BLACK, corners, 2)
        front = vehicle.get_front_center()
        pygame.draw.circle(self.screen, YELLOW, (int(front[0]), int(front[1])), 5)
    
    def draw_trajectory(self):
        if len(self.trajectory) > 1:
            phase1_points = [(int(x), int(y)) for x, y, _ in self.trajectory[:self.phase1_end]]
            phase2_points = [(int(x), int(y)) for x, y, _ in self.trajectory[self.phase1_end:]]
            if len(phase1_points) > 1:
                pygame.draw.lines(self.screen, CYAN, False, phase1_points, 2)
            if len(phase2_points) > 1:
                pygame.draw.lines(self.screen, ORANGE, False, phase2_points, 2)
            for i, (x, y, _) in enumerate(self.trajectory):
                if i % 10 == 0:
                    color = CYAN if i < self.phase1_end else ORANGE
                    pygame.draw.circle(self.screen, color, (int(x), int(y)), 3)
        # Visualização do ponto intermediário Ir
        if 'intermediate_point' in self.result and self.result['intermediate_point']:
            xi, yi, thetai = self.result['intermediate_point']
            pygame.draw.circle(self.screen, RED, (int(xi), int(yi)), 12, 3)
            pygame.draw.circle(self.screen, YELLOW, (int(xi), int(yi)), 6)
            text = f"Ir: ({xi:.0f}, {yi:.0f})"
            surface = self.font_small.render(text, True, WHITE)
            self.screen.blit(surface, (int(xi) + 15, int(yi) - 10))
            arrow_length = 30
            end_x = xi + arrow_length * math.cos(math.radians(thetai))
            end_y = yi + arrow_length * math.sin(math.radians(thetai))
            pygame.draw.line(self.screen, YELLOW, (int(xi), int(yi)), (int(end_x), int(end_y)), 3)
            pygame.draw.circle(self.screen, YELLOW, (int(end_x), int(end_y)), 4)
    
    def draw_info(self):
        y_pos = 10
        title = self.font_large.render("BALIZA - 2 FASES (Vieira et al.)", True, WHITE)
        self.screen.blit(title, (10, y_pos))
        y_pos += 40
        
        # Determina fase atual
        is_phase2 = self.current_step >= self.phase1_end
        phase_text = "FASE 2: Entrada (RÉ)" if is_phase2 else "FASE 1: Aproximação (FRENTE)"
        phase_color = ORANGE if is_phase2 else CYAN
        
        info = [
            f"Artigo: s(t) 5a ordem | x(s), y(s) 3a ordem",
            f"Fase Atual: {phase_text}",
            f"|S|total={self.result['path_length']:.2f}, |phi|max={self.result['max_steering']:.2f} deg",
            f"Frame: {self.current_step}/{len(self.trajectory)-1}",
            "",
            "ESPACO: Play/Pause | R: Reiniciar | +/-: Vel | Q: Sair"
        ]
        
        for i, text in enumerate(info):
            if text:
                if i == 1:  # Linha da fase
                    surface = self.font_medium.render(text, True, phase_color)
                else:
                    surface = self.font_small.render(text, True, WHITE)
                self.screen.blit(surface, (10, y_pos))
            y_pos += 20
        
        status = "REPRODUZINDO" if self.playing else "PAUSADO"
        color = GREEN if self.playing else YELLOW
        surface = self.font_medium.render(status, True, color)
        self.screen.blit(surface, (10, self.height - 40))
    
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.playing = not self.playing
                elif event.key == pygame.K_r:
                    self.current_step = 0
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.speed = min(5, self.speed + 1)
                elif event.key == pygame.K_MINUS:
                    self.speed = max(1, self.speed - 1)
                elif event.key == pygame.K_q:
                    return False
        return True
    
    def run(self):
        running = True
        while running:
            running = self.handle_events()
            
            if self.playing:
                self.current_step += self.speed
                if self.current_step >= len(self.trajectory):
                    self.current_step = len(self.trajectory) - 1
                    self.playing = False
            
            self.screen.fill((40, 40, 40))
            pygame.draw.rect(self.screen, (60, 60, 60), (10, 0, 760, 700))
            
            for i in range(0, 700, 40):
                pygame.draw.line(self.screen, YELLOW, (400, i), (400, i + 20), 3)
            
            self.draw_obstacles()
            self.draw_parking_spot()
            self.draw_trajectory()
            
            if self.current_step < len(self.trajectory):
                x, y, angle = self.trajectory[self.current_step]
                vehicle = Vehicle(x, y, angle)
                corners = vehicle.get_corners()
                is_parked = self.parking_spot.is_inside(corners, margin=2)
                color = GREEN if is_parked else BLUE
                self.draw_vehicle(x, y, angle, color)
            
            x0, y0, a0 = self.initial_pose
            self.draw_vehicle(x0, y0, a0, (100, 100, 200))
            
            self.draw_info()
            pygame.display.flip()
            self.clock.tick(self.fps)
        
        pygame.quit()


if __name__ == "__main__":
    print("\n" + "="*70)
    print("BALIZA COM DUAS FASES - MÉTODO DO ARTIGO")
    print("Vieira, Argento & Revoredo (IEEE)")
    print("Fase 1: Aproximação (frente) | Fase 2: Entrada (ré)")
    print("="*70 + "\n")
    
    viz = ParallelParkingVisualizer()
    viz.run()
