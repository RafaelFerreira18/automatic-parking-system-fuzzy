import pygame
import numpy as np
import math
from typing import List, Tuple, Optional
from simulation import ParkingSimulation, Vehicle, ParkingSpot, Obstacle

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
PURPLE = (150, 50, 200)
LIGHT_BLUE = (100, 150, 255)


class ParkingVisualizer:
    def __init__(self, simulation: ParkingSimulation, width: int = 1400, height: int = 700):
        pygame.init()
        pygame.font.init()
        
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Sistema de Estacionamento Autônomo - Controle Fuzzy")
        
        self.simulation = simulation
        self.clock = pygame.time.Clock()
        self.fps = 60

        self.font_large = pygame.font.SysFont('Arial', 20, bold=True)
        self.font_medium = pygame.font.SysFont('Arial', 16)
        self.font_small = pygame.font.SysFont('Arial', 13)
        
        self.sim_area = pygame.Rect(0, 0, 800, 600)
        self.info_area = pygame.Rect(800, 0, 600, 700)

        self.paused = False
        self.show_trajectory = True
        self.show_sensors = True
        self.simulation_speed = 1.0

        self.running = True
        
    def draw_vehicle(self, vehicle: Vehicle):
        """Desenha o veículo"""
        corners = vehicle.get_corners()
        
        if vehicle.is_colliding:
            color = RED
        elif vehicle.is_parked:
            color = GREEN
        else:
            color = BLUE
        
        pygame.draw.polygon(self.screen, color, corners, 0)
        pygame.draw.polygon(self.screen, BLACK, corners, 2)
        
        front_center = vehicle.get_front_center()
        pygame.draw.circle(self.screen, YELLOW, (int(front_center[0]), int(front_center[1])), 4)
        
        theta = math.radians(vehicle.angle + vehicle.steering_angle)
        indicator_length = 15
        end_x = front_center[0] + indicator_length * math.cos(theta)
        end_y = front_center[1] + indicator_length * math.sin(theta)
        pygame.draw.line(self.screen, YELLOW, front_center, (end_x, end_y), 3)
        
    def draw_trajectory(self, vehicle: Vehicle):
        if len(vehicle.trajectory) > 1:
            for i in range(1, len(vehicle.trajectory)):
                alpha = i / len(vehicle.trajectory)
                color = (
                    int(100 + 100 * alpha),
                    int(150 * alpha),
                    int(200 + 55 * alpha)
                )
                pygame.draw.circle(self.screen, color, 
                                 (int(vehicle.trajectory[i][0]), int(vehicle.trajectory[i][1])), 2)
    
    def draw_parking_spot(self, spot: ParkingSpot):
        rect = pygame.Rect(spot.x, spot.y, spot.length, spot.width)
        
        pygame.draw.rect(self.screen, (50, 80, 50), rect, 0)
        
        pygame.draw.rect(self.screen, WHITE, rect, 3)
        
        dash_length = 10
        for i in range(0, int(spot.length), dash_length * 2):
            pygame.draw.line(self.screen, WHITE,
                           (spot.x + i, spot.y),
                           (spot.x + i + dash_length, spot.y), 2)
            pygame.draw.line(self.screen, WHITE,
                           (spot.x + i, spot.y + spot.width),
                           (spot.x + i + dash_length, spot.y + spot.width), 2)
    
    def draw_obstacles(self, obstacles: List[Obstacle]):
        for obs in obstacles:
            rect = pygame.Rect(obs.x, obs.y, obs.width, obs.height)
            pygame.draw.rect(self.screen, DARK_GRAY, rect, 0)
            pygame.draw.rect(self.screen, BLACK, rect, 2)
    
    def draw_sensors(self, vehicle: Vehicle, parking_spot: ParkingSpot):
        if not self.show_sensors:
            return
        
        front_pos = vehicle.get_front_center()
        
        target_x = parking_spot.x + parking_spot.length
        pygame.draw.line(self.screen, RED, front_pos, (target_x, front_pos[1]), 2)
        pygame.draw.circle(self.screen, RED, (int(target_x), int(front_pos[1])), 5)
        
        text = self.font_small.render(f"{vehicle.sensor_front:.0f}cm", True, RED)
        self.screen.blit(text, (front_pos[0] + 10, front_pos[1] - 20))
        
        corners = vehicle.get_corners()
        right_side_y = (corners[2][1] + corners[3][1]) / 2
        right_side_x = (corners[2][0] + corners[3][0]) / 2
        target_y = parking_spot.y + parking_spot.width
        pygame.draw.line(self.screen, GREEN, (right_side_x, right_side_y), 
                        (right_side_x, target_y), 2)
        pygame.draw.circle(self.screen, GREEN, (int(right_side_x), int(target_y)), 5)
        
        text = self.font_small.render(f"{vehicle.sensor_lateral:.0f}cm", True, GREEN)
        self.screen.blit(text, (right_side_x + 10, (right_side_y + target_y) / 2))
        
        center = (int(vehicle.x), int(vehicle.y))
        radius = 30
        start_angle = 0
        end_angle = math.radians(vehicle.angle)
        
        if abs(vehicle.angle) > 1:
            points = [center]
            for angle in np.linspace(0, end_angle, 20):
                x = center[0] + radius * math.cos(angle)
                y = center[1] + radius * math.sin(angle)
                points.append((x, y))
            points.append(center)
            pygame.draw.polygon(self.screen, (*CYAN, 100), points, 0)
            pygame.draw.arc(self.screen, CYAN, 
                          pygame.Rect(center[0]-radius, center[1]-radius, radius*2, radius*2),
                          0, end_angle, 3)
        
        text = self.font_small.render(f"{vehicle.angle:.1f}°", True, CYAN)
        self.screen.blit(text, (center[0] + radius + 5, center[1] - 10))
    
    def draw_info_panel(self, state: dict):
        """Desenha o painel de informações lateral"""
        x_start = self.info_area.x + 10
        y_pos = 10
        
        title = self.font_large.render("CONTROLE FUZZY", True, WHITE)
        self.screen.blit(title, (x_start, y_pos))
        y_pos += 35
        
        pygame.draw.line(self.screen, GRAY, (x_start, y_pos), (x_start + 580, y_pos), 2)
        y_pos += 15
        
        if state['is_parked']:
            status_text = "STATUS: ESTACIONADO ✓"
            status_color = GREEN
        elif state['is_colliding']:
            status_text = "STATUS: COLISÃO ✗"
            status_color = RED
        else:
            status_text = "STATUS: ESTACIONANDO..."
            status_color = YELLOW
        
        status = self.font_medium.render(status_text, True, status_color)
        self.screen.blit(status, (x_start, y_pos))
        y_pos += 30
        
        time_text = self.font_medium.render(f"Tempo: {state['time_elapsed']:.1f}s", True, WHITE)
        self.screen.blit(time_text, (x_start, y_pos))
        y_pos += 25
        
        updates_text = self.font_small.render(f"Atualizações: {state['control_updates']}", True, GRAY)
        self.screen.blit(updates_text, (x_start, y_pos))
        y_pos += 30
        
        pygame.draw.line(self.screen, GRAY, (x_start, y_pos), (x_start + 580, y_pos), 1)
        y_pos += 10
        
        sensor_title = self.font_medium.render("ENTRADAS (Sensores):", True, LIGHT_BLUE)
        self.screen.blit(sensor_title, (x_start, y_pos))
        y_pos += 25
        
        df = state['fuzzy_inputs']['distancia_frontal']
        df_text = self.font_small.render(f"Distância Frontal: {df:.1f} cm", True, RED)
        self.screen.blit(df_text, (x_start + 10, y_pos))
        y_pos += 20
        self.draw_bar(x_start + 10, y_pos, 300, 15, df, 0, 200, RED)
        y_pos += 25
        
        dl = state['fuzzy_inputs']['distancia_lateral']
        dl_text = self.font_small.render(f"Distância Lateral: {dl:.1f} cm", True, GREEN)
        self.screen.blit(dl_text, (x_start + 10, y_pos))
        y_pos += 20
        self.draw_bar(x_start + 10, y_pos, 300, 15, dl, 0, 100, GREEN)
        y_pos += 25
        
        ang = state['fuzzy_inputs']['angulo_veiculo']
        ang_text = self.font_small.render(f"Ângulo do Veículo: {ang:.1f}°", True, CYAN)
        self.screen.blit(ang_text, (x_start + 10, y_pos))
        y_pos += 20
        self.draw_bar(x_start + 10, y_pos, 300, 15, ang, -90, 90, CYAN)
        y_pos += 30
        
        pygame.draw.line(self.screen, GRAY, (x_start, y_pos), (x_start + 580, y_pos), 1)
        y_pos += 10
        
        fuzz_title = self.font_medium.render("FUZZIFICAÇÃO:", True, ORANGE)
        self.screen.blit(fuzz_title, (x_start, y_pos))
        y_pos += 25
        
        y_pos = self.draw_membership_values(x_start + 10, y_pos, state)
        y_pos += 20
        
        pygame.draw.line(self.screen, GRAY, (x_start, y_pos), (x_start + 580, y_pos), 1)
        y_pos += 10
        
        output_title = self.font_medium.render("SAÍDAS (Controle):", True, LIGHT_BLUE)
        self.screen.blit(output_title, (x_start, y_pos))
        y_pos += 25
        
        ang_dir = state['fuzzy_outputs']['angulo_direcao']
        ang_dir_text = self.font_small.render(f"Ângulo de Direção: {ang_dir:.1f}°", True, PURPLE)
        self.screen.blit(ang_dir_text, (x_start + 10, y_pos))
        y_pos += 20
        self.draw_bar(x_start + 10, y_pos, 300, 15, ang_dir, -40, 40, PURPLE)
        y_pos += 25
        
        vel = state['fuzzy_outputs']['velocidade']
        vel_text = self.font_small.render(f"Velocidade: {vel:.1f} cm/s", True, YELLOW)
        self.screen.blit(vel_text, (x_start + 10, y_pos))
        y_pos += 20
        self.draw_bar(x_start + 10, y_pos, 300, 15, vel, 0, 20, YELLOW)
        y_pos += 30
        
        pygame.draw.line(self.screen, GRAY, (x_start, y_pos), (x_start + 580, y_pos), 1)
        y_pos += 10
        
        rules_title = self.font_medium.render("REGRAS ATIVAS:", True, GREEN)
        self.screen.blit(rules_title, (x_start, y_pos))
        y_pos += 25
        
        active_rules = self.simulation.fuzzy_system.get_active_rules_info()[:5]
        if active_rules:
            for i, (rule_str, activation) in enumerate(active_rules):
                if len(rule_str) > 65:
                    rule_str = rule_str[:62] + "..."
                
                rule_text = self.font_small.render(f"[{activation:.2f}] {rule_str}", True, WHITE)
                self.screen.blit(rule_text, (x_start + 10, y_pos))
                y_pos += 18
        else:
            no_rules = self.font_small.render("Nenhuma regra ativa", True, GRAY)
            self.screen.blit(no_rules, (x_start + 10, y_pos))
        
        y_pos += 30
        
        pygame.draw.line(self.screen, GRAY, (x_start, y_pos), (x_start + 580, y_pos), 1)
        y_pos += 10
        
        controls_title = self.font_small.render("CONTROLES:", True, GRAY)
        self.screen.blit(controls_title, (x_start, y_pos))
        y_pos += 20
        
        controls = [
            "ESPAÇO: Pausar/Continuar",
            "R: Reiniciar",
            "T: Mostrar/Ocultar Trajetória",
            "S: Mostrar/Ocultar Sensores",
            "Q: Sair"
        ]
        
        for ctrl in controls:
            ctrl_text = self.font_small.render(ctrl, True, GRAY)
            self.screen.blit(ctrl_text, (x_start + 10, y_pos))
            y_pos += 18
    
    def draw_bar(self, x: int, y: int, width: int, height: int, 
                 value: float, min_val: float, max_val: float, color: Tuple[int, int, int]):
        pygame.draw.rect(self.screen, DARK_GRAY, (x, y, width, height))

        normalized = (value - min_val) / (max_val - min_val)
        normalized = max(0, min(1, normalized))
        
        bar_width = int(width * normalized)
        if bar_width > 0:
            pygame.draw.rect(self.screen, color, (x, y, bar_width, height))
        
        pygame.draw.rect(self.screen, WHITE, (x, y, width, height), 1)
        
        if min_val < 0 < max_val:
            center_x = x + int(width * abs(min_val) / (max_val - min_val))
            pygame.draw.line(self.screen, WHITE, (center_x, y), (center_x, y + height), 2)
    
    def draw_membership_values(self, x: int, y: int, state: dict) -> int:
        memberships = self.simulation.fuzzy_system.last_input_memberships
        
        if not memberships:
            return y
        
        for var_name, terms in memberships.items():
            var_text = self.font_small.render(f"{var_name}:", True, WHITE)
            self.screen.blit(var_text, (x, y))
            y += 18
            
            sorted_terms = sorted(terms.items(), key=lambda item: item[1], reverse=True)[:3]
            for term_name, membership in sorted_terms:
                if membership > 0.01:
                    term_text = self.font_small.render(
                        f"  • {term_name}: {membership:.2f}",
                        True, (255, 255 * (1 - membership), 0)
                    )
                    self.screen.blit(term_text, (x, y))
                    y += 16
            
            y += 8
        
        return y
    
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                
                elif event.key == pygame.K_r:
                    self.simulation.reset_vehicle()
                
                elif event.key == pygame.K_t:
                    self.show_trajectory = not self.show_trajectory
                
                elif event.key == pygame.K_s:
                    self.show_sensors = not self.show_sensors
                
                elif event.key == pygame.K_q:
                    self.running = False
    
    def run(self):
        dt = 1.0 / self.fps
        
        while self.running:
            self.handle_events()

            if not self.paused:
                continue_sim = self.simulation.update(dt * self.simulation_speed)
                if not continue_sim:
                    self.paused = True
            
            self.screen.fill(BLACK)
            
            pygame.draw.rect(self.screen, (40, 60, 40), self.sim_area)
            
            state = self.simulation.get_state()
            
            self.draw_parking_spot(state['parking_spot'])
            self.draw_obstacles(state['obstacles'])
            
            if self.show_trajectory:
                self.draw_trajectory(state['vehicle'])
            
            self.draw_vehicle(state['vehicle'])
            
            if self.show_sensors:
                self.draw_sensors(state['vehicle'], state['parking_spot'])
            
            pygame.draw.rect(self.screen, (20, 20, 40), self.info_area)
            self.draw_info_panel(state)
            
            pygame.draw.line(self.screen, WHITE, (800, 0), (800, 700), 2)
            
            if self.paused:
                pause_text = self.font_large.render("PAUSADO", True, YELLOW)
                pause_rect = pause_text.get_rect(center=(400, 30))
                pygame.draw.rect(self.screen, BLACK, pause_rect.inflate(20, 10))
                self.screen.blit(pause_text, pause_rect)
            
            pygame.display.flip()
            self.clock.tick(self.fps)
        
        pygame.quit()


if __name__ == "__main__":
    from fuzzy_system import create_parking_fuzzy_system
    
    fis = create_parking_fuzzy_system()
    sim = ParkingSimulation(fis)
    
    viz = ParkingVisualizer(sim)
    
    print("\n" + "="*60)
    print("SISTEMA DE ESTACIONAMENTO AUTÔNOMO")
    print("="*60)
    print("\nControles:")
    print("  ESPAÇO: Pausar/Continuar")
    print("  R: Reiniciar")
    print("  T: Mostrar/Ocultar Trajetória")
    print("  S: Mostrar/Ocultar Sensores")
    print("  Q: Sair")
    print("\nIniciando visualização...")
    print("="*60 + "\n")
    
    viz.run()
