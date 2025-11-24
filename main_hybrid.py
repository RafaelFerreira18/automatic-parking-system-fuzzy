"""
Sistema Híbrido de IA para Estacionamento Paralelo Autônomo
Integra Algoritmo Genético (AG) + Lógica Fuzzy

Fase 1: AG otimiza trajetória offline
Fase 2: Fuzzy rastreia trajetória otimizada em tempo real
"""

from fuzzy_centered import create_centered_parking_system
from simulation import ParkingSimulation, ParkingSpot, Obstacle
from visualization import ParkingVisualizer
from hybrid_system import HybridParkingSystem


def main():
    print("\n" + "="*70)
    print(" "*10 + "SISTEMA HÍBRIDO DE IA PARA ESTACIONAMENTO AUTÔNOMO")
    print(" "*15 + "Algoritmo Genético + Lógica Fuzzy")
    print("="*70)
    
    # Configuração do ambiente
    parking_spot = ParkingSpot(x=600, y=250, length=150, width=80)
    obstacles = [
        Obstacle(600, 170, 150, 10),
        Obstacle(600, 410, 150, 10),
        Obstacle(750, 170, 10, 250),
    ]
    
    # Pose inicial e final para estacionamento paralelo
    initial_pose = (250.0, 350.0, -15.0)  # (x, y, angle)
    final_pose = (675.0, 290.0, 0.0)  # Centro da vaga, alinhado
    
    # Parâmetros do veículo (conforme Tabela II do artigo)
    vehicle_params = {
        'length': 50.0,
        'width': 25.0,
        'wheelbase': 35.0,  # 70% do comprimento
        'max_steering': 40.0
    }
    
    print("\n[CONFIGURAÇÃO]")
    print(f"  Pose Inicial: ({initial_pose[0]}, {initial_pose[1]}), ângulo: {initial_pose[2]}°")
    print(f"  Pose Final: ({final_pose[0]}, {final_pose[1]}), ângulo: {final_pose[2]}°")
    print(f"  Vaga: ({parking_spot.x}, {parking_spot.y}), {parking_spot.length}x{parking_spot.width}")
    print(f"  Obstáculos: {len(obstacles)}")
    
    # Cria sistema Fuzzy
    print("\n[1/4] Criando Sistema de Inferência Fuzzy...")
    fuzzy_system = create_centered_parking_system()
    print("  ✓ Sistema Fuzzy criado")
    
    # Cria sistema híbrido
    print("\n[2/4] Inicializando Sistema Híbrido (AG + Fuzzy)...")
    hybrid_system = HybridParkingSystem(
        fuzzy_system=fuzzy_system,
        initial_pose=initial_pose,
        final_pose=final_pose,
        parking_spot=parking_spot,
        obstacles=obstacles,
        vehicle_params=vehicle_params
    )
    print("  ✓ Sistema Híbrido inicializado")
    
    # Executa AG para otimizar trajetória
    print("\n[3/4] Executando Algoritmo Genético (Otimização Offline)...")
    print("  Este processo pode levar alguns minutos...")
    ga_result = hybrid_system.optimize_trajectory(
        population_size=50,
        generations=100,
        verbose=True
    )
    
    # Exibe resultados do AG
    print("\n" + "="*70)
    print("RESULTADOS DO ALGORITMO GENÉTICO")
    print("="*70)
    print(f"Parâmetros Otimizados:")
    print(f"  k₀ = {ga_result['k0']:.6f}")
    print(f"  k₁ = {ga_result['k1']:.6f}")
    print(f"  Vₛ = {ga_result['vs']} ({'Frente' if ga_result['vs'] == 1 else 'Ré'})")
    print(f"\nDesempenho:")
    print(f"  Comprimento da Trajetória |S| = {ga_result['path_length']:.4f}")
    print(f"  Ângulo Máximo de Esterçamento |φ|max = {ga_result['max_steering']:.4f}°")
    print(f"  Valor da Função Objetivo fₒ = {ga_result['objective_value']:.4f}")
    print(f"  Colisão Detectada: {'Sim' if ga_result['has_collision'] else 'Não'}")
    
    # Verifica trajetória
    if ga_result['trajectory']:
        traj_start = ga_result['trajectory'][0]
        traj_end = ga_result['trajectory'][-1]
        print(f"\nVerificação da Trajetória:")
        print(f"  Início: ({traj_start[0]:.1f}, {traj_start[1]:.1f}), ângulo: {traj_start[2]:.1f}°")
        print(f"  Fim: ({traj_end[0]:.1f}, {traj_end[1]:.1f}), ângulo: {traj_end[2]:.1f}°")
        print(f"  Pontos na trajetória: {len(ga_result['trajectory'])}")
        print(f"  Pose inicial desejada: ({initial_pose[0]}, {initial_pose[1]}), {initial_pose[2]}°")
        print(f"  Pose final desejada: ({final_pose[0]}, {final_pose[1]}), {final_pose[2]}°")
    
    print("="*70)
    
    # Inicializa simulação com sistema híbrido
    print("\n[4/4] Inicializando Simulação com Sistema Híbrido...")
    sim = ParkingSimulation(fuzzy_system, hybrid_system=hybrid_system)
    
    # Ajusta posição inicial do veículo
    sim.vehicle.x = initial_pose[0]
    sim.vehicle.y = initial_pose[1]
    sim.vehicle.angle = initial_pose[2]
    sim.vehicle.trajectory = [(initial_pose[0], initial_pose[1])]
    
    print(f"  ✓ Veículo posicionado em: ({sim.vehicle.x:.1f}, {sim.vehicle.y:.1f})")
    print(f"  ✓ Modo: Híbrido (AG + Fuzzy)")
    
    # Inicializa visualização
    print("\n[5/5] Preparando Interface Gráfica...")
    viz = ParkingVisualizer(sim, width=1400, height=700)
    print("  ✓ Resolução: 1400x700 pixels")
    print("  ✓ Taxa de atualização: 60 FPS")
    
    print("\n" + "="*70)
    print("LEGENDA DA VISUALIZAÇÃO:")
    print("="*70)
    print("  Trajetória VERDE: Trajetória otimizada pelo AG (referência)")
    print("  Trajetória AZUL: Trajetória real do veículo (rastreada pelo Fuzzy)")
    print("  Veículo AZUL: Em movimento")
    print("  Veículo VERDE: Estacionado")
    print("  Veículo VERMELHO: Colisão")
    print("="*70)
    print("\nCONTROLES:")
    print("="*70)
    print("  ESPAÇO ........ Pausar/Continuar simulação")
    print("  R ............. Reiniciar")
    print("  T ............. Mostrar/Ocultar trajetórias")
    print("  S ............. Mostrar/Ocultar sensores")
    print("  Q ............. Sair do programa")
    print("="*70)
    
    print("\n[INFO] Iniciando visualização em tempo real...")
    print("[INFO] O Fuzzy está rastreando a trajetória otimizada pelo AG\n")
    
    try:
        viz.run()
    except KeyboardInterrupt:
        print("\n\n[INFO] Programa interrompido pelo usuário")
    except Exception as e:
        print(f"\n\n[ERRO] Ocorreu um erro: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n" + "="*70)
        print("RESUMO FINAL")
        print("="*70)
        ga_params = hybrid_system.get_ga_parameters()
        if ga_params:
            print(f"Parâmetros AG: k₀={ga_params['k0']:.4f}, k₁={ga_params['k1']:.4f}, Vₛ={ga_params['vs']}")
            print(f"Desempenho: |S|={ga_params['path_length']:.2f}, |φ|max={ga_params['max_steering']:.2f}°")
        print("="*70)
        print("Obrigado por usar o Sistema Híbrido de Estacionamento Autônomo!")
        print("="*70 + "\n")


if __name__ == "__main__":
    main()

