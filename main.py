from fuzzy_centered import create_centered_parking_system
from simulation import ParkingSimulation
from visualization import ParkingVisualizer
from fuzzy_centered import FuzzyInferenceSystem


def main():
    
    print("\n" + "="*70)
    print(" "*15 + "SISTEMA DE ESTACIONAMENTO AUTÔNOMO")
    print(" "*20 + "Controle Fuzzy em Malha Fechada")
    print("="*70)
    
    print("\n[1/3] Criando Sistema de Inferência Fuzzy...")
    fis = create_centered_parking_system()
    
    print("[2/3] Inicializando Simulação Física...")
    sim = ParkingSimulation(fis)
    print(f"  ✓ Veículo posicionado em: ({sim.vehicle.x:.1f}, {sim.vehicle.y:.1f})")
    print(f"  ✓ Vaga em: ({sim.parking_spot.x:.1f}, {sim.parking_spot.y:.1f})")
    print(f"  ✓ Obstáculos: {len(sim.obstacles)}")
    
    print("[3/3] Preparando Interface Gráfica...")
    viz = ParkingVisualizer(sim, width=1400, height=700)
    print("  ✓ Resolução: 1400x700 pixels")
    print("  ✓ Taxa de atualização: 60 FPS")
    
    print("\n" + "="*70)
    print("CONTROLES:")
    print("="*70)
    print("  ESPAÇO ........ Pausar/Continuar simulação")
    print("  R ............. Reiniciar com nova posição")
    print("  T ............. Mostrar/Ocultar trajetória")
    print("  S ............. Mostrar/Ocultar sensores")
    print("  Q ............. Sair do programa")
    print("="*70)
    
    print("\n[INFO] Iniciando visualização em tempo real...")
    print("[INFO] Pressione ESPAÇO para pausar a qualquer momento\n")
    
    try:
        viz.run()
    except KeyboardInterrupt:
        print("\n\n[INFO] Programa interrompido pelo usuário")
    except Exception as e:
        print(f"\n\n[ERRO] Ocorreu um erro: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[INFO] Encerrando programa...")
        print("="*70)
        print("Obrigado por usar o Sistema de Estacionamento Autônomo!")
        print("="*70 + "\n")


if __name__ == "__main__":
    main()
