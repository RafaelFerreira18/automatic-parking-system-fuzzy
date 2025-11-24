
# Sistema de Estacionamento Autônomo Híbrido (Fuzzy + Algoritmo Genético)

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![PyGame](https://img.shields.io/badge/PyGame-2.5+-green.svg)
![NumPy](https://img.shields.io/badge/NumPy-1.26+-orange.svg)

**Sistema Inteligente de Estacionamento Paralelo Utilizando Lógica Fuzzy e Algoritmo Genético**

[Características](#-características) • [Instalação](#-instalação) • [Como Usar](#-como-usar) • [Arquitetura](#%EF%B8%8F-arquitetura)

</div>

---

## 📋 Descrição

Este projeto implementa um **Sistema de Estacionamento Autônomo Híbrido**, integrando:
- **Algoritmo Genético (AG):** Planeja a trajetória ótima do veículo até a vaga, considerando obstáculos e restrições físicas.
- **Lógica Fuzzy:** Realiza o controle em tempo real, ajustando direção e velocidade para seguir a trajetória e garantir centralização e segurança.

### 🎯 Características Principais

- ✅ **Planejamento de Trajetória Otimizada:** AG encontra o melhor caminho para estacionar, evitando colisões.
- ✅ **Controle Fuzzy em Tempo Real:** Corrige desvios, suaviza movimentos e garante parada centralizada.
- ✅ **Sistema Híbrido:** AG para navegação global, Fuzzy para ajustes locais e segurança.
- ✅ **Visualização Interativa:** Interface PyGame com painel de debug e exibição da trajetória planejada.

---

## ✨ Características Técnicas

### � Algoritmo Genético (AG)

- **Planejamento Offline:** AG gera uma sequência de poses (x, y, ângulo) que o veículo deve seguir.
- **Otimização de Parâmetros:** Minimiza comprimento da trajetória, ângulo máximo de esterçamento e risco de colisão.
- **Adaptação Dinâmica:** Permite reotimizar a trajetória se o veículo for reposicionado.

### 🧠 Sistema Fuzzy

- **Entradas:** Sensores de distância frontal, lateral, ângulo e profundidade na vaga.
- **Saídas:** Ângulo de direção e velocidade.
- **Função:** Ajusta comandos para seguir a trajetória do AG e evitar colisões.

### 🚗 Simulação

- **Modelo Cinemático:** Ackermann (bicicleta).
- **Obstáculos:** Considerados no planejamento e na simulação.
- **Precisão:** Centralização horizontal e vertical garantida.

---

## 🚀 Instalação

### Pré-requisitos
- Python 3.8 ou superior
- pip

### Passos

```bash
git clone <seu-repositorio>
cd trabPoly
pip install -r requirements.txt
```

---

## 🎮 Como Usar

### Executar Simulação Híbrida

```bash
python main.py
```

### Controles

- **ESPAÇO** - Pausar/Continuar
- **R** - Reiniciar simulação
- **ESC** - Sair

### Comportamento Esperado

1. O AG planeja a trajetória ótima até a vaga.
2. O veículo segue essa trajetória, ajustando direção e velocidade com o sistema Fuzzy.
3. O painel mostra a trajetória planejada, sensores e regras fuzzy ativadas.
4. O veículo para centralizado e sem colisões.

---

## 🏗️ Arquitetura

```
trabPoly/
│
├── main.py                 # Script principal
├── hybrid_system.py        # Sistema híbrido AG + Fuzzy
├── genetic_algorithm.py    # Algoritmo Genético para planejamento
├── fuzzy_centered.py       # Sistema fuzzy
├── simulation.py           # Física e sensores
├── visualization.py        # Interface PyGame
├── requirements.txt        # Dependências
└── README.md               # Documentação
```

### Fluxo do Sistema

1. **AG planeja a trajetória** → 2. **Fuzzy controla o veículo** → 3. **Simulação executa física e sensores** → 4. **Visualização exibe tudo em tempo real**

---

## 📊 Resultados

- ✅ **Trajetória otimizada:** Menor caminho, menor ângulo de esterçamento, sem colisões.
- ✅ **Centralização perfeita:** Veículo para no centro da vaga.
- ✅ **Robustez:** Sistema reotimiza se houver mudanças.
- ✅ **Explicabilidade:** Painel mostra regras fuzzy e trajetória AG.

---

## 🔬 Vantagens do Sistema Híbrido

- **Planejamento global + controle local:** AG resolve o “onde ir”, Fuzzy resolve o “como ir”.
- **Evita colisões e erros:** AG considera obstáculos, Fuzzy ajusta em tempo real.
- **Flexível e adaptável:** Pode ser usado em diferentes cenários e vagas.

---

## 📚 Referências

1. Mamdani, E. H. (1974). "Application of fuzzy algorithms for control of simple dynamic plant"
2. Zadeh, L. A. (1965). "Fuzzy sets" - Information and Control
3. Wang, L.-X. (1997). "A Course in Fuzzy Systems and Control"
4. Renan, E. T. (2021). "Trajectory Planning For Car-like Robots Through Curve Parametrization And Genetic Algorithm Optimization With Applications To Autonomous Parking"

---

## 📄 Licença

Projeto desenvolvido para fins acadêmicos.

---

**Sistema de Estacionamento Autônomo Híbrido (Fuzzy + AG)**

Desenvolvido com ❤️ usando IA Evolutiva e Lógica Fuzzy 🌫️

