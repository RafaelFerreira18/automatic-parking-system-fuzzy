#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Sistema Fuzzy APRIMORADO com controle de parada centralizada
"""

import numpy as np
from typing import Dict, List, Tuple, Optional

class FuzzyVariable:
    def __init__(self, name: str, universe: Tuple[float, float], resolution: int = 100):
        self.name = name
        self.min_val, self.max_val = universe
        self.resolution = resolution
        self.universe = np.linspace(self.min_val, self.max_val, resolution)
        self.terms = {}
    
    def add_term(self, term_name: str, mf_type: str, params: List[float]):
        if mf_type == "trimf":
            mf = self._triangular_mf(params)
        elif mf_type == "trapmf":
            mf = self._trapezoidal_mf(params)
        elif mf_type == "gaussmf":
            mf = self._gaussian_mf(params)
        else:
            raise ValueError(f"Tipo de função de pertinência não suportado: {mf_type}")
        
        self.terms[term_name] = mf
    
    def _triangular_mf(self, params: List[float]) -> np.ndarray:
        a, b, c = params
        mf = np.zeros_like(self.universe)
        
        # Slope up
        idx1 = np.where((self.universe >= a) & (self.universe <= b))
        if b != a:
            mf[idx1] = (self.universe[idx1] - a) / (b - a)
        
        # Slope down
        idx2 = np.where((self.universe >= b) & (self.universe <= c))
        if c != b:
            mf[idx2] = (c - self.universe[idx2]) / (c - b)
        
        return np.clip(mf, 0, 1)
    
    def _trapezoidal_mf(self, params: List[float]) -> np.ndarray:
        a, b, c, d = params
        mf = np.zeros_like(self.universe)
        
        # Slope up
        idx1 = np.where((self.universe >= a) & (self.universe <= b))
        if b != a:
            mf[idx1] = (self.universe[idx1] - a) / (b - a)
        
        # Flat top
        idx2 = np.where((self.universe >= b) & (self.universe <= c))
        mf[idx2] = 1.0
        
        # Slope down
        idx3 = np.where((self.universe >= c) & (self.universe <= d))
        if d != c:
            mf[idx3] = (d - self.universe[idx3]) / (d - c)
        
        return np.clip(mf, 0, 1)
    
    def fuzzify(self, crisp_value: float) -> Dict[str, float]:
        crisp_value = np.clip(crisp_value, self.min_val, self.max_val)
        
        memberships = {}
        idx = np.abs(self.universe - crisp_value).argmin()
        
        for term_name, mf in self.terms.items():
            memberships[term_name] = mf[idx]
        
        return memberships

class FuzzyRule:
    def __init__(self, antecedents: Dict[str, str], consequents: Dict[str, str]):
        self.antecedents = antecedents
        self.consequents = consequents

class FuzzyInferenceSystem:
    def __init__(self, name: str):
        self.name = name
        self.inputs = {}
        self.outputs = {}
        self.rules = []
        self.last_input_memberships = {}  # Para compatibilidade com visualização
    
    def add_input(self, input_var: FuzzyVariable):
        self.inputs[input_var.name] = input_var
    
    def add_output(self, output_var: FuzzyVariable):
        self.outputs[output_var.name] = output_var
    
    def add_rule(self, rule: FuzzyRule):
        self.rules.append(rule)
    
    def infer(self, input_values: Dict[str, float]) -> Dict[str, float]:
        # 1. FUZZIFICAÇÃO
        fuzzified_inputs = {}
        for input_name, input_value in input_values.items():
            input_var = self.inputs[input_name]
            fuzzified_inputs[input_name] = input_var.fuzzify(input_value)
        
        # Armazenar para compatibilidade com visualização
        self.last_input_memberships = fuzzified_inputs
        
        # 2. INFERÊNCIA - Cálculo das ativações das regras
        aggregated = {}
        for output_name in self.outputs:
            aggregated[output_name] = np.zeros_like(self.outputs[output_name].universe)
        
        for rule in self.rules:
            # Força da regra (mínimo das antecedentes)
            activation = 1.0
            for antecedent_var, antecedent_term in rule.antecedents.items():
                if antecedent_var in fuzzified_inputs and antecedent_term in fuzzified_inputs[antecedent_var]:
                    membership = fuzzified_inputs[antecedent_var][antecedent_term]
                    activation = min(activation, membership)
            
            # Aplicar às consequentes
            for consequent_var, consequent_term in rule.consequents.items():
                if consequent_var in self.outputs:
                    output_var = self.outputs[consequent_var]
                    if consequent_term in output_var.terms:
                        consequent_mf = output_var.terms[consequent_term]
                        implied_mf = np.minimum(activation, consequent_mf)
                        aggregated[consequent_var] = np.maximum(aggregated[consequent_var], implied_mf)
        
        # 3. DEFUZZIFICAÇÃO (Centroid)
        output_values = {}
        for output_name, aggregated_mf in aggregated.items():
            output_var = self.outputs[output_name]
            
            numerator = np.sum(output_var.universe * aggregated_mf)
            denominator = np.sum(aggregated_mf)
            
            if denominator != 0:
                output_values[output_name] = numerator / denominator
            else:
                output_values[output_name] = (output_var.min_val + output_var.max_val) / 2
        
        return output_values
    
    def get_active_rules_info(self):
        """Retorna informações sobre regras ativas para visualização"""
        if not hasattr(self, 'last_input_memberships'):
            return []
        
        rule_info = []
        for i, rule in enumerate(self.rules):
            # Calcula força da regra (mínimo das antecedentes)
            activation = 1.0
            for antecedent_var, antecedent_term in rule.antecedents.items():
                if antecedent_var in self.last_input_memberships and antecedent_term in self.last_input_memberships[antecedent_var]:
                    membership = self.last_input_memberships[antecedent_var][antecedent_term]
                    activation = min(activation, membership)
                else:
                    activation = 0.0  # Se alguma entrada não existe, regra não ativa
                    break
            
            if activation > 0:  # Apenas regras com ativação > 0
                # Constrói string da regra
                antecedent_str = " E ".join([f"{var}={term}" for var, term in rule.antecedents.items()])
                consequent_str = " E ".join([f"{var}={term}" for var, term in rule.consequents.items()])
                rule_str = f"SE {antecedent_str} ENTÃO {consequent_str}"
                rule_info.append((rule_str, activation))
        
        # Ordena por força de ativação (decrescente)
        rule_info.sort(key=lambda x: x[1], reverse=True)
        return rule_info

def create_centered_parking_system():
    """Sistema fuzzy com controle de parada centralizada"""
    
    fis = FuzzyInferenceSystem("Sistema Estacionamento com Parada Centralizada")
    
    # === ENTRADAS ===
    
    # 1. Distância frontal (até obstáculo/parede)
    dist_frontal = FuzzyVariable("distancia_frontal", (0, 400), resolution=400)
    dist_frontal.add_term("muito_perto", "trapmf", [0, 0, 5, 10])      
    dist_frontal.add_term("perto", "trimf", [8, 25, 50])               
    dist_frontal.add_term("medio", "trimf", [40, 100, 180])            
    dist_frontal.add_term("longe", "trimf", [150, 250, 350])           
    dist_frontal.add_term("muito_longe", "trapmf", [320, 380, 400, 400]) 
    fis.add_input(dist_frontal)
    
    # 2. Desalinhamento lateral (distância do centro da vaga) - CORRIGIDO
    # Valores: negativo = veículo acima do centro, 0 = centrado, positivo = abaixo do centro
    dist_lateral = FuzzyVariable("distancia_lateral", (-80, 80), resolution=160)
    dist_lateral.add_term("muito_acima", "trapmf", [-80, -80, -40, -20])  # Muito acima do centro
    dist_lateral.add_term("acima", "trimf", [-30, -15, -5])               # Acima do centro
    dist_lateral.add_term("centrado", "trimf", [-8, 0, 8])                # CENTRADO = 0 ± 8px
    dist_lateral.add_term("abaixo", "trimf", [5, 15, 30])                 # Abaixo do centro
    dist_lateral.add_term("muito_abaixo", "trapmf", [20, 40, 80, 80])     # Muito abaixo do centro
    fis.add_input(dist_lateral)
    
    # 3. Ângulo do veículo
    angulo = FuzzyVariable("angulo_veiculo", (-90, 90), resolution=180)
    angulo.add_term("forte_esquerda", "trapmf", [-90, -90, -60, -30])
    angulo.add_term("esquerda", "trimf", [-50, -25, -5])
    angulo.add_term("alinhado", "trimf", [-15, 0, 15])                
    angulo.add_term("direita", "trimf", [5, 25, 50])
    angulo.add_term("forte_direita", "trapmf", [30, 60, 90, 90])
    fis.add_input(angulo)
    
    # 4. NOVA ENTRADA: Profundidade na vaga (0-150px, 0=entrada, 65-75=centro, 150=final)
    profundidade = FuzzyVariable("profundidade_vaga", (0, 150), resolution=150)
    profundidade.add_term("entrada", "trapmf", [0, 0, 10, 30])         # Acabou de entrar (0-30)
    profundidade.add_term("inicio", "trimf", [20, 40, 58])             # Início da vaga (20-58)
    profundidade.add_term("centro", "trimf", [55, 70, 85])             # Centro ideal ⭐ (55-85, pico em 70)
    profundidade.add_term("fundo", "trimf", [80, 105, 130])            # Próximo do final (80-130)
    profundidade.add_term("final", "trapmf", [125, 140, 150, 150])     # Final da vaga (125-150)
    fis.add_input(profundidade)
    
    # === SAÍDAS ===
    
    # Ângulo de direção
    ang_direcao = FuzzyVariable("angulo_direcao", (-40, 40), resolution=80)
    ang_direcao.add_term("forte_esquerda", "trapmf", [-40, -40, -30, -20])
    ang_direcao.add_term("esquerda", "trimf", [-25, -15, -5])
    ang_direcao.add_term("reto", "trimf", [-5, 0, 5])
    ang_direcao.add_term("direita", "trimf", [5, 15, 25])
    ang_direcao.add_term("forte_direita", "trapmf", [20, 30, 40, 40])
    fis.add_output(ang_direcao)
    
    # Velocidade - NOVOS TERMOS PARA PARADA GRADUAL
    velocidade = FuzzyVariable("velocidade", (0, 100), resolution=100)
    velocidade.add_term("parado", "trapmf", [0, 0, 1, 3])              # Completamente parado
    velocidade.add_term("muito_lento", "trimf", [2, 5, 10])            # Quase parando
    velocidade.add_term("lento", "trimf", [8, 15, 25])                 # Entrada controlada
    velocidade.add_term("medio", "trimf", [20, 35, 55])                # Navegação normal  
    velocidade.add_term("rapido", "trapmf", [50, 70, 100, 100])        # Aproximação rápida
    fis.add_output(velocidade)
    
    # === REGRAS COM CONTROLE DE PARADA CENTRALIZADA ===
    
    # ⭐⭐⭐ REGRA #1 - PARADA NO CENTRO (MÁXIMA PRIORIDADE) ⭐⭐⭐
    # Esta é a regra MAIS IMPORTANTE - para completamente no centro
    fis.add_rule(FuzzyRule(
        {"profundidade_vaga": "centro"},
        {"velocidade": "parado", "angulo_direcao": "reto"}
    ))
    
    # REGRA #2 - APROXIMAÇÃO DO CENTRO (desacelera antes de chegar)
    fis.add_rule(FuzzyRule(
        {"profundidade_vaga": "inicio"},
        {"velocidade": "lento", "angulo_direcao": "reto"}
    ))
    
    # REGRA #3 - ENTRADA NA VAGA (velocidade controlada)
    fis.add_rule(FuzzyRule(
        {"profundidade_vaga": "entrada"},
        {"velocidade": "lento", "angulo_direcao": "reto"}
    ))
    
    # REGRA #4 - SEGURANÇA NO FUNDO (desacelera se passar do centro)
    fis.add_rule(FuzzyRule(
        {"profundidade_vaga": "fundo"},
        {"velocidade": "muito_lento", "angulo_direcao": "reto"}
    ))
    
    # REGRA #5 - PARADA NO FINAL (segurança máxima)
    fis.add_rule(FuzzyRule(
        {"profundidade_vaga": "final"},
        {"velocidade": "parado", "angulo_direcao": "reto"}
    ))
    
    # REGRA #6 - SEGURANÇA: Para se muito próximo de obstáculo
    fis.add_rule(FuzzyRule(
        {"distancia_frontal": "muito_perto"},
        {"velocidade": "parado", "angulo_direcao": "reto"}
    ))
    
    # REGRA #7 - APROXIMAÇÃO: Velocidade rápida se longe
    fis.add_rule(FuzzyRule(
        {"distancia_frontal": "longe"},
        {"velocidade": "rapido", "angulo_direcao": "reto"}
    ))
    
    # REGRA #8 - CORREÇÃO ANGULAR ESQUERDA
    fis.add_rule(FuzzyRule(
        {"angulo_veiculo": "esquerda"},
        {"angulo_direcao": "direita", "velocidade": "lento"}
    ))
    
    # REGRA #9 - CORREÇÃO ANGULAR DIREITA
    fis.add_rule(FuzzyRule(
        {"angulo_veiculo": "direita"},
        {"angulo_direcao": "esquerda", "velocidade": "lento"}
    ))
    
    # REGRA #10 - CORREÇÃO LATERAL: Veículo muito abaixo do centro
    fis.add_rule(FuzzyRule(
        {"distancia_lateral": "muito_abaixo"},
        {"angulo_direcao": "esquerda", "velocidade": "lento"}  # Vira pra cima
    ))
    
    # REGRA #11 - CORREÇÃO LATERAL: Veículo abaixo do centro
    fis.add_rule(FuzzyRule(
        {"distancia_lateral": "abaixo"},
        {"angulo_direcao": "esquerda", "velocidade": "lento"}  # Vira pra cima
    ))
    
    # REGRA #12 - CORREÇÃO LATERAL: Veículo acima do centro
    fis.add_rule(FuzzyRule(
        {"distancia_lateral": "acima"},
        {"angulo_direcao": "direita", "velocidade": "lento"}  # Vira pra baixo
    ))
    
    # REGRA #13 - CORREÇÃO LATERAL: Veículo muito acima do centro
    fis.add_rule(FuzzyRule(
        {"distancia_lateral": "muito_acima"},
        {"angulo_direcao": "direita", "velocidade": "lento"}  # Vira pra baixo
    ))
    
    print(f"Sistema fuzzy criado com {len(fis.rules)} regras")
    return fis

if __name__ == "__main__":
    # Teste do sistema com parada centralizada
    fuzzy = create_centered_parking_system()
    
    print("\n=== TESTE: PARADA CENTRALIZADA ===")
    
    # Posições dentro da vaga
    test_cases = [
        (610, 290, 10, "Logo após entrar"),     # profundidade=10
        (650, 290, 50, "Meio da vaga"),         # profundidade=50
        (675, 290, 75, "Centro ideal"),         # profundidade=75 ⭐
        (700, 290, 100, "Próximo do centro"),   # profundidade=100
        (730, 290, 130, "Próximo do final")     # profundidade=130
    ]
    
    for x, y, prof, desc in test_cases:
        inputs = {
            'distancia_frontal': 750 - x,       # Até parede traseira
            'distancia_lateral': 0,             # Centrado
            'angulo_veiculo': 0,                # Alinhado
            'profundidade_vaga': prof           # Nova entrada!
        }
        
        outputs = fuzzy.infer(inputs)
        vel = outputs['velocidade']
        
        print(f"{desc:20} | Prof={prof:3}px | Vel={vel:5.1f} | {'✅ PARA' if vel < 3 else '⚠️  MOVE'}")
    
    print(f"\n✅ Sistema configurado para parar no centro da vaga!")