# Projeto Robótica - Webots Python

Este projeto implementa um robô que deve encontrar a caixa leve e girar sobre o próprio eixo quando a encontrar.

## Arquivos do Projeto

### Controllers
- `robot_controller.py` - Controller principal do robô que implementa a busca pela caixa leve
- `PosicaoCaixas.py` - mostra posições das caixas


## Como Testar


### 1. Configuração no Webots
1. Abra o Webots
2. Carregue seu arquivo `Mundo.wbt`
3. O robô já está configurado com o controller correto
4. Execute a simulação (▶️)

### 4. Comportamento Esperado
1. O robô identifica a caixa leve no início
2. Move-se em direção à caixa, evitando obstáculos
3. Quando chega próximo (< 50cm), para de se mover linearmente
4. Inicia rotação contínua sobre o próprio eixo