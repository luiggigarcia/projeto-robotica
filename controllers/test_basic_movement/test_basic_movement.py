#!/usr/bin/env python3

"""
Controller simplificado para teste inicial
Testa apenas os movimentos básicos do robô
"""

from controller import Robot

# Constantes
TIME_STEP = 64
MAX_SPEED = 6.28

def main():
    # Inicializa o robô
    robot = Robot()
    
    # Inicializa os motores
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    # Configura os motores
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    print("Teste de movimentos básicos:")
    print("1. Movendo para frente por 3 segundos...")
    
    step_count = 0
    phase = 1
    
    while robot.step(TIME_STEP) != -1:
        step_count += 1
        
        if phase == 1:  # Mover para frente
            left_motor.setVelocity(MAX_SPEED * 0.5)
            right_motor.setVelocity(MAX_SPEED * 0.5)
            
            if step_count > 50:  # ~3 segundos
                phase = 2
                step_count = 0
                print("2. Girando para a esquerda por 2 segundos...")
        
        elif phase == 2:  # Girar esquerda
            left_motor.setVelocity(-MAX_SPEED * 0.3)
            right_motor.setVelocity(MAX_SPEED * 0.3)
            
            if step_count > 30:  # ~2 segundos
                phase = 3
                step_count = 0
                print("3. Girando sobre o próprio eixo...")
        
        elif phase == 3:  # Girar sobre o eixo
            left_motor.setVelocity(MAX_SPEED * 0.4)
            right_motor.setVelocity(-MAX_SPEED * 0.4)
            
            if step_count > 100:  # ~6 segundos
                phase = 4
                print("4. Parando o robô.")
        
        else:  # Parar
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

if __name__ == "__main__":
    main()