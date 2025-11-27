from controller import Robot, Motor, DistanceSensor, Supervisor
import math

# Constantes
TIME_STEP = 64
MAX_SPEED = 6.28  # Velocidade máxima dos motores
QTDD_CAIXA = 20   # Número total de caixas (CAIXA01 a CAIXA20)

class RobotController:
    def __init__(self):
        # Inicializa o supervisor (para acessar posições das caixas)
        self.robot = Supervisor()
        
        # Inicializa os motores das rodas
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        
        # Configura os motores para velocidade
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Inicializa sensores de distância
        self.distance_sensors = []
        sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
        
        for name in sensor_names:
            sensor = self.robot.getDevice(name)
            if sensor:
                sensor.enable(TIME_STEP)
                self.distance_sensors.append(sensor)
        
        # Estado do robô
        self.state = 'SEARCHING'  # Estados: SEARCHING, APPROACHING, SPINNING
        self.target_box = None
        self.boxes = []
        
        # Controle de estabilidade
        self.stable_counter = 0
        self.spin_delay_counter = 0
        self.search_counter = 0
        self.last_known_distance = float('inf')
        
        # Carrega as referências das caixas
        self.load_boxes()
        
        # Encontra a caixa leve
        self.find_light_box()
        
    def load_boxes(self):
        """Carrega as referências de todas as caixas"""
        for i in range(1, QTDD_CAIXA + 1):  # CAIXA01 a CAIXA20
            box_name = f"CAIXA{i:02d}"
            box_node = self.robot.getFromDef(box_name)
            if box_node:
                self.boxes.append({
                    'name': box_name,
                    'node': box_node,
                    'index': i
                })
                print(f"{i:2d}. {box_name} - Carregada com sucesso")
            else:
                print(f"Falha ao carregar a posição da {box_name}")
        
        print(f"\n{len(self.boxes)} CAIXAS CARREGADAS\n")
    
    def find_light_box(self):
        """Identifica qual é a caixa leve baseada na massa"""
        lightest_mass = float('inf')
        lightest_box = None
        boxes_with_mass = []
        
        for box in self.boxes:
            # Obtém a massa da caixa
            mass_field = box['node'].getField('mass')
            if mass_field:
                mass = mass_field.getSFFloat()
                print(f"{box['name']}: massa = {mass:.2f} kg")
                
                # Só considera caixas com massa > 0 (ignora caixas sem peso definido)
                if mass > 0.0:
                    boxes_with_mass.append((box, mass))
                    if mass < lightest_mass:
                        lightest_mass = mass
                        lightest_box = box
            else:
                print(f"{box['name']}: sem campo de massa")
        
        print(f"\nCaixas com massa definida: {len(boxes_with_mass)}")
        for box, mass in boxes_with_mass:
            status = "(LEVE)" if box == lightest_box else ""
            print(f"  {box['name']}: {mass:.2f} kg {status}")
        
        if lightest_box:
            self.target_box = lightest_box
            print(f"\nCaixa leve identificada: {lightest_box['name']} (massa: {lightest_mass:.2f} kg)\n")
        else:
            print("Nenhuma caixa leve encontrada!")
    
    def get_robot_position(self):
        """Obtém a posição atual do robô"""
        robot_node = self.robot.getSelf()
        if robot_node:
            return robot_node.getPosition()
        return [0, 0, 0]
    
    def get_distance_to_target(self):
        """Calcula a distância até a caixa alvo"""
        if not self.target_box:
            return float('inf')
        
        robot_pos = self.get_robot_position()
        target_pos = self.target_box['node'].getPosition()
        
        dx = target_pos[0] - robot_pos[0]
        dz = target_pos[2] - robot_pos[2]  # No Webots, Z é a profundidade
        
        return math.sqrt(dx*dx + dz*dz)
    
    def get_simple_direction_to_target(self):
        """Retorna uma direção simples para o alvo: 'left', 'right', ou 'forward'"""
        if not self.target_box:
            return 'forward'
        
        robot_pos = self.get_robot_position()
        target_pos = self.target_box['node'].getPosition()
        
        # Vetor direção do robô para o alvo
        dx = target_pos[0] - robot_pos[0]
        dz = target_pos[2] - robot_pos[2]
        
        # Obtém a orientação atual do robô
        robot_node = self.robot.getSelf()
        robot_rotation = robot_node.getOrientation()
        
        # Vetor direção frontal do robô (direção que ele está olhando)
        robot_forward_x = robot_rotation[0]
        robot_forward_z = robot_rotation[2]
        
        # Produto vetorial para determinar se o alvo está à esquerda ou direita
        cross_product = dx * robot_forward_z - dz * robot_forward_x
        
        # Produto escalar para determinar se está na frente ou atrás
        dot_product = dx * robot_forward_x + dz * robot_forward_z
        
        # Decide a direção baseada nos produtos
        if abs(cross_product) < 0.1 and dot_product > 0:
            return 'forward'  # Alvo está na frente
        elif cross_product > 0:
            return 'left'     # Alvo está à esquerda
        else:
            return 'right'    # Alvo está à direita
    
    def has_obstacle_ahead(self):
        """Verifica se há obstáculo à frente usando sensores de distância"""
        if not self.distance_sensors:
            return False
        
        # Verifica os sensores frontais (ps0, ps1, ps2, ps6, ps7)
        front_sensors = [0, 1, 2, 6, 7]
        
        for i in front_sensors:
            if i < len(self.distance_sensors):
                distance = self.distance_sensors[i].getValue()
                if distance > 80:  # Obstacle detected (sensor values are inverted in Webots)
                    return True
        
        return False
    
    def is_target_box_detected(self):
        """Verifica se o objeto detectado à frente é a caixa alvo"""
        if not self.target_box:
            return False
        
        robot_pos = self.get_robot_position()
        target_pos = self.target_box['node'].getPosition()
        
        # Calcula se a caixa está aproximadamente à frente do robô
        dx = target_pos[0] - robot_pos[0]
        dz = target_pos[2] - robot_pos[2]
        
        robot_node = self.robot.getSelf()
        robot_rotation = robot_node.getOrientation()
        robot_angle = math.atan2(robot_rotation[6], robot_rotation[0])
        
        target_angle = math.atan2(dz, dx)
        angle_diff = abs(target_angle - robot_angle)
        
        # Normaliza o ângulo
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        angle_diff = abs(angle_diff)
        
        # Se a caixa está à frente (ângulo pequeno) e próxima
        return angle_diff < 0.3 and self.get_distance_to_target() < 0.3
    
    def move_forward(self):
        """Move o robô para frente"""
        self.left_motor.setVelocity(MAX_SPEED * 0.7)  # Mais rápido
        self.right_motor.setVelocity(MAX_SPEED * 0.7)
    
    def turn_left(self):
        """Gira o robô para a esquerda"""
        self.left_motor.setVelocity(-MAX_SPEED * 0.4)  # Mais rápido
        self.right_motor.setVelocity(MAX_SPEED * 0.4)
    
    def turn_right(self):
        """Gira o robô para a direita"""
        self.left_motor.setVelocity(MAX_SPEED * 0.4)  # Mais rápido
        self.right_motor.setVelocity(-MAX_SPEED * 0.4)
    
    def spin_on_axis(self):
        """Gira o robô sobre o próprio eixo"""
        self.left_motor.setVelocity(MAX_SPEED * 0.4)
        self.right_motor.setVelocity(-MAX_SPEED * 0.4)
    
    def stop(self):
        """Para o robô"""
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def run(self):
        """Loop principal do robô"""
        print("🤖 Iniciando busca pela caixa leve...")
        
        if self.target_box:
            print(f"🎯 Alvo: {self.target_box['name']} (massa: {self.target_box['node'].getField('mass').getSFFloat():.2f}kg)")
        else:
            print("⚠️ ATENÇÃO: Nenhuma caixa leve encontrada no mundo!")
        
        while self.robot.step(TIME_STEP) != -1:
            distance_to_target = self.get_distance_to_target()
            
            if self.state == 'SEARCHING':
                # Verifica se tem uma caixa alvo definida
                if not self.target_box:
                    print("❌ Nenhuma caixa alvo encontrada! Procurando...")
                    # Gira lentamente procurando
                    self.turn_left()
                    return
                
                if distance_to_target < 0.25:  # Chegou próximo à caixa (25cm)
                    self.state = 'APPROACHING'
                    print(f"🎯 Caixa leve detectada próxima! Distância: {distance_to_target:.2f}m")
                    print("Mudando para modo de aproximação...")
                    self.stable_counter = 0
                    self.search_counter = 0
                else:
                    # Verifica se está progredindo em direção ao alvo
                    self.search_counter += 1
                    
                    # Se não está progredindo há muito tempo, procura novamente
                    if self.search_counter > 200 and distance_to_target > self.last_known_distance:
                        print("🔄 Robô pode estar perdido. Reiniciando busca...")
                        self.search_counter = 0
                        # Gira para procurar novamente
                        self.turn_left()
                        return
                    
                    self.last_known_distance = min(self.last_known_distance, distance_to_target)
                    
                    # Navega em direção à caixa usando lógica simples
                    direction = self.get_simple_direction_to_target()
                    
                    # Verifica obstáculo apenas se não estiver indo direto para a caixa alvo
                    if self.has_obstacle_ahead() and distance_to_target > 0.15:
                        # Desvia apenas se não for a caixa alvo
                        self.turn_left()
                        print("🚧 Obstáculo detectado, desviando...")
                        self.stable_counter = 0
                    elif direction == 'left':
                        self.stable_counter = 0
                        self.turn_left()
                        print(f"⬅️ Virando à esquerda para {self.target_box['name']} (dist: {distance_to_target:.2f}m)")
                    elif direction == 'right':
                        self.stable_counter = 0
                        self.turn_right()
                        print(f"➡️ Virando à direita para {self.target_box['name']} (dist: {distance_to_target:.2f}m)")
                    else:  # direction == 'forward'
                        self.stable_counter += 1
                        
                        if self.stable_counter > 2:  # Estável por menos ciclos (mais rápido)
                            # Move em direção à caixa com velocidade ajustada pela distância
                            if distance_to_target > 0.3:
                                self.move_forward()  # Velocidade alta
                            else:
                                # Velocidade moderada quando próximo
                                self.left_motor.setVelocity(MAX_SPEED * 0.4)
                                self.right_motor.setVelocity(MAX_SPEED * 0.4)
                            print(f"🎯 Indo para {self.target_box['name']}... Distância: {distance_to_target:.2f}m")
                        else:
                            # Ainda estabilizando
                            self.stop()
                            print(f"⚖️ Alinhando com o alvo... (dist: {distance_to_target:.2f}m)")
            
            elif self.state == 'APPROACHING':
                # Verifica se perdeu o alvo (muito longe novamente)
                if distance_to_target > 0.5:
                    print("❓ Perdeu o alvo durante aproximação. Voltando para busca...")
                    self.state = 'SEARCHING'
                    self.search_counter = 0
                    return
                
                # IMPORTANTE: Só para se estiver REALMENTE próximo da CAIXA ALVO (não qualquer obstáculo)
                if distance_to_target < 0.08:  # Muito próximo da caixa alvo específica
                    self.state = 'SPINNING'
                    print(f"🎯 Chegou até a {self.target_box['name']}! Iniciando rotação...")
                    self.stop()
                    self.spin_delay_counter = 0
                else:
                    # Move devagar em direção à caixa
                    direction = self.get_simple_direction_to_target()
                    
                    if direction == 'forward':
                        # Velocidade ajustada para aproximação mais rápida
                        if distance_to_target < 0.10:
                            speed = MAX_SPEED * 0.3  # Moderado quando próximo
                        elif distance_to_target < 0.20:
                            speed = MAX_SPEED * 0.5  # Normal
                        else:
                            speed = MAX_SPEED * 0.6  # Rápido para aproximação
                        
                        self.left_motor.setVelocity(speed)
                        self.right_motor.setVelocity(speed)
                        print(f"🎯 Aproximando da caixa... Distância: {distance_to_target:.3f}m")
                    elif direction == 'left':
                        self.turn_left()
                        print("⬅️ Ajustando à esquerda")
                    else:  # right
                        self.turn_right()
                        print("➡️ Ajustando à direita")
            
            elif self.state == 'SPINNING':
                # Gira sobre o próprio eixo imediatamente
                self.spin_on_axis()
                if self.spin_delay_counter == 0:  # Primeira vez girando
                    print("🌀 Girando sobre o próprio eixo!")
                    self.spin_delay_counter = 1  # Marca que já mostrou a mensagem

# Ponto de entrada
if __name__ == "__main__":
    controller = RobotController()
    controller.run()