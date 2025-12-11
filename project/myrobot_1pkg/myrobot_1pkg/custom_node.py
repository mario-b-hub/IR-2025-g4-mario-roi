import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        
        # 1. PUBLICADOR: Para enviar comandos de velocidad
        # Definimos self.cmd_vel_publisher_ DENTRO del __init__
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. SUSCRIPTOR: Para leer el láser
        self.subscriber_ = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )

        # 3. VARIABLE DE ESTADO:
        # Creamos una variable 'global' para la clase que guarde la distancia.
        # La iniciamos con un valor alto (camino libre) por seguridad.
        # VARIABLES DE ESTADO (Sensores Virtuales)
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        # 0: Avanzando, 1: Girando a Izquierda, 2: Girando a Derecha
        self.state = 0

        # 4. TIMER (CEREBRO):
        # Ejecutamos la lógica de control cada 0.2 segundos
        self.timer_ = self.create_timer(0.2, self.control_loop)
        
        self.get_logger().info('MyCustomNode iniciado: Robot autónomo listo.')

    def scan_callback(self, msg):
        # Función auxiliar interna para calcular el promedio de un sector
        def get_sector_distance(target_angle, window_deg=10):
            # 1. Convertir ventana de grados a número de índices
            # msg.angle_increment es radianes por índice
            window_rad = (window_deg * 3.14159) / 180.0
            half_window_indices = int(window_rad / msg.angle_increment / 2)
            
            # 2. Calcular índice central teórico para el ángulo deseado
            # Fórmula: index = (angulo_deseado - angulo_minimo) / incremento
            center_index = int((target_angle - msg.angle_min) / msg.angle_increment)
            
            # 3. Definir límites del array (start y end) con protección de desbordamiento
            start = max(0, center_index - half_window_indices)
            end = min(len(msg.ranges), center_index + half_window_indices + 1)
            
            # 4. Extraer y filtrar datos
            sector = msg.ranges[start:end]
            valid_readings = [r for r in sector if not (r == float('inf') or r == float('nan'))]
            
            if not valid_readings:
                return 10.0 # Sin datos válidos, asumimos libre
            return sum(valid_readings) / len(valid_readings)

        # ACTUALIZACIÓN DE ESTADOS
        # Frente (0 radianes)
        self.front_dist = get_sector_distance(0.0)
        
        # Izquierda (aprox +45 grados = +0.78 rad)
        self.left_dist = get_sector_distance(0.78)
        
        # Derecha (aprox -45 grados = -0.78 rad)
        self.right_dist = get_sector_distance(-0.78)
        
        # Debug para verificar integridad (puedes comentarlo luego)
        # self.get_logger().info(f'F: {self.front_dist:.2f} | L: {self.left_dist:.2f} | R: {self.right_dist:.2f}')  

    def decide_turn_direction(self):
        """
        Analiza qué lado tiene más espacio libre.
        Retorna -1 para Izquierda (sentido default) o +1 para Derecha.
        """
        if self.left_dist > self.right_dist:
            # Camino libre a la izquierda
            self.get_logger().info(f'Hueco detectado a IZQUIERDA (L:{self.left_dist:.1f} > R:{self.right_dist:.1f})')
            return -1 # Signo negativo (Anti-horario)
        else:
            # Camino libre a la derecha
            self.get_logger().info(f'Hueco detectado a DERECHA (R:{self.right_dist:.1f} > L:{self.left_dist:.1f})')
            return 1  # Signo positivo (Horario)

    def control_loop(self):
        # --- PARÁMETROS ---
        STOP_DIST = 1       
        RESUME_DIST = 2.5     
        KP = 0.45 
        CORRIDOR_LIMIT = 2.0 

        # --- MÁQUINA DE ESTADOS ---
        
        if self.state == 0: # ESTADO: AVANZAR
            if self.front_dist > STOP_DIST:
                # 1. Centrado en pasillo (Mantenemos lo que funciona)
                linear = 0.8
                angular_correction = 0.0

                if self.left_dist < CORRIDOR_LIMIT and self.right_dist < CORRIDOR_LIMIT:
                    error = self.left_dist - self.right_dist
                    angular_correction = KP * error
                    angular_correction = max(-0.4, min(0.4, angular_correction))
                
                self.move_robot(linear, angular_correction)
                
            else:
                # 2. ESQUINA DETECTADA -> DECISIÓN CRÍTICA
                self.get_logger().info(f'Pared frontal ({self.front_dist:.2f}m). Analizando giro...')
                
                # AQUÍ LLAMAMOS A LA NUEVA FUNCIÓN
                # Decidimos el sentido UNA VEZ y lo guardamos
                self.turn_direction = self.decide_turn_direction()
                
                # Cambiamos estado
                self.state = 1

        elif self.state == 1: # ESTADO: GIRAR (Dinámico)
            if self.front_dist > RESUME_DIST:
                self.state = 0 
                self.get_logger().info('Pasillo alineado. Avanzando.')
                self.move_robot(0.9, 0.0)
            else:
                # 3. EJECUCIÓN DEL GIRO
                # Usamos una velocidad base (0.5) multiplicada por la dirección decidida (-1 o +1)
                # Esto invierte el giro automáticamente si es necesario.
                velocidad_giro = 1 * self.turn_direction
                self.move_robot(0.0, velocidad_giro)


    def move_robot(self, linear, angular):
        # Función auxiliar para publicar el mensaje
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detenemos el robot al cerrar con Ctrl+C
        node.move_robot(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()