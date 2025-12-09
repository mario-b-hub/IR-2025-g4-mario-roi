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
        self.front_dist = 10.0 

        # 4. TIMER (CEREBRO):
        # Ejecutamos la lógica de control cada 0.2 segundos
        self.timer_ = self.create_timer(0.2, self.control_loop)
        
        self.get_logger().info('MyCustomNode iniciado: Robot autónomo listo.')

    def scan_callback(self, msg):
        # Esta función solo se encarga de ACTUALIZAR la "visión" del robot.
        # En Turtlebot3, ranges[0] es el frente.
        # Verificamos que existan datos para no romper el nodo
        if len(msg.ranges) > 0:
            self.front_dist = msg.ranges[0]
            # Si el sensor devuelve 'inf' (infinito), lo tratamos como distancia máxima
            if self.front_dist == float('inf'):
                self.front_dist = 10.0

    def control_loop(self):
        # Esta es la función que el Timer llama repetidamente.
        # Aquí va tu lógica "IF/ELSE" corregida.
        
        # Debug en terminal para saber qué ve el robot
        # self.get_logger().info(f'Distancia frontal: {self.front_dist}')

        if self.front_dist < 0.5:
            # Hay obstáculo: Giramos sobre el eje (sin avanzar)
            self.move_robot(0.0, -0.5) 
        else:
            # Camino libre: Avanzamos recto
            self.move_robot(0.2, 0.0) 

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
