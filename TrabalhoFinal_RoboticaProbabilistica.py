import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import sys
import os
import cv2 as cv
import transforms3d
import numpy as np
import math
from importlib import import_module
from A_star import main as astar_main


""" # Importação dinâmica dos módulos auxiliares
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
Find_Vel = import_module('FindVel')
OpenCV_Bridge = import_module('OpenCV_Bridge')


class ImageProcessor:
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        self.subscription = None

    def start_processing(self):
        if self.subscription is None:
            self.subscription = self.node.create_subscription(
                Image,
                '/camera',
                self.image_callback,
                10
            )
            self.node.get_logger().info("Processamento de imagem ativado.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        OpenCV_Bridge.main(cv_image)


class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')

        # Inicializa posição e orientação
        self.current_position = None
        self.current_orientation = None
        self.goal = [5.0, 5.0]

        # Inicializa o processador de imagem
        self.image_processor = ImageProcessor(self)
        self.image_processing_started = False

        # Subscrições e publicações
        self.create_subscription(Odometry, '/odometry', self.odom_callback, 50)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_yaw(orientation_q)

        self.current_position = position
        self.current_orientation = yaw

    def quaternion_to_yaw(self, q):
        # Conversão direta de quaternion para yaw (rotação no plano XY)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

    def lidar_callback(self, msg):
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("Aguardando dados de odometria...")
            return

        lidar_data = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        forces = Find_Vel.algorithm(
            self.current_position,
            self.goal,
            lidar_data,
            angle_min,
            angle_increment,
            self.current_orientation
        )

        self.get_logger().info(f"Forças calculadas: linear={forces[0]:.2f}, angular={forces[1]:.2f}")

        cmd_vel = Twist()
        cmd_vel.linear.x = forces[0]
        cmd_vel.angular.z = forces[1]
        self.cmd_vel_publisher.publish(cmd_vel)

        # Se o robô chegou no destino, inicia o processamento de imagem
        if forces[0] == 0 and forces[1] == 0 and not self.image_processing_started:
            self.image_processor.start_processing()
            self.image_processing_started = True


def main(args=None):
    rclpy.init(args=args)
    navigator = RobotNavigator()
    rclpy.spin(navigator)
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
 """

#Código antigo

firstRun = True

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
Find_Vel = import_module('FindVel')
OpenCV_Bridge = import_module('OpenCV_Bridge')

class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.current_position = None
        self.current_orientation = None

        # Lista de objetivos: adicione quantos quiser
        #self.goals = [[2.5, 2.5], [7.5, 2.5], [7.5, 7.5], [7.5, 12.5], [7.5, 17.5], [2.5, 17.5]]  # Exemplo
        self.goals = astar_main()  # Agora usa o resultado do A*
        self.goal_index = 0
        self.goal_tolerance = 0.3  # Distância para considerar o goal alcançado

        # Subscrições e publicações
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.odom_callback, 50)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.position_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        OpenCV_Bridge.main(cv_image)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        # Verificar o quaternion recebido diretamente
        #self.get_logger().info(f"Quaternion recebido: x={orientation_q.x}, y={orientation_q.y}, z={orientation_q.z}, w={orientation_q.w}")

        # Convertendo o quaternion para ângulo de Euler (yaw)
        (roll, pitch, yaw) = self.quaternion_to_euler(orientation_q)

        # Verificando o valor de yaw após conversão
        #self.get_logger().info(f"Yaw após conversão: {yaw}")

        self.current_position = position
        self.current_orientation = yaw  # Armazenando o yaw (ângulo) para uso posterior

    def quaternion_to_euler(self, orientation_q):
        """Converte quaternion para Euler utilizando transforms3d"""
        # Criando um quaternion com a orientação
        quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]

        # Usando transforms3d para converter para Euler
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)

        # Retorna os ângulos de Euler
        return roll, pitch, yaw
    
    def lidar_callback(self, msg):

        #self.camera_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)

        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("erro")
            return

        if self.goal_index >= len(self.goals):
            self.get_logger().info("Todos os objetivos foram alcançados.")
            self.stop_robot()
            return
        
        goal = self.goals[self.goal_index]
        current_pos = self.current_position
        dist_to_goal = math.hypot(goal[0] - current_pos.x, goal[1] - current_pos.y)

            # Verifica se chegou ao goal ANTES de calcular as forças
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Objetivo {self.goal_index + 1} alcançado: {goal}")
            self.goal_index += 1

            # Se acabou os goals, para
            if self.goal_index >= len(self.goals):
                self.get_logger().info("Todos os objetivos completados.")
                self.stop_robot()
                WINDOW_NAME = "Detecção de Quadrado"
                cv.namedWindow(WINDOW_NAME, cv.WINDOW_NORMAL)
                self.camera_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
                return
            #else:
            #    goal = self.goals[self.goal_index]  # Atualiza o próximo goal
        
         # Atualiza o goal para o próximo
        goal = self.goals[self.goal_index]


        lidar_data = msg.ranges

        # Pegando os parâmetros necessários do msg
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        #self.get_logger().info(f"Lidar data: {lidar_data[:5]}...")

        # Passando os parâmetros para o algoritmo
        forces = Find_Vel.algorithm(current_pos, goal, lidar_data, angle_min, angle_increment, self.current_orientation)

        # Segurança extra: se forças forem zero, mas ainda não atingiu o goal → força pequeno avanço
        if forces[0] == 0.0 and forces[1] == 0.0 and dist_to_goal > self.goal_tolerance:
            self.get_logger().warn(f"Forças zeradas mas ainda distante do goal {goal} (dist: {dist_to_goal:.2f}) → avanço forçado.")
            forces = (0.1, 0.0)  # pequeno avanço para destravar


        self.get_logger().info(f"Forças para goal {self.goal_index + 1} {goal}: {forces}")
        #self.get_logger().info(f"Forças calculadas: {forces}")

        cmd_vel = Twist()
        cmd_vel.linear.x = forces[0]
        cmd_vel.angular.z = forces[1]

        self.position_publisher.publish(cmd_vel)

        # def stop_robot(self):
        #     stop_msg = Twist()
        #     self.position_publisher.publish(stop_msg)
        #     self.get_logger().info("Robô parado.")
            
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.position_publisher.publish(stop_msg)
        self.get_logger().info("Robô parado.")

#        if (forces[0] == 0 and forces[1] == 0):
#            self.camera_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        

def main(args=None):
    rclpy.init(args=args)
    image_processing = ImageProcessing()
    rclpy.spin(image_processing)
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

