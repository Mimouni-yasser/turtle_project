import rclpy
# Import des librairies ROS2 en Python
from rclpy.node import Node
# Import du module Twist du package geometry_msgs
from geometry_msgs.msg import Twist

# Import du module LaserScan de sensor_msgs
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# Import de la bibliothèque QoS pour définir les profils et la fiabilité
from rclpy.qos import ReliabilityPolicy, QoSProfile

from custom_interfaces.srv import ProjectCustomService
from custom_interfaces.msg import AIres

import time

import numpy as np
import math
from enum import Enum


class IA_etat(Enum):
    INIT = 0
    DETECTION = 1
    VERS_0 = 2
    DETERMINER_DIR = 3
    VERS_PRO = 4
    FIN = 5
    


class TopicsEx2Node(Node):
    def __init__(self):
        # Constructeur de la classe
        super().__init__('topics_ex2_node')
        
        self.all_found = False
        self.i = 0
        
        self.num_array = dict()
        list_num = list(self.num_array.keys())

        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(ProjectCustomService, 'server', self.custom_service_callback)
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

        # Définition de la période du timer (toutes les 0.2 secondes)
        timer_period = 0.2
        # Données utilisateur

        self.tolerance = 0.01
        self.degre_detection_direction = 10
        self.degre_detection_rotation = 3
        self.detection_min = 0.0
        self.detection_max = 1.0
        self.vitesse_min = 0.05
        self.vitesse_max = 0.5
                
        self.range = 10.0  # éviter que le robot rentre en rotation au début
        
        self.current_digit = None
        self.next_digit = None

        self.range_arriere = 10.0  # éviter que le robot rentre en rotation au début
        self.yaw = 0.0
        self.yawav = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.rotation = 0.0
        self.etat = 0.0
        self.rangeG = [100] * 360 #j'initialise un tableau de 360 elements à une valeur de 100 pour ne pas avoir un index error IRL
   
        self.test = 0.0
        self.sup = 0.0
        self.inf = 0.0
        self.condition_corrige = 0
        self.tableau_range = []
        

        self.mode = ''
        self.etatav = 0.0
        self.Nbr_image = 0.0
        
        
        self.direction = 1.0
        self.detected = list()
        
        
        self.IA_etat = IA_etat.INIT
        self.sorted_dict = dict()
        # Création d'un timer avec un callback toutes les 0.2 secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.first_digit = None
        self.total_time = 0.0
        # Abonnement au topic /scan pour recevoir les données de LaserScan
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Abonnement au topic /odom pour recevoir les données d'odométrie
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.AIsub = self.create_subscription(
            AIres,
            '/AI_detect',
            self.AI_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def AI_callback(self, msg):
        if self.etat == 2.0:
            cmd = Twist()
            match self.IA_etat:
                case IA_etat.INIT:
                    pass
                case IA_etat.DETECTION:
                    if str(msg.num) == '[]':
                        return
                    self.num_array[str(msg.num)] = msg.timestamp
                    if len(self.num_array.keys()) == 1:
                        self.first_digit = str(msg.num)
                    self.get_logger().info("table %s"%str(self.num_array))
                    if len(self.num_array.keys()) == 8:
                        self.sorted_dict = dict(sorted(self.num_array.items(), key=lambda item: item[1]))
                        
                        self.IA_etat = IA_etat.VERS_0
                        
                        pass
                case IA_etat.VERS_0:
                    self.list_num = list(self.num_array.keys())
                    self.list_num.sort()
                    self.determine_direction(self.sorted_dict[self.first_digit], self.sorted_dict["['0']"])
                    
                    if (msg.num == "['0']"):
                        self.current_digit = "['0']"
                        self.IA_etat = IA_etat.VERS_PRO
                        cmd.linear.x = 0
                        cmd.angular.z = 0
                        self.publisher_.publish(cmd)
                        time.sleep(3)
                    pass
                case IA_etat.DETERMINER_DIR:
                    pass
                case IA_etat.VERS_PRO:
                    if msg.num == self.next_digit:
                        #self.detected.append(msg.num)
                        #TODO ajouter logique de detection
                        cmd.linear.x = 0
                        cmd.angular.z = 0
                        self.publisher_.publish(cmd)
                        time.sleep(3)
                        self.current_digit = self.next_digit
                        self.i+=1
                        self.next_digit = f"['{self.i}']"
                    self.determine_direction(self.num_array[self.current_digit], self.num_array[self.next_digit])
                    if self.i == 9:
                        self.IA_etat = IA_etat.FIN
                    pass
                case IA_etat.FIN:
                    pass
                case _:
                    pass
            
            
            
    def determine_direction(self, P1, P2):
        if (P1-P2) > (self.total_time/2) or (P1-P2 < 0 and P1-P2> -(self.total_time/2)):
            self.direction = -1.0
        else:
            self.direction = 1.0
        
    def listener_callback(self, msg_sub):
       # Récupération des distances des capteurs laser
        indice_rangeG = len(msg_sub.ranges) // 4
        indice_range_arriere = len(msg_sub.ranges) // 2
        indice_sup = int(indice_rangeG - self.degre_detection_direction)
        indice_inf = int(indice_rangeG + self.degre_detection_direction)                
        indice_rangeG_sup = int(indice_rangeG + self.degre_detection_rotation)
        indice_rangeG_inf = int(indice_rangeG - self.degre_detection_rotation)        
 
        self.range_arriere = msg_sub.ranges[indice_range_arriere]
        self.range = msg_sub.ranges[0]
        self.tableau_range = msg_sub.ranges        
        self.rangeG = msg_sub.ranges[indice_rangeG_inf:indice_rangeG_sup]        
        self.sup = msg_sub.ranges[indice_sup]
        self.inf = msg_sub.ranges[indice_inf]
        self.get_logger().info('Range: "%s"\n' % str(self.range))

    def timer_callback(self):
        # Création d'un message Twist
        msg = Twist()

        self.get_logger().info('Yaw avant: "%s"\n' % str(self.yawav))
        self.get_logger().info('Yaw : "%s"\n' % str(self.yaw))
        
        #configuration de la condition de rotation
        
        if self.yawav <= -1.57 and self.yawav >= -3.14 and self.direction == 1.0 or self.yawav >= 1.57 and self.yawav <= 3.14 and self.direction == -1.0: 
            self.condition_corrige = 4.71
        else:
            self.condition_corrige = 0
        
        #remplacement des tableaux
        # Convertir en tableau NumPy si nécessaire
        self.rangeG = np.array(self.rangeG)
        self.tableau_range = np.array(self.tableau_range)

        self.rangeG[np.isinf(self.rangeG)] = 100  # Remplacer les inf
        self.rangeG[np.isnan(self.rangeG)] = 100  # Remplacer les NaN
        self.tableau_range[np.isinf(self.tableau_range)] = 100  # Remplacer les inf
        self.tableau_range[np.isnan(self.tableau_range)] = 100  # Remplacer les NaN                   
        
        # Condition de machine d'état
        
        if self.mode == 'Demarrage':
            self.etat = 0.0
            self.mode = '' #reinitialisation du mode
        elif self.mode == 'Start':
            self.etat = self.etatav
            self.mode = '' #reinitialisation du mode
        elif self.mode == 'Normal':
            self.etat = 2.0
            self.mode = '' #reinitialisation du mode
        elif self.mode == 'Stop' :
            self.etat = 4.0 
            self.mode = '' #reinitialisation du mode 
        else :
            self.mode = self.mode   
                    
        if self.etat == 0.0:
            if any(distance <= 0.3 for distance in self.tableau_range):
                self.etat = 1.0
                self.get_logger().info('Transition à l\'état 1 : démarrage de la rotation pour éviter l\'obstacle')

        elif self.etat == 1.0:
            if any(value <= min(self.tableau_range) for value in self.rangeG):
                self.etat = 2.0
                self.mode = 0.0
                self.get_logger().info('Transition à l\'état 2 : démarrage du suivi de bord')

        elif self.etat == 2.0:
            if (self.range < 0.3 and self.direction == 1) or (self.range_arriere < 0.3 and self.direction == -1):  # Présence d'un obstacle détecté
                self.etat = 3.0
                self.get_logger().info('Transition à l\'état 3 : rotation pour éviter l\'obstacle')            
                
            elif self.image_detect() != []:
                self.etat = 5.0
            
        elif self.etat == 3.0:
            if self.direction == 1.0: #si on va en marche avant
                if (abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 0) or (
            abs(self.yaw - self.yawav) <= self.condition_corrige and abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 4.71):
                    self.etat = 2.0
                    self.get_logger().info(f'Retour à l\'état 2 : rotation terminée, différence des yaw : {abs(self.yaw - self.yawav)}')
                    
                    
            elif self.direction == -1.0: #si on va en marche arrière
                if (abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 0) or (
            abs(self.yaw - self.yawav) >= self.condition_corrige and abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 4.71):
                    self.etat = 2.0
                    self.get_logger().info(f'Retour à l\'état 2 : rotation terminée, différence des yaw : {abs(self.yaw - self.yawav)}')                    
            
        self.get_logger().info('Etape: "%s"\n' % str(self.etat))
        self.get_logger().info('Etape: "%s"\n' % str(self.condition_corrige))

        longueur = abs(self.sup - self.inf)
        vitesse = self.map_range(longueur, self.detection_min, self.detection_max, self.vitesse_min, self.vitesse_max)

        # Machine d'état
               
        if self.etat == 0.0: #mode de detection du bord
            msg.linear.x = 0.1
            self.etatav = 0.0
            self.get_logger().info("Demarrage du robot\n")
        elif self.etat == 1.0:
            msg.linear.x = 0.0
            msg.angular.z = -0.2
            self.etatav = 1.0
            self.get_logger().info("Rotation de démarrage\n")
        elif self.etat == 2.0:

            if self.sup <= self.inf - self.tolerance:  # Je suis en train d'aller vers la gauche
                msg.linear.x = 0.1*self.direction
                msg.angular.z = -abs(vitesse)
                self.get_logger().info("Je corrige vers la droite\n")
            elif self.sup >= self.inf + self.tolerance:  # Je suis en train d'aller vers la droite
                msg.linear.x = 0.1*self.direction
                msg.angular.z = abs(vitesse)
                self.get_logger().info("Je corrige vers la gauche\n")
            else:
                msg.linear.x = 0.1*self.direction
                msg.angular.z = 0.0
                self.get_logger().info("Avancement du robot\n")
          
            self.yawav = self.yaw
            self.etatav = 2.0

        elif self.etat == 3.0:
            msg.linear.x = 0.0
            msg.angular.z = -0.2*self.direction
            self.get_logger().info("Rotation\n")
            self.etatav = 3.0
            
        elif self.etat == 4.0: 
            msg.linear.x = 0.0
            msg.angular.z = 0.0  
             
        elif self.etat == 5.0:
                        
            if self.Nbr_image <= 10.0:
                self.Nbr_image = self.Nbr_image + 1
                self.setpoint[self.Nbr_image] = self.enregistrement_setpoint()
            elif self.image_detect() == self.Nbr_image:     #si je déctecte la bonne image       
                start_time = self.get_clock().now().seconds_nanoseconds()[0]
                while self.get_clock().now().seconds_nanoseconds()[0] - start_time < 3.0: #on s'arrête pour montrer qu'on a trouvé 
                    msg.linear.x = 0.0 
                    msg.angular.z = 0.0                      
                    self.publisher_.publish(msg)
                    time.sleep(0.1)  # small delay to avoid spamming
                self.etat = 2.0
 
   
        # Publication du message sur le topic
        self.publisher_.publish(msg)


    def odom_callback(self, msg):
        quaternion = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(quaternion)
        self.get_logger().info('Yaw: "%s"\n' % str(self.yaw))

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def map_range(self, value, from_min, from_max, to_min, to_max):
        """
        Map a value from one range to another.
        """
        value = max(min(value, from_max), from_min)
        return to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)

    def custom_service_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response

        # create a Twist message
        msg = Twist()

        self.mode = request.mode
        self.vitesse_max = request.vitesse  
        self.degre_detection_rotation = request.detect_rot  
        self.degre_detection_direction = request.detect_dir  
        self.tolerance = request.tolerance    
                    
        # Message publication
        self.publisher_.publish(msg)
        self.get_logger().info(f'mode:  {request.mode} vitesse: {request.vitesse} degre_rot: {request.detect_rot} degre_dir: {request.detect_dir} tolerance: {request.tolerance }')
        
        # Set response
        response.success = True
        return response   
        
    def image_detect(self):
        return []     
    def enregistrement_setpoint (self):
        return 1.0        
        
def main(args=None):
    # Initialisation de la communication ROS
    rclpy.init(args=args)

    # Création du noeud
    topics_ex2_node = TopicsEx2Node()

    # Exécution du noeud en attente d'événements
    rclpy.spin(topics_ex2_node)
    

    # Destruction du noeud après arrêt
    topics_ex2_node.destroy_node()

    # Fermeture de la communication ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()