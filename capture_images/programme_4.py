import rclpy
# Import des librairies ROS2 en Python
from rclpy.node import Node
# Import du module Twist du package geometry_msgs
from geometry_msgs.msg import Twist

#for the LCD
from std_msgs.msg import String

# Import du module LaserScan de sensor_msgs
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# Import de la bibliothèque QoS pour définir les profils et la fiabilité
from rclpy.qos import ReliabilityPolicy, QoSProfile

#importation d'interface permettant de comuniquer entre différent noeuds
from custom_interfaces.srv import ProjectCustomService #interface de communication entre le RobotNode et le client
from custom_interfaces.msg import AIres #interface de communication entre le RobotNode et ImageCaptureNode

import time

import numpy as np
import math
from enum import Enum


NUM_DIGITS = 10 #nombre de numéro à détecter

class IA_etat(Enum):
    INIT = 0
    DETECTION = 1
    VERS_0 = 2
    DETERMINER_DIR = 3
    VERS_PRO = 4
    FIN = 5

class Robot_etat(Enum):
    DEMARRAGE = 0
    ROTATION_DEMARRAGE = 1
    AVANCEMENT = 2
    ROTATION = 3
    ARRET = 4    


class RobotNode(Node):
    def __init__(self):
        # Constructeur de la classe
        super().__init__('robot_node')
        
        self.all_found = False #bool indiquant la fin de la recherche des numéros
        self.i = 1
        
        self.num_array = dict()
        list_num = list(self.num_array.keys())

        #creation de l'objet service server
        self.srv = self.create_service(ProjectCustomService, 'server', self.custom_service_callback)
        #creation du publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)        
        
        #for publishing to the LCD
        self.lcd_publisher = self.create_publisher(String, 'LCD_topic', 10)
        
        
        # Définition de la période du timer (toutes les 0.2 secondes)
        timer_period = 0.2

        # Données utilisateur
        self.tolerance = 0.01 #tolérance sur la coplénéarité du robot par rapport au mur
        self.degre_detection_direction = 10 #angle entre la longueur sup (inf) et le 90 degré
        self.degre_detection_rotation = 3 #angle de détection lors de la rotation de démarrage
        self.detection_min = 0.0 #différence minimim entre les longueurs sup et inf
        self.detection_max = 1.0 #différence maximum entre les longueurs sup et inf
        self.vitesse_min = 0.05 #vitesse angulaire minimum du robot
        self.vitesse_max = 2.0 #vitesse angulaire maximum du robot
                
        self.range = 10.0  # initialisation du range à 10m afin d'éviter que le robot rentre en rotation au dès le lancement du programme
        
        self.current_digit = None #variable indiquant le dernier nombre trouvé
        self.next_digit = None #variable indiquant le prochain nombre à trouver

        self.range_arriere = 10.0  # initialisation du range à 10m afin d'éviter que le robot rentre en rotation au dès le lancement du programme
        self.yaw = 0.0 #variable contenant la valeur du lacet
        self.yawav = 0.0 #variable contenant la valeur du lacet avnt le commencement de la rotation
        self.roll = 0.0 #variable contenant la valeur du roulis
        self.pitch = 0.0 #variable contenant la valeur du tangage
        self.etat = Robot_etat.DEMARRAGE #initialisation de l'état du robot
        self.rangeG = [100] * 360 #initialisation d'un tableau de 360 elements à une valeur de 100 pour ne pas avoir un index error IRL
   
        self.sup = 0.0 #variable contenant la distance du robot avec son environnement à 10° de l'horizontale
        self.inf = 0.0 #variable contenant la distance du robot avec son environnement à 10° de l'horizontale
        self.condition_corrige = 0 #variable indiquant la condition nécessaire pour effectuer une rotation de 90°
        self.tableau_range = [] #tableau contenant la distance du robot avec son environnement sur 360°
        

        self.mode = '' #variable contenant le mode choisis par le client
        self.etatav = Robot_etat.DEMARRAGE #variable contenant l'etat du robot avant son arret
        self.Nbr_image = 0.0
        NUM_DIGITS
        
        self.strip = "[']"
        
        self.direction = 1 #variable indiquant la direction que doit prendre le robot pour atteindre le prochain nombre le plus rapidemment possible
        self.detected = list()
        
        
        self.IA_etat = IA_etat.INIT
        self.sorted_dict = dict()
        # Création d'un timer avec un callback toutes les 0.2 secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.first_digit = None
        self.total_time = 0.0
        self.LCD_informed = False

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
        LCD_msg = String()
        if self.etat == Robot_etat.AVANCEMENT:
            cmd = Twist()
            match self.IA_etat:
                case IA_etat.INIT:
                    self.IA_etat = IA_etat.DETECTION
                    pass
                case IA_etat.DETECTION:
                    self.get_logger().info("state detection")
                    if len(self.num_array.keys()) == NUM_DIGITS:
                        if msg.num == self.first_digit:
                            self.total_time = msg.timestamp - self.num_array[str(msg.num)]
                            self.first_digit = msg.num
                            self.sorted_dict = dict(sorted(self.num_array.items(), key=lambda item: item[1]))
                            self.IA_etat = IA_etat.VERS_0
                            return
                    if str(msg.num) == '[]':
                        return
                    if str(msg.num) not in self.num_array.keys():
                        LCD_msg.data = f"ORDER: {str(msg.num).strip(self.strip)}"
                        self.num_array[str(msg.num)] = msg.timestamp
                        self.lcd_publisher.publish(LCD_msg)
                    if len(self.num_array.keys()) == 1:
                        self.first_digit = msg.num
                    self.get_logger().info("table %s"%str(self.num_array))

                case IA_etat.VERS_0:
                    self.get_logger().info("state vers 0")
                    LCD_msg.data = f"ORDER: done"
                    self.lcd_publisher.publish(LCD_msg)
                    self.list_num = list(self.num_array.keys())
                    self.list_num.sort()
                    
                    
                    if not self.LCD_informed:
                        LCD_msg.data = f"GOING: {str(self.first_digit).strip(self.strip)}"
                        self.lcd_publisher.publish(LCD_msg) #little hack to show the first digit
                        LCD_msg.data = f"GOING: 0"
                        self.lcd_publisher.publish(LCD_msg)
                        self.LCD_informed = True
                    self.determine_direction(self.sorted_dict[self.first_digit], self.sorted_dict["['0']"])
                    
                    if (msg.num == "['0']"):
                        self.get_logger().info("all nums detected")
                        self.current_digit = "['0']"
                        self.IA_etat = IA_etat.VERS_PRO
                        self.LCD_informed = False
                        self.next_digit = "['1']"
                    pass
                case IA_etat.DETERMINER_DIR:
                    pass
                case IA_etat.VERS_PRO:
                    if not self.LCD_informed:
                        LCD_msg.data = f"GOING: {str(self.next_digit).strip(self.strip)}"
                        self.lcd_publisher.publish(LCD_msg)
                        self.LCD_informed = True
                    self.get_logger().info("state vers prochaine %s" % self.next_digit)
                    if msg.num == self.next_digit:
                        #self.detected.append(msg.num)
                        #TODO ajouter logique de detection
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        self.publisher_.publish(cmd)
                        time.sleep(1)
                        self.current_digit = self.next_digit
                        self.i+=1
                        self.next_digit = f"['{self.i}']"
                        self.LCD_informed = False
                    if self.i >= NUM_DIGITS:
                        self.IA_etat = IA_etat.FIN
                        return
                    self.determine_direction(self.num_array[self.current_digit], self.num_array[self.next_digit])

                    pass
                case IA_etat.FIN:
                    pass
                case _:
                    pass
            
            
            
    def determine_direction(self, P1, P2):
        #fonction déterminant la distance la plus courte jusqu'au prochain numéro
        if (P1-P2) > (self.total_time/2) or (P1-P2 < 0 and P1-P2> -(self.total_time/2)):
            self.direction = 1.0
        else:
            self.direction = -1.0
            
        self.get_logger().info("directions determined: %f" %self.direction)
        
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

    def timer_callback(self):
        # Création d'un message Twist
        msg = Twist()
        
        #configuration de la condition pour effectuer une rotation de 90° en fonction du quadrant       
        if self.yawav <= -1.57 and self.yawav >= -3.14 and self.direction == 1.0 or self.yawav >= 1.57 and self.yawav <= 3.14 and self.direction == -1.0: 
            self.condition_corrige = 4.71
        else:
            self.condition_corrige = 0


        #remplacement des inf et NaN par 100 pour éviter les problèmes lors de la rotation de démarrage
        self.rangeG = np.array(self.rangeG)
        self.tableau_range = np.array(self.tableau_range)
        self.rangeG[np.isinf(self.rangeG)] = 100 
        self.rangeG[np.isnan(self.rangeG)] = 100
        self.tableau_range[np.isinf(self.tableau_range)] = 100
        self.tableau_range[np.isnan(self.tableau_range)] = 100                
        
        # Condition de machine d'état
        #choix de l'état en fonction du client
        if self.mode == 'Demarrage':
            self.etat = Robot_etat.DEMARRAGE
            self.mode = '' #reinitialisation du mode
        elif self.mode == 'Start':
            self.etat = self.etatav
            self.mode = '' #reinitialisation du mode
        elif self.mode == 'Normal':
            self.etat = Robot_etat.AVANCEMENT
            self.mode = '' #reinitialisation du mode
        elif self.mode == 'Stop' :
            self.etat = Robot_etat.ARRET
            self.mode = '' #reinitialisation du mode 
        else :
            self.mode = self.mode   
                    
        if self.etat == Robot_etat.DEMARRAGE:
            if any(distance <= 0.3 for distance in self.tableau_range):
                self.etat = Robot_etat.ROTATION_DEMARRAGE
                #self.get_logger().info('Transition à l\'état 1 : démarrage de la rotation pour éviter l\'obstacle')

        elif self.etat == Robot_etat.ROTATION_DEMARRAGE:
            if any(value <= min(self.tableau_range) for value in self.rangeG):
                self.etat = Robot_etat.AVANCEMENT
                #self.get_logger().info('Transition à l\'état 2 : démarrage du suivi de bord')

        elif self.etat == Robot_etat.AVANCEMENT:
            if (self.range < 0.3 and self.direction == 1) or (self.range_arriere < 0.3 and self.direction == -1):  # Présence d'un obstacle détecté
                self.etat = Robot_etat.ROTATION
                #self.get_logger().info('Transition à l\'état 3 : rotation pour éviter l\'obstacle')            
            
        elif self.etat == Robot_etat.ROTATION: 
            if self.direction == 1: 
                if (abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 0) or (
            abs(self.yaw - self.yawav) <= self.condition_corrige and abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 4.71):
                    self.etat = Robot_etat.AVANCEMENT
                    #self.get_logger().info(f'Retour à l\'état 2 : rotation terminée, différence des yaw : {abs(self.yaw - self.yawav)}')
                                        
            elif self.direction == -1:
                if (abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 0) or (
            abs(self.yaw - self.yawav) >= self.condition_corrige and abs(self.yaw - self.yawav) >= 1.57 and self.condition_corrige == 4.71):
                    self.etat = Robot_etat.AVANCEMENT
                    #self.get_logger().info(f'Retour à l\'état 2 : rotation terminée, différence des yaw : {abs(self.yaw - self.yawav)}')                    

        longueur = abs(self.sup - self.inf) #difference des distances supérieur et inférieur
        vitesse = self.map_range(longueur, self.detection_min, self.detection_max, self.vitesse_min, self.vitesse_max) #mapping de la différence des distances pour choisir la vitesse angulaire

        # Machine d'état
               
        if self.etat == Robot_etat.DEMARRAGE: #avancement linéaire du robot jusqu'à la détection d'un obstacle
            msg.linear.x = 0.1
            self.etatav = 0.0
            #self.get_logger().info("Demarrage du robot\n")
        elif self.etat == Robot_etat.ROTATION_DEMARRAGE: #Rotation jusqu'à ce que le robot soit parrallèle au mur
            msg.linear.x = 0.0
            msg.angular.z = -0.2
            self.etatav = Robot_etat.ROTATION_DEMARRAGE
            #self.get_logger().info("Rotation de démarrage\n")
        elif self.etat == Robot_etat.AVANCEMENT: #avancement du robot de manière parrallèle au mur
            if self.sup <= self.inf - self.tolerance:  # correction vers la droite
                msg.linear.x = 0.1*self.direction
                msg.angular.z = -abs(vitesse)
                #self.get_logger().info("Je corrige vers la droite\n")
            elif self.sup >= self.inf + self.tolerance: #correction vers la gauche
                msg.linear.x = 0.1*self.direction
                msg.angular.z = abs(vitesse)
                #self.get_logger().info("Je corrige vers la gauche\n")
            else:                                       #avancement du robot
                msg.linear.x = 0.1*self.direction
                msg.angular.z = 0.0
                #self.get_logger().info("Avancement du robot\n")
          
            self.yawav = self.yaw
            self.etatav = Robot_etat.AVANCEMENT

        elif self.etat == Robot_etat.ROTATION: #rotation de 90° du robot
            msg.linear.x = 0.0
            msg.angular.z = -0.4*self.direction
            #self.get_logger().info("Rotation\n")
            self.etatav = Robot_etat.ROTATION
            
        elif self.etat == Robot_etat.ARRET: #arret du robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0  
               
        # Publication du message sur le topic
        self.publisher_.publish(msg)


    def odom_callback(self, msg):
        quaternion = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(quaternion) #conversion du lacet, tangage et roulis en radian 
        #self.get_logger().info('Yaw: "%s"\n' % str(self.yaw))

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
        #mapping d'une valeur
        value = max(min(value, from_max), from_min)
        return to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)

    def custom_service_callback(self, request, response): #fonction callback du serveur 

        # creation d'un message twist
        msg = Twist()
        #enregistrement des données envoyé par le client
        self.mode = request.mode
        self.vitesse_max = request.vitesse  
        self.degre_detection_rotation = request.detect_rot  
        self.degre_detection_direction = request.detect_dir  
        self.tolerance = request.tolerance    
                    
        # Message publication
        self.publisher_.publish(msg)
        #self.get_logger().info(f'mode:  {request.mode} vitesse: {request.vitesse} degre_rot: {request.detect_rot} degre_dir: {request.detect_dir} tolerance: {request.tolerance }')
        
        # envoi de la response
        response.success = True
        return response   
        
        
def main(args=None):
    # Initialisation de la communication ROS
    rclpy.init(args=args)

    # Création du noeud
    robot_node = RobotNode()

    # Exécution du noeud en attente d'événements
    rclpy.spin(robot_node)
    

    # Destruction du noeud après arrêt
    robot_node.destroy_node()

    # Fermeture de la communication ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()