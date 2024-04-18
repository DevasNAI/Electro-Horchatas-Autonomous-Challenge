#!/usr/bin/env python3

## @file
#  @brief Este script controla el movimiento de un robot diferencial en ROS.
#
#  El script utiliza mensajes Twist para controlar la velocidad lineal y angular
#  del robot y publica las velocidades de las ruedas izquierda y derecha para
#  mover el robot correctamente.

# Importar todas las librerías necesarias
import rospy
import numpy as np 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped

# Definir variables

## @var wl
#  Velocidad de la rueda izquierda (en rad/s).
wl = Float32()

## @var wr
#  Velocidad de la rueda derecha (en rad/s).
wr = Float32()

## @var vel
#  Velocidad lineal y angular.
vel = Twist()

## @var pose
#  Pose del robot en el espacio.
pose = PoseStamped()

## @var vel_x
#  Velocidad lineal del robot (en m/s).
vel_x = 0.0

## @var ang_z
#  Velocidad angular del robot (en rad/s).
ang_z = 0.0

## @var l
#  Distancia entre las ruedas izquierda y derecha del robot (en metros).
l = 0.19

## @var r
#  Radio de las ruedas del robot (en metros).
r = 0.05

# Variables de tiempo

## @var lastTime
#  Tiempo de la última actualización.
lastTime = 0.0

## @var dt
#  Diferencia de tiempo desde la última actualización.
dt = 0.0

## Función para envolver el ángulo theta dentro del rango [-π, π].
#
#  @param theta Ángulo a envolver.
#  @return Ángulo envuelto dentro del rango [-π, π].
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

# Definir funciones de callback

## Callback para suscribirse al comando de velocidad del robot.
#
#  @param msg Mensaje Twist que contiene la velocidad lineal y angular del robot.
#
def sub_vel(msg):
    global vel_x, ang_z
    cmd_vel = msg
    vel_x = cmd_vel.linear.x
    ang_z = cmd_vel.angular.z

# Código principal del sistema
if __name__ == "__main__":
    # Inicializar el nodo ROS
    rospy.init_node("math_operations")
    rate = rospy.Rate(100) # 100 Hz

    # Publicador

    ## @var pub_r
    #  Publicador para la velocidad de la rueda derecha.
    pub_r = rospy.Publisher("/wr", Float32, queue_size=1)

    ## @var pub_l
    #  Publicador para la velocidad de la rueda izquierda.
    pub_l = rospy.Publisher("/wl", Float32, queue_size=1)

    ## @var pose_pub
    #  Publicador para la pose del robot.
    pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=1)

    # Suscriptor

    ## @var sub_cmd_vel
    #  Suscriptor para el comando de velocidad del robot.
    rospy.Subscriber("/cmd_vel", Twist, sub_vel)

    lastTime = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        currentTime = rospy.get_time()
        
        # Calcular la diferencia de tiempo
        dt = currentTime - lastTime

        # Calcular la velocidad y velocidad angular
        vel_x += vel_x * dt
        ang_z += ang_z * dt

        # Calcular wr y wl
        wr = ( ((ang_z * l) / r) + ( (2*vel_x) / r ) ) / 2
        wl = ( (2 * vel_x) / r ) - wr

        # Publicar los temas wr y wl
        pub_r.publish(wr)
        pub_l.publish(wl)

        # Calcular y publicar la pose
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x += vel_x*dt*np.cos(pose.pose.orientation.z)
        pose.pose.position.y += vel_x*dt*np.sin(pose.pose.orientation.z)
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z += ang_z*dt
        pose.pose.orientation.z = wrap_to_Pi(ang_z)
        pose.pose.orientation.w = 0
        pose_pub.publish(pose)

        # Guardar el tiempo
        lastTime = currentTime
        rate.sleep()
