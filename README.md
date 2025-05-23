# Sistema-hardware-in-the-loop-para-un-Robot-manipulador-serial-RR

![Image_Alt](https://github.com/ChivaCable/Dise-o-mecatr-nico-Dise-o-de-un-sistema-hardware-in-the-loop-para-un-Robot-manipulador-serial-RR/blob/f272a01cd7a706f6146c9a469f38829558094698/Estructura/Piezas/ensamble%20final.png)

# Gz y Rviz

https://github.com/user-attachments/assets/f8c596f3-bc3e-4921-8bc3-12ca6fd54891

# Espacio de trabajo

En este espacio de trabajo usamos ROS 2 y es una estructura de carpetas donde se organizan y construyen los paquetes del sistema robótico. Para trabajar en el espacio de trabajo se debe seguir estos pasos.

# Construir el espacio de trabajo


```bash
cd ros-ws  
colcon build
```

# Construir el espacio de trabajo
Es necesario cargar ROS2 al sistema y cargar el espacio de trabajo personalizado cada que se realicen cambios.

```bash
source install/setup.bash
source /opt/ros/jazzy/setup.bash
```
# Ejecutar lanzador
El lanzador es el codigo que permite iniciar los multiples nodos que tiene el robot y las configuraciones del mismo de manera simultanea, de manera que con solo un comando se ejecuta el sistema

```bash
ros2 launch my_robot_planar gazebo_launch.py
```
# Topicos
Gracias a los tópicos en ROS 2, podemos enviar comandos para mover el robot de forma sencilla y eficiente. Esto permite que diferentes nodos se comuniquen y coordinen sus acciones en tiempo real.

# Publicar topicos
En una nueva terminal carga ROS 2 del sistema 
```bash
source /opt/ros/jazzy/setup.bash
```
Lo siguiente es publicar los topicos en la terminal
Topicos de Gz (usa radianes para mover joint1 y joint2)
```bash
ros2 topic pub /topic_1 std_msgs/msg/Float64 "{data: 1.5}" --once
ros2 topic pub /topic_2 std_msgs/msg/Float64 "{data: 1.5}" --once
```
Topicos del Arduino (usa coordenadas)
```bash
ros2 topic pub /position_command geometry_msgs/msg/Point "{x: -7.97, y: 8.29, z: 0.0}" -1
```

![Image_Alt]([https://github.com/ChivaCable/Dise-o-mecatr-nico-Dise-o-de-un-sistema-hardware-in-the-loop-para-un-Robot-manipulador-serial-RR/blob/f272a01cd7a706f6146c9a469f38829558094698/Estructura/Piezas/ensamble%20final.png](https://github.com/ChivaCable/Dise-o-mecatr-nico-Dise-o-de-un-sistema-hardware-in-the-loop-para-un-Robot-manipulador-serial-RR/blob/e35848d80f0b053c5a8befcfcf4da4d8263dbdc0/Archivos%20PCB/Archivos%20Visuales/PCB%20reprecentacion%20grafica.png)
