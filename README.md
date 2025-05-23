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
