# Tproject
Second Semester Masters mini Project

Modern Embedded systems require complex solutions that needs a high performance Linux , GUI interfaces, GPUs and Manipulations associated with it.
Along with this, it may also require hard-realitme application such as capturing and reacting on the sensor data. In these type of scenario, A mulitcore devices containing different microcontroller architecture such as Cortex A7 and Cortex M4 could prove beneficial.One such application is implemented as a demo in this project.

In this project, the data from the a Lidar sensor is obtained and the data is plotted on a Display.
The STM32MP1 device is used in this project that consists of display that is to display the data obtained from the Lidar sensor.

First a firmware is developed capturing the data from the RP Lidar. Second, An custom embedded linux is created using Yocto Project with Qt meta layers. The data obtained from Cortex M4 is Plotted on to the graph using Qt.
The data between the two architecture is shared using RPMsg inter-processor mechanism


