## qr_detector
QR codes detector based on zbar library (http://zbar.sourceforge.net). The detector is dedicated to ROS systems.

Command to install zbar library on Ubuntu
`sudo apt install libzbar-dev`

Subscribes:
- **/image** (sensor_msgs/Image) - the topic with RGB images which contains QR codes.

Publishes:
- **/qr_codes** (std_msgs/String) - message from each detected QR code is published as a string.


The package was tested on ROS Kinetic and Melodic but it will be working also with older ROS versions (as long as they supports C++11)
