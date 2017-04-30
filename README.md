## qr_detector
QR codes detector based on zbar library (http://zbar.sourceforge.net). The detector is dedicated to ROS framework.

Subscribes:
- **/image** (sensor_msgs/Image) - the topic with RGB images which contains QR codes.

Publishes:
- **/qr_codes** (std_msgs/String) - message from each detected QR code is published as a string.


The package was tested on ROS Kinetic but it should work too with older ROS versions if they used gcc which supports c++11.
