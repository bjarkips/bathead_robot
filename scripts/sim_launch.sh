#!/bin/sh

roslaunch pioneer3at hardware.launch &\

sleep 5s

gzclient &\

sleep 15s

unbuffer gz topic -e /gazebo/default/Pioneer3AT/sonar_link/sonar_sensor_left/sonar\
	| grep --line-buffered 'range: '\
	| sed -u 's/  range: /---\ndata: /'\
	| rostopic pub -r 50 bathead/range/left std_msgs/Float64 &\

unbuffer gz topic -e /gazebo/default/Pioneer3AT/sonar_link/sonar_sensor_right/sonar\
	| grep --line-buffered 'range: '\
	| sed -u 's/  range: /---\ndata: /'\
	| rostopic pub -r 50 bathead/range/right std_msgs/Float64 &\
	
gnome-terminal\
	--geometry=70x20+0+500\
	--tab -e \
	"bash -c \"rosrun bathead_robot bathead_control_node_sim; exec bash\""
