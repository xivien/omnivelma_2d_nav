Square test: mode is in constructor in file square_test.py, path in path.py
./base.sh
roslaunch omnivelma_2d_nav square_test.launch 

Straight_test:
./base.sh
roslaunch omnivelma_2d_nav straight_test.launch 
rostopic pub --once /target std_msgs/Float32 "data: 2.0"

Turn test:
./base.sh
roslaunch omnivelma_2d_nav turn_test.launch 
rostopic pub --once /target std_msgs/Float32 "data: 2.0"