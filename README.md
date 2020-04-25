# omnivelma_2d_nav

Installation: 
download folder install_scripts with setup.bash and rosinstall file.
In the folder:
chmod u+x setup.bash
./setup.bash [root of new workspace] [FULL path to the velmwheel rosinstall file]

After installing go to omnivelma_2d_nav
Possible use cases:
Firstly run "source init.sh" 
1. Navigation on a static global map:
    - EKF filter for global localization
    - EKF filter for odometry
    - Move base package for Navigation

usage: ./nav_static_map.sh

2. Navigation on a dynamic map and map building:
    - SLAM from Hector_mapping package used with global EKF localization filter
    - EKF filter for odometry
    - Move base package for Navigation 

usage: 
        ./nav_dynamic_map.sh
optional: 
        Second terminal: rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
Saving map: 
        rosrun map_server map_saver -f PATH

3. Running base simulation without navigation or SLAM

usage: 
        ./base.sh
