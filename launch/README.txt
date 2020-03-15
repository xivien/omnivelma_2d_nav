Launch files:
/additional - additional packages for testing 

/ext_packages - launch files for external packages
    - amcl_omni - configure AMCL 
    - ekf_map - Extended Kalman Filter for global localization (fuse AMCL, odometry, IMU)
    - ekf_odom - Extended Kalman Filter for fusing odom, IMU and Pose from Laser scan
    - laser_scan_matcher - Pose in odom frame from laser scans (another source of odometry)
    - laser_scan_multi_merger - merge 2 LiDARs into one scan
    - rtabmap - 3D mapping

gazebo - bash script: launching gazebo simulation of a robot
move_base_gmapping - navigation with dynamic map (NO EKF map)
move_base - navigation with statci map
omnivelma_base - launching base external packages needed by all applications
omnivelma - launching gazebo script (needed to launch everything from one file)
