# Localization
  The Filters are implemented in ROS system and the filters that use GPS measurements, it's subscribing the publisher               "convert_lon_lat_node.py"

## Converting Longitude and Latitude to meters
  This program converts the GPS measurements of longitude and latitude in meters x,y

## Kalman Filter
  Simple Kalman Filter for GPS
  - kf_gps.py:
    - GPS
  
## Extended Kalman Filter
  Filter that use the Sensor Fusion to estimated the location of a robot
  - ekf_sensor.py:
    - GPS
    - IMU
    - Odometry
    
   
