JJROBOTS B-ROBOT EVO 2 with ROS and Simulink
======================

This is a fork version version of the popular self Balancing Arduino robot. 
You can control your robot via Smartphone/Tablet with the Control app. The parts are 3D printed.
This fork version aims to add ROS and Simulink compatibility!

See on [JJROBOTS](http://www.jjrobots.com/b-robot-evo-2-much-more-than-a-self-balancing-robot/) the B-robot documentation, features, how it works, build manuals and store. Scientific information in the references. 

The ROS package is in underdevelopment [here](https://github.com/fredvaz/self_balancing_robot).

# Goals  <!-- New Features --> 

#### Hardware

- Arduino MEGA 2560
- Raspberry Pi 3/B+
- Mini LiDAR

#### Software

- Simulink PID Simulation
- ROS compatibility


## Dependencies

- git
- [Arduino](https://www.arduino.cc/en/Main/Software)
- [ROS](http://wiki.ros.org/kinetic/Installation)
- To Run the PID Simulation:
  - [MATLAB](https://www.mathworks.com/products/matlab.html)
  - [Simulink](https://www.mathworks.com/products/simulink.html)


## Installation 

Clone this package in your workspace

```
cd ~/your_workspace
git clone https://github.com/fredvaz/B-ROBOT_EVO2.git
```


<!-- ## Running with ROS -->
<!-- ## Creating a Map -->
<!-- ## Diagram of the software components -->


## References

- Zheng, B. G., Huang, Y. B., & Qiu, C. Y. (2014). LQR+ PID control and implementation of two-wheeled self-balancing robot. In Applied Mechanics and Materials (Vol. 590, pp. 399-406). Trans Tech Publications.

- Ding, Y., Gafford, J., & Kunio, M. (2012). Modeling, Simulation and Fabrication of a Balancing Robot. Harvard University, Massachusettes Institute of Technology, 151.
       
    
