# Industrial_Robotics_A2

This project is for 2024 Industrial Robotics 41013. The code will simulate two robots, the TM5700 and PR2 in a kitchen environment to autonomously work together to prepare ingredients.

<div style="display: flex; justify-content: space-around;">
  <img src="https://onboardsolutions.com.au/wp-content/uploads/2020/01/TM5-7001-300x338.jpg" alt="TM5 Robot" width="45%">
  <img src="https://cdn.sanity.io/images/7p2whiua/production/02a2c2c3f15a300d5d41b6b8aa8e902b5bf25003-2048x1536.jpg" alt="TM5 Robot" width="45%">
</div>

# PR2
The PR2 is a 20 DOF robot featuring two 7-DOF arms to be similar to human movement and an array of sensors on the head to percceive similarly to a human. We utilise these features to have a human-like robot personal chef that can use human objects such as knives.
Our PR2 features a laser scanner capable of find the location of the ingredient placed in front of it and uses various types of motion such as Inverse Kinematics, RMRC and waypoints with cubic spline trajectory to complete its task.

# TM5700
# Prior to running any code

In MATLAB CL run: `setup`

This will configure paths for this project necessary for Omron and the PR2 models to function.

# To run main code

Either run the `main.m` file via an editor or type `main` in the MATLAB CL to run.

