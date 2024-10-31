# Industrial_Robotics_A2

This project is for 2024 Industrial Robotics 41013. The code will simulate two robots, Wii Wii and Mii, the PR2 and TM5700 cooking companions that autonomously work together in a kitchen envrionment.

<div style="display: flex; justify-content: space-around;">
  <img src="https://onboardsolutions.com.au/wp-content/uploads/2020/01/TM5-7001-300x338.jpg" alt="TM5 Robot" width="45%">
  <img src="https://cdn.sanity.io/images/7p2whiua/production/02a2c2c3f15a300d5d41b6b8aa8e902b5bf25003-2048x1536.jpg" alt="TM5 Robot" width="45%">
</div>

# Wii Wii (PR2)
The PR2 is a 20 DOF robot featuring two 7-DOF arms to be similar to human movement and an array of sensors on the head to percceive similarly to a human. We utilise these features to have a human-like robot personal chef that can use human objects such as knives.
Our PR2 features a laser scanner capable of find the location of the ingredient placed in front of it and uses various types of motion such as Inverse Kinematics, RMRC and waypoints with cubic spline trajectory to complete its task. In our simulation we utilise 15 DOF, the spine and two 7 DOF arms.

# Mii (TM5700)
The TM5700 is a 6 DOF arm that can do basic tasks compared to the PR2. It uses Visual Servoing to find the banana to do pick it up and place it on the cutting board for the PR2. It utilises RMRC and Inverse Kinematics for motion.

# Prior to running any code

- Have MATLAB 2024 or higher installed
  - Simulink and Simulink 3D animation will be needed to run the physical game controller   
- Intall Peter Corkes Robotics Toolbox into your MATLAB System
- Clone this repository into your workspace, ensure you are on branch `main` as this is the final and working code

Once cloned make sure your are in the  in the same dierectory as the `main.m` script and in the MATLAB CL run: `setup` or go the the `setup.m` file and click run in MATLAB (or VSCode if you have the MATLAB extension)
If this is not done the code will not run as package directories are used for the robots and the toolbox will also be pathed to the code workspace


# To run main code

Either run the `main.m` file via an editor or type `main` in the MATLAB CL to run.

