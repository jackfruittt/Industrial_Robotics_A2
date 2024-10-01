Lab Assignment 1 | Jackson Russell | 14250803

Folder Descriptions:

images: Contains images to load textures into the environment such as floor surfaces

plyFiles: Contains ply files that can be loaded into the environment via the EnvironmentLoader.m script. Some of these ply files were obtained
from the UTS ply file directory. This simplifies the path requirement to the local assignment folder so that anyone that wishes to run this assigment 
wont have issues with loading ply's

Test Videos: Contains screen recordings from me testing certain animations and functions. Some videos are also just me running the code and an error
occurs because I made mistakes 

Files:

There are 7 matlab files. The code is to be run from the main.m file, the others are classes contain functions and processes to be used in main.m, some of these run separately. 
rosbag_playback.m is for one of the extension tasks and to be run independently. 


HOW TO RUN PROGRAM

Program 1: Linear UR3 Picking Up Bricks

1. In EnvironmentLoader.m navigate to line 19 and let compEnv = 1
2. Run main.m, it will take a few seconds to load but will run automatically from start to end once loaded
3. Profile will load after the program has run, this is for one of the extension tasks

Program 2: LinearUR3 workspace plotter + volume plot

1. In EnvironmentLoader.m navigate to line 19 and let compEnv = 0
2. Place a breakpoint on line 14 in main.m, if you don't it'll still run fine it will just throw and error that doesn't matter
3. Click run to run main.m and then click step (where the run button is once you click run and only if you added the break point) to step into that line. This will load the environment
4. You should see the workspace plot with the linear UR3 loaded in an orientation define by q on line 25 on EnvironmentLoader.m and a point cloud around the Linear UR3 which indicates its workable volume
5. You can use the sliders to see how the robot moves within the plotted volume
6. If you wish to change the loaded in orientation, change q in EnvironmentLoader.m then re run the program. 