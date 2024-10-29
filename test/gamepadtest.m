clc; clf;
grid on; hold on;
axis([-1.9 3.5 -2.5 2.5 0 2.3]);

view(120, 30);
%view(90,10);

env = EnvironmentLoader();
robot = robotControl(env);

if isempty(eStop)
    eStop = serialport('COM4', 9600);
end 
global TM5GripperState;
TM5GripperState = 'closed';

% For controlling robot using HID game controller
gamepad = env.teensyGamepad;
kV = 0.2;
kW = 1.0;
duration = 300;
dt = 0.1;
lambda = 0.1;
disp('environment loaded, testing gamepad')

% Change this value to swap between control modes
controlPreference = 0;

if controlPreference == 1
    disp('Selected Control:  EndEffector')
    savedQ = robot.gamepadEndEffectorFrameControl(gamepad, lambda, kV, kW, duration, dt, 'TM5', eStop);
    disp('Control session complete. Saved configurations:');
    disp(savedQ);
else
    disp('Selected Control:  WorldFrame')
    savedQ = robot.gamepadWorldFrameControl(gamepad, lambda, kV, kW, duration, dt, 'TM5', eStop);
    disp('Control session complete. Saved configurations:');
    disp(savedQ);
end

