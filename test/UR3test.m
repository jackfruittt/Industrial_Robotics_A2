clear; clc;
load("qCombined.mat");  % Combined IBVS + Banana movement trajectory

baseTr = trotz(-pi/2); 

robot = UR3(baseTr);  % Initialize the robot
robot.model.delay = 0;

% Optional: Define an offset configuration (if needed)
offsetQ = [pi/2 -pi/2 0 -pi/2 pi 0];  % Adjust as required

% Apply the offset to all configurations
qWithOffset = qCombined + offsetQ;

% Start the robot in the first configuration of the modified trajectory
robot.model.animate(qWithOffset(1, :));
drawnow();  % Ensure the plot updates before animation starts

pause(3)

% Animate through the entire trajectory
for i = 1:size(qWithOffset, 1)
    robot.model.animate(qWithOffset(i, :));  % Animate with offset configuration
    fprintf('i: %d ', i);
    fprintf('q: %.4f %.4f %.4f %.4f %.4f %.4f\n', qWithOffset(i,:));
    if(i > 0) % q1
        disp('From Initial to Scanning Pose')
    end
    if(i > 50) % q2
        disp('IBVS')
    end
    if(i > 127) % q3
        disp('Rotating End Effector 90d')
    end
    if(i > 177) % q4
        disp('Go down to banana')
    end
    if(i > 227) % q5
        disp('Go up with banana')
    end
    if(i > 277) % q6
        disp('Go down to chopping board')
    end
    if(i > 327) % q7
        disp('Go back to default')
    end
    pause(0.1);  % Control animation speed
    drawnow();
end
