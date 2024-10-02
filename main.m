clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Load environment
env = EnvironmentLoader();
view(30, 20);  

% Define the function to animate PR2 arms and grippers
function animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps)

    % Compute joint angles for home and target poses (Right Arm)
    q1 = env.pr2Right.model.ikcon(homePosr);
    q2 = env.pr2Right.model.ikcon(Tbr);
    
    % Compute joint angles for home and target poses (Left Arm)
    q3 = env.pr2Left.model.ikcon(homePosl);
    q4 = env.pr2Left.model.ikcon(Tbl);
    
    % LSPB trajectory for smooth transition
    sr = lspb(0, 1, numSteps); % Right arm blend
    sl = lspb(0, 1, numSteps); % Left arm blend
    
    % Initialize arrays to store trajectory joint values
    qPrer = nan(numSteps, 7);
    qPrel = nan(numSteps, 7);
    
    % Generate joint trajectories
    for i = 1:numSteps
        qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
        qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
    end
    
    % Initialize the grippers
    gripperl1 = PR2LeftGripper();
    gripperr1 = PR2RightGripper();
    gripperl2 = PR2LeftGripper();
    gripperr2 = PR2RightGripper();
    offset = troty(-pi/2) * transl(0.05, 0, 0); % Offset for the gripper position

    % Plot the motion between poses and animate grippers
    for i = 1:numSteps
        % Animate the PR2 arms
        env.pr2Right.model.animate(qPrer(i, :)); 
        env.pr2Left.model.animate(qPrel(i, :)); 
        
        % Update the end-effector positions for the grippers
        T_leftEndEffector = env.pr2Left.model.fkine(qPrel(i, :)).T;
        T_rightEndEffector = env.pr2Right.model.fkine(qPrer(i, :)).T;

        % Set the gripper bases to follow the end effectors
        gripperl1.model.base = T_leftEndEffector  * offset;
        gripperr1.model.base = T_leftEndEffector * offset;
        gripperl2.model.base = T_rightEndEffector * offset;
        gripperr2.model.base = T_rightEndEffector * offset;

        % Animate the grippers with neutral joint angles [0 0]
        gripperl1.model.animate([0 0]);
        gripperr1.model.animate([0 0]);
        gripperl2.model.animate([0 0]);
        gripperr2.model.animate([0 0]);
        
        drawnow(); % Update the figure
    end
end

numSteps = 100; 
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.594, -0.863, 0.647);
homePosl = transl(0.821, 0, 1);
Tbl = transl(0.594, 0.423, 0.647);

% Call the function to animate the PR2 arms and grippers
animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps);

hold off;




                 


