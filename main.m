clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Load environment
env = EnvironmentLoader();
view(90, 30);  

gripperLeftState = 'closed';
gripperRightState = 'closed';

function animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps)
    global gripperLeftState gripperRightState;
    offset = troty(-pi/2) * transl(0.05, 0, 0); 
    
    q1 = env.pr2Right.model.ikcon(homePosr);
    q2 = env.pr2Right.model.ikcon(Tbr);
    
    q3 = env.pr2Left.model.ikcon(homePosl);
    q4 = env.pr2Left.model.ikcon(Tbl);
    
    % LSPB trajectory for smooth transition
    sr = lspb(0, 1, numSteps); % Right arm blend
    sl = lspb(0, 1, numSteps); % Left arm blend
    
    qPrer = nan(numSteps, 7);
    qPrel = nan(numSteps, 7);
    
    for i = 1:numSteps
        qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
        qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
    end

    % Plot the motion between poses and animate robot with grippers
    for i = 1:numSteps
        env.pr2Right.model.animate(qPrer(i, :)); 
        env.pr2Left.model.animate(qPrel(i, :)); 
        
        T_leftEndEffector = env.pr2Left.model.fkine(qPrel(i, :)).T;
        T_rightEndEffector = env.pr2Right.model.fkine(qPrer(i, :)).T;

        env.gripperl1.model.base = T_leftEndEffector  * offset;
        env.gripperr1.model.base = T_leftEndEffector * offset;
        env.gripperl2.model.base = T_rightEndEffector * offset;
        env.gripperr2.model.base = T_rightEndEffector * offset;

        % Animate grippers based on their current state
        animateGrippers(env.gripperl1, env.gripperr1, gripperLeftState);
        animateGrippers(env.gripperl2, env.gripperr2, gripperRightState);
        
        drawnow(); % Update the figure
    end
end

function animateGrippers(gripperLeft, gripperRight, state)
    if strcmp(state, 'open')
        gripperLeft.model.animate([deg2rad(18), deg2rad(-18)]);
        gripperRight.model.animate([deg2rad(-18), deg2rad(18)]);
    else
        gripperLeft.model.animate([0, 0]);
        gripperRight.model.animate([0, 0]);
    end
end

function animateRightPR2ArmsAndGrippers(env, homePosr, Tbr, numSteps)
    global gripperRightState;
    offset = troty(-pi/2) * transl(0.05, 0, 0); 
    
    q1 = env.pr2Right.model.ikcon(homePosr);
    q2 = env.pr2Right.model.ikcon(Tbr);
    
    % LSPB trajectory for smooth transition
    sr = lspb(0, 1, numSteps); % Right arm blend
    
    qPrer = nan(numSteps, 7);
    
    for i = 1:numSteps
        qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
    end

    % Plot the motion between poses and animate robot with grippers
    for i = 1:numSteps
        env.pr2Right.model.animate(qPrer(i, :)); 
        
        T_rightEndEffector = env.pr2Right.model.fkine(qPrer(i, :)).T;

        env.gripperl2.model.base = T_rightEndEffector * offset;
        env.gripperr2.model.base = T_rightEndEffector * offset;

        animateGrippers(env.gripperl2, env.gripperr2, gripperRightState); % Assuming both grippers are closed
        
        drawnow(); % Update the figure
    end
end

function animateLeftPR2ArmsAndGrippers(env, homePosl, Tbl, numSteps)
    global gripperLeftState;
    offset = troty(-pi/2) * transl(0.05, 0, 0); 
    
    q3 = env.pr2Left.model.ikcon(homePosl);
    q4 = env.pr2Left.model.ikcon(Tbl);
    
    sl = lspb(0, 1, numSteps); % Left arm blend
    
    qPrel = nan(numSteps, 7);
    
    for i = 1:numSteps
        qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
    end

    % Plot the motion between poses and animate robot with grippers
    for i = 1:numSteps
        env.pr2Left.model.animate(qPrel(i, :)); 
        
        T_leftEndEffector = env.pr2Left.model.fkine(qPrel(i, :)).T;

        env.gripperl1.model.base = T_leftEndEffector  * offset;
        env.gripperr1.model.base = T_leftEndEffector * offset;

        animateGrippers(env.gripperl1, env.gripperr1, gripperLeftState); % Assuming both grippers are closed
        
        drawnow(); % Update the figure
    end
end

function gripperBothAnimate(env, numSteps, openOrClose)
    global gripperLeftState gripperRightState;

    % Define the joint limits
    qLeftOpen = [deg2rad(18), deg2rad(-18)]; 
    qLeftClose = [0, 0];
    qRightOpen = [deg2rad(-18), deg2rad(18)];  
    qRightClose = [0, 0];  

    if strcmp(openOrClose, 'open')
        qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
        qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
        gripperLeftState = 'open';
        gripperRightState = 'open';
    else
        qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
        qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
        gripperLeftState = 'closed';
        gripperRightState = 'closed';
    end

    % Animate the gripper opening or closing
    for i = 1:numSteps
        env.gripperl1.model.animate(qMatrixLeft(i, :));
        env.gripperr1.model.animate(qMatrixRight(i, :));
        env.gripperl2.model.animate(qMatrixLeft(i, :));
        env.gripperr2.model.animate(qMatrixRight(i, :));
        drawnow();
    end
end

function gripperLeftAnimate(env, numSteps, openOrClose)
    global gripperLeftState;

    % Define the joint limits
    qLeftOpen = [deg2rad(18), deg2rad(-18)]; 
    qLeftClose = [0, 0];
    qRightOpen = [deg2rad(-18), deg2rad(18)];  
    qRightClose = [0, 0];  

    if strcmp(openOrClose, 'open')
        qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
        qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
        gripperLeftState = 'open';
    else
        qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
        qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
        gripperLeftState = 'closed';
    end

    % Animate the gripper opening or closing
    for i = 1:numSteps
        env.gripperl1.model.animate(qMatrixLeft(i, :));
        env.gripperr1.model.animate(qMatrixRight(i, :));
        drawnow();
    end
end

function gripperRightAnimate(env, numSteps, openOrClose)
    global gripperRightState;

    % Define the joint limits
    qLeftOpen = [deg2rad(18), deg2rad(-18)]; 
    qLeftClose = [0, 0];
    qRightOpen = [deg2rad(-18), deg2rad(18)];  
    qRightClose = [0, 0];  

    if strcmp(openOrClose, 'open')
        qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
        qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
        gripperRightState = 'open';
    else
        qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
        qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
        gripperRightState = 'closed';
    end

    % Animate the gripper opening or closing
    for i = 1:numSteps
        env.gripperl2.model.animate(qMatrixLeft(i, :));
        env.gripperr2.model.animate(qMatrixRight(i, :));
        drawnow();
    end
end

function LeftGripperOpen(env, numSteps)
    gripperLeftAnimate(env, numSteps, 'open');
end

function LeftGripperClose(env, numSteps)
    gripperLeftAnimate(env, numSteps, 'close');
end

function RightGripperOpen(env, numSteps)
    gripperRightAnimate(env, numSteps, 'open');
end

function RightGripperClose(env, numSteps)
    gripperRightAnimate(env, numSteps, 'close');
end

function bothGripperOpen(env, numSteps)
    gripperBothAnimate(env, numSteps, 'open');
end

function bothGripperClose(env, numSteps)
    gripperBothAnimate(env, numSteps, 'close');
end

numSteps = 100; 
homePosr = transl(0.821, -0.440, 1);
homePosl = transl(0.821, 0, 1);

Tbr = transl(0.594, -0.863, 0.647);
Tbl = transl(0.594, 0.423, 0.647);

Tb2r = transl(0.431, -0.572, 1.256);
Tb2l = transl(0.252, 0.606, 0.894);

Tb3r = transl(0, -0.937, 1.071);
Tb3l = transl(0, 0.490, 1.083);

Tb4r = transl(0.116, -0.669, 0.334);
Tb4l = transl(0.99, 0.294, 0.286);

bothGripperClose(env, 50);
animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps);
LeftGripperOpen(env, 50);
LeftGripperClose(env, 50);
RightGripperOpen(env, 50);
RightGripperClose(env, 50);
bothGripperOpen(env, 50);
bothGripperClose(env, 50);
LeftGripperOpen(env, 50);
animatePR2ArmsAndGrippers(env, Tbr, Tb2r, Tbl, Tb2l, numSteps);
animateRightPR2ArmsAndGrippers(env, Tb2r, Tb3r, numSteps);
RightGripperOpen(env, 50);
animateLeftPR2ArmsAndGrippers(env, Tb2l, Tb3l, numSteps);
animatePR2ArmsAndGrippers(env, Tb3r, Tb4r, Tb3l, Tb4l, numSteps)
bothGripperClose(env, 50);




                 


