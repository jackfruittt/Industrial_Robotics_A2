clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Load environment
env = EnvironmentLoader();
view(30, 20);  

function animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps)
    offset = troty(-pi/2) * transl(0.05, 0, 0); 
    
    q1 = env.pr2RightArm.model.ikcon(homePosr);
    q2 = env.pr2RightArm.model.ikcon(Tbr);
    
    q3 = env.pr2LeftArm.model.ikcon(homePosl);
    q4 = env.pr2LeftArm.model.ikcon(Tbl);
    
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
        env.pr2RightArm.model.animate(qPrer(i, :)); 
        env.pr2LeftArm.model.animate(qPrel(i, :)); 
        % This section for gripper transformation
        T_leftEndEffector = env.pr2LeftArm.model.fkine(qPrel(i, :)).T;
        T_rightEndEffector = env.pr2RightArm.model.fkine(qPrer(i, :)).T;

        env.gripperl1.model.base = T_leftEndEffector  * offset;
        env.gripperr1.model.base = T_leftEndEffector * offset;
        env.gripperl2.model.base = T_rightEndEffector * offset;
        env.gripperr2.model.base = T_rightEndEffector * offset;
        % End Section ^
        env.gripperl1.model.animate([0 0]);
        env.gripperr1.model.animate([0 0]);
        env.gripperl2.model.animate([0 0]);
        env.gripperr2.model.animate([0 0]);
        
        drawnow(); % Update the figure
    end
end

numSteps = 100; 
% Animate arm + grippers
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.594,0.423, 0.647);
homePosl = transl(0.821, 0, 1);
Tbl = transl(0.594,-0.863, 0.647);

animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps);

function animatePR2Base(env, baseStartPos, baseEndPos, numSteps)
    offset = troty(-pi/2) * transl(0.05, 0, 0);

    q1 = env.pr2Base.model.ikcon(baseStartPos);
    q2 = env.pr2Base.model.ikcon(baseEndPos);

    sb = lspb(0, 1, numSteps);

    qMatrix = zeros(numSteps, 3);

    for i = 1:numSteps
        qMatrix(i,:) = (1 - sb(i)) * q1 + sb(i) * q2; 
    end

    for i = 1:numSteps
        
        % Shifts the arm base up or down relative to the base movement
        env.pr2LeftArm.model.base = env.pr2Base.model.base.T * transl(-0.1, 0.18, 0.2+qMatrix(i,1));
        env.pr2RightArm.model.base = env.pr2Base.model.base.T * transl(-0.1, -0.18, 0.2+qMatrix(i,1));

        % Animate the base with the qmatrix
        env.pr2Base.model.animate(qMatrix(i,:));
        qLeft = env.pr2LeftArm.model.getpos();
        qRight = env.pr2RightArm.model.getpos();

        % Include Gripper movements with end effector base movements
        leftEndEffectorTr = env.pr2LeftArm.model.fkine(qLeft).T;
        rightEndEffectorTr = env.pr2RightArm.model.fkine(qRight).T;

        env.gripperl1.model.base = leftEndEffectorTr * offset;
        env.gripperr1.model.base = leftEndEffectorTr * offset;
        env.gripperl2.model.base = rightEndEffectorTr * offset;
        env.gripperr2.model.base = rightEndEffectorTr * offset;

        qLeftGripper1 = env.gripperl1.model.getpos();
        qRightGripper1 = env.gripperr1.model.getpos();
        qLeftGripper2 = env.gripperl2.model.getpos();
        qRightGripper2 = env.gripperr2.model.getpos();
        
        env.pr2LeftArm.model.animate(qLeft);
        env.pr2RightArm.model.animate(qRight);
        env.gripperl1.model.animate(qLeftGripper1);
        env.gripperr1.model.animate(qRightGripper1);
        env.gripperl2.model.animate(qLeftGripper2);
        env.gripperr2.model.animate(qRightGripper2);
        
        drawnow();
    end
end

% Torso Go Up
qStart = [0 0 0];
startTr = env.pr2Base.model.fkine(qStart);

qEnd = [-0.3 0 0];
endTr = env.pr2Base.model.fkine(qEnd);

animatePR2Base(env, startTr, endTr, numSteps)

% Torso Go Down
qStart1 = [-0.3 0 0];
qEnd1 = [0 0 0];

startTr1 = env.pr2Base.model.fkine(qStart1);
endTr1 = env.pr2Base.model.fkine(qEnd1);

animatePR2Base(env, startTr1, endTr1, numSteps)

hold off;




                 


