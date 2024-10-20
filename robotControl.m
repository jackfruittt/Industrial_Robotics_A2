classdef robotControl
    properties
        
        env
        
    end
    
    methods
        function robot = robotControl(environment)
            robot.env = environment;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PR2 FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function movePR2ArmsRMRC(robot, leftStartTr, leftEndTr, rightStartTr, rightEndTr, steps, deltaTime, lambda, epsilon, eStop)

            global PR2GripperLeftState PR2GripperRightState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            % Use ikcon to get initial joint angles from given pose
            qStartLeft = robot.env.pr2LeftArm.model.ikcon(leftStartTr);
            qStartRight = robot.env.pr2RightArm.model.ikcon(rightStartTr);
            
            qMatrixLeft = zeros(steps, 7);
            qMatrixRight = zeros(steps, 7);

            % Set first joint angles with the calculated starting ones
            qMatrixLeft(1,:) = qStartLeft;
            qMatrixRight(1,:) = qStartRight;

            % Get cartesian traj from start to end given n steps
            leftArmCTraj = ctraj(leftStartTr, leftEndTr, steps);
            rightArmCTraj = ctraj(rightStartTr, rightEndTr, steps);

            % RMRC loop 
            for i = 1:steps-1
                % Get current joint angles
                qLeft = qMatrixLeft(i, :);
                qRight = qMatrixRight(i, :);

                % Get current pose using fkine
                tLeft = robot.env.pr2LeftArm.model.fkine(qLeft).T;
                tRight = robot.env.pr2RightArm.model.fkine(qRight).T;

                % Compute cartesian velocity
                vLeft = tr2delta(tLeft, leftArmCTraj(:,:,i+1)) / deltaTime;
                vRight = tr2delta(tRight, rightArmCTraj(:,:,i+1)) / deltaTime;

                % Compute jacobian at current joint configuration
                jLeft = robot.env.pr2LeftArm.model.jacob0(qLeft);
                jRight = robot.env.pr2RightArm.model.jacob0(qRight);

                % Handle singularities through Damped Least Squares (DLS) for left and right
                if abs(det(jLeft * jLeft')) < epsilon
                    qLeftDot = (jLeft' / (jLeft * jLeft' + lambda^2 * eye(6))) * vLeft;
                else
                    qLeftDot = jLeft \ vLeft;
                end

                if abs(det(jRight * jRight')) < epsilon
                    qRightDot = (jRight' / (jRight * jRight' + lambda^2 * eye(6))) * vRight;
                else
                    qRightDot = jRight \ vRight;
                end

                % Euler integration to update joint angles for both arms
                qMatrixLeft(i+1, :) = qLeft + qLeftDot' * deltaTime;
                qMatrixRight(i+1, :) = qRight + qRightDot' * deltaTime;
            end

            % Animate the robot movement
            for i = 1:steps
                robot.checkPause(eStop);
                robot.env.pr2LeftArm.model.animate(qMatrixLeft(i,:));
                robot.env.pr2RightArm.model.animate(qMatrixRight(i,:));

                T_leftEndEffector = robot.env.pr2LeftArm.model.fkine(qMatrixLeft(i, :)).T;
                T_rightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrixRight(i, :)).T;
                
                robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                % Animate grippers based on their current state
                robot.animatePR2Grippers(robot.env.gripperl1, robot.env.gripperr1, PR2GripperLeftState);
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);

                drawnow();
            end

        end

        function animatePR2Base(robot, baseStartPos, baseEndPos, numSteps, eStop)
            offset = troty(-pi/2) * transl(0.05, 0, 0);

            q1 = robot.env.pr2Base.model.ikcon(baseStartPos);
            q2 = robot.env.pr2Base.model.ikcon(baseEndPos);

            sb = lspb(0, 1, numSteps);

            qMatrix = zeros(numSteps, 3);

            for i = 1:numSteps
                qMatrix(i,:) = (1 - sb(i)) * q1 + sb(i) * q2; 
            end

            for i = 1:numSteps
                robot.checkPause(eStop);

                robot.env.pr2LeftArm.model.base = robot.env.pr2Base.model.base.T * transl(-0.1, -0.18, 0.2+qMatrix(i,1));
                robot.env.pr2RightArm.model.base = robot.env.pr2Base.model.base.T * transl(-0.1, 0.18, 0.2+qMatrix(i,1));

                robot.env.pr2Base.model.animate(qMatrix(i,:));

                qLeft = robot.env.pr2LeftArm.model.getpos();
                qRight = robot.env.pr2RightArm.model.getpos();

                leftEndEffectorTr = robot.env.pr2LeftArm.model.fkine(qLeft).T;
                rightEndEffectorTr = robot.env.pr2RightArm.model.fkine(qRight).T;

                robot.env.gripperl1.model.base = leftEndEffectorTr * offset;
                robot.env.gripperr1.model.base = leftEndEffectorTr * offset;
                robot.env.gripperl2.model.base = rightEndEffectorTr * offset;
                robot.env.gripperr2.model.base = rightEndEffectorTr * offset;

                qLeftGripper1 = robot.env.gripperl1.model.getpos();
                qRightGripper1 = robot.env.gripperr1.model.getpos();
                qLeftGripper2 = robot.env.gripperl2.model.getpos();
                qRightGripper2 = robot.env.gripperr2.model.getpos();

                robot.env.pr2LeftArm.model.animate(qLeft);
                robot.env.pr2RightArm.model.animate(qRight);
                robot.env.gripperl1.model.animate(qLeftGripper1);
                robot.env.gripperr1.model.animate(qRightGripper1);
                robot.env.gripperl2.model.animate(qLeftGripper2);
                robot.env.gripperr2.model.animate(qRightGripper2);

                drawnow();
            end
        end


        function animatePR2ArmsAndGrippers(robot, rightStartPos, rightEndPos, leftStartPos, leftEndPos, numSteps, eStop)
            global PR2GripperLeftState PR2GripperRightState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            q1 = robot.env.pr2RightArm.model.ikcon(rightStartPos);
            q2 = robot.env.pr2RightArm.model.ikcon(rightEndPos);
            
            q3 = robot.env.pr2LeftArm.model.ikcon(leftStartPos);
            q4 = robot.env.pr2LeftArm.model.ikcon(leftEndPos);
            
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
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.pr2RightArm.model.animate(qPrer(i, :));
                robot.env.pr2LeftArm.model.animate(qPrel(i, :));
                
                T_leftEndEffector = robot.env.pr2LeftArm.model.fkine(qPrel(i, :)).T;
                T_rightEndEffector = robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T;
                
                robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                % Animate grippers based on their current state
                robot.animatePR2Grippers(robot.env.gripperl1, robot.env.gripperr1, PR2GripperLeftState);
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                
                drawnow(); % Update the figure
            end
        end
        
        function animatePR2Grippers(~, gripperLeft, gripperRight, state)
            if strcmp(state, 'open')
                gripperLeft.model.animate([deg2rad(18), deg2rad(-18)]);
                gripperRight.model.animate([deg2rad(-18), deg2rad(18)]);
            else
                gripperLeft.model.animate([0, 0]);
                gripperRight.model.animate([0, 0]);
            end
        end
        
        function animatePR2RightArmsAndGrippers(robot, rightStartPos, rightEndPos, numSteps, eStop)
            global PR2GripperRightState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            q1 = robot.env.pr2RightArm.model.ikcon(rightStartPos);
            q2 = robot.env.pr2RightArm.model.ikcon(rightEndPos);
            
            % LSPB trajectory for smooth transition
            sr = lspb(0, 1, numSteps); % Right arm blend
            qPrer = nan(numSteps, 7);
            
            for i = 1:numSteps
                qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
            end
            
            % Plot the motion between poses and animate robot with grippers
            for i = 1:numSteps
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.pr2RightArm.model.animate(qPrer(i, :));
                
                T_rightEndEffector = robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T;
                
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                
                drawnow(); % Update the figure
            end
        end

        function animateRightPR2ArmsAndGrippersWithKnife(robot, rightStartPos, rightEndPos, numSteps, eStop)
            global gripperRightState;
            
            gripperOffset = troty(-pi/2) * transl(0.05, 0, 0); 
            
            q1 = robot.env.pr2RightArm.model.ikcon(rightStartPos);
            q2 = robot.env.pr2RightArm.model.ikcon(rightEndPos);
            
            % LSPB trajectory for smooth transition
            sr = lspb(0, 1, numSteps); % Right arm blend
            qPrer = nan(numSteps, 7);
            
            for i = 1:numSteps
                qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
            end
        
            % Plot the motion between poses and animate robot with grippers
            for i = 1:numSteps
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.pr2RightArm.model.animate(qPrer(i, :)); 
                
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T;
        
                robot.env.gripperl2.model.base = tRightEndEffector * gripperOffset;
                robot.env.gripperr2.model.base = tRightEndEffector * gripperOffset;
                robot.env.pr2KnifeArm.attachToEndEffector(robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T);
        
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, gripperRightState);
                
                drawnow(); % Update the figure
            end
        end
        
        function animatePR2RightArmsAndGrippersWithWaypoints(robot, qWaypoints, numSteps, eStop)
            global PR2GripperRightState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            %Use Spline Tracjectory for smooth velocity and acceleration between waypoints
            % Time vector for the waypoints and interp
            tWaypoints = linspace(0, 1, size(qWaypoints, 1));
            tInterp = linspace(0, 1, numSteps);
            
            qMatrix = zeros(numSteps, size(qWaypoints, 2));
            
            % Spline interpolation for each joint
            for jointIdx = 1:size(qWaypoints, 2)
                % Interpolate each joint angle using cubic splines
                qMatrix(:, jointIdx) = spline(tWaypoints, qWaypoints(:, jointIdx), tInterp);
            end
            
            for i = 1:numSteps
                robot.checkPause(eStop);
                
                robot.env.pr2RightArm.model.animate(qMatrix(i, :));
                
                T_rightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrix(i, :)).T;
                
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                
                drawnow();
            end
        end
        
        
        function animatePR2LeftArmsAndGrippers(robot, leftStartPos, leftEndPos, numSteps, eStop)
            global PR2GripperLeftState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            q3 = robot.env.pr2LeftArm.model.ikcon(leftStartPos);
            q4 = robot.env.pr2LeftArm.model.ikcon(leftEndPos);
            
            sl = lspb(0, 1, numSteps); % Left arm blend
            qPrel = nan(numSteps, 7);
            
            for i = 1:numSteps
                qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
            end
            
            % Plot the motion between poses and animate robot with grippers
            for i = 1:numSteps
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.pr2LeftArm.model.animate(qPrel(i, :));
                
                T_leftEndEffector = robot.env.pr2LeftArm.model.fkine(qPrel(i, :)).T;
                
                robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl1, robot.env.gripperr1, PR2GripperLeftState);
                
                drawnow(); % Update the figure
            end
        end
        
        function animatePR2BothGrippers(robot, numSteps, openOrClose)
            global PR2GripperLeftState PR2GripperRightState;
            %robot.PR2GripperLeftState;
            %robot.PR2GripperRightState;
            
            % Define the joint limits
            qLeftOpen = [deg2rad(18), deg2rad(-18)];
            qLeftClose = [0, 0];
            qRightOpen = [deg2rad(-18), deg2rad(18)];
            qRightClose = [0, 0];
            
            if strcmp(openOrClose, 'open')
                qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
                qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
                PR2GripperLeftState = 'open';
                PR2GripperRightState = 'open';
            else
                qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
                qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
                PR2GripperLeftState = 'closed';
                PR2GripperRightState = 'closed';
            end
            
            % Animate the gripper opening or closing
            for i = 1:numSteps
                robot.env.gripperl1.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr1.model.animate(qMatrixRight(i, :));
                robot.env.gripperl2.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr2.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end
        
        function animatePR2LeftGripper(robot, numSteps, openOrClose)
            global PR2GripperLeftState;
            %robot.PR2GripperLeftState;
            
            % Define the joint limits
            qLeftOpen = [deg2rad(18), deg2rad(-18)];
            qLeftClose = [0, 0];
            qRightOpen = [deg2rad(-18), deg2rad(18)];
            qRightClose = [0, 0];
            
            if strcmp(openOrClose, 'open')
                qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
                qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
                PR2GripperLeftState = 'open';
            else
                qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
                qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
                PR2GripperLeftState = 'closed';
            end
            
            % Animate the gripper opening or closing
            for i = 1:numSteps
                robot.env.gripperl1.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr1.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end
        
        function animatePR2RightGripper(robot, numSteps, openOrClose)
            global PR2GripperRightState;
            %robot.PR2GripperRightState;
            
            % Define the joint limits
            qLeftOpen = [deg2rad(18), deg2rad(-18)];
            qLeftClose = [0, 0];
            qRightOpen = [deg2rad(-18), deg2rad(18)];
            qRightClose = [0, 0];
            
            if strcmp(openOrClose, 'open')
                qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
                qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
                PR2GripperRightState = 'open';
            else
                qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
                qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
                PR2GripperRightState = 'closed';
            end
            
            % Animate the gripper opening or closing
            for i = 1:numSteps
                robot.env.gripperl2.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr2.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end

        function animatePR2RightGripperforKnife(robot, numSteps, openOrClose)
            global PR2GripperRightState;
            %robot.PR2GripperRightState;
            
            % Define the joint limits
            qLeftOpen = [deg2rad(18), deg2rad(-18)];
            qLeftClose = [deg2rad(6), deg2rad(-6)];
            qRightOpen = [deg2rad(-18), deg2rad(18)];
            qRightClose = [deg2rad(-6), deg2rad(6)]; 
            
            if strcmp(openOrClose, 'open')
                qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
                qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
                PR2GripperRightState = 'open';
            else
                qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
                qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
                PR2GripperRightState = 'closed';
            end
            
            % Animate the gripper opening or closing
            for i = 1:numSteps
                robot.env.gripperl2.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr2.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end
        
        function PR2LeftGripperOpen(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2LeftGripper(numSteps, 'open');
        end
        
        function PR2LeftGripperClose(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2LeftGripper(numSteps, 'close');
        end
        
        function PR2RightGripperOpen(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2RightGripper(numSteps, 'open');
        end
        
        function PR2RightGripperClose(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2RightGripper(numSteps, 'close');
        end

        function PR2GrabKnife(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2RightGripperforKnife(numSteps, 'close');
        end

        function PR2ReleaseKnife(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2RightGripperforKnife(numSteps, 'open');
        end
        
        function PR2BothGrippersOpen(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2BothGrippers(numSteps, 'open');
        end
        
        function PR2BothGripperClose(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.animatePR2BothGrippers(numSteps, 'close');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OMRON TM5 FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        function animateTM5(robot, startPos, endPos, numSteps, eStop)
            q1 = robot.env.tm5700.model.ikcon(startPos);
            q2 = robot.env.tm5700.model.ikcon(endPos);
            
            % LSPB trajectory for smooth transition
            sr = lspb(0, 1, numSteps); 
            
            qPrer = nan(numSteps, 6);
            
            for i = 1:numSteps
                qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
            end
            
            % Plot the motion between poses and animate robot with grippers
            for i = 1:numSteps
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.tm5700.model.animate(qPrer(i, :));
                
                %Build Gripper and Modify later
                %T_leftEndEffector = robot.env.pr2Left.model.fkine(qPrel(i, :)).T;
                %T_rightEndEffector = robot.env.pr2Right.model.fkine(qPrer(i, :)).T;
                
                %Ditto ^^
                %robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                %robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                %robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                %robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                % Animate grippers based on their current state
                %robot.animatePR2Grippers(robot.env.gripperl1, robot.env.gripperr1, PR2GripperLeftState);
                %robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                
                drawnow(); % Update the figure
            end
        end

        function animateTM5RMRC(robot, qStart, startPos, endPos, numSteps, deltaTime, epsilon, eStop)
            W = diag([1 1 1 0 0 0]);            % Weight matrix W for applying linear/angular vel influence

            m = zeros(numSteps, 1);                % Manipulability
            qMatrix = zeros(numSteps, 6);          % Joint Angles
            qDot = zeros(numSteps, 6);             % Joint Velocities
            p = zeros(3, numSteps);                % X Y Z point
            theta  = zeros(3, numSteps);           % R P Y 

            startPosRot = startPos(1:3,1:3)                                  % Extract Rotation Matrix from T1
            startPosRPY = tr2rpy(startPosRot,'deg',true,'order','xyz')' % Get roll pitch yaw using tr2rpy from rotation matrix
            P1 = startPos(1:3, 4);  % Initial XYZ point from T1
            P2 = endPos(1:3, 4);  % Final XYZ point from T2                                      

            % Initialise trajectory with initial pose, trapezoidal
            s = lspb(0, 1, numSteps);
            for i=1:numSteps
                p(:, i) = (1 - s(i)) * P1 + s(i) * P2;  % Interpolate XYZ points
                theta(:, i) = deg2rad(startPosRPY);     % Try to keep orientation the same
            end

            % Get current q configuration using T1 and qStart
            qMatrix(1,:) = robot.env.tm5700.model.ikcon(startPos,qStart);

            % RMRC Loop
            for i = 1:numSteps-1
                % Get current transform from current q configuration
                T = robot.env.tm5700.model.fkine(qMatrix(i,:)).T;

                % Get position error
                pError = p(:,i+1) - T(1:3,4);

                rD = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); % R(t+1)
                rC = T(1:3,1:3);                                    % R(t)
                skew = ((rD*rC')-eye(3))*(1/deltaTime);             % S(w) = dT^-1*((R(t+1)*R(t)'))-I
                
                % Get linear and angular velocities
                linearVelocity = pError*(1/deltaTime);
                angularVelocity = [skew(3,2); skew(1,3); skew(2,1)]; % phiDot, thetaDot, psiDot
                
                pDot = W * [linearVelocity;angularVelocity];

                % Get robot jacobian from current q configuration
                J = robot.env.tm5700.model.jacob0(qMatrix(i,:));

                % Check if manipulability is within threshold to adjust lambda value
                if sqrt(det(J*J')) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end

                % Calculate inverse jacobian
                jInv = inv(J'*J + lambda *eye(6))*J';
                qDot(i,:) = (jInv*pDot)';

                % Check if calculated q configurations are within lower and upper qlim bounds of tm5700
                for j = 1:robot.env.tm5700.model.n
                    if qMatrix(i,j) + deltaTime*qDot(i,j) < robot.env.tm5700.model.qlim(j,1)
                        qDot(i,j) = 0; % Stop if it exceeds lower bounds
                    elseif robot.env.tm5700.model.qlim(j,2) < qMatrix(i,j) + deltaTime*qDot(i,j) 
                        qDot(i,j) = 0; % Stop if it exceeds upper bounds
                    end
                end

                % Euler
                qMatrix(i+1,:) = qMatrix(i,:) + deltaTime*qDot(i,:);
            end
            
            % Animate qMatrix
            for i=1:numSteps
                robot.checkPause(eStop)
                robot.env.tm5700.model.animate(qMatrix(i,:));

                % Include gripper Here 

                drawnow();
            end
        end

        function animateTM5IBVS(robot, q0, camera, pStar, P, fps, lambda, eStop)
            robotTr = robot.env.tm5700.model.fkine(q0).T;
            robot.env.tm5700.model.animate(q0');
            drawnow;
        
            %camera.T = robotTr * cameraOffset; % Uncomment to apply offset, edit input param
            camera.T = robotTr;
            camera.plot_camera('label', 'scale', 0.05, 'frustum', true);
            plot_sphere(P, 0.03, 'b');
        
            % Project the 3D points to the image plane
            camera.clf();
            p = camera.plot(P);  % Initial projection
            camera.plot(pStar, '*');  % Desired points in the image
            camera.hold(true);
            camera.plot(P, 'pose', robotTr, 'o');  % 3D points with camera pose
            
            % Label each point in the image view
            textHandlesP = gobjects(1, size(p, 2));  % Labels for projected points
            textHandlesPStar = gobjects(1, size(pStar, 2));  % Labels for desired points
            
            % Label the initial projected points (P)
            for i = 1:size(p, 2)
                textHandlesP(i) = text(p(1, i), p(2, i), sprintf('P%d', i), ...
                    'Color', 'blue', 'FontSize', 12, 'Parent', gca(camera.figure));
            end
            
            % Label the desired points (pStar)
            for i = 1:size(pStar, 2)
                textHandlesPStar(i) = text(pStar(1, i), pStar(2, i), sprintf('p^*%d', i), ...
                    'Color', 'green', 'FontSize', 12, 'Parent', gca(camera.figure));
            end
        
            pause(2);
            camera.hold(true);
            
            % Initialize history storage for plotting results
            history = [];
            steps = 0;
            errorThreshold = [10, 1000];
            depth = mean(P(1, :));

            pause(5)
        
            while true
                robot.checkPause(eStop);
                steps = steps + 1;
                uv = camera.plot(P);
                for i = 1:size(uv, 2)
                    if isvalid(textHandlesP(i))
                        delete(textHandlesP(i));  % Delete old label
                    end
                    textHandlesP(i) = text(uv(1, i), uv(2, i), sprintf('P%d', i), ...
                                     'Color', 'blue', 'FontSize', 12, 'Parent', gca(camera.figure));
                end
        
                error = pStar - uv;
                error = error(:);
                J = camera.visjac_p(uv, depth);
        
                % Check if the error is within the acceptable range
                errorNorm = norm(error)  % Check error magnitude
                if errorNorm < errorThreshold (1)
                    disp('Error within acceptable range. Exiting...');
                    break;  % Exit the visual servoing loop if the error is below the minimum threshold
                elseif errorThreshold(2) < errorNorm  
                    disp('Error too large. Exiting...');
                    break;
                end
                
                % compute the velocity of camera in camera frame
                try
                    v = lambda * pinv(J) * error;
                catch
                    status = -1; 
                    return
                end
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
        
                J_robot = robot.env.tm5700.model.jacobe(q0);
                J_robotInv = pinv(J_robot);
        
                % Calculate joint velocities
                qp = J_robotInv * v;
                % Limit joint velocities
                qp = max(min(qp, pi), -pi);
                % Update joint angles
                q = q0 + (1 / fps) * qp;
                robot.env.tm5700.model.animate(q');  % Animate the robot
                % Update camera pose
                Tc = robot.env.tm5700.model.fkine(q);
                %camera.T = Tc.T * cameraOffset;
                camera.T = Tc.T;

                % Include gripper here

                % Pause to match the frame rate
                pause(1 / fps);
        
                % Stop the loop after a fixed number of steps
                if steps > 200
                    break;
                end
        
                % Update the joint configuration for the next iteration
                q0 = q;
            end
        end

        function animateTM5WithWaypoints(robot, qWaypoints, numSteps, eStop)
            
            %Use Spline Tracjectory for smooth velocity and acceleration between waypoints
            % Time vector for the waypoints and interp
            tWaypoints = linspace(0, 1, size(qWaypoints, 1));
            tInterp = linspace(0, 1, numSteps);
            
            qMatrix = zeros(numSteps, size(qWaypoints, 2));
            
            % Spline interpolation for each joint
            for jointIdx = 1:size(qWaypoints, 2)
                % Interpolate each joint angle using cubic splines
                qMatrix(:, jointIdx) = spline(tWaypoints, qWaypoints(:, jointIdx), tInterp);
            end
            
            for i = 1:numSteps
                robot.checkPause(eStop);
                
                robot.env.tm5700.model.animate(qMatrix(i, :));
                
                %T_rightEndEffector = robot.env.pr2Right.model.fkine(qMatrix(i, :)).T;
                
                %robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                %robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                %robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                
                drawnow();
            end
        end
        
        function checkPause(~, eStop)
            if eStop.BytesAvailable > 0
                message = fgetl(eStop);
                if strcmp(message, 'STOP')
                    fprintf('Paused...\n');
                    while true
                        if eStop.BytesAvailable > 0
                            message = fgetl(eStop);
                            if strcmp(message, 'RUN')
                                fprintf('Resumed...\n');
                                break;
                            end
                        end
                        pause(0.1);
                    end
                end
            end
        end
    end
end