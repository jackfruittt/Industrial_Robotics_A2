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


        function animateTM5(robot, currentQ, startPos, endPos, numSteps, eStop)
            
            global TM5GripperState;
            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);
            
            % Set delays to 0 to animate faster
            robot.env.tm5700.model.delay = 0;
            robot.env.tm5700GripperL.model.delay = 0;
            robot.env.tm5700GripperR.model.delay = 0;
            
            % Using ikcon to generate start
            q1 = robot.env.tm5700.model.ikcon(startPos, currentQ);
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
                endEffector = robot.env.tm5700.model.fkine(qPrer(i, :));
                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;

                robot.animateTM5Gripper(TM5GripperState);

                % Display Data
                disp('Animating TM5 from q1 to q2 ...');
                fprintf('q1: %.3f %.3f %.3f %.3f %.3f %.3f\n', q1);
                fprintf('currentQ: %.3f %.3f %.3f %.3f %.3f %.3f\n', qPrer(i,:));
                fprintf('q2: %.3f %.3f %.3f %.3f %.3f %.3f\n', q2);

                if qPrer(i,:) == q2
                    disp('Animating TM5 from q1 to q2 finished');
                end
                
                drawnow(); % Update the figure
            end
        end

        function animateTM5WithBanana(robot, currentQ, startPos, endPos, numSteps, eStop)
            
            global TM5GripperState;
            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);

            % Set delays to 0 to animate faster
            robot.env.tm5700.model.delay = 0;
            robot.env.tm5700GripperL.model.delay = 0;
            robot.env.tm5700GripperR.model.delay = 0;
            robot.env.tm5700Banana.model.delay = 0;
            
            q1 = robot.env.tm5700.model.ikcon(startPos, currentQ);
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
                endEffector = robot.env.tm5700.model.fkine(qPrer(i, :));
                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;

                % Attach 'robot' banana to end effector for currentQ
                robot.env.tm5700Banana.attachToEndEffector(endEffector.T);
                robot.animateTM5Gripper(TM5GripperState);
                
                % Display Data
                disp('Animating TM5 from q1 to q2 ...');
                fprintf('q1: %.3f %.3f %.3f %.3f %.3f %.3f\n', q1);
                fprintf('currentQ: %.3f %.3f %.3f %.3f %.3f %.3f\n', qPrer(i,:));
                fprintf('q2: %.3f %.3f %.3f %.3f %.3f %.3f\n', q2);

                if qPrer(i,:) == q2
                    disp('Animating TM5 from q1 to q2 finished');
                end
                
                drawnow(); % Update the figure
            end
        end

        function animateTM5RMRC(robot, qStart, startPos, endPos, numSteps, deltaTime, epsilon, eStop)
            % Based on Week 7 and Week 9 Theory / Lab
            global TM5GripperState;
            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);

            % Set delays to 0 to animate faster
            robot.env.tm5700.model.delay = 0;
            robot.env.tm5700GripperL.model.delay = 0;
            robot.env.tm5700GripperR.model.delay = 0;

            % Weight matrix W for applying linear/angular vel influence
            W = diag([1 1 1 0 0 0]);            

            m = zeros(numSteps, 1);                % Manipulability
            qMatrix = zeros(numSteps, 6);          % Joint Angles
            qDot = zeros(numSteps, 6);             % Joint Velocities
            p = zeros(3, numSteps);                % X Y Z point
            theta  = zeros(3, numSteps);           % R P Y 

            startPosRot = startPos(1:3,1:3);                                  % Extract Rotation Matrix from T1
            startPosRPY = tr2rpy(startPosRot,'deg',true,'order','xyz')';      % Get roll pitch yaw using tr2rpy from rotation matrix
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
                
                % Apply weight matrix to influence the end effector velocity
                pDot = W * [linearVelocity;angularVelocity];

                % Get robot jacobian from current q configuration
                J = robot.env.tm5700.model.jacob0(qMatrix(i,:));

                % Check if manipulability is within threshold to adjust lambda value
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2; % Damping gets larger as m(i) goes to 0 (singularity)
                else
                    lambda = 0; % No damping applied
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

                % Euler Integration to get next q configuration
                qMatrix(i+1,:) = qMatrix(i,:) + deltaTime*qDot(i,:);
            end
            
            % Animate qMatrix
            for i=1:numSteps
                robot.checkPause(eStop)
                robot.env.tm5700.model.animate(qMatrix(i,:));

                endEffector = robot.env.tm5700.model.fkine(qMatrix(i,:));

                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;

                robot.animateTM5Gripper(TM5GripperState);

                % Display Data
                disp('End Effector Position: ')
                disp(endEffector.T);
                fprintf('q: %.3f %.3f %.3f %.3f %.3f %.3f\n', qMatrix(i,:));
                fprintf('step: %d\n',i);

                if(i == numSteps)
                    disp('Final End Effector Position: ');
                    disp(endEffector.T);
                    disp('Desired End Effector Position: ');
                    disp(endPos);
                    disp('Position Error: ');
                    endEffector = endEffector.T;
                    disp(endPos(1:3, 4) - endEffector(1:3, 4));
                end

                drawnow();
            end
        end

        function animateTM5WithBananaRMRC(robot, qStart, startPos, endPos, numSteps, deltaTime, epsilon, eStop)
            % Based on Week 7 and Week 9 Theory / Lab
            global TM5GripperState;

            % Set delays to 0 to animate faster
            robot.env.tm5700.model.delay = 0;
            robot.env.tm5700GripperL.model.delay = 0;
            robot.env.tm5700GripperR.model.delay = 0;
            robot.env.tm5700Banana.model.delay = 0;

            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);

            % Weight matrix W for applying linear/angular vel influence
            W = diag([1 1 1 0 0 0]);            

            m = zeros(numSteps, 1);                % Manipulability
            qMatrix = zeros(numSteps, 6);          % Joint Angles
            qDot = zeros(numSteps, 6);             % Joint Velocities
            p = zeros(3, numSteps);                % X Y Z point
            theta  = zeros(3, numSteps);           % R P Y 

            startPosRot = startPos(1:3,1:3);                                  % Extract Rotation Matrix from T1
            startPosRPY = tr2rpy(startPosRot,'deg',true,'order','xyz')';      % Get roll pitch yaw using tr2rpy from rotation matrix
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
                
                % Apply weight matrix to influence the end effector velocity
                pDot = W * [linearVelocity;angularVelocity];

                % Get robot jacobian from current q configuration
                J = robot.env.tm5700.model.jacob0(qMatrix(i,:));

                % Check if manipulability is within threshold to adjust lambda value
                if sqrt(det(J*J')) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2; % Damping gets larger as m(i) goes to 0 (singularity)
                else
                    lambda = 0; % No damping applied
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

                % Euler Integration to get next q configuration
                qMatrix(i+1,:) = qMatrix(i,:) + deltaTime*qDot(i,:);
            end
            
            % Animate qMatrix
            for i=1:numSteps
                robot.checkPause(eStop);
                robot.env.tm5700.model.animate(qMatrix(i,:));

                endEffector = robot.env.tm5700.model.fkine(qMatrix(i,:));

                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;

                robot.env.tm5700Banana.attachToEndEffector(endEffector.T);
                robot.animateTM5Gripper(TM5GripperState);

                % Display Data
                disp('End Effector Position: ')
                disp(endEffector.T);
                fprintf('q: %.3f %.3f %.3f %.3f %.3f %.3f\n', qMatrix(i,:));
                fprintf('step: %d\n',i);
                
                if(i == numSteps)
                    disp('Final End Effector Position: ');
                    disp(endEffector.T);
                    disp('Desired End Effector Position: ');
                    disp(endPos);
                    disp('Position Error: ');
                    endEffector = endEffector.T;
                    disp(endPos(1:3, 4) - endEffector(1:3, 4));
                end

                drawnow();
            end
        end

        function animateTM5IBVS(robot, q0, pStar, P, fps, lambda, eStop)
            % Based on Week 10 Theory / Lab
            global TM5GripperState;
            qDotLim = [-0.5 0.5];
            dt = (1/fps);

            % Set delays to 0 to animate faster
            robot.env.tm5700.model.delay = 0;
            robot.env.tm5700GripperL.model.delay = 0;
            robot.env.tm5700GripperR.model.delay = 0;

            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);

            % Get current robot position from given q configuration
            robotTr = robot.env.tm5700.model.fkine(q0).T;
            robot.env.tm5700.model.animate(q0');                

            robot.env.tm5700GripperL.model.base = robotTr * tm5GripperOffset;
            robot.env.tm5700GripperR.model.base = robotTr * tm5GripperOffset;

            robot.animateTM5Gripper(TM5GripperState);

            drawnow();
        
            % Camera Position defined from end effector location * offset
            robot.env.tm5700Camera.T = robotTr * cameraOffset;
            robot.env.tm5700Camera.plot_camera('label', 'scale', 0.05, 'frustum', true);
            plot_sphere(P, 0.03, 'b');  % Desired point features in world frame
        
            % Project the 3D points to the image plane
            robot.env.tm5700Camera.clf();
            p = robot.env.tm5700Camera.plot(P);  % Initial projection of point features to image
            robot.env.tm5700Camera.plot(pStar, '*');  % Plot desired points in the image
            robot.env.tm5700Camera.hold(true);
            robot.env.tm5700Camera.plot(P, 'pose', robotTr, 'o');  % Plot 3D points with camera pose
            
            % Label each point in the image view
            textHandlesP = gobjects(1, size(p, 2));  % Labels for projected points
            textHandlesPStar = gobjects(1, size(pStar, 2));  % Labels for desired points
            
            % Label the initial projected points (P)
            for i = 1:size(p, 2)
                textHandlesP(i) = text(p(1, i), p(2, i), sprintf('P%d', i), ...
                    'Color', 'blue', 'FontSize', 12, 'Parent', gca(robot.env.tm5700Camera.figure));
            end
            
            % Label the desired points (pStar)
            for i = 1:size(pStar, 2)
                textHandlesPStar(i) = text(pStar(1, i), pStar(2, i), sprintf('p^*%d', i), ...
                    'Color', 'green', 'FontSize', 12, 'Parent', gca(robot.env.tm5700Camera.figure));
            end
        
            pause(3);
            robot.env.tm5700Camera.hold(true);
            
            steps = 0; 
            errorThreshold = [10, 2000]; % Set pixel error value 
            depth = mean(P(1, :)); % Estimate depth 
        
            while true
                robot.checkPause(eStop);
                steps = steps + 1; % Increment Step count
                uv = robot.env.tm5700Camera.plot(P); % Current Projected Features

                % For updating plot display of desired features moving towards target
                for i = 1:size(uv, 2)
                    if isvalid(textHandlesP(i))
                        delete(textHandlesP(i));  % Delete old label
                    end
                    textHandlesP(i) = text(uv(1, i), uv(2, i), sprintf('P%d', i), ...
                                     'Color', 'blue', 'FontSize', 12, 'Parent', gca(robot.env.tm5700Camera.figure));
                end
                
                % Calculate pixel error
                error = pStar - uv;
                error = error(:);

                % Calcuate image jacobian from current features and depth (Interaction matrix)
                J_image = robot.env.tm5700Camera.visjac_p(uv, depth);
        
                % Check if the error is within the acceptable range
                errorNorm = norm(error);  % Check error magnitude
                if errorNorm < errorThreshold (1)
                    disp('Error within acceptable range. Exiting...');
                    disp('Final End Effector Position: ');
                    disp(robot.env.tm5700.model.fkine(q0));
                    break;  % Exit the visual servoing loop if the error is below the minimum threshold
                elseif errorThreshold(2) < errorNorm  
                    disp('Error too large. Exiting...');
                    break;
                end
                
                % Compute the velocity of camera in camera frame
                v = lambda * pinv(J_image) * error;

                J_robot = robot.env.tm5700.model.jacobe(q0); % jacobe for end effector frame
                invJ_robot = pinv(J_robot);
        
                % Calculate joint velocities using computed velocity and the pseudo inv robot jacobian
                qDot = invJ_robot * v;
                % Limit joint velocities
                qDot = max(min(qDot, qDotLim(2)), qDotLim(1));
                % Update joint angles
                q = q0 + (dt * qDot);
                robot.env.tm5700.model.animate(q');  % Animate the robot
                
                % Update camera pose
                endEffector = robot.env.tm5700.model.fkine(q);
                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;

                % Update Gripper
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;

                robot.animateTM5Gripper(TM5GripperState);

                % Display Data
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
                fprintf('steps: %d\n',steps);
                fprintf('error: %.3f\n',errorNorm);

                % Pause to match the frame rate
                pause(dt);
        
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

        function animateTM5Gripper(robot, openOrClose)
            global TM5GripperState;
            % To open or close gripper instantly
            if strcmp(openOrClose, 'open')
                robot.env.tm5700GripperL.model.animate([deg2rad(30), deg2rad(-30)]);
                robot.env.tm5700GripperR.model.animate([deg2rad(-30), deg2rad(30)]);
                TM5GripperState = 'open';
            elseif strcmp(openOrClose, 'closed')
                robot.env.tm5700GripperL.model.animate([deg2rad(16), deg2rad(-16)]);
                robot.env.tm5700GripperR.model.animate([deg2rad(-16), deg2rad(16)]);
                TM5GripperState = 'closed';
            end
        end

        function animateTM5GripperMotion(robot, openOrClose, eStop)
            global TM5GripperState;
            numSteps = 50;

            % To open or close gripper over a number of steps using jtraj

            qLeftOpen = [deg2rad(30), deg2rad(-30)];
            qLeftClose = [deg2rad(16), deg2rad(-16)];
            qRightOpen = [deg2rad(-30), deg2rad(30)];
            qRightClose = [deg2rad(-16), deg2rad(16)];

            if strcmp(openOrClose, 'open')
                qMatrixL = jtraj(qLeftClose, qLeftOpen, numSteps);
                qMatrixR = jtraj(qRightClose, qRightOpen, numSteps);
                TM5GripperState = 'open';
            elseif strcmp(openOrClose, 'closed')
                qMatrixL = jtraj(qLeftOpen, qLeftClose, numSteps);
                qMatrixR = jtraj(qRightOpen, qRightClose, numSteps);
                TM5GripperState = 'closed';
            end

            for i = 1:numSteps
                robot.checkPause(eStop);
                robot.env.tm5700GripperL.model.animate(qMatrixL(i,:));
                robot.env.tm5700GripperR.model.animate(qMatrixR(i,:));
                drawnow();
            end

        end
        
        function checkPause(~, eStop)
            if eStop.BytesAvailable > 0
                message = strtrim(fgetl(eStop));
                if strcmp(message, 'STOP')
                    fprintf('Paused...\n');
                    while true
                        if eStop.BytesAvailable > 0
                            message = strtrim(fgetl(eStop));
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

        function savedQ =  gamepadEndEffectorFrameControl(robot, gamepad, lambda, kV, kW, duration, dt, robotName, eStop)
            % Based on Week 11 Theory / Lab
            global TM5GripperState;
            selectedRobot = robot.selectRobotToControl(robotName);
            
            % Get current q
            q = selectedRobot.getpos();

            % To animate faster by setting delay = 0
            selectedRobot.delay = 0;

            maxSavedQ = 10; % Edit this value to store more q configurations
            savedQ = cell(maxSavedQ, 1); % For storing desired q configurations
            qCount = 0;

            % Offsets
            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);

            steps = 0;
            stickDriftThreshold = 0.1; % Tested this value, works good
            qDotLim = [-0.5 0.5]; % Define joint speed limits

            tic;
            while (toc < duration)
                robot.checkPause(eStop)
                % Increase Step Count
                steps = steps + 1;

                % read joystick data
                [axes, buttons, ~] = read(gamepad);

                % Get linear and angular velocities from mapped axes and buttons
                vx = kV*robot.gamepadRemoveStickDrift(axes(1), stickDriftThreshold);
                vy = kV*robot.gamepadRemoveStickDrift(axes(2), stickDriftThreshold);
                vz = kV*(buttons(4) - buttons(2));

                wx = kW*robot.gamepadRemoveStickDrift(axes(3), stickDriftThreshold);
                wy = kW*robot.gamepadRemoveStickDrift(axes(4), stickDriftThreshold);
                wz = kW*(buttons(9) - buttons(7));

                % Combine velocities to get velocity vector x
                x = [vx; vy; vz; wx; wy; wz];

                % Get jacobian with respect to end effector frame
                J = selectedRobot.jacobe(q);

                % DLS (kinda)
                invJ = inv((J*J')+lambda^2*eye(6))*J';
                
                % Get joint velocities
                qDot = invJ*x;

                % Check joint limits if they exceed
                for i = 1:length(q)
                    if q(i) + dt * qDot(i) < robot.env.tm5700.model.qlim(i, 1)
                        qDot(i) = 0;  % Stop if it exceeds lower bound
                    elseif q(i) + dt * qDot(i) > robot.env.tm5700.model.qlim(i, 2)
                        qDot(i) = 0;  % Stop if it exceeds upper bound
                    end
                end

                % Reset Robot Position by pressing B1
                if (buttons(1))
                    q = zeros(1,6);
                    selectedRobot.animate(q);
                end

                % Save q configuration by pressing B3
                if (buttons(3))
                    disp('____________________________________');
                    disp('Saving current q configurations ...');
                    qCount = qCount + 1;
                    fprintf('Number of empty q configurations : %d\n', (maxSavedQ - qCount))
                    savedQ{qCount} = q;
                    disp('____________________________________');
                end

                % Exit loop if max number of q's are reached or B8 is pressed early
                if qCount == 10 || buttons(8)
                    disp('Max Number of Q Configurations saved, Exiting Control Loop');
                    break
                end

                % Gripper control, B5 = Right Joystick, B10 = Left Joystick
                if buttons(5)
                    robot.animateTM5GripperMotion('open',eStop);
                elseif buttons(10)
                    robot.animateTM5GripperMotion('closed',eStop);
                end

                % Set speed limits for joints based off qDotLim values
                qDot = max(min(qDot, qDotLim(2)), qDotLim(1));
                
                % Update q Value 
                q = q + (qDot' * dt);

                % Animate robot to match new q values
                selectedRobot.animate(q);
                endEffector = selectedRobot.fkine(q);
                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;
                robot.animateTM5Gripper(TM5GripperState);

                % Display Data
                fprintf('x: %.3f %.3f %.3f %.3f %.3f %.3f\n', x');
                fprintf('q: %.3f %.3f %.3f %.3f %.3f %.3f\n', q);
                fprintf('qDot: %.3f %.3f %.3f %.3f %.3f %.3f\n', qDot');
                fprintf('Loop: %d\n',steps);

                drawnow();

                while (toc < dt*steps)
                    robot.checkPause(eStop)
                end
            end
        end

        function savedQ = gamepadWorldFrameControl(robot, gamepad, lambda, kV, kW, duration, dt, robotName, eStop)
            % Based on Week 11 Theory / Lab
            global TM5GripperState;
            selectedRobot = robot.selectRobotToControl(robotName);

            % Get current q
            q = selectedRobot.getpos();

            % To animate faster by setting delay = 0
            selectedRobot.delay = 0;
            maxSavedQ = 10; % Edit this value to store more q configurations
            savedQ = cell(maxSavedQ, 1); % For storing desired q configurations
            qCount = 0;

            % Offsets
            tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);
            cameraOffset = transl(0,0.075,0.05);

            steps = 0;
            stickDriftThreshold = 0.1; % Tested this value, works good
            qDotLim = [-0.5 0.5]; % Define joint speed limits

            tic;
            while (toc < duration)
                robot.checkPause(eStop)
                % Increase Step Count
                steps = steps + 1;

                % read joystick data
                [axes, buttons, ~] = read(gamepad);

                % Get linear and angular velocities from mapped axes and buttons
                vx = kV*robot.gamepadRemoveStickDrift(axes(1), stickDriftThreshold);
                vy = kV*robot.gamepadRemoveStickDrift(axes(2), stickDriftThreshold);
                vz = kV*(buttons(4) - buttons(2));

                wx = kW*robot.gamepadRemoveStickDrift(axes(3), stickDriftThreshold);
                wy = kW*robot.gamepadRemoveStickDrift(axes(4), stickDriftThreshold);
                wz = kW*(buttons(9) - buttons(7));

                % Combine velocities to get velocity vector x
                x = [vx; vy; vz; wx; wy; wz];

                % Get jacobian with respect to end effector frame
                J = selectedRobot.jacob0(q);
                
                % DLS (kinda)
                invJ = inv((J*J')+lambda^2*eye(6))*J';

                % Get joint velocities
                qDot = invJ*x;

                % Check joint limits if they exceed
                for i = 1:length(q)
                    if q(i) + dt * qDot(i) < robot.env.tm5700.model.qlim(i, 1)
                        qDot(i) = 0;  % Stop if it exceeds lower bound
                    elseif q(i) + dt * qDot(i) > robot.env.tm5700.model.qlim(i, 2)
                        qDot(i) = 0;  % Stop if it exceeds upper bound
                    end
                end

                % Reset Robot Position by pressing B1
                if (buttons(1))
                    q = zeros(1,6);
                    selectedRobot.animate(q);
                end

                % Save q configuration by pressing B3
                if (buttons(3))
                    disp('____________________________________');
                    disp('Saving current q configurations ...');
                    qCount = qCount + 1;
                    fprintf('Number of empty q configurations : %d\n', (maxSavedQ - qCount))
                    savedQ{qCount} = q;
                    disp('____________________________________');
                end

                % Exit loop if max number of q's are reached or B8 is pressed early
                if qCount == 10 || buttons(8)
                    disp('Max Number of Q Configurations saved, Exiting Control Loop');
                    break
                end

                % Gripper control, B5 = Right Joystick, B10 = Left Joystick
                if buttons(5)
                    robot.animateTM5GripperMotion('open',eStop);
                elseif buttons(10)
                    robot.animateTM5GripperMotion('closed',eStop);
                end

                % Set speed limits for joints based off qDotLim values
                qDot = max(min(qDot, qDotLim(2)), qDotLim(1));
                
                % Update q Value 
                q = q + (qDot' * dt);

                % Animate robot to match new q values
                selectedRobot.animate(q);
                endEffector = selectedRobot.fkine(q);
                robot.env.tm5700Camera.T = endEffector.T * cameraOffset;
                robot.env.tm5700GripperL.model.base = endEffector.T * tm5GripperOffset;
                robot.env.tm5700GripperR.model.base = endEffector.T * tm5GripperOffset;
                robot.animateTM5Gripper(TM5GripperState);

                % Display Data
                fprintf('x: %.3f %.3f %.3f %.3f %.3f %.3f\n', x');
                fprintf('q: %.3f %.3f %.3f %.3f %.3f %.3f\n', q);
                fprintf('qDot: %.3f %.3f %.3f %.3f %.3f %.3f\n', qDot');
                fprintf('Loop: %d\n',steps);

                drawnow();

                while (toc < dt*n)
                    robot.checkPause(eStop)
                end
            end
        end

        function value = gamepadRemoveStickDrift(~, inputValue, stickDriftThreshold)
            % Create deadzone to prevent stick drift
            if abs(inputValue) < stickDriftThreshold
                value = 0;
            else
                value = sign(inputValue);
            end
        end

        function selectedRobot = selectRobotToControl(robot, robotName)
            % Implement gamepad controls to support PR2 Arm
            switch robotName
                case 'PR2LeftArm' 
                    selectedRobot = robot.env.pr2LeftArm.model;
                case 'PR2RightArm'
                    selectedRobot = robot.env.pr2RightArm.model;
                case 'TM5'
                    selectedRobot = robot.env.tm5700.model;
            end
        end
    end
end