classdef robotControl
    properties
        
        env
        
    end
    
    methods
        function robot = robotControl(environment)
            robot.env = environment;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PR2 FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        
        function animatePR2LeftArmsAndGrippersWithWaypoints(robot, qWaypoints, numSteps, eStop)
            global PR2GripperLeftState;
            
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
                
                robot.env.pr2LeftArm.model.animate(qMatrix(i, :));
                
                T_rightEndEffector = robot.env.pr2LeftArm.model.fkine(qMatrix(i, :)).T;
                
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperLeftState);
                
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