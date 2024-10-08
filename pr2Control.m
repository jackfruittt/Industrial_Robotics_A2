classdef pr2Control
    properties
    
        env
    
    end

    methods
        function robot = pr2Control(environment)
            robot.env = environment;
        end

        function animatePR2ArmsAndGrippers(robot, rightStartPos, rightEndPos, leftStartPos, leftEndPos, numSteps, eStop)
            global gripperLeftState gripperRightState;

            offset = troty(-pi/2) * transl(0.05, 0, 0); 
            
            q1 = robot.env.pr2Right.model.ikcon(rightStartPos);
            q2 = robot.env.pr2Right.model.ikcon(rightEndPos);
            
            q3 = robot.env.pr2Left.model.ikcon(leftStartPos);
            q4 = robot.env.pr2Left.model.ikcon(leftEndPos);
            
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
                
                robot.env.pr2Right.model.animate(qPrer(i, :)); 
                robot.env.pr2Left.model.animate(qPrel(i, :)); 
                
                T_leftEndEffector = robot.env.pr2Left.model.fkine(qPrel(i, :)).T;
                T_rightEndEffector = robot.env.pr2Right.model.fkine(qPrer(i, :)).T;
        
                robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
        
                % Animate grippers based on their current state
                robot.animateGrippers(robot.env.gripperl1, robot.env.gripperr1, gripperLeftState);
                robot.animateGrippers(robot.env.gripperl2, robot.env.gripperr2, gripperRightState);
                
                drawnow(); % Update the figure
            end
        end    
        
        function animateGrippers(~, gripperLeft, gripperRight, state)
            if strcmp(state, 'open')
                gripperLeft.model.animate([deg2rad(18), deg2rad(-18)]);
                gripperRight.model.animate([deg2rad(-18), deg2rad(18)]);
            else
                gripperLeft.model.animate([0, 0]);
                gripperRight.model.animate([0, 0]);
            end
        end
        
        function animateRightPR2ArmsAndGrippers(robot, rightStartPos, rightEndPos, numSteps, eStop)
            global gripperRightState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0); 
            
            q1 = robot.env.pr2Right.model.ikcon(rightStartPos);
            q2 = robot.env.pr2Right.model.ikcon(rightEndPos);
            
            % LSPB trajectory for smooth transition
            sr = lspb(0, 1, numSteps); % Right arm blend
            qPrer = nan(numSteps, 7);
            
            for i = 1:numSteps
                qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
            end
        
            % Plot the motion between poses and animate robot with grippers
            for i = 1:numSteps
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.pr2Right.model.animate(qPrer(i, :)); 
                
                T_rightEndEffector = robot.env.pr2Right.model.fkine(qPrer(i, :)).T;
        
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
        
                robot.animateGrippers(robot.env.gripperl2, robot.env.gripperr2, gripperRightState);
                
                drawnow(); % Update the figure
            end
        end

        function animateRightPR2ArmsAndGrippersWithWaypoints(robot, qWaypoints, numSteps, eStop)
            global gripperRightState;
            
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
                
                robot.env.pr2Right.model.animate(qMatrix(i, :)); 
                
                T_rightEndEffector = robot.env.pr2Right.model.fkine(qMatrix(i, :)).T;
        
                robot.env.gripperl2.model.base = T_rightEndEffector * offset;
                robot.env.gripperr2.model.base = T_rightEndEffector * offset;
        
                robot.animateGrippers(robot.env.gripperl2, robot.env.gripperr2, gripperRightState);
                
                drawnow();
            end
        end
        
        
        function animateLeftPR2ArmsAndGrippers(robot, leftStartPos, leftEndPos, numSteps, eStop)
            global gripperLeftState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0); 
            
            q3 = robot.env.pr2Left.model.ikcon(leftStartPos);
            q4 = robot.env.pr2Left.model.ikcon(leftEndPos);
            
            sl = lspb(0, 1, numSteps); % Left arm blend
            qPrel = nan(numSteps, 7);
            
            for i = 1:numSteps
                qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
            end
        
            % Plot the motion between poses and animate robot with grippers
            for i = 1:numSteps
                robot.checkPause(eStop); % Check for pause signal
                
                robot.env.pr2Left.model.animate(qPrel(i, :)); 
                
                T_leftEndEffector = robot.env.pr2Left.model.fkine(qPrel(i, :)).T;
        
                robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                robot.env.gripperr1.model.base = T_leftEndEffector * offset;
        
                robot.animateGrippers(robot.env.gripperl1, robot.env.gripperr1, gripperLeftState);
                
                drawnow(); % Update the figure
            end
        end
        
        function gripperBothAnimate(robot, numSteps, openOrClose)
            global gripperLeftState gripperRightState;
            %robot.gripperLeftState;
            %robot.gripperRightState;
        
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
                robot.env.gripperl1.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr1.model.animate(qMatrixRight(i, :));
                robot.env.gripperl2.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr2.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end
        
        function gripperLeftAnimate(robot, numSteps, openOrClose)
            global gripperLeftState;
            %robot.gripperLeftState;
        
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
                robot.env.gripperl1.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr1.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end
        
        function gripperRightAnimate(robot, numSteps, openOrClose)
            global gripperRightState;
            %robot.gripperRightState;
        
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
                robot.env.gripperl2.model.animate(qMatrixLeft(i, :));
                robot.env.gripperr2.model.animate(qMatrixRight(i, :));
                drawnow();
            end
        end
        
        function LeftGripperOpen(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.gripperLeftAnimate(numSteps, 'open');
        end
        
        function LeftGripperClose(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.gripperLeftAnimate(numSteps, 'close');
        end
        
        function RightGripperOpen(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.gripperRightAnimate(numSteps, 'open');
        end
        
        function RightGripperClose(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.gripperRightAnimate(numSteps, 'close');
        end
        
        function bothGripperOpen(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.gripperBothAnimate(numSteps, 'open');
        end
        
        function bothGripperClose(robot, numSteps, eStop)
            robot.checkPause(eStop);
            robot.gripperBothAnimate(numSteps, 'close');
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