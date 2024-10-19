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
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T;
                
                robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                
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

        function animatePR2GripperRightKnife(~, gripperLeft, gripperRight, state)
            if strcmp(state, 'open')
                gripperLeft.model.animate([deg2rad(18), deg2rad(-18)]);
                gripperRight.model.animate([deg2rad(-18), deg2rad(18)]);
            else
                gripperLeft.model.animate([deg2rad(6), deg2rad(-6)]);
                gripperRight.model.animate([deg2rad(-6), deg2rad(6)]);
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
                
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T;
                
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                
                drawnow(); % Update the figure
            end
        end

        function animateRightPR2ArmsAndGrippersWithKnife(robot, rightStartPos, rightEndPos, numSteps, eStop)
            global PR2GripperRightStateKnife;
            
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
                
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T;
        
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                robot.env.pr2KnifeArm.attachToEndEffector(robot.env.pr2RightArm.model.fkine(qPrer(i, :)).T);
        
                robot.animatePR2GripperRightKnife(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightStateKnife);
                
                drawnow(); % Update the figure
            end
        end

        function animatepr2RightHybridControl(robot, rightStartTr, rightEndTr, steps, deltaTime, lambda, epsilon, eStop)

            global PR2GripperRightStateKnife;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            qStartRight = robot.env.pr2RightArm.model.ikcon(rightStartTr);
            
            qMatrixRight = zeros(steps, 7);
        
            qMatrixRight(1,:) = qStartRight;
        
            rightArmCTraj = ctraj(rightStartTr, rightEndTr, steps);
        
            % RMRC loop with fallback to IK
            for i = 1:steps-1
                
                qRight = qMatrixRight(i, :);
                tRight = robot.env.pr2RightArm.model.fkine(qRight).T;
        
                % Compute Cartesian velocity for RMRC
                vRight = tr2delta(tRight, rightArmCTraj(:,:,i+1)) / deltaTime;
        
                % Compute Jacobian at current joint configuration
                jRight = robot.env.pr2RightArm.model.jacob0(qRight);
        
                % Damped Least Squares (DLS) to handle singularities
                if abs(det(jRight * jRight')) < epsilon
                    qRightDot = (jRight' / (jRight * jRight' + lambda^2 * eye(6))) * vRight;
                else
                    qRightDot = jRight \ vRight;
                end
        
                % Check if RMRC can continue or fallback to IK based on joint velocity
                %if any(abs(qRightDot) > 2)  % If joint velocity is too high or unstable
                
                % Compute Yoshikawa's manipulability measure
                manipulability = sqrt(det(jRight * jRight'));

                %Combine YMM with singularity handling 
                if manipulability < epsilon ||  any(abs(qRightDot) > 2)
                    disp('Switching to IK due to low manipulability, singularities or instability');
                    % Use IK to compute the next joint configuration
                    qMatrixRight(i+1,:) = robot.env.pr2RightArm.model.ikcon(rightArmCTraj(:,:,i+1));
                else
                    % RMRC - update joint angles using Euler integration
                    qMatrixRight(i+1,:) = qRight + qRightDot' * deltaTime;
                end
            end
        
            for i = 1:steps
    
                robot.checkPause(eStop);

                robot.env.pr2RightArm.model.animate(qMatrixRight(i,:));

                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrixRight(i,:)).T;
        
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                robot.env.pr2KnifeArm.attachToEndEffector(tRightEndEffector);
        
                robot.animatePR2GripperRightKnife(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightStateKnife);
                drawnow();
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
                
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrix(i, :)).T;
                
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                
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
                
                tRightEndEffector = robot.env.pr2LeftArm.model.fkine(qMatrix(i, :)).T;
                
                robot.env.gripperl1.model.base = tRightEndEffector * offset;
                robot.env.gripperr1.model.base = tRightEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl1, robot.env.gripperr1, PR2GripperLeftState);
                
                drawnow();
            end
        end
        
        function animatePR2ArmsRMRC(robot, leftStartTr, leftEndTr, rightStartTr, rightEndTr, steps, deltaTime, lambda, epsilon, eStop)

            global PR2GripperRightState;
            
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
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrixRight(i, :)).T;
                
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                drawnow();
            end

        end

        function animatePR2RightArmRMRC(robot, rightStartTr, rightEndTr, steps, deltaTime, lambda, epsilon, eStop)

            global PR2GripperRightState;
            
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            % Use ikcon to get initial joint angles from given pose
            qStartRight = robot.env.pr2RightArm.model.ikcon(rightStartTr);
            
            qMatrixRight = zeros(steps, 7);

            % Set first joint angles with the calculated starting ones
            qMatrixRight(1,:) = qStartRight;

            % Get cartesian traj from start to end given n steps
            rightArmCTraj = ctraj(rightStartTr, rightEndTr, steps);

            % RMRC loop 
            for i = 1:steps-1
                % Get current joint angles
                qRight = qMatrixRight(i, :);

                % Get current pose using fkine
                tRight = robot.env.pr2RightArm.model.fkine(qRight).T;

                % Compute cartesian velocity
                vRight = tr2delta(tRight, rightArmCTraj(:,:,i+1)) / deltaTime;

                % Compute jacobian at current joint configuration
                jRight = robot.env.pr2RightArm.model.jacob0(qRight);

                % Handle singularities through Damped Least Squares (DLS) for left and right

                if abs(det(jRight * jRight')) < epsilon
                    qRightDot = (jRight' / (jRight * jRight' + lambda^2 * eye(6))) * vRight;
                else
                    qRightDot = jRight \ vRight;
                end

                % Euler integration to update joint angles for both arms
                qMatrixRight(i+1, :) = qRight + qRightDot' * deltaTime;
            end

            % Animate the robot movement
            for i = 1:steps
                robot.checkPause(eStop);
                robot.env.pr2RightArm.model.animate(qMatrixRight(i,:));
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrixRight(i, :)).T;
                
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                robot.env.pr2KnifeArm.attachToEndEffector(robot.env.pr2RightArm.model.fkine(qMatrixRight(i, :)).T);
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
                drawnow();
            end

        end

        function animatepr2RightRMRCNullSpace(robot, rightStartTr, rightEndTr, steps, deltaTime, lambda, epsilon, secondaryTaskWeight)
            global  PR2GripperRightStateKnife;
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            qRight = robot.env.pr2RightArm.model.ikcon(rightStartTr);
            qMatrixRight = zeros(steps, length(qRight)); 
            qMatrixRight(1,:) = qRight;
        
            T_rightCTraj = ctraj(rightStartTr, rightEndTr, steps);
        
            for i = 1:steps-1
              
                qRight = qMatrixRight(i, :);
                T_current = robot.env.pr2RightArm.model.fkine(qRight).T;
        
                % Compute Cartesian velocity
                vRight = tr2delta(T_current, T_rightCTraj(:,:,i+1)) / deltaTime;
        
                % Compute the Jacobian at the current joint configuration
                J = robot.env.pr2RightArm.model.jacob0(qRight);
        
                % Compute the primary task joint velocities using Damped Least Squares (DLS)
                if abs(det(J * J')) < epsilon
                    J_pseudoInverse = (J' / (J * J' + lambda^2 * eye(6)));  % DLS pseudo-inverse
                else
                    J_pseudoInverse = pinv(J);  
                end
        
                qDot_primary = J_pseudoInverse * vRight;
        
                % Define the secondary task (e.g., posture control, joint limit avoidance)
                q_preferred = [0 -pi/2 0 0 0 0 0]; 
                z = secondaryTaskWeight * (q_preferred - qRight)';
        
                % Null-space projection to resolve redundancy
                qDot_secondary = (eye(size(J, 2)) - J_pseudoInverse * J) * z;
        
                % Combine primary and secondary tasks
                qDot = qDot_primary + qDot_secondary;
        
                qMatrixRight(i+1, :) = qRight + qDot' * deltaTime;
            end
        
            for i = 1:steps
                robot.env.pr2RightArm.model.animate(qMatrixRight(i,:));
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qMatrixRight(i, :)).T;
                
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
                robot.env.pr2KnifeArm.attachToEndEffector(robot.env.pr2RightArm.model.fkine(qMatrixRight(i, :)).T);
                robot.animatePR2GripperRightKnife(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightStateKnife);
                drawnow();
            end
        end
        

        function animatePR2SingleRightJoint(robot, jointToMove, targetAngleDeg, steps)
            global PR2GripperRightState;
            
            currentConfig = robot.env.pr2RightArm.model.getpos();  
            targetAngleRad = deg2rad(targetAngleDeg);
            angles = linspace(currentConfig(jointToMove), targetAngleRad, steps);
            qTarget = currentConfig;
            offset = troty(-pi/2) * transl(0.05, 0, 0);
            
            for i = 1:length(angles)
                qTarget(jointToMove) = angles(i);  
                robot.env.pr2RightArm.model.animate(qTarget);  
                tRightEndEffector = robot.env.pr2RightArm.model.fkine(qTarget).T;
                robot.env.gripperl2.model.base = tRightEndEffector * offset;
                robot.env.gripperr2.model.base = tRightEndEffector * offset;
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
                robot.animatePR2Grippers(robot.env.gripperl2, robot.env.gripperr2, PR2GripperRightState);
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
            global PR2GripperRightStateKnife;
            %robot.PR2GripperRightState;
            
            % Define the joint limits
            qLeftOpen = [deg2rad(18), deg2rad(-18)];
            qLeftClose = [deg2rad(6), deg2rad(-6)];
            qRightOpen = [deg2rad(-18), deg2rad(18)];
            qRightClose = [deg2rad(-6), deg2rad(6)]; 
            
            if strcmp(openOrClose, 'open')
                qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
                qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);
                PR2GripperRightStateKnife = 'open';
            else
                qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
                qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);
                PR2GripperRightStateKnife = 'closed';
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
                %tRightEndEffector = robot.env.pr2Right.model.fkine(qPrer(i, :)).T;
                
                %Ditto ^^
                %robot.env.gripperl1.model.base = T_leftEndEffector  * offset;
                %robot.env.gripperr1.model.base = T_leftEndEffector * offset;
                %robot.env.gripperl2.model.base = tRightEndEffector * offset;
                %robot.env.gripperr2.model.base = tRightEndEffector * offset;
                
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
                
                %tRightEndEffector = robot.env.pr2Right.model.fkine(qMatrix(i, :)).T;
                
                %robot.env.gripperl2.model.base = tRightEndEffector * offset;
                %robot.env.gripperr2.model.base = tRightEndEffector * offset;
                
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