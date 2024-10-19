%% TM5-700 Visual Servoing with Camera
clc; clf;

grid on; hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(120, 40);

eStop = serial('COM3', 'BaudRate', 9600); 
env = EnvironmentLoader();

%gripperLeftState = 'closed';
%gripperRightState = 'closed';
robot = robotControl(env);
view(75, 25); 

pStar = [262 762 762 262;  % uTL, uTR, uBL, uBR
         762 762 262 262];  % vTL, vTR, vBL, vBR

%pStar = [100 500 500 100;  % uTL, uTR, uBL, uBR
%        500 500 100 100];  % vTL, vTR, vBL, vBR

%P = [1.2, 1.2, 1.2, 1.2;
%    -0.25, 0.25, 0.25, -0.25;
%    1, 1, 0.5, 0.5];

%    P4    P3   P2    P1
%P = [0.9 , 1.1, 1.1, 0.9;
%     0.5, 0.5, 0.6, 0.6;
%     0.82, 0.82, 0.82, 0.82];

%    P4    P3   P2    P1
P = [0.9 , 1.1, 1.1, 0.9;
     0.4, 0.4, 0.6, 0.6;
     0.82, 0.82, 0.82, 0.82];


fps = 25;
lambda = 1;
cameraOffset = transl(0,0.075,0.05);
%q0 = [0.9*pi; -pi/3; pi/3; pi/6; -pi/2; 0];
q0 = [pi/2; pi/8; -pi/2; 0; -pi/2; 0];

camera = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
         'resolution', [1024 1024], 'centre', [512 512], 'name', 'TM5-Camera');

robot.TM5700_IBVS(q0, camera, pStar, P, fps, lambda, eStop)

function TM5700_IBVS(robot, q0, camera, cameraOffset, pStar, P, fps, lambda)
    robotTr = robot.env.tm5700.model.fkine(q0).T;
    robot.env.tm5700.model.animate(q0');
    drawnow;

    camera.T = robotTr * cameraOffset;
    camera.plot_camera('label', 'scale', 0.05, 'frustum', true);
    plot_sphere(P, 0.05, 'b');

    % Project the 3D points to the image plane
    camera.clf();
    p = camera.plot(P);  % Initial projection
    camera.plot(pStar, '*');  % Desired points in the image
    camera.hold(true);
    camera.plot(P, 'pose', Tc0, 'o');  % 3D points with camera pose
    
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
    errorThreshold = 10;
    depth = mean(P(1, :));

    while true
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
        errorNorm = norm(e)  % Check error magnitude
        if errorNorm < errorThreshold
            disp('Error within acceptable range. Exiting...');
            break;  % Exit the visual servoing loop if the error is below the minimum threshold
        end
        
        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
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
        robot.model.animate(q');  % Animate the robot
        % Update camera pose
        Tc = robot.model.fkine(q);
        camera.T = Tc.T * cameraOffset;

        % Pause to match the frame rate
        pause(1 / fps);

        % Stop the loop after a fixed number of steps
        if ksteps > 200
            break;
        end

        % Update the joint configuration for the next iteration
        q0 = q;
    end
end





% %% Initialize the Robot and Setup
% robot = TM5.TM5700();  % Initialize TM5-700 Robot
% 
% %Set workspace limits for better visualization
% axis([-1.9 3.5 -2.5 2.5 0 2.3]);
% 
% % cameraOffset for TM5700 from endEffector
% cameraOffset = transl(0,0.075,0.05);
% 
% % Create target points in the image plane
% pStar = [262 762 762 262;  % uTL, uTR, uBL, uBR
%         762 762 262 262];  % vTL, vTR, vBL, vBR
% 
% % Create 3D points representing the target object
% P = [1.2, 1.2, 1.2, 1.2;
%     -0.25, 0.25, 0.25, -0.25;
%     1, 1, 0.5, 0.5];
% 
% % Initial joint configuration for TM5-700 (6 DOF)
% q0 = [0.9*pi; -pi/3; pi/3; pi/6; -pi/2; 0];
% 
% % Setup the camera
% cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
%     'resolution', [1024 1024], 'centre', [512 512], 'name', 'TM5-Camera');
% 
% % Frame rate
% fps = 25;
% 
% % Controller gain and target depth
% lambda = 1.6;
% depth = mean(P(1, :));  % Average depth of the 3D points
% 
% %% Initialize Visualization
% Tc0 = robot.model.fkine(q0).T;  % Initial pose of the robot
% robot.model.animate(q0');        % Animate to initial pose
% drawnow;
% 
% cam.T = Tc0 * cameraOffset;  % Set the camera pose to match the robot’s end-effector
% cam.plot_camera('label', 'scale', 0.05, 'frustum', true);  % Plot the camera
% plot_sphere(P, 0.05, 'b');  % Plot the target points in 3D
% 
% input('Press Enter to continue.');
% 
% %% Image View Initialization
% % Project the 3D points to the image plane
% cam.clf();
% p = cam.plot(P);  % Initial projection
% cam.plot(pStar, '*');  % Desired points in the image
% cam.hold(true);
% cam.plot(P, 'pose', Tc0, 'o');  % 3D points with camera pose
% 
% % Label each point in the image view
% textHandlesP = gobjects(1, size(p, 2));  % Labels for projected points
% textHandlesPStar = gobjects(1, size(pStar, 2));  % Labels for desired points
% 
% % Label the initial projected points (P)
% for i = 1:size(p, 2)
%     textHandlesP(i) = text(p(1, i), p(2, i), sprintf('P%d', i), ...
%         'Color', 'blue', 'FontSize', 12, 'Parent', gca(cam.figure));
% end
% 
% % Label the desired points (pStar)
% for i = 1:size(pStar, 2)
%     textHandlesPStar(i) = text(pStar(1, i), pStar(2, i), sprintf('p^*%d', i), ...
%         'Color', 'green', 'FontSize', 12, 'Parent', gca(cam.figure));
% end
% 
% pause(2);
% cam.hold(true);
% 
% % Initialize history storage for plotting results
% history = [];
% 
% %% Visual Servoing Control Loop
% ksteps = 0;
% 
% errorThreshold = 10;
% 
% while true
%     ksteps = ksteps + 1;
%     % Get current image plane projection
%     uv = cam.plot(P);
%     % Update labels for the projected points (P)
%     for i = 1:size(uv, 2)
%         if isvalid(textHandlesP(i))
%             delete(textHandlesP(i));  % Delete old label
%         end
%         textHandlesP(i) = text(uv(1, i), uv(2, i), sprintf('P%d', i), ...
%             'Color', 'blue', 'FontSize', 12, 'Parent', gca(cam.figure));
%     end
%     % Compute error between desired and current projections
%     e = pStar - uv;
%     e = e(:)  % Flatten to a column vector
%     % Compute the IBVS Jacobian
%     J = cam.visjac_p(uv, depth);
%     % Check if the error is within the acceptable range
%     errorNorm = norm(e)  % Check error magnitude
%     if errorNorm < errorThreshold
%         disp('Error within acceptable range. Exiting...');
%         break;  % Exit the visual servoing loop if the error is below the minimum threshold
%     end
%     % Compute camera velocity
%     try
%         v = lambda * pinv(J) * e
%     catch
%         warning('Singularity detected! Exiting...');
%         break;
%     end
%     % Compute the robot's Jacobian and its inverse
%     J_robot = robot.model.jacobe(q0);
%     J_inv = pinv(J_robot);
%     % Calculate joint velocities
%     qp = J_inv * v;
%     % Limit joint velocities
%     qp = max(min(qp, pi), -pi);
%     % Update joint angles
%     q = q0 + (1 / fps) * qp;
%     robot.model.animate(q');  % Animate the robot
%     % Update camera pose
%     Tc = robot.model.fkine(q);
%     cam.T = Tc.T * cameraOffset;
% 
%     % Store data for plotting later
%     hist.uv = uv(:);
%     hist.vel = v;
%     hist.e = e;
%     hist.en = norm(e);
%     hist.jcond = cond(J);
%     hist.Tcam = Tc;
%     hist.qp = qp;
%     hist.q = q;
%     history = [history hist];
% 
%     % Pause to match the frame rate
%     pause(1 / fps);
% 
%     % Stop the loop after a fixed number of steps
%     if ksteps > 200
%         break;
%     end
% 
%     % Update the joint configuration for the next iteration
%     q0 = q;
% end
% 
% %% 1.5 Plot Results
% figure();
% plot_p(history, pStar, cam);
% figure();
% plot_camera(history);
% figure();
% plot_vel(history);
% figure();
% plot_robjointpos(history);
% figure();
% plot_robjointvel(history);
% 
% %% Plotting Functions
% function plot_p(history, uv_star, camera)
%     if isempty(history), return; end
%     clf; hold on;
%     uv = [history.uv]';
%     for i = 1:size(uv, 2) / 2
%         plot(uv(:, (i * 2 - 1):(i * 2)));
%     end
%     plot_poly(reshape(uv(1, :), 2, []), 'o--');
%     plot_poly(uv_star, '*:');
%     axis([0 camera.npix(1) 0 camera.npix(2)]);
%     set(gca, 'YDir', 'reverse');
%     grid on;
%     xlabel('u (pixels)');
%     ylabel('v (pixels)');
%     hold off;
% end
% 
% function plot_vel(history)
%     if isempty(history), return; end
%     clf;
%     vel = [history.vel]';
%     plot(vel(:, 1:3), '-'); hold on;
%     plot(vel(:, 4:6), '--'); hold off;
%     ylabel('Cartesian Velocity');
%     grid on;
%     xlabel('Time');
%     legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z');
% end
% 
% function plot_camera(history)
%     %VisualServo.plot_camera Plot camera trajectory
%     %
%     % VS.plot_camera() plots the camera pose versus time.
%     %
%     % See also VS.plot_p, VS.plot_vel, VS.plot_error,
%     % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
% 
%     if isempty(history)
%         return
%     end
%     clf
%     % Cartesian camera position vs time
% 
%     % History: 1x201 Struct Array
%     %disp('History:')
%     %disp(history)
%     %size(history)
% 
%     %disp('History Tcam:')
%     %history.Tcam;
%     %size([history.Tcam(1,1,:)])
% 
%     % Cartesian camera position vs time
%     T = [history.Tcam];
%     S = size(T);
% 
%     % UPDATE: [history.Tcam] is size (1, 201), not (4, 804)
%     % Therefore the below reshape command is unnecessary.
%     % The below reshape command should be uncommented if the fkine
%     % function on Line 164 calls '.T', as that changes the sizes of
%     % the history.Tcam matrix.
% 
%     %T = reshape([history.Tcam], 4, 4, []);
% 
%     subplot(211)
%     plot(transl(T));
%     ylabel('camera position')
%     grid
%     subplot(212)
%     plot(tr2rpy(T))
%     ylabel('camera orientation')
%     grid
%     xlabel('Time')
%     xaxis(length(history));
%     legend('R', 'P', 'Y');
%     subplot(211)
%     legend('X', 'Y', 'Z');
% end
% 
% function plot_robjointpos(history)
%     if isempty(history), return; end
%     clf;
%     pos = [history.q]';
%     plot(pos, '-');
%     ylabel('Joint Angles');
%     grid on;
%     xlabel('Time');
%     legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6');
% end
% 
% function plot_robjointvel(history)
%    if isempty(history), return; end
%    clf;
%    vel = [history.qp]';
%    plot(vel, '-');
%    ylabel('Joint Velocities');
%    grid on;
%    xlabel('Time');
%    legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_5', '\omega_6');
%end
