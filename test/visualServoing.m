clf;
clc;
grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);
view(90, 10);

% Initialize robot
tm5 = TM5900.TM5900();
%tm5.model.teach();

% Define initial AR tag 3D points (corners of a square in world coordinates)
arTagCorners3D = [
     0.2, -0.2, 0.2;  % Bottom-left
     0.6, -0.2, 0.2;  % Bottom-right
     0.6,  0.2, 0.2;  % Top-right
     0.2,  0.2, 0.2]; % Top-left

% Plot the initial AR tag
h_patch = fill3(arTagCorners3D(:,1), arTagCorners3D(:,2), arTagCorners3D(:,3), 'r');
h_corners = plot3(arTagCorners3D(:,1), arTagCorners3D(:,2), arTagCorners3D(:,3), 'bo', 'MarkerSize', 5, 'LineWidth', 2);

% Initial joint config
q_start = [0, 0, 0, 0, 0, 0];

%{
% Define waypoints
q_waypoints = [
    q_start;                                        
    deg2rad([0 0 40.5 0 90 0]); 
    deg2rad([0 -19.5 64.9 0 90 0]);
    deg2rad([0 -19.5 112 0 90 0]);
    deg2rad([21.1 -11.4 130 -24.3 90 0]);
    tm5.model.ikine(transl(0.4, 0, 0.3) * trotx(pi)); 
];

% Move to initial desired pose using waypoints
for waypoint_idx = 1:size(q_waypoints, 1)
    q_target = q_waypoints(waypoint_idx, :);
    for i = 1:50

        q_current = (1 - i/50) * q + (i/50) * q_target;
        
        tm5.model.animate(q_current);
        pause(0.05);
        drawnow();
        
        q = q_current;
    end
end
%}
% AR tag y-axis translation per step
y_translation_step = -0.3 / 50; % Translate the square by -0.3 in 50 steps

% Move the AR tag and have the end-effector follow it
for i = 1:50
    % Move the AR tag by translating it along the y-axis
    arTagCorners3D(:, 2) = arTagCorners3D(:, 2) + y_translation_step;
    
    % Update AR tag plot
    set(h_patch, 'XData', arTagCorners3D(:,1), 'YData', arTagCorners3D(:,2), 'ZData', arTagCorners3D(:,3));
    set(h_corners, 'XData', arTagCorners3D(:,1), 'YData', arTagCorners3D(:,2), 'ZData', arTagCorners3D(:,3));
    
    % Update the desired end-effector pose to follow the AR tag
    T_desired = transl(0.4, 0 + i * y_translation_step, 0.3) * trotx(pi);
    
    q_desired = tm5.model.ikcon(T_desired, q); % Use the current q as the initial guess
    
    % Linear interpolation
    q_current = (1 - i/50) * q + (i/50) * q_desired;
    
    tm5.model.animate(q_current);
    pause(0.05);
    drawnow();
    
    q = q_current;
end









