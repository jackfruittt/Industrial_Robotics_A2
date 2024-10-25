classdef pr2Laser
    methods(Static)
        function [firstCoord, lastCoord] = laser_scan(sensor_position, laser_rotation, radii, centerPoint)
            
            % Ellipsoid parameters
            [phi, theta] = meshgrid(linspace(0, 2*pi, 30), linspace(0, pi, 30));
            X = centerPoint(1) + radii(1) * sin(theta) .* cos(phi);
            Y = centerPoint(2) + radii(2) * sin(theta) .* sin(phi);
            Z = centerPoint(3) + radii(3) * cos(theta);
            
            % Plot the ellipsoid
            hold on;
            surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
            distanceLabel = text(0, 0, 0, 'Distance: ', 'FontSize', 10, 'Color', 'k');
            
            % Define the laser angles and range
            theta_range = linspace(-pi/4, pi/4, 100);
            rot = [cos(laser_rotation), 0, sin(laser_rotation); %Create a rotation matrix for Ry, usage of troty doesn't work
                   0, 1, 0;
                   -sin(laser_rotation), 0, cos(laser_rotation)];
            
            % Initialize output coordinates
            firstCoord = [];
            lastCoord = [];
            distanceList = []; % Store distances to find first and last non-inf points
            
            while true
                for theta = theta_range

                    laser_direction = [0, tan(theta), -1] * rot;  
                    distance = Inf;
                    
                    for t = 0:0.01:100 % Extend laser line infinitely
                        % Calculate the intersection point along the laser path
                        x_intersect = sensor_position(1) + t * laser_direction(1);
                        y_intersect = sensor_position(2) + t * laser_direction(2);
                        z_intersect = sensor_position(3) + t * laser_direction(3);
                        
                        % Check if the point lies on the surface of the ellipsoid
                        if (x_intersect - centerPoint(1))^2/(radii(1)^2) + ...
                           (y_intersect - centerPoint(2))^2/(radii(2)^2) + ...
                           (z_intersect - centerPoint(3))^2/(radii(3)^2) <= 1
                            % Intersection found
                            distance = norm([x_intersect, y_intersect, z_intersect] - sensor_position);
                            distanceList = [distanceList; distance]; % Store distance
                            break; 
                        end
                    end
                    
                    if distance < Inf
                        % Capture the intersection point
                        intersection_point = [x_intersect, y_intersect, z_intersect];
                        if isempty(firstCoord) % Get first intersection point
                            firstCoord = intersection_point;
                        end
                        lastCoord = intersection_point; % Get last intersection point
                        
                        plot3(intersection_point(1), intersection_point(2), intersection_point(3), 'ro', 'MarkerSize', 5); 
                        plot3([sensor_position(1), intersection_point(1)], ...
                              [sensor_position(2), intersection_point(2)], ...
                              [sensor_position(3), intersection_point(3)], 'k--'); 
                    end
                    
                    % Check distance is Inf
                    if distance == Inf
                        fprintf('Angle: %.2f degrees, Distance: Inf (No Intersection)\n', theta * (180/pi));
                    else
                        fprintf('Angle: %.2f degrees, Distance: %.2f units\n', theta * (180/pi), distance);
                    end
                    
                    set(distanceLabel, 'String', sprintf('Distance: %.2f', distance), ...
                        'Position', sensor_position + [0, 0, 0]); 
                    pause(0.1);
                end
                
                % Exit loop when co-ords founds
                if ~isempty(firstCoord) && ~isempty(lastCoord)
                    break; 
                end
            end  
        end              
    end
end


















