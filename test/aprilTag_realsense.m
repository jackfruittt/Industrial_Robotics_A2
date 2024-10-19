% Initialize camera object
camera = realsense.pipeline();
config = realsense.config();
config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
config.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);

% Load the camera intrinsic parameters
data = load("camIntrinsicsAprilTag.mat");
intrinsics = data.intrinsics;  % Camera intrinsics
tagSize = 0.1;  % Specify tag size in meters

% Start streaming
camera.start(config);

fig1 = figure(1);  % First figure for the 3D position of the AR tag
clf(fig1); 
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
xlim([-1, 1]); 
ylim([-1, 1]);
zlim([0, 2]);  
view(3);  
title('AR Tag Position Relative to Camera');
rotate3d on; 

% Plot the camera at the origin
plot3(0, 0, 0, 'bo', 'MarkerSize', 10, 'DisplayName', 'Camera');

hTagPosition = [];
hXAxis = [];
hYAxis = [];
hZAxis = [];
hTagText = [];

[sphereX, sphereY, sphereZ] = sphere(); 

fig2 = figure(2); 
clf(fig2); 
title('Camera Feed with AR Tag Detection');

% Ccapture and process frames
for i = 1:200 
 
    frames = camera.wait_for_frames();
    color_frame = frames.get_color_frame();
    depth_frame = frames.get_depth_frame();
    
    % Convert to MATLAB image format
    color_image = permute(reshape(color_frame.get_data(), [3, color_frame.get_width(), color_frame.get_height()]), [3 2 1]);
    depth_image = reshape(depth_frame.get_data(), [depth_frame.get_width(), depth_frame.get_height()]);
    
    % Convert the color image to grayscale for AprilTag detection
    I = rgb2gray(color_image);
    
    % Detect AprilTags in the image
    tagFamily = "tag36h11";  
    [id, loc, pose] = readAprilTag(I, tagFamily, intrinsics, tagSize);
    
    %figure(2);  % Switch to figure 2
    imshow(color_image);  % Display the color image
    hold on;
    
    if ~isempty(id)
        % Extract the 3D position (translation) of the first detected tag
        tagPosition = pose(1).Translation;  
        
        % Extract the rotation matrix from the pose
        R = pose(1).R(1:3, 1:3);  % Extract 3x3 rotation matrix
        
        % Define unit vectors 
        xAxis = [0.1; 0; 0];  
        yAxis = [0; 0.1; 0];
        zAxis = [0; 0; 0.1];
        
        % Rotate the axes according to the tag's orientation
        rotatedXAxis = R * xAxis;
        rotatedYAxis = R * yAxis;
        rotatedZAxis = R * zAxis;
        
        % Display the detected tag's ID and position in the command window
        disp("Detected Tag ID: " + id(1));
        disp("Tag Position (X, Y, Z): " + num2str(tagPosition));
        
        % Highlight the detected AR tag in the camera feed (figure 2)
        markerRadius = 8;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,1), repmat(markerRadius, numCorners, 1)];
        color_image = insertShape(color_image, "FilledCircle", markerPosition, ShapeColor="red", Opacity=1);
        
        % Insert the tag ID in the camera feed
        color_image = insertText(color_image, loc(1,:,1), ['ID: ' num2str(id(1))], 'FontSize', 18, 'TextColor', 'yellow', 'BoxOpacity', 0.8);
        
        % Visualize the transform frames in the 2D image
        worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];  % Origin, X-axis, Y-axis, Z-axis
        imagePoints = world2img(worldPoints, pose(1), intrinsics);
        
        % Draw the axes on the 2D image
        color_image = insertShape(color_image, 'Line', [imagePoints(1,:) imagePoints(2,:)], 'Color', 'red', 'LineWidth', 4);  % X-axis
        color_image = insertShape(color_image, 'Line', [imagePoints(1,:) imagePoints(3,:)], 'Color', 'green', 'LineWidth', 4);  % Y-axis
        color_image = insertShape(color_image, 'Line', [imagePoints(1,:) imagePoints(4,:)], 'Color', 'blue', 'LineWidth', 4);  % Z-axis
        
        imshow(color_image);  % Refresh the displayed image in figure 2
        
        %figure(1);  % Switch to figure 1 (3D plot)
        
        if ~isempty(hTagPosition)
            delete(hTagPosition);
            delete(hXAxis);
            delete(hYAxis);
            delete(hZAxis);
            delete(hTagText);
        end
        
        % Scale the sphere to a reasonable size (e.g., 0.05 meters in radius)
        hTagPosition = surf(tagPosition(1) + 0.05 * sphereX, ...
                            tagPosition(2) + 0.05 * sphereY, ...
                            tagPosition(3) + 0.05 * sphereZ, ...
                            'EdgeColor', 'none', 'FaceColor', 'red');  % Sphere
        
        % Visualize the rotated transform frames in 3D space
        hXAxis = quiver3(tagPosition(1), tagPosition(2), tagPosition(3), ...
                         rotatedXAxis(1), rotatedXAxis(2), rotatedXAxis(3), 'r', 'LineWidth', 2);  % X-axis (red)
        hYAxis = quiver3(tagPosition(1), tagPosition(2), tagPosition(3), ...
                         rotatedYAxis(1), rotatedYAxis(2), rotatedYAxis(3), 'g', 'LineWidth', 2);  % Y-axis (green)
        hZAxis = quiver3(tagPosition(1), tagPosition(2), tagPosition(3), ...
                         rotatedZAxis(1), rotatedZAxis(2), rotatedZAxis(3), 'b', 'LineWidth', 2);  % Z-axis (blue)
        
        % Display the tag ID
        hTagText = text(tagPosition(1), tagPosition(2), tagPosition(3), ['ID: ' num2str(id(1))], 'FontSize', 12, 'Color', 'yellow');
        
        drawnow(); 
    else
        imshow(color_image);
    end
    
    %pause(0.1); 
end

% Stop streaming
camera.stop();










