clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Create the env
env = EnvironmentLoader();
steps = 100;

numSteps = 100; 
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.594, -0.863, 0.647);

homePosl = transl(0.821, 0, 1);
Tbl = transl(0.594, 0.423, 0.647);


% Compute joint angles for home and target poses
q1 = env.pr2Right.model.ikcon(homePosr);
q2 = env.pr2Right.model.ikcon(Tbr);

q3 = env.pr2Left.model.ikcon(homePosl);
q4 = env.pr2Left.model.ikcon(Tbl);

sr = lspb(0, 1, numSteps); % Trajectory blend between 0 and 1 over numSteps
qPrer = nan(numSteps, 7);

sl = lspb(0, 1, numSteps); % Trajectory blend between 0 and 1 over numSteps
qPrel = nan(numSteps, 7);

% Generate joint trajectory
for i = 1:numSteps
    qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
    qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
end
% Plot the motion between poses and draw a red line of the end-effector path
for i = 1:numSteps
env.pr2Right.model.animate(qPrer(i, :)); 
drawnow();
env.pr2Left.model.animate(qPrel(i, :)); 
drawnow();
end

hold off;



                 


