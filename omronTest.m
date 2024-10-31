clc;
clf;

%tm5 = TM5.TM5700();

tm5Gripper = TM5.TM5700();
%pr2gripper = PR2.PR2LeftGripper();

q = zeros(1,6);
qGripper = zeros(1, 3);

%tm5.model.animate(q);
%tm5.model.teach();

tm5Gripper.model.animate(qGripper);
tm5Gripper.model.teach(qGripper);

%{
numSteps = 50;
leftStartPos = transl(0, -0.136, 1.092);

leftEndPos = transl(-0.115, -0.145, 0.207);

q3 = tm5.model.ikcon(leftStartPos);
q4 = tm5.model.ikcon(leftEndPos);

sl = lspb(0, 1, numSteps); % Left arm blend
qPrel = nan(numSteps, 6);

for i = 1:numSteps
    qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
end

% Plot the motion between poses and animate robot with grippers
for i = 1:numSteps
    %robot.checkPause(eStop); % Check for pause signal
    
    tm5.model.animate(qPrel(i, :)); 
    
    drawnow(); %
end
