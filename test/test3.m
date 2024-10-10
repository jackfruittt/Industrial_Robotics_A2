clc; clf; clear;


% Set Axis Bounds
axis ([-1.5 1.5 -1.5 1.5 0 2])
hold on

% % define the workspace vectors:
% %   qz         zero joint angle configuration
% %   qr         vertical 'READY' configuration
% %   qstretch   arm is stretched out in the X direction
% %   qn         arm is at a nominal non-singular configuration
% %
% qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
% %qr = [0 -pi/2 -pi/2 0 0 0 0]; % ready pose, arm up
% %qs = [0 0 -pi/2 0 0 0 0];
% %qn = [0 pi/4 pi/2 0 pi/4  0 0];
% 

PR2 = PR2(trotx(pi) * transl(0,0,-1.025));

for i = 0: -0.01: -0.3
    q = [i 0 0];
    PR2.base.model.animate(q);
    PR2.base.model.base
    PR2.leftArm.model.base = PR2.base.model.base.T * transl(-0.1, 0.18, 0.2+i);
    PR2.rightArm.model.base = PR2.base.model.base.T * transl(-0.1, -0.18, 0.2+i);
    PR2.leftArm.model.animate([0 0 0 0 0 0 0]);
    PR2.rightArm.model.animate([0 0 0 0 0 0 0]);
    pause(0.1)
end

for i = -0.3: 0.01: 0
    q = [i 0 0];
    PR2.base.model.animate(q);
    PR2.base.model.base
    PR2.leftArm.model.base = PR2.base.model.base.T * transl(-0.1, 0.18, 0.2+i);
    PR2.rightArm.model.base = PR2.base.model.base.T * transl(-0.1, -0.18, 0.2+i);
    PR2.leftArm.model.animate([0 0 0 0 0 0 0]);
    PR2.rightArm.model.animate([0 0 0 0 0 0 0]);
    pause(0.1)
end

PR2.plotPR2();