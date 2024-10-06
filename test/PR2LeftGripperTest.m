L1 = 67.568 / 1000;   % Link length a in meters
b = 48.972 / 1000;    % Distance b in meters
L0 = 34.708 / 1000;   % Distance L0 in meters
r = 91.5 / 1000;      % Distance r in meters
c = 26.01/1000;
theta0 = deg2rad(2.976); % Initial angle in radians
phi0 = deg2rad(29.987);  % Initial angle in radians

% DH parameters for the two-link gripper
%Link(         [θ      d        a        αlpha      sigma])
link(1) = Link([0      0        r        0           0]);
link(2) = Link([0      0        c        0           0]);

link(1).qlim = [-pi pi];
link(2).qlim = [-pi pi];

model = SerialLink(link, 'name', 'PR2 Gripper');

model.teach([0 0]);