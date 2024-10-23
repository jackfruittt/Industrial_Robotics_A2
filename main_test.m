
clf;

grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(90, 10);  

%{
obj.pr2Base = PR2.PR2Base();
obj.pr2LeftArm = PR2.PR2LeftArm(obj.pr2Base.model.base.T);
obj.pr2RightArm = PR2.PR2RightArm(obj.pr2Base.model.base.T);
q = [-0.3 0 0];
obj.pr2Base.model.animate(q);
obj.pr2LeftArm.model.teach();
obj.pr2RightArm.model.teach();
%}
% Load environment
fakeKnife = PlaceObject("plyFiles/Scenery/knife.ply", [0.89, -0.59, 0.86]);
env = EnvironmentLoader();
%obj.pr2LeftArm.model.teach();
%obj.pr2RightArm.model.teach();
%qStart = [0 0 0];
%startTr = robot.env.pr2Base.model.fkine(qStart);

%qEnd = [-0.3 0 0];
%endTr = robot.env.pr2Base.model.fkine(qEnd);

%robot.animatePR2Base(startTr, endTr, numSteps, eStop);

%{
NOTE ALL POSITIONS HERE (If spine raised add 0.3 in z)

%% Home Pos
r = [0.721 -0.180 0.825]
l = [0.721 0.180 0.825]

r = [-0.100 -0.680 1.146]
l = [-0.100  0.680 1.146]

%% Knife Pos
r ~= [0.557, -0.617, 0.863] 



[1.071 -0.180 1.125]


ALL WAYPOINTS NEW
q1 = deg2rad([-90 125 0 0 0 0 0]);
q2 = deg2rad([-85.6 125 0 0 0 0 0]);
q3 = deg2rad([-85.6 125 0 8.6 0 0 0]);
q4 = deg2rad([-80.2 125 0 13.8 0 0 0]);
q5 = deg2rad([-71.8 125 0 22.2 0 0 0]);
q6 = deg2rad([-67.2 125 0 35.9 0 0 0]);
q7 = deg2rad([-26.6 125 -13.9 35.9 0 0 0]);
q8 = deg2rad([-26.6 125 -53.8 35.9 0 0 0]);
q9 = deg2rad([-15.1 125 -53.8 74.3 0 0 0]);
q10 = deg2rad([-15.1 137 -53.8 74.3 0 0 0]);
q11 = deg2rad([-15.1 140 -53.8 74.3 0 0 0]);
q12 = deg2rad([-15.1 140 -211 74.3 0 0 0]);
q13 = deg2rad([-15.1 140 -211 74.3 -45 0 0]);
q14 = deg2rad([-15.1 140 -211 74.3 -90 0 0]);
q15 = deg2rad([-15.1 140 -211 74.3 -180 0 0]);
q16 = deg2rad([-15.1 140 -211 74.3 -180 0 0]);
q17 = deg2rad([-15.1 140 -211 74.3 -180 20 0]);
q18 = deg2rad([-15.1 140 -211 74.3 -180 45 0]);
q19 = deg2rad([-15.1 140 -211 74.3 -180 53.3 0]);
q20 = deg2rad([-5.18 140 -180 74.3 -180 53.3 0]);
q21 = deg2rad([-5.18 140 -180 90 -180 53.3 0]);
q22 = deg2rad([-5.18 131 -180 82.1 -180 53.3 0]);
q23 = deg2rad([-5.18 96.9 -180 75.5 -180 53.3 0])
%}