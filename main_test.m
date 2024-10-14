clc;
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


%}