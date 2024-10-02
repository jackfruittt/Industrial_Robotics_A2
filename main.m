clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Load environment
env = EnvironmentLoader();
view(30, 20);  

function animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps)
    offset = troty(-pi/2) * transl(0.05, 0, 0); 
    
    q1 = env.pr2Right.model.ikcon(homePosr);
    q2 = env.pr2Right.model.ikcon(Tbr);
    
    q3 = env.pr2Left.model.ikcon(homePosl);
    q4 = env.pr2Left.model.ikcon(Tbl);
    
    % LSPB trajectory for smooth transition
    sr = lspb(0, 1, numSteps); % Right arm blend
    sl = lspb(0, 1, numSteps); % Left arm blend
    
    qPrer = nan(numSteps, 7);
    qPrel = nan(numSteps, 7);
    
    for i = 1:numSteps
        qPrer(i, :) = (1 - sr(i)) * q1 + sr(i) * q2;
        qPrel(i, :) = (1 - sl(i)) * q3 + sl(i) * q4;
    end

    % Plot the motion between poses and animate robot with grippers
    for i = 1:numSteps
        env.pr2Right.model.animate(qPrer(i, :)); 
        env.pr2Left.model.animate(qPrel(i, :)); 
        
        T_leftEndEffector = env.pr2Left.model.fkine(qPrel(i, :)).T;
        T_rightEndEffector = env.pr2Right.model.fkine(qPrer(i, :)).T;

        env.gripperl1.model.base = T_leftEndEffector  * offset;
        env.gripperr1.model.base = T_leftEndEffector * offset;
        env.gripperl2.model.base = T_rightEndEffector * offset;
        env.gripperr2.model.base = T_rightEndEffector * offset;

        env.gripperl1.model.animate([0 0]);
        env.gripperr1.model.animate([0 0]);
        env.gripperl2.model.animate([0 0]);
        env.gripperr2.model.animate([0 0]);
        
        drawnow(); % Update the figure
    end
end

numSteps = 100; 
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.594, -0.863, 0.647);
homePosl = transl(0.821, 0, 1);
Tbl = transl(0.594, 0.423, 0.647);

animatePR2ArmsAndGrippers(env, homePosr, Tbr, homePosl, Tbl, numSteps);

hold off;




                 


