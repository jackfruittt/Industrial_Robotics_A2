clc;
clf;

grid on;
hold on
axis([-4 4 -2 2 -2 2]);

% Create the PR2 robot model
pr2Left = PR2Left();

% Set up lighting and shading
light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud;  
material shiny;   
camlight('headlight');
camlight('left');

qz = [0 pi/2 0 0 0 0 0];
pr2Left.model.teach();

hold off;


                 


