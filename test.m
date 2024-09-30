% Load PR2 model
PR2;
figure;
left.teach(qz, 'workspace', [-1.5 1.5 -1.5 1.5 -1 1.5]);

hold on;

right.teach(qz, 'workspace', [-1.5 1.5 -1.5 1.5 -1 1.5]);

hold off; 

