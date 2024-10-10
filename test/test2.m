clf;
clc;
hold on;

gripperl1 = PR2LeftGripper();
gripperr1 = PR2RightGripper();

gripperr1.model.teach();
gripperl1.model.teach();
gripperr1.model.animate();
gripperl1.model.animate();