clc;
r = UR5();
q = [0 pi 0 0 0 0];
qt = transl(0.4, 0.2, 0.2);
qr = trotx(pi/2);
disp("SE3");
a = r.model.fkine(q)
size(a)

disp("Transformation matrix");
b = r.model.fkine(q).T
size(b)

disp("Rotational matrix");
c = r.model.fkine(q).R
size(c)

disp("Translation matrix");
d = r.model.fkine(q).t
size(d)

e = r.model.ikcon(qt)
size(e)

f = r.model.ikcon(b)
size(f)

g = r.model.ikcon(a)
size(g)

h = r.model.fkine(g)
size(h)

i = r.model.fkine(g).T
size(i)



