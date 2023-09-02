function jianliqiuti(pos,r)
[x,y,z]=sphere;
mesh(r*x+pos(1),r*y+pos(2),r*z+pos(3));
hold on;