function axispos = Forward_kinematic(theta,n)
a=[0,-425,-392,0,0,0]; 
d=[89.2,0,0,109.3,94.75,82.5]; 
alpha=[pi/2,0,0,pi/2,-pi/2,0];
if n==1
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    axispos=T01(1:3,4)';
elseif n==2
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    point2_homogeneous_coordinates=(T01*[0;0;130;1])'; 
    axispos=point2_homogeneous_coordinates(1,1:3);
elseif n==3
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T04=T01*T14;
    point3_homogeneous_coordinates=(T04*[0;0;130;1])';
    axispos=point3_homogeneous_coordinates(1,1:3);
elseif n==4
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T04=T01*T14;
    axispos=T04(1:3,4)';
elseif n==5
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T45=Link_T(alpha(3),a(3),theta(3),d(3));
    T05=T01*T14*T45;
    axispos=T05(1:3,4)';
elseif n==6
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T45=Link_T(alpha(3),a(3),theta(3),d(3));
    T56=Link_T(alpha(4),a(4),theta(4),d(4));
    T06=T01*T14*T45*T56;
    axispos=T06(1:3,4)';
elseif n==7
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T45=Link_T(alpha(3),a(3),theta(3),d(3));
    T56=Link_T(alpha(4),a(4),theta(4),d(4));
    T67=Link_T(alpha(5),a(5),theta(5),d(5));
    T07=T01*T14*T45*T56*T67;
    axispos=T07(1:3,4)';
elseif n==8
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T45=Link_T(alpha(3),a(3),theta(3),d(3));
    T56=Link_T(alpha(4),a(4),theta(4),d(4));
    T67=Link_T(alpha(5),a(5),theta(5),d(5));
    T78=Link_T(alpha(6),a(6),theta(6),d(6));
    T08=T01*T14*T45*T56*T67*T78;
    axispos=T08(1:3,4)';
elseif n==0 %输出全部8个关键点的xyz坐标值
    axispos=zeros(8,3);
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    axispos(1,:)=T01(1:3,4)';
    
    point2_homogeneous_coordinates=(T01*[0;0;130;1])';
    axispos(2,:)=point2_homogeneous_coordinates(1,1:3);
    
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T04=T01*T14;
    point3_homogeneous_coordinates=(T04*[0;0;130;1])';
    axispos(3,:)=point3_homogeneous_coordinates(1,1:3);
    axispos(4,:)=T04(1:3,4)';
    
    T45=Link_T(alpha(3),a(3),theta(3),d(3));
    T56=Link_T(alpha(4),a(4),theta(4),d(4));
    T67=Link_T(alpha(5),a(5),theta(5),d(5));
    T78=Link_T(alpha(6),a(6),theta(6),d(6));
    T05=T04*T45;    T06=T05*T56;    T07=T06*T67;    T08=T07*T78;
    axispos(5,:)=T05(1:3,4)';
    axispos(6,:)=T06(1:3,4)';
    axispos(7,:)=T07(1:3,4)';
    axispos(8,:)=T08(1:3,4)';
    
elseif n==-1 
    kp1_7=[385,0,90,1;
        385,0,170,1;
        210,0,170,1;
        210,0,90,1;
        50,0,0,1;
        50,0,170,1;
        -50,0,170,1]';
    kp8_13=[342,0,65,1;
        342,0,-30,1;
        196,0,-30,1;
        196,0,65,1;
        40,0,-40,1;
        -40,0,-40,1]';
    kp14_15=[0,-50,-50,1;
        0,50,-50,1]';
    kp16_17=[0,50,-50,1;
        0,-50,-50,1]';
    kp18_19=[0,-40,0,1;
        0,40,0,1]';
    T01=Link_T(alpha(1),a(1),theta(1),d(1));
    T14=Link_T(alpha(2),a(2),theta(2),d(2));
    T45=Link_T(alpha(3),a(3),theta(3),d(3));
    T56=Link_T(alpha(4),a(4),theta(4),d(4));
    T67=Link_T(alpha(5),a(5),theta(5),d(5));
    T78=Link_T(alpha(6),a(6),theta(6),d(6));
    T04=T01*T14;    T05=T04*T45;    T06=T05*T56;
    T07=T06*T67;    T08=T07*T78;
    
    pos=[(T04*kp1_7)';(T05*kp8_13)';(T06*kp14_15)';(T07*kp16_17)';(T08*kp18_19)'];
    axispos=pos(:,1:3); 
end
end

function T = Link_T(u,a,v,d)
alpha=u;
theta=v;
T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
    sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
    0,sin(alpha),cos(alpha),d;
    0,0,0,1];
end