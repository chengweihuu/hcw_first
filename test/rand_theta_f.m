%Randomize the joint angles, convert the angle system to radians, and take the angles of the first three axes according to the quadrant.
function rands_theta=rand_theta_f(Interval)
if rand()<Interval(3,1)
    Sign=[1 1 1];
elseif rand()<Interval(3,2)
    Sign=[-1 1 1];
elseif rand()<Interval(3,3)
    Sign=[-1 -1 1];
elseif rand()<Interval(3,4)
    Sign=[1 -1 1];
elseif rand()<Interval(3,5)
    Sign=[1 1 -1];
elseif rand()<Interval(3,6)
    Sign=[-1 1 -1];
elseif rand()<Interval(3,7)
    Sign=[-1 -1 -1];
else
    Sign=[1 -1 -1];
end
T=[180,180,180,180,180,360];
s1=(2*rand(1,6)-1).*3.1415;
s2=Sign.*(rand(1,3)).*3.1415;
s1(1:3)=s2(1:3);
rands_theta=s1.*T/3.1415;
rands_theta=deg2rad(rands_theta);
end