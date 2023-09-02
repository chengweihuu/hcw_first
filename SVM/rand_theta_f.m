function rands_theta=rand_theta_f()
T=[180,180,180,180,180,360];
s1=(2*rand(1,6)-1).*3.1415;
rands_theta=s1.*T/3.1415;
rands_theta=deg2rad(rands_theta);
end