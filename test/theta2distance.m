
function distance=theta2distance(theta1,theta2)
w=1;
theta1_xyz=Forward_kinematic(theta1,8);
theta2_xyz=Forward_kinematic(theta2,8);
distance=w*sqrt(sum((theta2_xyz-theta1_xyz).^2));
end