function obstalceCircle=BuildBall(CenterPos,radius)
num_points=radius*10;
TempCenter=repmat(CenterPos,num_points,1);

theta = 2 * pi * rand(1, num_points);      
theta=theta';
phi = acos(2 * rand(1, num_points) - 1);   
phi=phi';

x = CenterPos(:,1) + radius * sin(phi) .* cos(theta);       
y = CenterPos(:,2) + radius * sin(phi) .* sin(theta);       
z = CenterPos(:,3) + radius * cos(phi);                     
obstalceCircle=[x,y,z];
%scatter3(obstalceCircle(:,1),obstalceCircle(:,2),obstalceCircle(:,3),".")
end