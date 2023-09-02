function obst=jianlichangfangti(x,y,z)
obst=[];
for i=1:size(y,1)
    a=size(z,1);
    temp=[repmat(x(1),a,1),repmat(y(i),a,1),z];
    obst=[obst;temp];
    temp=[repmat(x(end),a,1),repmat(y(i),a,1),z];
    obst=[obst;temp];
end
for i=1:size(x,1)
    a=size(z,1);
    temp=[repmat(x(i),a,1),repmat(y(1),a,1),z];
    obst=[obst;temp];
    temp=[repmat(x(i),a,1),repmat(y(end),a,1),z];
    obst=[obst;temp];
end
for i=1:size(x,1)
    a=size(y,1);
    temp=[repmat(x(i),a,1),y,repmat(z(1),a,1)];
    obst=[obst;temp];
    temp=[repmat(x(i),a,1),y,repmat(z(end),a,1)];
    obst=[obst;temp];
end
end