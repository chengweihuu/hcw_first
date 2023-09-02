function position_read()
clear all
clc
 
disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%
r1=[];
r2=[];
r3=[];
r4=[];
r5=[];
r6=[];
 
if (clientID>-1)
    disp('Connected to remote API server');
% vrep.simxSynchronous(clientID,true);%
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%启动仿真
 
  %
 
  [res,NiryoOneJoint1] = vrep.simxGetObjectHandle(clientID,'j1_space',vrep.simx_opmode_oneshot_wait);
  [res,NiryoOneJoint2] = vrep.simxGetObjectHandle(clientID,'j2_space',vrep.simx_opmode_oneshot_wait);
  [res,NiryoOneJoint3] = vrep.simxGetObjectHandle(clientID,'j3_space',vrep.simx_opmode_oneshot_wait);
  [res,NiryoOneJoint4] = vrep.simxGetObjectHandle(clientID,'j4_space',vrep.simx_opmode_oneshot_wait);
  [res,NiryoOneJoint5] = vrep.simxGetObjectHandle(clientID,'j5_space',vrep.simx_opmode_oneshot_wait);
  [res,NiryoOneJoint6] = vrep.simxGetObjectHandle(clientID,'j6_space',vrep.simx_opmode_oneshot_wait);
 
% 
% NiryoOneJoint=zeros(1,6);
%  for i=1:6
%      [res,NiryoOneJoint(i)] = vrep.simxGetObjectHandle(clientID,['handle_joint' num2str(i)],vrep.simx_opmode_blocking);
%  end
 
    while(vrep.simxGetConnectionId(clientID) ~= -1), % while v-rep connection is still active
         t = vrep.simxGetLastCmdTime(clientID) / 1000.0; % get current simulation time
         if (t > 30) break;
         end 
 
  [res,r1angle]=vrep.simxGetJointPosition(clientID,NiryoOneJoint1,vrep.simx_opmode_oneshot_wait);
  [res,r2angle]=vrep.simxGetJointPosition(clientID,NiryoOneJoint2,vrep.simx_opmode_oneshot_wait);
  [res,r3angle]=vrep.simxGetJointPosition(clientID,NiryoOneJoint3,vrep.simx_opmode_oneshot_wait);
  [res,r4angle]=vrep.simxGetJointPosition(clientID,NiryoOneJoint4,vrep.simx_opmode_oneshot_wait);
  [res,r5angle]=vrep.simxGetJointPosition(clientID,NiryoOneJoint5,vrep.simx_opmode_oneshot_wait);
  [res,r6angle]=vrep.simxGetJointPosition(clientID,NiryoOneJoint6,vrep.simx_opmode_oneshot_wait);
 
  r1= [r1 r1angle];
  r2= [r2 r2angle];
  r3= [r3 r3angle];
  r4= [r4 r4angle];
  r5= [r5 r5angle];
  r6= [r6 r6angle];
 
    end
 
r=[r1' r2' r3' r4' r5' r6'];
 

fid=fopen('angle.txt','wt');
[m,n]=size(r);
for i=1:1:m
for j=1:1:n
if j==n
fprintf(fid,'%g\n',r(i,j));
else
fprintf(fid,'%g\t',r(i,j));
end
end
end
fclose(fid);
 
% Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID);
 
 %Now close the connection to V-rep
  vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); %停止仿真
  vrep.simxFinish(clientID);
  
else
    disp('Failed connecting to remote API server');
end
 
 
vrep.delete(); % call the destructor!
    
disp('Program ended');
end