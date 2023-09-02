clear;
clc;
% create complex environment
x=[-1000:50:1000]';y=[300:25:400]';z=[-400:25:1000]';obst1=jianlichangfangti(x,y,z);
x=[-400:50:400]';y=[300:25:400]';z=[200:25:600]';obst2=jianlichangfangti(x,y,z);
obs_jiao=intersect(obst1,obst2,'row');
obs_bing=union(obst1,obst2,'row');
obs1=setdiff(obs_bing,obs_jiao,'row');
x=[300:30:500]';y=[0:40:200]';z=[-400:40:400]';obst3=jianlichangfangti(x,y,z);
x=[-700:30:-400]';y=[-400:30:0]';z=[-200:30:600]';obst4=jianlichangfangti(x,y,z);
obst5=build_cylinder([400 -500 -200],600,200);
obst6=build_ball([0 -400 800 ],150);
obs2=[obst3;obst4;obst5;obst6];

scatter3(obs1(:,1),obs1(:,2),obs1(:,3),'.','blue');hold on
scatter3(obs2(:,1),obs2(:,2),obs2(:,3),'.','black');hold on
obs=[obs1;obs2];
hold on
X_home=[0,0,0,0,0,0]; 
X_start=deg2rad([90 0 0 0 0 0]);
X_goal=deg2rad([-82.8 -32.4 43 -115.2 97.2 82.8]);
start_point=Forward_kinematic(X_start,8);
goal_point=Forward_kinematic(X_goal,8);
scatter3(start_point(1),start_point(2),start_point(3),'*','r'); hold on;
scatter3(goal_point(1),goal_point(2),goal_point(3),'*','g');hold on;

%% Modeling the robot
%       theta     d           a        alpha     offset
L1=Link([pi/2     89.2        0         pi/2       0     ]); 
L2=Link([-pi/2    0        -425            0       0     ]);
L3=Link([0        0        -392            0       0     ]);
L4=Link([0        109.3       0         pi/2       0     ]);
L5=Link([0        94.75       0        -pi/2       0     ]);
L6=Link([0        82.5        0            0       0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','FANUC 200iB/125L'); 

%% Set rotation angle limits 
l1u = 180.0;l1d = -180.0;
l2u = 180.0;l2d = -180.0;
l3u = 180.0;l3d = -180.0;
l4u = 180.0;l4d = -180.0;
l5u = 180.0;l5d = -180.0;
l6u = 360.0;l6d = -360.0;
robot.teach;
%%
DCL=[];ECL=[];allpoint=[];
X_mid1 = X_start+(X_goal-X_start)/3 ;
X_mid2 = X_start+2*(X_goal-X_start)/3 ;

Sigma = [0.25 0 0; 0 0.25 0;0 0 0.25]; 
mu =X_start(1:3); 
dim = size(Sigma, 1); 
n = 600; 
L_chol = chol(Sigma, 'lower'); 
tmp = randn(n, dim); 
mu_vec = repmat(mu, n, 1); 
samples = (L_chol*tmp')'; 
theta1 = mu_vec + samples;

Sigma = [0.25 0 0; 0 0.25 0;0 0 0.25]; 

mu =X_goal(1:3); % average value
dim = size(Sigma, 1); % sample dimension
n = 600; % sample size
L_chol = chol(Sigma, 'lower'); % get the loweer triangle matrix, L_chol*Lchol' = Sigma
tmp = randn(n, dim); % tmp ~ N(0, I)
mu_vec = repmat(mu, n, 1); % size(mu_vec) = (n, dim)
samples = (L_chol*tmp')'; % samples ~ C*N(0,I), size(samples) = (n, dim)
theta2 = mu_vec + samples;

Sigma = [0.25 0 0; 0 0.25 0;0 0 0.25]; 
mu =X_mid1(1:3); 
dim = size(Sigma, 1); 
n = 600; 
L_chol = chol(Sigma, 'lower'); % get the loweer triangle matrix, L_chol*Lchol' = Sigma
tmp = randn(n, dim); % tmp ~ N(0, I)
mu_vec = repmat(mu, n, 1); % size(mu_vec) = (n, dim)
samples = (L_chol*tmp')'; % samples ~ C*N(0,I), size(samples) = (n, dim)
theta3 = mu_vec + samples;

Sigma = [0.25 0 0; 0 0.25 0;0 0 0.25]; 
mu =X_mid2(1:3); 
dim = size(Sigma, 1); 
n = 600; 
L_chol = chol(Sigma, 'lower'); % get the loweer triangle matrix, L_chol*Lchol' = Sigma
tmp = randn(n, dim); % tmp ~ N(0, I)
mu_vec = repmat(mu, n, 1); % size(mu_vec) = (n, dim)
samples = (L_chol*tmp')'; % samples ~ C*N(0,I), size(samples) = (n, dim)
theta4 = mu_vec + samples;

sample_theta=[theta1;theta2;theta3;theta4];
for i = 1:4*n
    A=rand_theta_f;
    sample_theta(i,4:6)=A(4:6);
end

for i =1:size(sample_theta,1)
    if CollisionCheck_3(sample_theta(i,1:6),obs)
        sample_theta(i,7) = -1;
        sample_theta(i,8:10) = Forward_kinematic(sample_theta(i,1:6),8);
    else
        sample_theta(i,7) = 1;
        sample_theta(i,8:10) = Forward_kinematic(sample_theta(i,1:6),8);
    end
end

for i =1:size(sample_theta,1)
    if sample_theta(i,7) == -1
        DCL=[DCL;sample_theta(i,8:10) sample_theta(i,7)];
    else
        ECL=[ECL;sample_theta(i,8:10) sample_theta(i,7)];
    end
end

% Take the first 300 as ECL_training and DCL_training
ECL_training = ECL(1:3:end,:);
DCL_training = DCL(1:3:end,:);
data = [DCL_training;ECL_training];
scatter3(data(:,1),data(:,2),data(:,3),'.');



















