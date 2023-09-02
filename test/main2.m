clear;
clc;
%% Complex Environment
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
% initial setting
X_home=[0,0,0,0,0,0];
X_start=deg2rad([90 0 0 0 0 0]);
X_goal=deg2rad([-82.8 -32.4 43 -115.2 97.2 82.8]);
start_point=Forward_kinematic(X_start,8);
goal_point=Forward_kinematic(X_goal,8);
scatter3(start_point(1),start_point(2),start_point(3),'*','r'); hold on;
scatter3(goal_point(1),goal_point(2),goal_point(3),'*','g');hold on;

%       theta     d           a        alpha     offset
L1=Link([pi/2     89.2        0         pi/2       0     ]); 
L2=Link([-pi/2    0        -425            0       0     ]);
L3=Link([0        0        -392            0       0     ]);
L4=Link([0        109.3       0         pi/2       0     ]);
L5=Link([0        94.75       0        -pi/2       0     ]);
L6=Link([0        82.5        0            0       0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','FANUC 200iB/125L'); 

%% configure limit
l1u = 180.0;l1d = -180.0;
l2u = 180.0;l2d = -180.0;
l3u = 180.0;l3d = -180.0;
l4u = 180.0;l4d = -180.0;
l5u = 180.0;l5d = -180.0;
l6u = 360.0;l6d = -360.0;
robot.teach;

%% home
jbar=20;
q0=zeros(jbar,6);
q0(:,1)=linspace(X_home(1),X_start(1),jbar)';
q0(:,2)=linspace(X_home(2),X_start(2),jbar)';
q0(:,3)=linspace(X_home(3),X_start(3),jbar)';
q0(:,4)=linspace(X_home(4),X_start(4),jbar)';
q0(:,5)=linspace(X_home(5),X_start(5),jbar)';
q0(:,6)=linspace(X_home(6),X_start(6),jbar)';
robot.plot(q0); 
hold on;
xlabel('X axle');ylabel('Y axle');zlabel('Z axle'); 
hold on;
%% Random selection representing 8 quadrants
Interval1=[1 1 1 1 1 1 1 1;1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
Interval1(3,1)=Interval1(2,1);
for i= 2:size(Interval1,2)
    Interval1(3,i)=Interval1(3,i-1)+Interval1(2,i);
end

Interval2=[1 1 1 1 1 1 1 1;1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
Interval2(3,1)=Interval2(2,1);
for i= 2:size(Interval2,2)
    Interval2(3,i)=Interval2(3,i-1)+Interval2(2,i);
end
%% rrt-connect
Start2Goal_cost = theta2distance(X_start,X_goal);
Tree1.v=[];Tree2.v=[];
Tree1.v(1).theta1 = X_start(1); Tree1.v(1).theta2 = X_start(2); Tree1.v(1).theta3 = X_start(3); 
Tree1.v(1).theta4 = X_start(4); Tree1.v(1).theta5 = X_start(5); Tree1.v(1).theta6 = X_start(6);
Tree1.v(1).theta1Prev =X_start(1); Tree1.v(1).theta2Prev =X_start(2); Tree1.v(1).theta3Prev =X_start(3);
Tree1.v(1).theta4Prev =X_start(4); Tree1.v(1).theta5Prev =X_start(5); Tree1.v(1).theta6Prev =X_start(6);
Tree1.v(1).dist = 0;            % Euclidean distance
Tree1.v(1).indPrev = 0;         % Parent Node Index
count1 = 0;
Tree1.v(1).HaveSon = 0;         % This node has several children
Tree1.v(1).goal_cost = Start2Goal_cost;
Tree1.v(1).grad = 0;            % Expected distance gradient
Tree1.v(1).Ban = 0;             % if Ban=1,not travel throughout
Tree1.v(1).Quadrant = 0;        % sampling Quadrant
% --------------------------------------------------------------------------------------------------------------
Tree2.v(1).theta1 = X_goal(1); Tree2.v(1).theta2 = X_goal(2); Tree2.v(1).theta3 = X_goal(3); 
Tree2.v(1).theta4 = X_goal(4); Tree2.v(1).theta5 = X_goal(5); Tree2.v(1).theta6 = X_goal(6);
Tree2.v(1).theta1Prev =X_goal(1); Tree2.v(1).theta2Prev =X_goal(2); Tree2.v(1).theta3Prev =X_goal(3);
Tree2.v(1).theta4Prev =X_goal(4); Tree2.v(1).theta5Prev =X_goal(5); Tree2.v(1).theta6Prev =X_goal(6);
Tree2.v(1).dist = 0;            
Tree2.v(1).indPrev = 0;         
count2 = 0;
Tree2.v(1).HaveSon = 0;
Tree2.v(1).goal_cost = Start2Goal_cost;
Tree2.v(1).grad = 0;            
Tree2.v(1).Ban = 0;
Tree2.v(1).Quadrant = 0;       

step1=0.01 ;                    
step2=0.01 ;
HaveSonThr = 5 ;                % Maximum number of child nodes
tree1ExpansionFail=false;       % sets to true if expansion after set number of attempts fails
tree2ExpansionFail=false;       % Set to true if expansion fails after a set number of attempts


tic
while ~tree1ExpansionFail || ~tree2ExpansionFail  
    if ~tree1ExpansionFail 
        [Tree1,count1,Tree1Expansionflag,Tree2num,step1,Interval1]=...
            Cspace_BiRRT(X_start,X_goal,Tree1,Tree2,obs,count1,step1,Interval1); 
        for i =1:size(Tree1.v,2)
            if (Tree1.v(i).Ban == 0 && Tree1.v(i).HaveSon == HaveSonThr) || (Tree1.v(i).Ban == 0 && Tree1.v(i).grad > 0.2)
                Tree1.v(i).Ban = 1;
                Interval1(1,Tree1.v(i).Quadrant) = Interval1(1,Tree1.v(i).Quadrant)*0.9+0.02;
%                 % show points
%                 ABCDE=[Tree1.v(i).theta1,Tree1.v(i).theta2,Tree1.v(i).theta3,...
%                     Tree1.v(i).theta4,Tree1.v(i).theta5,Tree1.v(i).theta6];
%                 ABCDE_xyz=Forward_kinematic(ABCDE,8);
%                 scatter3(ABCDE_xyz(1),ABCDE_xyz(2),ABCDE_xyz(3),150,'.','black');
            end
        end
%         % Step  Connect the resulting new node to the parent node and show it on the graph
%         figure(1)
%         s_theta = [Tree1.v(end).theta1,Tree1.v(end).theta2,Tree1.v(end).theta3,Tree1.v(end).theta4,Tree1.v(end).theta5,Tree1.v(end).theta6]; 
%         p_theta = [Tree1.v(end).theta1Prev,Tree1.v(end).theta2Prev,Tree1.v(end).theta3Prev,Tree1.v(end).theta4Prev,Tree1.v(end).theta5Prev,Tree1.v(end).theta6Prev];
%         s_xyz=Forward_kinematic(s_theta,8);
%         p_xyz=Forward_kinematic(p_theta,8);
%         plot3([p_xyz(1),s_xyz(1)],[p_xyz(2),s_xyz(2)],[p_xyz(3),s_xyz(3)],'LineWidth',1,'color','green') ;grid on;
    end
    if Tree1Expansionflag
        break;
    end
    if ~tree2ExpansionFail
        [Tree2,count2,Tree2Expansionflag,Tree1num,step2,Interval2]=...
        Cspace_BiRRT(X_goal,X_start,Tree2,Tree1,obs,count2,step2,Interval2); % RRT 1 expands from goal towards source
        for i =1:size(Tree2.v,2)
            if (Tree2.v(i).Ban == 0 && Tree2.v(i).HaveSon == HaveSonThr) || (Tree2.v(i).Ban == 0 && Tree2.v(i).grad > 0.2)
                Tree2.v(i).Ban = 1;
                % Rewards and penalties for this direction
                Interval2(1,Tree2.v(i).Quadrant) = Interval2(1,Tree2.v(i).Quadrant)*0.9+0.02;

            end
        end
        % Step  Connect the resulting new node to the parent node and show it on the graph
%         figure(1)
%         s_theta = [Tree2.v(end).theta1,Tree2.v(end).theta2,Tree2.v(end).theta3,...
%             Tree2.v(end).theta4,Tree2.v(end).theta5,Tree2.v(end).theta6]; 
%         p_theta = [Tree2.v(end).theta1Prev,Tree2.v(end).theta2Prev,Tree2.v(end).theta3Prev,...
%             Tree2.v(end).theta4Prev,Tree2.v(end).theta5Prev,Tree2.v(end).theta6Prev];
%         s_xyz=Forward_kinematic(s_theta,8);
%         p_xyz=Forward_kinematic(p_theta,8);
%         plot3([p_xyz(1),s_xyz(1)],[p_xyz(2),s_xyz(2)],[p_xyz(3),s_xyz(3)],'LineWidth',1,'color','red') ;grid on;
    end
    if Tree2Expansionflag
        break;
    end
end
%% pursue one's traces
if Tree1Expansionflag
    path1.pos(1).theta1 = Tree2.v(Tree2num).theta1 ; path1.pos(1).theta2 = Tree2.v(Tree2num).theta2 ; 
    path1.pos(1).theta3 = Tree2.v(Tree2num).theta3 ; path1.pos(1).theta4 = Tree2.v(Tree2num).theta4 ;
    path1.pos(1).theta5 = Tree2.v(Tree2num).theta5 ; path1.pos(1).theta6 = Tree2.v(Tree2num).theta6 ;
    pathIndex = Tree2.v(Tree2num).indPrev ;
    j=0;
    % Going back to the beginning along the end
    while 1
        path1.pos(j + 2).theta1 = Tree2.v(pathIndex).theta1 ; path1.pos(j + 2).theta2 = Tree2.v(pathIndex).theta2;
        path1.pos(j + 2).theta3 = Tree2.v(pathIndex).theta3 ; path1.pos(j + 2).theta4 = Tree2.v(pathIndex).theta4;
        path1.pos(j + 2).theta5 = Tree2.v(pathIndex).theta5 ; path1.pos(j + 2).theta6 = Tree2.v(pathIndex).theta6;
        pathIndex = Tree2.v(pathIndex).indPrev;
        % Backtrack to the first node and exit
        if pathIndex == 1
            break
        end
        j = j + 1;
    end
    j = j + 1;
    % The last coordinate of the path is x_goal
    path1.pos(j+2).theta1 = X_goal(1);path1.pos(j+2).theta2 = X_goal(2);path1.pos(j+2).theta3 = X_goal(3);
    path1.pos(j+2).theta4 = X_goal(4);path1.pos(j+2).theta5 = X_goal(5);path1.pos(j+2).theta6 = X_goal(6);
    %------------------------------------------------------------------------------------------------------------
    j=1;
    path2.pos(1).theta1 = Tree1.v(end).theta1; path2.pos(1).theta2 = Tree1.v(end).theta2;
    path2.pos(1).theta3 = Tree1.v(end).theta3; path2.pos(1).theta4 = Tree1.v(end).theta4;
    path2.pos(1).theta5 = Tree1.v(end).theta5; path2.pos(1).theta6 = Tree1.v(end).theta6;
    % Index of the parent node of the last sampling point
    pathIndex = Tree1.v(end).indPrev;
    while 1
        path2.pos(j + 1).theta1 = Tree1.v(pathIndex).theta1;path2.pos(j + 1).theta2 = Tree1.v(pathIndex).theta2;
        path2.pos(j + 1).theta3 = Tree1.v(pathIndex).theta3;path2.pos(j + 1).theta4 = Tree1.v(pathIndex).theta4;
        path2.pos(j + 1).theta5 = Tree1.v(pathIndex).theta5;path2.pos(j + 1).theta6 = Tree1.v(pathIndex).theta6;
        pathIndex = Tree1.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j = j + 1;
    end
    path2.pos(end).theta1 = X_start(1); path2.pos(end).theta2 = X_start(2); path2.pos(end).theta3 = X_start(3);
    path2.pos(end).theta4 = X_start(4); path2.pos(end).theta5 = X_start(5); path2.pos(end).theta6 = X_start(6);
end 
if Tree2Expansionflag
    path1.pos(1).theta1 = Tree2.v(end).theta1 ; path1.pos(1).theta2 = Tree2.v(end).theta2 ; 
    path1.pos(1).theta3 = Tree2.v(end).theta3 ; path1.pos(1).theta4 = Tree2.v(end).theta4 ; 
    path1.pos(1).theta5 = Tree2.v(end).theta5 ; path1.pos(1).theta6 = Tree2.v(end).theta6 ;
    pathIndex = Tree2.v(end).indPrev ;
    j=0;
    while 1
        path1.pos(j + 2).theta1 = Tree2.v(pathIndex).theta1 ; path1.pos(j + 2).theta2 = Tree2.v(pathIndex).theta2;
        path1.pos(j + 2).theta3 = Tree2.v(pathIndex).theta3 ; path1.pos(j + 2).theta4 = Tree2.v(pathIndex).theta4;
        path1.pos(j + 2).theta5 = Tree2.v(pathIndex).theta5 ; path1.pos(j + 2).theta6 = Tree2.v(pathIndex).theta6;
        pathIndex = Tree2.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j = j + 1;
    end
    j = j + 1;
    path1.pos(j+2).theta1 = X_goal(1);path1.pos(j+2).theta2 = X_goal(2);path1.pos(j+2).theta3 = X_goal(3);
    path1.pos(j+2).theta4 = X_goal(4);path1.pos(j+2).theta5 = X_goal(5);path1.pos(j+2).theta6 = X_goal(6);
    %------------------------------------------------------------------------------------------------------------
    j=1;
    path2.pos(1).theta1 = Tree1.v(Tree2num).theta1; path2.pos(1).theta2 = Tree1.v(Tree2num).theta2; path2.pos(1).theta3 = Tree1.v(Tree2num).theta3;
    path2.pos(1).theta4 = Tree1.v(Tree2num).theta4; path2.pos(1).theta5 = Tree1.v(Tree2num).theta5; path2.pos(1).theta6 = Tree1.v(Tree2num).theta6;
    pathIndex = Tree1.v(Tree2num).indPrev;
    while 1
        path2.pos(j + 1).theta1 = Tree1.v(pathIndex).theta1;path2.pos(j + 1).theta2 = Tree1.v(pathIndex).theta2;
        path2.pos(j + 1).theta3 = Tree1.v(pathIndex).theta3;path2.pos(j + 1).theta4 = Tree1.v(pathIndex).theta4;
        path2.pos(j + 1).theta5 = Tree1.v(pathIndex).theta5;path2.pos(j + 1).theta6 = Tree1.v(pathIndex).theta6;
        pathIndex = Tree1.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j = j + 1;
    end
    path2.pos(end).theta1 = X_start(1); path2.pos(end).theta2 = X_start(2); path2.pos(end).theta3 = X_start(3);
    path2.pos(end).theta4 = X_start(4); path2.pos(end).theta5 = X_start(5); path2.pos(end).theta6 = X_start(6);
end 


%% Add path to q
q=[];
for i =size(path2.pos,2):-1:1
    q=[q;path2.pos(i).theta1,path2.pos(i).theta2,path2.pos(i).theta3,path2.pos(i).theta4,path2.pos(i).theta5,path2.pos(i).theta6];
end
for i =1:size(path1.pos,2)
    q=[q;path1.pos(i).theta1,path1.pos(i).theta2,path1.pos(i).theta3,path1.pos(i).theta4,path1.pos(i).theta5,path1.pos(i).theta6];
end
%% GetKeyPoint,input:q
q_final=[];
i=1;j=3;
q_final=[q_final;q(i,:)];
while j <= size(q,1)
    [q0,~] = jtraj(q(i,:),q(j,:),10);
    flag_collision=0;
    for k=1:size(q0,1)
        if CollisionCheck_SVM(q0(k,1:6),obs) == 1 
            flag_collision=1; % collision flag
        end
    end
    if flag_collision == 1
        i=j-1;
        j=i+2;
        q_final=[q_final;q(i,:)];
    else
        j=j+1;
    end
end
q_final=[q_final;q(end,:)];


%% animated demo
% figure(1)  
% %robot.plot(q_final(:,1:6));
% T = robot.fkine(q_final(:,1:6)); 
% p = transl(T);
% plot3(p(:,1),p(:,2),p(:,3),'LineWidth',2,'color','b') ;grid on;
% xlabel('X(mm)');ylabel('Y(mm)');zlabel('Z£¨mm£©');
% hold on
% for k=1:size(q_final,1)
%     if CollisionCheck_3(q_final(k,1:6),obs) 
%         q_final(k,7)=1;
%     else
%         q_final(k,7)=0;
%     end
% end
for k=1:size(q_final,1) 
    [~,q_final(k,7)]=CollisionCheck_SVM(q_final(k,1:6),obs) ;
end

%% quadratic optimization
GaussionNum=20;
Q_houxuan=cell(1,size(q_final,1));
Q_houxuan{1,1}=q_final(1,1:6);
for i=2:size(q_final,1)-1
    u=q_final(i,1:6);
    ZZZ=[GaussionSample(GaussionNum) zeros(GaussionNum,3)];
    QQ=repmat(u,GaussionNum,1)+ZZZ;
    Q_houxuan{1,i}=QQ;
end
Q_houxuan{1,end}=q_final(end,1:6);
for i=2:size(Q_houxuan,2)-1
    for j=1:size(Q_houxuan{1,i},1)
        [~,Q_houxuan{1,i}(j,7)]=CollisionCheck_SVM(Q_houxuan{1,i}(j,1:6),obs);
    end
end
distance_init=0;distance_init2=[0 0 0 0 0 0];
for i =1:size(q_final,1)-1 
    distance_init=distance_init+theta2distance(q_final(i,1:6),q_final(i+1,1:6));
    distance_init2=abs(q_final(i+1,1:6)-q_final(i,1:6))+distance_init2;
end 
distance_erci=3000;cishu=1;distance_erci2=[];
while 0.8*distance_init < distance_erci && cishu < 40
    Q_formal=q_final(1,1:6);
    for i=2:size(Q_houxuan,2)-1
        ypred=q_final(i,7);
        if ypred < 0.4 
            Q_formal=[Q_formal;q_final(i,1:6)];
        else
            Index=find(Q_houxuan{1,i}(:,7)<ypred,1);
            Q_formal=[Q_formal;Q_houxuan{1,i}(Index,1:6)];
            if rand()>0.5
                Q_houxuan{1,i}(Index,:)=[];
            end
        end
    end
    Q_formal=[Q_formal;q_final(end,1:6)];
    cishu=cishu+1;
    distance_erci=0;distance_erci2=[0 0 0 0 0 0];
    for i =1:size(Q_formal,1)-1
        distance_erci=distance_erci+theta2distance(Q_formal(i,1:6),Q_formal(i+1,1:6));
        distance_erci2=abs(Q_formal(i+1,1:6)-Q_formal(i,1:6))+distance_erci2;
    end
end

% Input: Q_formal
Q_formal2=Q_formal(1,:);
for i =2:size(Q_formal,1)-1
    AAA1=Q_formal(i-1,:)+0.9*(Q_formal(i,:)-Q_formal(i-1,:));
    AAA2=Q_formal(i,:)+0.1*(Q_formal(i+1,:)-Q_formal(i,:));
    if ~CollisionCheck_SVM(AAA1,obs) && ~CollisionCheck_SVM(AAA2,obs)
        Q_formal2=[Q_formal2;AAA1;AAA2];
    else
        Q_formal2=[Q_formal2;Q_formal(i,:)];
    end
end
Q_formal2=[Q_formal2;Q_formal(end,:)];
toc


distance_final=0;distance_final2=[0 0 0 0 0 0];
for i =1:size(Q_formal2,1)-1
    distance_final=distance_final+theta2distance(Q_formal2(i,1:6),Q_formal2(i+1,1:6));
    distance_final2=abs(Q_formal2(i+1,1:6)-Q_formal2(i,1:6))+distance_final2;
end 


figure(1)  
robot.plot(Q_formal2(:,1:6));
T = robot.fkine(Q_formal2(:,1:6)); 
p = transl(T);
plot3(p(:,1),p(:,2),p(:,3),'LineWidth',2,'color','r') ;grid on;
xlabel('X axle(mm)');ylabel('Y axle(mm)');zlabel('Z axle£¨mm£©');
hold on


    



