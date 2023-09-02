%% GetKeyPoint,input£ºq
% for i = 1:size(q,1)-2
%     for j = i:size(q,1)
%         [q0,~] = jtraj(q(i,:),q(j,:),10);
%   
load q
load obs
q_final=[];
i=1;j=3;
q_final=[q_final;q(i,:)];
while j <= size(q,1)
    [q0,~] = jtraj(q(i,:),q(j,:),10);
    flag_collision=0;
    for k=1:size(q0,1)
        if CollisionCheck_3(q0(k,1:6),obs) == 1 
            flag_collision=1; 
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

figure(1)  
robot.plot(q_final(:,1:6));
T = robot.fkine(q_final(:,1:6)); 
p = transl(T);
plot3(p(:,1),p(:,2),p(:,3),'LineWidth',1,'color','g') ;grid on;
xlabel('XÖá(mm)');ylabel('YÖá(mm)');zlabel('ZÖá£¨mm£©');
hold on
figure(2)
robot.plot(q(:,1:6));
T = robot.fkine(q(:,1:6)); 
p = transl(T);
plot3(p(:,1),p(:,2),p(:,3),'LineWidth',1,'color','b') ;grid on;
xlabel('XÖá(mm)');ylabel('YÖá(mm)');zlabel('ZÖá£¨mm£©');
hold on