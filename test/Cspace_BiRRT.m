function [TreeM,count,TreeExpansionFlag,TreeNnum,step,Interval]=...
    Cspace_BiRRT(start_node,final_node,TreeM,TreeN,obst,count,step,Interval)
%% Parameter initialization
num=[];
TreeExpansionFlag=0;
failedAttempts=1;
TreeNnum=1;
while failedAttempts<=100
    if rand()>0.618                        
        point_new_rand = rand_theta_f(Interval);
    else
        if size(TreeN.v,2)>4
            min_theta=[TreeN.v(end).theta1,TreeN.v(end).theta2,TreeN.v(end).theta3,...
                TreeN.v(end).theta4,TreeN.v(end).theta5,TreeN.v(end).theta6];
            for i=1:4 
                THETA = [TreeN.v(end-i).theta1,TreeN.v(end-i).theta2,TreeN.v(end-i).theta3,...
                    TreeN.v(end-i).theta4,TreeN.v(end-i).theta5,TreeN.v(end-i).theta6];
                min_theta=min_theta+THETA;    
            end
            point_new_rand = min_theta/5;
        else
            point_new_rand = final_node;
        end
    end
    
    % Step 2: Iterate through all the nodes of the tree and find the closest point from the tree to the node that is the parent of that node
    MinDis=theta2distance(point_new_rand,[TreeM.v(1).theta1,TreeM.v(1).theta2,TreeM.v(1).theta3,...
        TreeM.v(1).theta4,TreeM.v(1).theta5,TreeM.v(1).theta6]);
    minIndex = 1;
    for i = 2 : size(TreeM.v, 2)
        if TreeM.v(i).Ban == 1
            continue
        end
        % Distance between two nodes
        distance = theta2distance(point_new_rand,[TreeM.v(i).theta1,TreeM.v(i).theta2,TreeM.v(i).theta3,...
            TreeM.v(i).theta4,TreeM.v(i).theta5,TreeM.v(i).theta6]);
        % Save the minimum distance and index each time
        if(distance < MinDis)
            MinDis = distance;
            minIndex = i;
        end
    end
    point_near(1)=TreeM.v(minIndex).theta1;point_near(2)=TreeM.v(minIndex).theta2;
    point_near(3)=TreeM.v(minIndex).theta3;point_near(4)=TreeM.v(minIndex).theta4;
    point_near(5)=TreeM.v(minIndex).theta5;point_near(6)=TreeM.v(minIndex).theta6;
    step=step+0.005; 
    d_theta=(point_new_rand-point_near)*step; 
    point_new=point_near+d_theta; 
    
    %% -------Collision detection and Gaussian sampling-------------------------------------------------
    BBB=point_new;
    j=1;
    FFF=1 ; 
    CCC=0 ; 
    while FFF == 1 && j <= 100
        if CollisionCheck_SVM(point_new,obst) && step <= 0.02
            sample_theta1=GaussionSample(3);
            sample_theta=mean(sample_theta1) ;
            point_new=(BBB+point_near)/2+[sample_theta 0 0 0];
            j=j+1;
            continue;
        elseif CollisionCheck_SVM(point_new,obst)
            FFF = 0;
            CCC=1;
            step=0.005;
            j=j+1;
            failedAttempts=failedAttempts+1;
        else
            FFF=0;
        end
    end
    if CCC==1 
        continue;
    end
%%
    count = count + 1;
    % Step 4
    TreeM.v(count).theta1 = point_new(1) ; TreeM.v(count).theta2 = point_new(2) ; 
    TreeM.v(count).theta3 = point_new(3) ; TreeM.v(count).theta4 = point_new(4) ;
    TreeM.v(count).theta5 = point_new(5) ; TreeM.v(count).theta6 = point_new(6) ;
    TreeM.v(count).theta1Prev = point_near(1) ; TreeM.v(count).theta2Prev = point_near(2) ; 
    TreeM.v(count).theta3Prev = point_near(3) ; TreeM.v(count).theta4Prev = point_near(4) ; 
    TreeM.v(count).theta5Prev = point_near(5) ; TreeM.v(count).theta6Prev = point_near(6) ;
    TreeM.v(count).dist = MinDis ;          
    TreeM.v(count).indPrev = minIndex ;     
    TreeM.v(count).HaveSon = 0 ;
    TreeM.v(count).Ban = 0 ;
    TreeM.v(count).goal_cost = theta2distance(point_new,final_node) ; 
    TreeM.v(count).grad = 0 ;   
    TreeM.v(minIndex).HaveSon = TreeM.v(minIndex).HaveSon+1 ;
    TreeM.v(count).grad = (TreeM.v(count).goal_cost-TreeM.v(minIndex).goal_cost)/TreeM.v(minIndex).goal_cost ;
    % Sampling direction classification
    if point_new_rand(1) >0 && point_new_rand(2) >0 && point_new_rand(3) >0
        TreeM.v(count).Quadrant = 1 ;
    elseif point_new_rand(1) <0 && point_new_rand(2) >0 && point_new_rand(3) >0
        TreeM.v(count).Quadrant = 2 ;
    elseif point_new_rand(1) <0 && point_new_rand(2) <0 && point_new_rand(3) >0
        TreeM.v(count).Quadrant = 3 ;
    elseif point_new_rand(1) >0 && point_new_rand(2) <0 && point_new_rand(3) >0
        TreeM.v(count).Quadrant = 4 ;
    elseif point_new_rand(1) >0 && point_new_rand(2) >0 && point_new_rand(3) <0
        TreeM.v(count).Quadrant = 5 ;
    elseif point_new_rand(1) <0 && point_new_rand(2) >0 && point_new_rand(3) <0
        TreeM.v(count).Quadrant = 6 ;
    elseif point_new_rand(1) <0 && point_new_rand(2) <0 && point_new_rand(3) <0
        TreeM.v(count).Quadrant = 7 ;
    else
        TreeM.v(count).Quadrant = 8 ;
    end
    % 
    if TreeM.v(count).grad < 0
        Interval(1,TreeM.v(count).Quadrant) = Interval(1,TreeM.v(count).Quadrant)*1.12-0.04;
    elseif TreeM.v(count).grad < 0.1
        Interval(1,TreeM.v(count).Quadrant) = Interval(1,TreeM.v(count).Quadrant)*1.05-0.04;
    elseif TreeM.v(count).grad > 0.12 && TreeM.v(minIndex).grad >0.12
        Interval(1,TreeM.v(count).Quadrant) = Interval(1,TreeM.v(count).Quadrant)*0.88+0.02;
    end
    
    % Update Interval
    Interval(2,1)=Interval(1,1)/sum(Interval(1,:));
    Interval(3,1)=Interval(2,1);
    for i =2:size(Interval,2)
        Interval(2,i)=Interval(1,i)/sum(Interval(1,:));
        Interval(3,i)=Interval(3,i-1)+Interval(2,i);
    end
    for i = 2 : size(TreeN.v, 2)
        if TreeN.v(i).Ban == 1
            continue;
        end  
        distance1 = theta2distance(point_new,[TreeN.v(i).theta1,TreeN.v(i).theta2,TreeN.v(i).theta3,...
            TreeN.v(i).theta4,TreeN.v(i).theta5,TreeN.v(i).theta6]);
        if(distance1 < 50)
            TreeNnum=i;
            TreeExpansionFlag = 1;
            break;
        end
    end
    break;
end
end


    















