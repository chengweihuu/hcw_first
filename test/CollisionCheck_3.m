function sign=CollisionCheck_3(configuration,obst)
% sign=0 means no collision occurred, sign=1 means collision occurred
keypoint_pos=Forward_kinematic(configuration,0);%Positive kinematics, 0 means output coordinate values for all 8 coordinate systems
keypoint_pos=keypoint_pos(:,1:3);
Bounding_box_parameters={60,60,60,50,50,50,50;...
    [keypoint_pos(1,:);keypoint_pos(2,:)],[keypoint_pos(2,:);keypoint_pos(3,:)],...
    [keypoint_pos(3,:);keypoint_pos(4,:)],[keypoint_pos(4,:);keypoint_pos(5,:)],...
    [keypoint_pos(5,:);keypoint_pos(6,:)],[keypoint_pos(6,:);keypoint_pos(7,:)],...
    [keypoint_pos(7,:);keypoint_pos(8,:)]}; 
obst_nei=obst; 
if isempty(obst_nei) 
    sign=0; 
else
    for i=1:7 
        threshold=Bounding_box_parameters{1,i}; 
        key1=Bounding_box_parameters{2,i}(1,:);
        key2=Bounding_box_parameters{2,i}(2,:); 
        vec1=key2-key1; 
        dominant_vector1=obst_nei-key1;
        s=(dominant_vector1*vec1')/(norm(vec1)^2); 
        a1=find(s<0); 
        if isempty(a1)
            sign=0;
        else
            c1=obst_nei(a1,:); 
            D1=sum((c1-key1).^2,2).^0.5; 
            d1=min(D1);        
            if d1<threshold
                sign=1;break;
            else
                sign=0;
            end
        end
        a2=find(s>=1); 
        if isempty(a2)
            sign=0;
        else
            c2=obst_nei(a2,:); 
            D2=sum((c2-key2).^2,2).^0.5;
            d2=min(D2);
            if d2<threshold
                sign=1;break;
            else
                sign=0;
            end
        end
        
        a31=find(s>0);a32=find(s<1);a3=intersect(a31,a32); 
        c3=obst_nei(a3,:);
        if isempty(c3)
            sign=0; 
        else
            for ii=1:size(c3,1) 
                %D3=norm(cross(key2-key1,c3(ii,:)-key1))/norm(key2-key1);
                LOA=norm(c3(ii,:)-key1);
                LOB=norm(c3(ii,:)-key2);
                LAB=norm(key2-key1);
                p=(LOA+LOB+LAB)/2;
                D3=2/LAB*sqrt(p*(p-LOA)*(p-LOB)*(p-LAB));
                if D3<threshold
                    sign=1;
                    break;
                else
                    sign=0;
                    continue;
                end
            end
            if sign==1
                break; 
            end
        end
    end
end
