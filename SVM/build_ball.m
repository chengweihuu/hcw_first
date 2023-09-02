function ball_point=build_ball(center,radius)
ball_point=[];
for theta=0:15:345
    for fai=-180:15:165
        ball_point=[ball_point;center(1)+radius*cosd(fai)*cosd(theta) ...
            center(2)+radius*cosd(fai)*sind(theta) center(3)+radius*sind(fai)];
    end
end
