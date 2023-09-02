function cylinder_point=build_cylinder(center,height,radius)
cylinder_point=[];
for r=0:20:radius
    for dushu=0:15:350
        cylinder_point=[cylinder_point;center(1)+r*cosd(dushu) center(2)+r*sind(dushu) center(3)+0];
    end
end
for i=20:20:height-20
    for dushu=0:10:350
        cylinder_point=[cylinder_point;center(1)+radius*cosd(dushu) center(2)+radius*sind(dushu) center(3)+i];
    end
end
for r=0:20:radius
    for dushu=0:15:350
        cylinder_point=[cylinder_point;center(1)+r*cosd(dushu) center(2)+r*sind(dushu) center(3)+height];
    end
end