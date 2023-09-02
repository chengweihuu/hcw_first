function  A = Getcolumn(axis,r,step)

column_num = size(axis,1);%Ô²ÖùÊýÁ¿

Hight = sqrt(sum((axis(1,1:3)-axis(1,4:6)).*(axis(1,1:3)-axis(1,4:6)),2)); % n*1

axis_vec = (axis(:,4:6) - axis(:,1:3))./Hight;  % n*3

for i = 1:column_num
    avi = axis_vec(i,:);
    trans(:,:,i) = [ [null(avi) avi' axis(i,1:3)'];0 0 0 1 ]; 
end

% Point cloud of a standard cylindrical surface
x = repmat(cos((1:8)*2*pi/8),column_num,1).*r'; % n*50
y = repmat(sin((1:8)*2*pi/8),column_num,1).*r'; % n*50


%% Superimposed along the vector direction

x = [x x(:,1)];
y = [y y(:,1)];


for i = 1:column_num
    lp(:,:,i) = trans(:,:,i)*[x(i,:);y(i,:);zeros(1,9);ones(1,9)];
end
A = lp(1:3,:);
A = A';
A0 = A;
vec = (axis(:,4:6) - axis(:,1:3))/norm(axis(:,4:6) - axis(:,1:3)); 
for i = 0:step: Hight
    B = A0+vec*i; 
    A = [A;B];
end

% hold on ;scatter3(A(:,1),A(:,2),A(:,3),'.r'); hold on;

end
