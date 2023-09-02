function sample_theta=GaussionSample()
% p1=sample;p2=[-sample(2),sample(1),0];p3=cross(p1,p2);
% n1=p1/norm(p1);
% n2=p1/norm(p2);
% n3=p1/norm(p3);
% Sigma=[n1' n2' n3']*[0.1 0 0;0 0.1 0;0 0 0.1]*[n1' n2' n3'];
Sigma = [1/100 0 0; 0 1/100 0;0 0 1/100]; 
mu = [0, 0, 0];  
dim = size(Sigma, 1); 
n = 1; 
L_chol = chol(Sigma, 'lower'); % get the loweer triangle matrix, L_chol*Lchol' = Sigma
tmp = randn(n, dim); % tmp ~ N(0, I)
mu_vec = repmat(mu, n, 1); % ,size(mu_vec) = (n, dim)
samples = (L_chol*tmp')'; % samples ~ C*N(0,I), size(samples) = (n, dim)
theta = mu_vec + samples;
sample_theta=theta;
%scatter3(sample_theta(:,1),sample_theta(:,2),sample_theta(:,3),'.');
end