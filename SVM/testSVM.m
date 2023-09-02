% function iscollision = testSVM(data)
data = [DCL_training;ECL_training];
Xapp = data(:,1:3); Yapp = data(:,end);

[nApp, p] = size(Xapp);
moyenne = mean(Xapp);  
variance = std(Xapp);  
% Center and reduce
Xapp = (Xapp - ones(nApp, 1) * moyenne) ./ (ones(nApp, 1) * variance);


%% Calculate K kernel and G matrix
kernel = 'gaussian'; 
kerneloption = 2;

K = svmkernel(Xapp, kernel, kerneloption, Xapp);

C1 = 1000;
CMoins1 = 1000;

G = (Yapp*Yapp').*K; % Gram matrix
vecteurC = zeros(nApp, 1);
vecteurC(find(Yapp == 1)) = C1;
vecteurC(find(Yapp == -1)) = CMoins1;
matriceC = diag(1 ./ vecteurC); 

H = G + matriceC;

e = ones(nApp,1);
epsilon = 10^-5;

%% Solve with monqp
tic

lambda = eps^.5;
lambda = 1;
[alpha2, b, pos] = monqp(H, e, Yapp, 0, inf, lambda, 0);

% pos correspond aux positions des alphas differents de 0. alpha = a(pos)
a4 = zeros(nApp, 1);
a4(pos) = alpha2;%【alpha是啥呀】
toc

% %% 预测
% collision_pro_d = zeros(size(DCL_test,2),1);%【预测的是DCL_test的数据】
% for i = 1:size(DCL_test,1)
%     tem1 = DCL_test(i,:);
%     test = tem1;
%     Xtest = test(:,1:3);
%     Ytest = test(:,end);
%     [nTest, p] = size(Xtest);
%     Xtest = (Xtest - ones(nTest, 1) * moyenne) ./ (ones(nTest, 1) * variance);%标准化
%     Kgrid = svmkernel(Xtest, kernel, kerneloption, Xapp(pos, :));%【pos是高斯帅选过的点】
%     ypred = Kgrid*(Yapp(pos).*alpha) + b;  % 关键点到平面的距离【alpha是系数】
%     % Calculate new labels
%     ypred(find(ypred > 0)) = 1;
%     ypred(find(ypred < 0)) = -1;
    
    % Calculate the error rate (in percent)
%     erreur = (length(find(ypred - Ytest == 0)) / nTest) * 100;   % 碰撞的概率
%     collision_pro_d(i,1) = ypred;
% end

% %% 预测
% collision_pro_e = zeros(size(ECL_test,2),1);%【预测的是DCL_test的数据】
% for i = 1:size(ECL_test,1)
%     tem1 = ECL_test(i,:);
%     test = tem1;
%     Xtest = test(:,1:3);
%     Ytest = test(:,end);
%     [nTest, p] = size(Xtest);
%     Xtest = (Xtest - ones(nTest, 1) * moyenne) ./ (ones(nTest, 1) * variance);%标准化
%     
%     Kgrid = svmkernel(Xtest, kernel, kerneloption, Xapp(pos, :));%【pos是高斯帅选过的点】
%     ypred = Kgrid*(Yapp(pos).*alpha) + b;  % 关键点到平面的距离【alpha是系数】
% %     % Calculate new labels
% %     ypred(find(ypred > 0)) = 1;
% %     ypred(find(ypred < 0)) = -1;
%     
%     % Calculate the error rate (in percent)
% %    erreur = (length(find(ypred - Ytest == 0)) / nTest) * 100;   % 碰撞的概率
%     collision_pro_e(i,1) = ypred;
% end
