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
a4(pos) = alpha2;%��alpha��ɶѽ��
toc

% %% Ԥ��
% collision_pro_d = zeros(size(DCL_test,2),1);%��Ԥ�����DCL_test�����ݡ�
% for i = 1:size(DCL_test,1)
%     tem1 = DCL_test(i,:);
%     test = tem1;
%     Xtest = test(:,1:3);
%     Ytest = test(:,end);
%     [nTest, p] = size(Xtest);
%     Xtest = (Xtest - ones(nTest, 1) * moyenne) ./ (ones(nTest, 1) * variance);%��׼��
%     Kgrid = svmkernel(Xtest, kernel, kerneloption, Xapp(pos, :));%��pos�Ǹ�˹˧ѡ���ĵ㡿
%     ypred = Kgrid*(Yapp(pos).*alpha) + b;  % �ؼ��㵽ƽ��ľ��롾alpha��ϵ����
%     % Calculate new labels
%     ypred(find(ypred > 0)) = 1;
%     ypred(find(ypred < 0)) = -1;
    
    % Calculate the error rate (in percent)
%     erreur = (length(find(ypred - Ytest == 0)) / nTest) * 100;   % ��ײ�ĸ���
%     collision_pro_d(i,1) = ypred;
% end

% %% Ԥ��
% collision_pro_e = zeros(size(ECL_test,2),1);%��Ԥ�����DCL_test�����ݡ�
% for i = 1:size(ECL_test,1)
%     tem1 = ECL_test(i,:);
%     test = tem1;
%     Xtest = test(:,1:3);
%     Ytest = test(:,end);
%     [nTest, p] = size(Xtest);
%     Xtest = (Xtest - ones(nTest, 1) * moyenne) ./ (ones(nTest, 1) * variance);%��׼��
%     
%     Kgrid = svmkernel(Xtest, kernel, kerneloption, Xapp(pos, :));%��pos�Ǹ�˹˧ѡ���ĵ㡿
%     ypred = Kgrid*(Yapp(pos).*alpha) + b;  % �ؼ��㵽ƽ��ľ��롾alpha��ϵ����
% %     % Calculate new labels
% %     ypred(find(ypred > 0)) = 1;
% %     ypred(find(ypred < 0)) = -1;
%     
%     % Calculate the error rate (in percent)
% %    erreur = (length(find(ypred - Ytest == 0)) / nTest) * 100;   % ��ײ�ĸ���
%     collision_pro_e(i,1) = ypred;
% end
