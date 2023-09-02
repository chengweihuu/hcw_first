function [sign,ypred]=CollisionCheck_SVM(configuration,obst)
load ComplexEnv.mat % Input the SVM-trained data for the corresponding scenario
position = Forward_kinematic(configuration,8);
Xtest = position;
Xtest = (Xtest - moyenne) ./ variance;
Kgrid = svmkernel(Xtest, kernel, kerneloption, Xapp(pos, :));
ypred = Kgrid*(Yapp(pos).*alpha2) + b;  

if ypred < 0
    sign = 1;
elseif ypred > 0.6
    sign = 0;
else
    sign = CollisionCheck_3(configuration,obst);
end
end