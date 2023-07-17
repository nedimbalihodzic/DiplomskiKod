clear all
clc
close all
filename = 'data.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

% for i=2:n
%     figure(i)
%   %  yy = i;
%   %  plot(Arr(:,yy),'r');
%   %  ylabel(cell2mat(VariableNames(yy)))
%      yy = i;
%      plot(Arr(:,1),Arr(:,yy),'r');
%      ylabel(cell2mat(VariableNames(yy)))
%      xlabel(cell2mat(VariableNames(1)))
% end

% figure 
% plot(Arr(:,1), Arr(:,2));
% xlabel('Vrijeme [s]'); 
% ylabel('T_x'); 
% xlim([0 3]);
% title('Moment sile oko x ose');
% 
% figure 
% plot(Arr(:,1), Arr(:,3));
% xlabel('Vrijeme [s]'); 
% ylabel('T_y'); 
% xlim([0 3]);
% title('Moment sile oko y ose');
% 
% figure 
% plot(Arr(:,1), Arr(:,4));
% xlabel('Vrijeme [s]'); 
% ylabel('T_z'); 
% xlim([0 3]);
% title('Moment sile oko z ose');


figure 
plot(Arr(:,1), Arr(:,4));
xlabel('Vrijeme [s]'); 
ylabel('a_z (mj\_Data)'); 
xlim([0 11]);
title('Vertikalno ubrznje dobijeno iz mj\_Data');

figure
plot(Arr(:,1), Arr(:,7));
xlabel('Vrijeme [s]'); 
ylabel('a_z (formula)');
xlim([0 11]);
title('Vertikalno ubrznje dobijeno formulom'); 

figure
plot(Arr(:,1), Arr(:,4)-Arr(:,7));
xlabel('Vrijeme [s]'); 
ylabel('\Delta q');
xlim([0 11]);
title('Grafik greške');
% 
% 
% drag = (Arr(:,4) - Arr(:,7)); 
% figure
% plot(Arr(:,1), drag); 
% x = lsqr( Arr(:,10) , drag );

%  t = Arr(:,1);
%  coeff = drag./Arr(:,10); 
% 
%  figure 
%  plot(t,coeff);
% 
% linear1 = coeff(9:24); 
% t1 = t(9:24); 
% figure 
% plot(t1, linear1);
% 
% linear2 = coeff(25:47); 
% linear3 = coeff(48:64); 
% linear5 = coeff(65:75); 
