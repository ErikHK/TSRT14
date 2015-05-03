function [ x ] = tdoa_localisation( tphat, will_plot )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

robot_start = [0.085; 0.465];
mic_place_2 = [0 0.50 0 0.99 0.61 0.99 1.22 0.99 1.22 0 0.60 0 0 0];
mic_place_2_2 = [0 0.50; 0 0.99; 0.61 0.99; 1.22 0.99; 1.22 0; 0.60 0; 0 0]';

s = exsensor('tdoa2', 7, 1, 2);
s.x0 = robot_start;
s.th = mic_place_2;
s.pe = calc_R(tphat);
%yy = zeros(s.nn(3),1);
k = 1;
%yy = zeros(88,21);
yy = [];
m=1;
k=1;
yy2 = zeros(88,21);
for m=1:88,
    for i =[1:6],
        for j = [i+1:7],
            yy(m,k) = tphat(m,i)-tphat(m,j);
            k=k+1;
        end
    end
    k=1;
end

for i=1:88,
    [shat, xhat] = nls(s, sig(yy(i,:)), 'thmask', zeros(s.nn(4) ,1));
    %sig_yy = sig(y_i);
    x(:,i) = shat.x0;
    x_cov(:,:,i) = cov(shat.px0);
    s.x0 = shat.x0;
    if(will_plot)
        plot(shat, 'conf', 90)
        hold on
    end
end


end

