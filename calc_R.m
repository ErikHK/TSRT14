function [ R ] = calc_R(tphat)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
nr_of_mics = 7;
mean_tphat = mean(tphat,2);
nr_of_samples = length(tphat);
e = zeros(nr_of_samples,nr_of_mics);

for i=[1:nr_of_mics]
    e(:,i) = tphat(:,i)-mean_tphat;
end

mic_bias = zeros(nr_of_mics,1);
mic_var = zeros(nr_of_mics,1);
for i=[1:nr_of_mics]
    mic_bias(i) = mean(e(:,i));
    mic_var(i) = var(e(:,i));
    %[N, l] = hist(e(:,i),20);
    %Wb= l(2)-l(1); %Bin width
    %Ny = nr_of_samples; %Nr of samples
    %figure(i)
    %bar(l, N/(Ny*Wb));
    %hold on;
    %pe = ndist(mic_bias(i), mic_var(i));
    %plot(pe);
end
%k=1;
%for i =[1:nr_of_mics]
%    for j = [i+1:nr_of_mics]
%        y_var(k) = mic_var(i) + mic_var(j);
%        k= k+1;
%    end
%end

y_var = [];
for k = 1:6,
    for l = k+1:7,
        y_var = [y_var mic_var(l) + mic_var(k)];
    end
end


R = diag(y_var);
end

