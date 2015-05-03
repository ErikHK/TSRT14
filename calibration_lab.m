%% 7.2.1 sensor calibration
addpath('../toolkit');
load('calibration.mat')
tphat = tphat*343;
mean_tphat = mean(tphat,2);
 nr_of_mics = length(mics);
nr_of_samples = length(tphat);
e = zeros(nr_of_samples,nr_of_mics);

for i=[1:nr_of_mics]
    e(:,i) = tphat(:,i)-mean_tphat;
end

mic_bias = zeros(nr_of_mics,1);
mic_var = zeros(nr_of_mics,1);
for i=[1:nr_of_mics]
%k = 1;
%for i=[2, 6]
    mic_bias(i) = mean(e(:,i));
    mic_var(i) = var(e(:,i));
    [N, l] = hist(e(:,i),20);
    Wb= l(2)-l(1); %Bin width
    Ny = nr_of_samples; %Nr of samples
    figure(i)
    %subplot(2,1,k)
    bar(l, N/(Ny*Wb));
    hold on;
    pe = ndist(mic_bias(i), mic_var(i));
    plot(pe);
    %k=k+1;
end

%% 7.2.2 signal modeling
mic_place_1 = [0 0.958 0 0.828 0 0.678 0 0.525 0 0.38 0 0.21 0 0.035];
robot_start = [0.085; 0.465];
mic_place_2 = [0 0.50 0 0.99 0.61 0.99 1.22 0.99 1.22 0 0.60 0 0 0];

s1 = exsensor('tdoa2',7,1,2);
s1.x0 = robot_start;
s1.th = mic_place_1;

s2 = exsensor('tdoa2',7,1,2);
s2.x0 = robot_start;
s2.th = mic_place_2;

y_var = zeros(s1.nn(3),1);
y_var2 = zeros(s2.nn(3),1);
k = 1;
for i =[1:nr_of_mics]
    for j = [i+1:nr_of_mics]
        y_var(k) = mic_var(i) + mic_var(j);
        k= k+1;
    end
end
%s2.pe = diag(y_var_2);
s1.pe = diag(y_var);
s2.plot;
s1.plot;
%% 7.2.4 configuration analysis
%R = diag(y_var);
%crlb(s1);


clear all
addpath('../toolkit');
load('one_side.mat')
%load('all_around.mat')
gridd = 1:20;
grid_pos = (gridd-5)*.1;

mic_place_1 = [0 0.958 0 0.828 0 0.678 0 0.525 0 0.38 0 0.21 0 0.035];
robot_start = [0.085; 0.465];
mic_place_2 = [0 0.50 0 0.99 0.61 0.99 1.22 0.99 1.22 0 0.60 0 0 0];

tphat = tphat*340;
mean_tphat = mean(tphat,2);
 nr_of_mics = length(mics);
nr_of_samples = length(tphat);
e = zeros(nr_of_samples,nr_of_mics);

for i=[1:nr_of_mics]
    e(:,i) = tphat(:,i)-mean_tphat;
end

s2 = exsensor('tdoa2',7,1,2);
s2.x0 = robot_start;
s2.th = mic_place_1;

mic_bias = zeros(nr_of_mics,1);
mic_var = zeros(nr_of_mics,1);
for i=[1:nr_of_mics]
    mic_bias(i) = mean(e(:,i));
    mic_var(i) = var(e(:,i));
    [N, l] = hist(e(:,i),20);
    Wb= l(2)-l(1); %Bin width
    Ny = nr_of_samples; %Nr of samples
    %figure(i)
    %bar(l, N/(Ny*Wb));
    %hold on;
    pe = ndist(mic_bias(i), mic_var(i));
    %plot(pe);
end


y_var = zeros(s2.nn(3),1);
k = 1;
for i =[1:nr_of_mics]
    for j = [i+1:nr_of_mics]
        y_var(k) = mic_var(i) + mic_var(j);
        k= k+1;
    end
end
%s2.pe = diag(y_var_2);
R = diag(y_var);
s2.pe = R;
V = zeros(length(gridd), length(gridd));
for x1 = gridd,
    for x2 = gridd,
        s2.x0 = [grid_pos(x1); grid_pos(x2)];
        cramer = crlb(s2);
        V(x1, x2) = sqrt(cramer.Px(1,1,1) + cramer.Px(1,2,2));
        %h_x = simulate(sm, 1);
        %y = simulate(sm, 1);
        %a = (y - h_x);
        %[xhat, shat] = ls(sm, y);
        %V(x1, x2) = sqrt(trace([xhat.Px(:,:,1); xhat.Px(:,:,2)]'));
    end
end

%s2.plot
%crlb(s2)

figure(2)
%V(1, 10) = 1000000;
contour(grid_pos, grid_pos, V', 50)

%% 7.2.5 Localisation, NLS Gauss
clear all
robot_start = [0.085; 0.465];
mic_place_2 = [0 0.50 0 0.99 0.61 0.99 1.22 0.99 1.22 0 0.60 0 0 0];
mic_place_2_2 = [0 0.50; 0 0.99; 0.61 0.99; 1.22 0.99; 1.22 0; 0.60 0; 0 0]';
addpath('../toolkit');
%load('all_around.mat')
load('all_around.mat');
tphat = tphat*340; %seconds to meter
bias = abs(mean(tphat(1,:))-mean(tphat(2,:)));
s = exsensor('tdoa1', 7, 1, 2);
s.x0 = [robot_start; bias];
s.th = mic_place_2;
%s.x0(3) = bias;
Fs = 2; %2 Hz
%new_pos = zeros(2,88);
for i=1:88,
    y_i = sig(tphat(i,:), Fs);
    xhat = nls(s, y_i, 'thmask', zeros(1,s.nn(4)));
    new_pos(:,i) = xhat.x0(1:2);
    s.x0 = [xhat.x0(1:2); xhat.x0(3)+bias];
end

SFlabCompEstimGroundTruth(new_pos, mic_place_2_2);

%% 7.2.5 Localisation, TDOA approach
clear all
robot_start = [0.085; 0.465];
mic_place_2 = [0 0.50 0 0.99 0.61 0.99 1.22 0.99 1.22 0 0.60 0 0 0];
mic_place_2_2 = [0 0.50; 0 0.99; 0.61 0.99; 1.22 0.99; 1.22 0; 0.60 0; 0 0]';
addpath('../toolkit');
%load('all_around.mat')
load('all_around.mat');
tphat = tphat*340; %seconds to meter

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
    
    x(:,i) = shat.x0;
    x_cov(:,:,i) = cov(shat.px0);
    %s.x0 = shat.x0;
    plot(shat, 'conf', 90)
    hold on
end

%% 7.2.6 Tracking
clear all
dt = 0.5; %time between measurements
Fs = 1/dt;

% [x,y,vx,vy]
model_matrix = [1 0 dt/2 0;0 1 0 dt/2;0 0 1 0;0 0 0 1];

robot_start = [0.085; 0.465];
mic_place = [0 0.50 0 0.99 0.61 0.99 1.22 0.99 1.22 0 0.60 0 0 0];
mic_place_2 = [0 0.50; 0 0.99; 0.61 0.99; 1.22 0.99; 1.22 0; 0.60 0; 0 0]';
addpath('../toolkit');
%load('all_around.mat')
load('all_around.mat');
tphat = tphat*343; %seconds to meter

x = tdoa_localisation(tphat, 0);

model = exnl('cv2d'); %linear velocity
model.x0 = [robot_start; 0; 0];
Q = [0 0 0 0; 0 0 0 0; 0 0 0.004 0; 0 0 0 0.004];
P0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
model.px0 = P0;
model.pe = [.2 0;0 .3]; %ÄNDRA DETTA FÖR GUDS SKULL!!!
y = sig(x', Fs);
%x_CV = ekf(model, y, zeros(4,4), Q, P0);
x_CV = ekf(model, y);
SFlabCompEstimGroundTruth(x_CV.y', mic_place_2);
%plot(x_CV)
figure;
plot(x_CV.y(:,1), x_CV.y(:,2));
hold on
plot(x(1,:), x(2,:), 'r');

%%
