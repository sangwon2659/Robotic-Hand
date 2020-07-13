% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.001;
filename = "../../data/tangential/2020-05-21-13-44-38.bag";
gif_fname = "dis_12.5.gif";
bag = rosbag(filename);

k = 1;
for i = 1 : length(bag.AvailableTopics.Row)
    if ((string(bag.AvailableTopics.Row{i}) ~= "/rosout") && (string(bag.AvailableTopics.Row{i}) ~= "/rosout_agg"))
        if (string(bag.AvailableTopics.Row{i}) == "/torque")
            [t_temp,temp] = topic_read(bag,bag.AvailableTopics.Row{i},'Tor');
        else
            [t_temp,temp] = topic_read(bag,bag.AvailableTopics.Row{i},'Data');
        end
        Data.(['t_' bag.AvailableTopics.Row{i}(2:end)]) = t_temp;
        varname(k) = string([bag.AvailableTopics.Row{i}(2:end)]);
        Data.(varname(k)) = temp;
        k = k+1;
    end
    clear t_temp temp
end
varname(k) = 'FT_f';
clear i bag temp_data k

%% Data filtering
Data.FT_f = lowpass(Data.FT, 5, 1/0.008);
Data.t_FT_f = Data.t_FT;
% Data.tact = movmean(Data.tact, [8 0]);
Data.tact = lowpass(Data.tact, 5, 1/0.01);
% Data.tact = smoothdata(Data.tact, 'sgolay');

%% Load cell data interpolation
range_temp_min = [];
range_temp_max = [];
for i = 1 : length(varname)
    range_temp_min = [range_temp_min min(Data.(['t_' char(varname(i))]))];
    range_temp_max = [range_temp_max max(Data.(['t_' char(varname(i))]))];
end
t_range = max(range_temp_min) : t_step : min(range_temp_max) ;
t = t_range-max(range_temp_min);

% interp1
for i = 1 : length(varname)
    Data_i.(varname(i))=interp1(Data.(['t_' char(varname(i))]),Data.(varname(i)),t_range);
end

%% Data processing
% signal matrix
sig = Data_i.tact;

% convert signal to resistance
R0=180*1000; % comparison resistance 500k ohm
R = R0*sig./(4096-sig); % resistance matrix
R_rel = R - mean(R(100:500,:),1); % relative resistance matrix

I = 5./R; % current matrix
I_rel = I - mean(I(100:500,:),1); % relative current matrix
I_ratio = I_rel./mean(I(100:500,:),1);
I_ratio = I_ratio.*(I_ratio>0);
[a, Imax] = max(I_ratio,[],2);

% Data_i.FT = Data_i.FT - mean(Data_i.FT(1 : 500,:));
Data_i.FT_f = Data_i.FT_f - mean(Data_i.FT_f(1 : 500,:));

FT_res = [1/32 1/32 1/32 1/1504 1/1504 1/1504];
Data_i.FT_r = Data_i.FT_f.*FT_res;

%% Calibration
cell_7 = load('cell_7_fit.mat');
cell_8 = load('cell_8_fit.mat');
cell_9 = load('cell_9_fit.mat');
cell_10 = load('cell_10_fit.mat');
cell_11 = load('cell_11_fit.mat');
cell_12 = load('cell_12_fit.mat');
cell_13 = load('cell_13_fit.mat');

I_ratio_cali = I_ratio; 
I_ratio_cali(:,1) = cell_7.cell_fit(I_ratio(:,7));
I_ratio_cali(:,2) = cell_8.cell_fit(I_ratio(:,8));
I_ratio_cali(:,3) = cell_9.cell_fit(I_ratio(:,9));
I_ratio_cali(:,4) = cell_10.cell_fit(I_ratio(:,10));
I_ratio_cali(:,5) = cell_11.cell_fit(I_ratio(:,11));
I_ratio_cali(:,6) = cell_12.cell_fit(I_ratio(:,12));
% I_ratio_cali(:,5) = cell_5.cell_fit(I_ratio(:,5));
I_ratio_cali = I_ratio_cali/5;
% I_ratio_cali(:,4:5) = I_ratio_cali(:,4:5).*(I_ratio_cali(:,4:5)>0.15);

%% Contact Force Distribution Estimation using Gaussian Process regression
% x = [1:0.1:2, 3, 4, 5, 6:0.1:7];
% xtest = 1:0.1:7;
% x = [0:0.5:6.5, 13, 19.5, 26, 32.5:0.5:39]; cell 3:5
x = [2:1:20, 24, 27, 30, 34:1:52];
dx = 0.01;
xtest = 0:dx:54;
y1 = zeros(1,18); % cell 1~2
y3 = zeros(1,19); % cell 6~7

% for i = 1:length(I_ratio_cali(:,1))/10
for i = 500:1500
    y2 = I_ratio_cali(i*10, 2:5);
    y(i,:) = [y1 y2 y3];
    gprmdl{i} = fitrgp(x',y(i,:)','Basis','linear','FitMethod','exact','PredictMethod','exact');
    [ypred, ysd, yint] = predict(gprmdl{i}, xtest');
    ypred_ts(i,:,:) = ypred;
    ysd_ts(i,:,:) = ysd;
    yint_ts(i,:,:) = yint;
    FT(i,:) = Data_i.FT_r(i*10,:);
    ts(i) = t(i*10);
    disp(i)
end

%% FT Sensor XY-plane Data
FT_XY = sqrt(FT(:,1).^2 + FT(:,2).^2);
plot(FT_XY)

%% Contact Force Estimation
F_est = sum(ypred_ts.*dx,2);
for i = 500:1500
    I_ratio_cali_only_pressed(i,:) = I_ratio_cali(i*10,4);
end
figure(1)
clf
plot(ts, -FT(:,1))
hold on
plot(ts, FT(:,2))
plot(ts, FT_XY)
plot(ts, F_est)
plot(ts, I_ratio_cali_only_pressed)
grid on
grid minor
xlim([0 15])
ylim([-1 6]),
legend('FT_X', 'FT_Y', 'FT_{XY}', 'Tactile', 'Cell#4')
xlabel('Time (s)')
ylabel('Force (N)')

%% Torque from Normal Force Estimation


%% Torque on Motor Estimation

%% Tangential Force Estimation

%% Tangential Force and Ground Truth Tangential Force Comparison

%% Contact Force Distribution estimation results plot
figure(2)
clf
for i = 1:size(ypred_ts,1)
    if mod(i,10) == 0
        hold off
        plot(x,y(i,:),'o')
        hold on
        plot(xtest,ypred_ts(i,:))        
%         stem(19.5, -Data_i.FT_r(i*10,1))
        plot(xtest,yint_ts(i,:,1),'k--')
        plot(xtest,yint_ts(i,:,2),'k--')
        grid on
        grid minor
        ylim([-2 5])
        xlabel('Cell #')
        ylabel('Normal Force (N)')
        legend('Measured','Estimated distribution','FT sensor','95% interval','FontSize',13)
        title(['Contact Normal Force Distribution Estimation, Time: ' num2str(i/100), '(s)'], 'FontSize',13)
        drawnow limitrate
        
         % Save GIF
%          frame = getframe(7);
%          img = frame2im(frame);
%          [A,map] = rgb2ind(img,256);
%          
%          if i == 100
%              imwrite(A,map,gif_fname,'gif','LoopCount',Inf,'DelayTime',0.1);
%          else
%              imwrite(A,map,gif_fname,'gif','WriteMode','append','DelayTime',0.1);
%          end
    end
end
    
