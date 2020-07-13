% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.001;
filename = "data/dis_12.5.bag";
gif_fname = "data/dis_12.5.gif";
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
R0=475*1000; % comparison resistance 500k ohm
R = R0*sig./(4096-sig); % resistance matrix
R_rel = R - mean(R(100:500,:),1); % relative resistance matrix

I = 5./R; % current matrix
I_rel = I - mean(I(100:500,:),1); % relative current matrix
I_ratio = I_rel./mean(I(100:500,:),1);
I_ratio = I_ratio.*(I_ratio>0);
[a, Imax] = max(I_ratio,[],2);

% Data_i.FT = Data_i.FT - mean(Data_i.FT(1 : 500,:));
Data_i.FT_f = Data_i.FT_f - mean(Data_i.FT_f(1 : 500,:));

FT_res = [1/32 1/32 1/32 1/1504 1/1054 1/1054];
Data_i.FT_r = Data_i.FT_f.*FT_res;


%% Calibration
cell_2 = load('data/cell_2_fit_piece.mat');
cell_3 = load('data/cell_3_fit_piece.mat');
cell_4 = load('data/cell_4_fit_piece.mat');

I_ratio_cali = I_ratio;

%Manual Setting
%{
for a = 1:length(I_ratio(:,1))
    
    if I_ratio(a,2) < 0.5
        I_ratio_cali(a,2) = sqrt(0.4*I_ratio(a,2));
    else
        I_ratio_cali(a,2) = cell_2.cell_fit(I_ratio(a,2));
    end
    
    if I_ratio(a,4) < 0.5
        I_ratio_cali(a,4) = sqrt(1.8*I_ratio(a,4));
    else
        I_ratio_cali(a,4) = cell_4.cell_fit(I_ratio(a,4));
    end
end
%}

I_ratio_cali(:,2) = cell_2.cell_fit(I_ratio(:,2));
I_ratio_cali(:,3) = cell_3.cell_fit(I_ratio(:,3));
I_ratio_cali(:,4) = cell_4.cell_fit(I_ratio(:,4));

I_ratio_cali = I_ratio_cali/5;

%% Contact Force Distribution Estimation using Gaussian Process regression
% x = [1:0.1:2, 3, 4, 5, 6:0.1:7];
% xtest = 1:0.1:7;
x = [0:0.5:6.5, 13, 19.5, 26, 32.5:0.5:39];
xtest = 0:0.1:39;
y1 = zeros(1,14); % cell 1
y3 = zeros(1,14); % cell 5~7

for i = 1:length(I_ratio_cali(:,1))/100
    y2 = I_ratio_cali(i*100, 2:4);
    y(i,:) = [y1 y2 y3];
    gprmdl{i} = fitrgp(x',y(i,:)','Basis','linear','FitMethod','exact','PredictMethod','exact');
    [ypred, ysd, yint] = predict(gprmdl{i}, xtest');
    ypred_ts(i,:,:) = ypred;
    ysd_ts(i,:,:) = ysd;
    yint_ts(i,:,:) = yint;
    disp(i)
end

%% Integral (Every 0.1mm)

sum = zeros(1,length(ypred_ts(:,1)));
Data_com = zeros(1,length(ypred_ts(:,1)));
t_ = 0:length(ypred_ts(:,1))-1;
for k = 1:length(ypred_ts(:,1))
    Data_com(k) = -Data_i.FT_r(100*k,1);
    for l = 1:length(ypred_ts(1,:))
        sum(k) = sum(k) + ypred_ts(k,l)*0.1;
    end
end

%% Plotting Estimate and FT values
figure(1)
plot(t_,sum)
hold on
plot(t_,Data_com)
grid on
grid minor
legend('Estimate','FT')
xlabel('Time')
ylabel('Force')

%{
figure(2)
clf
hold on
plot(t,Data_i.FT_f)
legend('Fx','Fy','Fz','Tx','Ty','Tz')
grid on
grid minor

figure(3)
clf
hold on
plot(t,I_ratio(:,4))
plot(t,-Data_i.FT_r(:,1))
grid on
grid minor

figure(4)
clf
hold on
plot(Data_i.FT_r)
legend('Fx','Fy','Fz','Tx','Ty','Tz')
grid on
grid minor

figure(5)
clf
plot(movmean(I_ratio(:,3),100),movmean(-Data_i.FT_r(:,1),100),'.')
axis square
grid on
grid minor


    
%% Contact Force Distribution estimation results plot
figure(7)
clf
for i = 1:size(ypred_ts,1)
    if mod(i,100) == 0
        hold off
        plot(x,y(i,:),'o')
        hold on
        plot(xtest,ypred_ts(i,:))        
        stem(19.5, -Data_i.FT_r(i*10,1))
        plot(xtest,yint_ts(i,:,1),'k--')
        plot(xtest,yint_ts(i,:,2),'k--')
        grid on
        grid minor
        ylim([-0.1 1.0])
        xlabel('Cell #')
        ylabel('Normal Force (N)')
        legend('Measured','Estimated distribution','FT sensor','95% interval','FontSize',13)
        title(['Contact Normal Force Distribution Estimation, Time: ' num2str(i/100), '(s)'], 'FontSize',13)
        drawnow limitrate
        
%         % Save GIF
%         frame = getframe(7);
%         img = frame2im(frame);
%         [A,map] = rgb2ind(img,256);
%         
%         if i == 100
%             imwrite(A,map,gif_fname,'gif','LoopCount',Inf,'DelayTime',0.1);
%         else
%             imwrite(A,map,gif_fname,'gif','WriteMode','append','DelayTime',0.1);
%         end
    end
end

%% Estimated Normal Force and Position
% dx = 
% for i = 1:size(gprmdl,2)

%}
    
