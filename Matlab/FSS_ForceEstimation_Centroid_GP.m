%% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.001;
filename = "../../data/FSS/0924/Channel_234.bag";
gif_fname = "ForceEstimation.gif";
bag = rosbag(filename);
cellNum = 4;

%% Variable Setting
k = 1;
for i = 1 : length(bag.AvailableTopics.Row)
    if ((string(bag.AvailableTopics.Row{i}) ~= "/rosout")...
            && (string(bag.AvailableTopics.Row{i}) ~= "/rosout_agg"))
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
Data_i.tact = Data_i.tact-mean(Data_i.tact(500:1000,:));
Data_i.tact = max(Data_i.tact,0.0001);

Data_i.FT_f = Data_i.FT_f - mean(Data_i.FT_f(50 : 500,:));
FT_res = [1/32 1/32 1/32 1/1504 1/1504 1/1504];
Data_i.FT_r = Data_i.FT_f.*FT_res;
Data_i.FT_r = min(Data_i.FT_r,-0.0001);

%% Calibration
cell_1 = load('../../data/FSS/1014/CellFit_1.mat');
cell_2 = load('../../data/FSS/1014/CellFit_2.mat');
cell_3 = load('../../data/FSS/1014/CellFit_3.mat');
cell_4 = load('../../data/FSS/1014/CellFit_4.mat');

cali_data = zeros(length(Data_i.tact(:,1)),cellNum);
cali_data(:,1) = cell_1.cell_fit_piece(Data_i.tact(:,1));
cali_data(:,2) = cell_2.cell_fit_piece(Data_i.tact(:,2));
cali_data(:,3) = cell_3.cell_fit_piece(Data_i.tact(:,3));
cali_data(:,4) = cell_4.cell_fit_piece(Data_i.tact(:,4));

%% Single Point Summation
Summation = zeros(length(cali_data),1);
for k = 1:length(cali_data)
    Summation(k) = sum(cali_data(k,:));
end

%% FT Sensor XY-plane Data
FT_XY = sqrt(Data_i.FT_r(:,1).^2 + Data_i.FT_r(:,2).^2);

%% Gaussian Process
widthFSS = 5.6;
gapFSS = 1;
x_temp = -2:0.1:widthFSS+2*(widthFSS+gapFSS)+2;
x = [-2, 0, widthFSS/2, widthFSS/2+(widthFSS+gapFSS), (widthFSS/2)+2*(widthFSS+gapFSS),...
    widthFSS+2*(widthFSS+gapFSS), widthFSS+2*(widthFSS+gapFSS)+2];
y_temp = zeros(length(cali_data(:,1)),2);
y = [y_temp,cali_data(:,2:4), y_temp];

for i = 1:length(cali_data(:,1))/10
    gprmdl{i} = fitrgp(x',y(i*10,:)','Basis','linear','FitMethod','exact','PredictMethod','exact');
    [ypred, ysd, yint] = predict(gprmdl{i}, x_temp');
    ypred_ts(i,:,:) = ypred;
    ysd_ts(i,:,:) = ysd;
    yint_ts(i,:,:) = yint;
    disp(i)
end

%% Finding Centroid (Every 0.1mm)

delta = 0.1;
dis_times_deltaArea_sum = 0;
centroid = zeros(1,length(ypred_ts(:,1)));

for i = 1:length(ypred_ts(:,1))
    dis_times_deltaArea_sum = 0;
    for k = 0:length(ypred_ts(1,:))-1
        dis_times_deltaArea = (k/10)*delta*ypred_ts(i,k+1);
        dis_times_deltaArea_sum = dis_times_deltaArea_sum + dis_times_deltaArea;
    sumArea = sum(ypred_ts(i,:))*0.1;
    centroid(i) = dis_times_deltaArea_sum/sumArea;
    end
end

%% Plots
%Raw Data from FSS
figure(1)
plot(Data_i.tact)
title('Raw Data from Sensor')
grid on
xlabel('time')
legend('Channel_1', 'Channel_2', 'Channel_3', 'Channel_4')

%Calibrated Data of FSS
figure(2)
plot(cali_data)
title('Calibrated Data')
grid on
xlabel('time')
ylabel('Force(N)')
legend('Channel_1', 'Channel_2', 'Channel_3', 'Channel_4')

%Summed values of the calibrated data
figure(3)
plot(Summation);
hold on
plot(FT_XY);
grid on
xlabel('time')
ylabel('Force(N)')
legend('Fn Tot from FSS', 'FT XY')
title('Fn from FSS vs. F_{xy} from FT Sensor')

%Centroid
figure(4)
plot(centroid)
xlabel('time')
ylabel('Centroid(mm)')
title('Centroid of the GP graphs')

%% Creating GIF file
figure(5)
clf
for i = 1:size(ypred_ts,1)
    if mod(i,100) == 0
        hold off
        plot(x,y(i,:),'o')
        hold on
        plot(x_temp,ypred_ts(i,:))        
        stem(32, -Data_i.FT_r(i*10,1))
        plot(x_temp,yint_ts(i,:,1),'k--')
        plot(x_temp,yint_ts(i,:,2),'k--')
        grid on
        grid minor
        ylim([-1 10])
        xlabel('Distance (mm)')
        ylabel('Force (N)')
        legend('Measured','Estimated distribution','FT sensor','95% interval','FontSize',13)
        title(['Force Distribution Estimation (Gaussian Process), Time: ' num2str(i/100), '(s)']...
            , 'FontSize',13)
        drawnow limitrate
        
         % Save GIF
         frame = getframe(3);
         img = frame2im(frame);
         [A,map] = rgb2ind(img,1028);
         
         if i == 100
             imwrite(A,map,gif_fname,'gif','LoopCount',Inf,'DelayTime',0.1);
         else
             imwrite(A,map,gif_fname,'gif','WriteMode','append','DelayTime',0.1);
         end
    end
end























