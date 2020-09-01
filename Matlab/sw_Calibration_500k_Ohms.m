% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.005;
cell_num = 5;
filename = "../../data/0821_WholeCell/Whole_Old_Algo_Channel_2.bag";
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
Data.FT_f = lowpass(Data.FT, 1, 1/0.008);
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
sig = Data_i.tact;
Vin = sig/4096*5;
Vin_ = -(Vin-mean(Vin(500:1000,:),1));

Data_i.FT_f = Data_i.FT_f - mean(Data_i.FT_f(50 : 500,:));
FT_res = [1/32 1/32 1/32 1/1504 1/1504 1/1504];
Data_i.FT_r = Data_i.FT_f.*FT_res;

Data_i.FT_r = min(Data_i.FT_r,-0.0001);
Vin_ = max(Vin_,0.0001);

%% Calibration with Polynomial
%Linear Fitting
ft_linear = fittype('a1*x+b','indep','x');
cell_fit_linear = fit(Vin_(500:end-1000,cell_num),-Data_i.FT_r(500:end-1000,1), ft_linear, 'StartPoint',[0.5, 0.8], 'Robust','LAR','MaxIter',10000);

cali_data_linear = zeros(length(Vin_(:,1)),1);
for k = 1:length(Vin_(:,1))
    temp = cell_fit_linear(Vin_(k,cell_num));
    if Vin_(k,cell_num) < 0 || abs(temp) > 10
        cali_data_linear(k) = 0;
    else
        cali_data_linear(k) = cell_fit_linear(Vin_(k,cell_num));
    end
end

%Piecewise Fitting
ft_piece = fittype('a0*x^5+a1*x^4+a2*x^3+a3*x^2+a4*x','indep','x');
%ft_piece = fittype('a0*sqrt(x)+a2*x^2');
cell_fit_piece = fit(Vin_(500:end-1000,cell_num),-Data_i.FT_r(500:end-1000,1), ft_piece, 'StartPoint',[0.1, 0.1, 0.1, 30, 0.27], 'Robust','LAR','MaxIter',10000);

cali_data_piece = zeros(length(Vin_(:,1)),1);
for l = 1:length(Vin_(:,1))
    temp = cell_fit_piece(Vin_(l,cell_num));
    if Vin_(l,cell_num) < 0 || abs(temp) > 5
        cali_data_piece(l) = 0;
    else
        cali_data_piece(l) = temp;
    end
end

%Displaying Results
disp('Linear Fit');
disp(cell_fit_linear);
disp('Piecewise Fit');
disp(cell_fit_piece);

%% Plotting
%FT Sensor vs. Tactile Sensor Raw Data
figure(1)
clf
hold on
plot(t,Vin_(:,cell_num))
plot(t,-Data_i.FT_r(:,1))
grid on
grid minor
xlabel('time (s)')
ylabel('Force (N)')
legend('Tactile','FT sensor')
title('Raw Data')

%Loading and Unloading Analysis
figure(2)
clf
plot(movmean(Vin_(:,cell_num),100),movmean(-Data_i.FT_r(:,1),100),'.')
title('Tactile vs. FT')
xlabel('Tactile Sensor')
ylabel('FT Sensor')
axis square
grid on
grid minor

figure(3)
plot(Data.tact(:,1:3))
legend('Channel3','Channel2','Channel1')

%Fitted Curve_Linear
figure(3)
title('Linear');
clf
plot(cell_fit_linear, Vin_(500:end-1000,cell_num),-Data_i.FT_r(500:end-1000,1))
xlabel('Tactile sensor')
ylabel('FT sensor')
% axis equal
grid on
grid minor

%Fitted Curve_Piece
figure(4)
title('Piece');
clf
plot(cell_fit_piece, Vin_(500:end-1000,cell_num),-Data_i.FT_r(500:end-1000,1))
xlabel('Tactile sensor')
ylabel('FT sensor')
% axis equal
grid on
grid minor

%Calibrated Results_Linear
figure(5)
title('Calibration Results (Linear)');
clf
plot(cali_data_linear)
hold on
plot(-Data_i.FT_r(:,1))
grid on
grid minor
xlabel('time (s)')
ylabel('Normal Force (N)')
legend('Tactile','FT sensor')

%Calibrated Results_Piece
figure(6)
title('Calibration Results (Piece)');
clf
plot(cali_data_piece)
hold on
plot(-Data_i.FT_r(:,1))
grid on
grid minor
xlabel('time (s)')
ylabel('Normal Force (N)')
legend('Tactile','FT sensor')

