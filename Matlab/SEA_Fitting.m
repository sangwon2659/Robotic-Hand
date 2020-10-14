% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.005;
cell_num = 6;
filename = "../../data/0821_WholeCell/2020-08-25-11-31-56.bag";
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

FT_res = [1/32 1/32 1/32 1/1504 1/1504 1/1504];
Data_i.FT_r = Data_i.FT_f.*FT_res;

%% Calibration
Delta_SEA = max(-(Data_i.tact(:,6)-mean(Data_i.tact(50:100,6))),0);
Delta_FT = Data_i.FT_r(:,6) - mean(Data_i.FT_r(50 : 100,6));
ft_linear = fittype('a1*x','indep','x');
cell_fit_linear = fit(Delta_SEA,Delta_FT,...
    ft_linear, 'StartPoint',[0.5], 'Robust','LAR','MaxIter',10000);
disp('Linear Fit');
disp(cell_fit_linear);

figure(1)
plot(Delta_SEA(50:end),Delta_FT(50:end))
title('Delta SEA vs. Delta FT')
hold on
plot(cell_fit_linear)
xlabel('Delta SEA (0.01 Degree)')
ylabel('Delta FT (Nm)')
grid on

figure(2)
plot(Delta_FT(50:end))
hold on
plot(Delta_SEA(50:end))
legend('Delta FT','Delta SEA')
grid on
title('Delta FT & Delta SEA')
ylabel('Nm/0.01 Degree')