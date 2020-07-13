% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.005;
cell_num = 4;
filename = "data/cell4.bag"; %% normal_test_1_2.bag /  test_sensor2.bag
bag = rosbag(filename);
%test_normal_sine.bag
%test_normal_step.bag
% test_tang_13.bagclose a
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

figure(1)
plot(Data.FT)
grid on
grid minor
Data.FT_f = lowpass(Data.FT, 1, 1/0.008);
Data.t_FT_f = Data.t_FT;
Data.tact = lowpass(Data.tact, 5, 1/0.009);
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
R_rel = R - mean(R(10:1000,:),1); % relative resistance matrix

I = 5./R; % current matrix
I_rel = I - mean(I(10:1000,:),1); % relative current matrix
I_ratio = I_rel./mean(I(10:1000,:),1);
[a, Imax] = max(I_ratio,[],2);
I_ratio = I_ratio.*(I_ratio>0);

% Data_i.FT = Data_i.FT - mean(Data_i.FT(1 : 500,:));
Data_i.FT_f = Data_i.FT_f - mean(Data_i.FT_f(1 : 500,:));

FT_res = [1/32 1/32 1/32 1/1504 1/1054 1/1054];
Data_i.FT_r = Data_i.FT_f.*FT_res;

%% 

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
plot(t,I_ratio(:,cell_num))
plot(t,-Data_i.FT_r(:,1))
grid on
grid minor
xlabel('time (s)')
ylabel('Force (N)')
legend('Tactile','FT sensor')
title('Raw Data')

figure(4)
clf
hold on
plot(Data_i.FT_r)
legend('Fx','Fy','Fz','Tx','Ty','Tz')
grid on
grid minor

figure(5)
clf
plot(movmean(I_ratio(:,cell_num),100),movmean(-Data_i.FT_r(:,1),100),'.')
axis square
grid on
grid minor

%% Calibration with Polynomial
ft = fittype('a1*x^3+a2*x^2+a3*x','indep','x');
cell_fit = fit(I_ratio(500:end-1000,cell_num),-Data_i.FT_r(500:end-1000,1), ft)
% save('cell_'+string(cell_num)+'_fit.mat', 'cell_fit')
figure(7)
clf
plot(cell_fit, I_ratio(500:end-1000,cell_num),-Data_i.FT_r(500:end-1000,1))
xlabel('Tactile sensor')
ylabel('FT sensor')
axis equal
grid on
grid minor

figure(8)
clf
plot(t,cell_fit(I_ratio(:,cell_num)))
hold on
plot(t,-Data_i.FT_r(:,1))
grid on
grid minor
title('Calibration results')
xlabel('time (s)')
ylabel('Normal Force (N)')
legend('Tactile','FT sensor')
xlim([0,1])
ylim([0,1])

%% Calibration with GP
sample = sort(randi([1000 size(I_ratio,1)],1,5000));
gprmdl = fitrgp(I_ratio(sample,cell_num),-Data_i.FT_r(sample,1),'Basis','linear','FitMethod','none',...
    'PredictMethod','exact','KernelParameters',[3 1],'Sigma',0.06);

figure(9)
clf
ypred = predict(gprmdl,(0:0.001:2.5)');
plot(I_ratio(sample,cell_num),-Data_i.FT_r(sample,1),'.')
hold on
plot(0:0.001:2.5,ypred,'LineWidth',3)
grid on
grid minor

figure(10)
clf
hold on
plot(t,predict(gprmdl,(I_ratio(:,cell_num))))
plot(t,-Data_i.FT_r(:,1))
grid on
grid minor