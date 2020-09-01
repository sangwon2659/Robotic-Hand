%% Sensor system validation
clear all
close all

%% Bag Read
varname = strings;
t_step = 0.001;
filename = "../../data/0819/BagFiles/0819_2/Channel_345.bag";
gif_fname = "Force_Torque_Estimation.gif";
bag = rosbag(filename);
cellWidth = 5;
cellSeparation = 3;
cellNum = 5;
motorFirstCellDis = 40;
initialCell = 3;
finalCell = 5;
initialChannel = 1;
finalChannel = 5;
calibrationWidth = 5;

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

%% Data Sorting
Data.tact = Data.tact(:,initialChannel:finalChannel);

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
sig = Data_i.tact;
Vin = sig/4096*5;
Vin_ = -(Vin-mean(Vin(500:1000,:),1));

Data_i.FT_f = Data_i.FT_f - mean(Data_i.FT_f(50 : 500,:));
FT_res = [1/32 1/32 1/32 1/1504 1/1504 1/1504];
Data_i.FT_r = Data_i.FT_f.*FT_res;

%% Calibration
%{
for z = 1:cellNum
    if isfile(sprintf('../../data/calibration_rect/cell_%d_fit_piece.mat',z))
        

if isfile(filename)
     % File exists.
else
     % File does not exist.
%}

%cell_1 = load('../../data/0730/CellFit/CellFit_1.mat');
%cell_2 = load('../../data/0730/CellFit/CellFit_2.mat');
cell_3 = load('../../data/0819/CellFit/CellFit_3.mat');
cell_4 = load('../../data/0819/CellFit/CellFit_4.mat');
cell_5 = load('../../data/0819/CellFit/CellFit_5.mat');
%cell_6 = load('../../data/0730/CellFit/CellFit_6.mat');
%cell_7 = load('../../data/0730/CellFit/CellFit_7.mat');

cali_data = zeros(length(Vin_(500:end-1000,1)),finalCell-initialCell+1);
for k = 1:length(Vin_(:,3))
    if Vin_(k,3) < 0
        cali_data(k,1) = 0;
    else
        cali_data(k,1) = cell_3.cell_fit_piece(Vin_(k,3));
    end
end
for k = 1:length(Vin_(:,4))
    if Vin_(k,4) < 0
        cali_data(k,2) = 0;
    else
        cali_data(k,2) = cell_4.cell_fit_piece(Vin_(k,4));
    end
end

for k = 1:length(Vin_(:,5))
    if Vin_(k,5) < 0
        cali_data(k,3) = 0;
    else
        cali_data(k,3) = cell_5.cell_fit_piece(Vin_(k,5));
    end
end

%% Summation
Summation = zeros(length(cali_data),1);
for k = 1:length(cali_data)
    Summation(k) = sum(cali_data(k,:));
end
figure(1)
plot(Summation);

%% Determining Contact Intervals
%Data point가 많으면 굳이 이렇게 안해도 되지 않을까?
%GP를 한 distribution이 발산하는 건 어떻게 막을까?

%% Contact Force Distribution Estimation using Gaussian Process regression
%Setting position of cells wrt their centers
%Left and right end set to zero with an offset of half the width of the cell
%{
x = [motorFirstCellDis-(cellWidth/2) motorFirstCellDis:cellWidth+cellSeparation:...
    motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1) motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+(cellWidth/2)]; 
xtest = motorFirstCellDis-(cellWidth/2):0.1:motorFirstCellDis+(cellWidth+cellSeparation)*cellNum+(cellWidth/2);
x = [motorFirstCellDis-8:1:motorFirstCellDis motorFirstCellDis:cellWidth+cellSeparation:...
    motorFirstCellDis+(cellWidth+cellSeparation)*(4-1) motorFirstCellDis+(cellWidth+cellSeparation)*(4-1):1:motorFirstCellDis+(cellWidth+cellSeparation)*(4-1)+20];
%}
%{
x = [0:(motorFirstCellDis-(cellWidth/2)+(initialCell-1)*(cellWidth+cellSeparation))/Interval:...
    motorFirstCellDis-(cellWidth/2)+(initialCell-1)*(cellWidth+cellSeparation)... 
    motorFirstCellDis+(initialCell-1)*(cellWidth+cellSeparation):cellWidth+cellSeparation:...
    motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)...
    motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+(cellWidth/2):...
    (motorFirstCellDis-(cellWidth/2)+(initialCell-1)*(cellWidth+cellSeparation))/Interval:...
    motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+motorFirstCellDis];
xtest = 0:0.1:...
    (motorFirstCellDis-(cellWidth/2)+(initialCell-1)*(cellWidth+cellSeparation))/Interval:...
    motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+motorFirstCellDis;
%}
%{
Length = length(Vin_(1:10:length(Vin_),initialCell:finalCell));
y_ = zeros(Length,length(0:(motorFirstCellDis-(cellWidth/2)+(initialCell-1)...
    *(cellWidth+cellSeparation))/Interval:motorFirstCellDis-(cellWidth/2)+...
    (initialCell-1)*(cellWidth+cellSeparation)));
y3_ = zeros(Length,length(motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+(cellWidth/2):...
    (motorFirstCellDis-(cellWidth/2)+(initialCell-1)*(cellWidth+cellSeparation))/Interval:...
    motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+motorFirstCellDis));
y = [y_ Vin_(1:10:length(Vin_),:) y3_];
%}

%Variables
Interval = 20;
Separation = cellWidth+cellSeparation;
initialPoint = motorFirstCellDis-((cellWidth+cellSeparation)/2)...
    +(initialCell-1)*(cellWidth+cellSeparation);
initialCellPoint = initialPoint+((cellWidth+cellSeparation)/2);
finalCellPoint =  motorFirstCellDis+(cellWidth+cellSeparation)*(finalCell-1);
finalPoint = motorFirstCellDis+(cellWidth+cellSeparation)*(finalCell-1)...
    +((cellWidth+cellSeparation)/2);
EndPoint = motorFirstCellDis+(cellWidth+cellSeparation)...
    *(finalCell-1)+motorFirstCellDis;

%X array
x = [0:initialPoint/Interval:initialPoint initialCellPoint:Separation:finalCellPoint...
    finalPoint:(EndPoint-finalPoint)/Interval:EndPoint];
xtest = 0:0.1:EndPoint;

%Y array
Length = length(Vin_(1:10:length(Vin_),initialCell:finalCell));
y_ = zeros(Length,length(0:initialPoint/Interval:initialPoint));
y2_ = zeros(Length,length(finalPoint:(EndPoint-finalPoint)/Interval:EndPoint));

y = [y_ Vin_(1:10:length(Vin_),initialCell:finalCell) y2_];


%% Gaussian Process

for i = 1:length(Vin_(:,1))/10
    gprmdl{i} = fitrgp(x',y(i,:)','Basis','linear','FitMethod','exact','PredictMethod','exact');
    [ypred, ysd, yint] = predict(gprmdl{i}, xtest');
    ypred_ts(i,:,:) = ypred;
    ysd_ts(i,:,:) = ysd;
    yint_ts(i,:,:) = yint;
    disp(i)
end

%% FT Sensor XY-plane Data
figure(5)
for i = 1:length(Vin_(:,1))
    FT(i,:) = Data_i.FT_r(i,:);
end

FT_XY = sqrt(FT(:,1).^2 + FT(:,2).^2);
plot(FT_XY)

%% Normal Force Integral (Every 0.1mm)

summationForce = zeros(1,length(ypred_ts(:,1)));
Data_com = Data_i.FT_r(1:10:(length(Data_i.FT_r)-rem(length(Data_i.FT_r),10)),1);
t_ = 0:length(ypred_ts(:,1))-1;

ypred_ts_pos = max(0,ypred_ts);

for k = 1:length(ypred_ts_pos(:,1))
    summationForce(k) = sum(ypred_ts_pos(k,:))*0.1;
end

%% Plotting Estimated Normal Force and Selected F values
figure(1)
plot(t_,summationForce)
hold on
plot(t_,-Data_com)
grid on
grid minor
legend('Estimate','F')
xlabel('Time(s)')
ylabel('Force(N)')
hold off

%% Normal Force Test

Fn_test = zeros(1,length(Vin_(:,1)));
Vin__ = Vin_*calibrationWidth;
for k = 1:length(Vin_(1,:))
    Fn_test(k) = sum(Vin_(k,2:4));
end
figure(10)
plot(Fn_test);
hold on
plot(-Data_com);
grid on
grid minor

%% Finding Torque from Normal Force Data
summationTorque = zeros(1,length(ypred_ts(:,1)));
for j = 1:length(ypred_ts(:,1))
    k = motorFirstCellDis;
    for i = 1:length(ypred_ts(1,:))
        summationTorque(j) = summationTorque(j) + ypred_ts(j,i)*0.1*k;
        k = k+0.1;
    end
end

%% Plotting Estimated Torque and Selected T values
figure(2)
plot(t_,summationTorque)
hold on
%plot(t_,Data_com)
grid on
grid minor
%legend('Estimate','T')
xlabel('Time(s)')
ylabel('Torque(Nmm)')
hold off

%% Estimating Tangential Force from Normal Force and Torque
%Normal Force와 torque 정보로 어떻게 tangential force mapping 할까?

%% Creating GIF file
figure(3)
clf
for i = 1:size(ypred_ts,1)
    if mod(i,100) == 0
        hold off
        plot(x,y(i,:),'o')
        hold on
        plot(xtest,ypred_ts(i,:))        
        stem(30, -Data_i.FT_r(i*10,1))
        plot(xtest,yint_ts(i,:,1),'k--')
        plot(xtest,yint_ts(i,:,2),'k--')
        grid on
        grid minor
        ylim([-1 10])
        xlim([0 motorFirstCellDis+(cellWidth+cellSeparation)*(cellNum-1)+(cellWidth/2)+20])
        xlabel('Cell #')
        ylabel('Normal Force (N)/Force Density (N/m)')
        legend('Measured','Estimated distribution','FT sensor','95% interval','FontSize',13)
        title(['Contact Normal Force Distribution Estimation, Time: ' num2str(i/100), '(s)']...
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