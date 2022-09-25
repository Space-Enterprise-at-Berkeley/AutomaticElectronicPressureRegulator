%% load('models.mat')

%% This section gets data for system identification app


% Read csv files

setpointF = csvread('fuel_angle_setpoint.csv');
xsetF = setpointF(:,1);
ysetF = setpointF(:,2);

angleF = csvread('fuel_encoder_angle.csv');
xencF = angleF(:,1);
yencF = angleF(:,2);

setpointL = csvread('lox_angle_setpoint.csv');
xsetL = setpointL(:,1);
ysetL = setpointL(:,2);

angleL = csvread('lox_encoder_angle.csv');
xencL = angleL(:,1);
yencL = angleL(:,2);

% existing data has unequal timesteps, resample for equal timestep
resampledF = resample(yencF, xencF);
resampledsetF = resample(ysetF, xsetF);
resampledL = resample(yencL, xencL);
resampledsetL = resample(ysetL, xsetL);

% converts data into time domain data set
% iddata requires y (output), u (input), and Ts (sample time)
t = size(ysetL,1);
dataF = iddata(resampledF, resampledsetF, (resampledF(t)-resampledF(1)));
dataL = iddata(resampledL, resampledsetL, (resampledL(t)-resampledL(1)));

% plot(resampledF)
% plot(resampledsetF)
% plot(resampledL)
% plot(resampledsetL)


%% Using system identification
% systemIdentification

% select "time domain data" under "import data"
% Workspace Variable Input: resampledsetF    Output: resampledF
% Data Information data name: dataF
% Start time: 0.803 fuel, 0.8 lox    End time: 4.0 (both)

% under "Estimate", select "Quick Start"
% open the "n4s3" model from the generated models
% "Export" will save an idss object in workspace called "n4s3"

% repeat with resampledsetL, resampledL, dataL for lox

% A, B, C, D, K can be called by modelname.A