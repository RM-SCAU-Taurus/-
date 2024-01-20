% Allan Variance: Data simulation and analysis demo script.
%
% Author: Juan Jurado, Air Force Institute of Technology, 2017

%Changed by Zhao YongZheng @HIT 20181105

% Initial MATLAB macros for figure formatting
clear all; clc; close all;
plot_them = 1;%if data is short :plot_them = 0

% Now feed simulated data to Allan Deviation script to compute Alla
% Deviation and coresponding T for later analysis.

% stim300 100hz deg/s
%load('wz'); 
 filename= 'D:\RM\Taurus_IMU4.1.1\data3.csv'
 data=csvread(filename)
 Y=data
%Y  = wz;% matrix  n*1  % deg /s 
%dt =1/8;  %s
dt=0.001
[AllanSigma, T] = ComputeAVAR(Y,dt); 

% AllanSigma is expressed in deg/s (gyro) or m/s^2 (accel), we'll convert
% the denominator to hours prior to anlayzing so we don't have to scale the
% outputs later.
AllanSigma = AllanSigma*3600; % Now in deg/hr (gyros) or m/s^2/hr(accels)
T = T/3600; % Now in hrs

% Finally, feed AllanSigma and T to analyzer script to extract noise
% parameters and compare to those we first introducted into Y.

% The script takes in Log-Log slopes of interest along with respective T
% values to extrapolate to prior to reading desired Allan Deviation. Per
% the literature, the slopes of interest are: -1, -1/2, 0 and 1/2;
slopes = [-1;-0.5;0;0.5;1];

% Also per literature, the corresponding T values to extrapolate to are:
% T=sqrt(3), T=1, N/A, and T=3. Note we've already converted T to hours at
% this point, so there is no need to multiply these values by 3600.
Ts = [sqrt(3);1;NaN;3;sqrt(2)];

% The script will analyse AllanSigma, look for the slopes, extrapolate to
% Ts and report the appropriate AllanSigma found. The last parameter is a
% plot option.
[sigmasOut,Tbias,figs,hs] = AnalyzeAVAR(AllanSigma,T,slopes,Ts,1,1);

% We finally take the output of the analyzer and apply any other scaling
% per the literature. The only scaling needed at this point is for bias
% instability, where we divide by sqrt((2*log(2)/pi)).
sigmasOut(3) = sigmasOut(3)/sqrt((2*log(2)/pi));


fprintf(' Quantization:%0.2e [deg]\n', sigmasOut(1));
fprintf('Random Walk:%0.2e [deg/sqrt{hr}]\n',sigmasOut(2));
fprintf('Bias Instability:%0.2e [deg/hr]\n',sigmasOut(3));
fprintf( 'Rate Random Walk:%0.2e [deg/hr/sqrt{hr}]\n', sigmasOut(4));
fprintf( 'Rate Ramp:%0.2e [deg/hr/hr]\n', sigmasOut(5));

% sigmasOut(1) % Quantization  deg (gyros) OR m/s (accels)
% sigmasOut(2) %  Random Walk  deg/sqrt(hr) (gyros) OR m/s/sqrt(hr) (accels)
% sigmasOut(3)%  Bias Instability  deg/hr (gyros) OR m/s/hr (accels)
% sigmasOut(4)% Rate Random Walk deg/hr/sqrt(hr) (gyros) OR m/s/hr/sqrt(hr)(accels)
% sigmasOut(5) % Rate Ramp deg/hr/hr
 if plot_them
sigmaQ = sprintf('Quantization:%0.2e [deg]',...
    sigmasOut(1));
sigmaRW = sprintf('Random Walk:%0.2e [deg/sqrt(hr)]',...
    sigmasOut(2));
sigmaBias = sprintf('Bias Instability:%0.2e [deg/hr]',...
    sigmasOut(3));
sigmaRRW = sprintf(...
    'Rate Random Walk:%0.2e [deg/hr/sqrt(hr)]',...
    sigmasOut(4));
sigmaRR = sprintf(...
    'Rate Ramp:%0.2e [deg/hr/hr]',...
    sigmasOut(5));
figure(figs(1)); hold on; title('Simulated Data: Allan Deviation');
set(gcf,'Position',[0 0 800 800]);
legend(hs(1,2:end),sigmaQ,sigmaRW,sigmaBias,sigmaRRW,sigmaRR,'Location',...
    'NorthEast');
xlabel('Averaging Time, $\tau$ (hr)');
ylabel('Allan Deviation, $\sigma(\tau)$ (deg/hr)');
grid minor;

% Choose if FOGM approximation to bias stability is desired. If not,
% simulation will use "Pink" noise generator for 1/f flicker noise. 
figure(figs(2)); hold on; title('Simulated Data: Allan Deviation Slope');
set(gcf,'Position',[0 0 800 800]);
legend(hs(2,2:end),'-1: Quantization','-1/2: Random Walk',...
    '0: Bias Instability','+1/2: Rate Random Walk',...
    '+1: Rate Ramp','Location','NorthEast');
xlabel('Averaging Time, $\tau$ (hr)');
ylabel('Allan Deviation Slope');
grid minor;
 
end





%% Bias stability and Bias

N=10;%N s
num=N/dt;%%m每组内数据个数
m=floor(length(Y)/num);%%共可以分成m组数据
gx=zeros(m,1);

Xbais_all=mean(Y);

for i=1:m-1
    gx(i)=mean(Y(1+(i-1)*num:1+i*num,1));
end


gx= gx(2:end-2,:);

Xbais_stability=std(gx)*3600;

fprintf('Bias Stability 10s:%0.2e [deg/hr]\n',Xbais_stability);

%%%%%%%%%%%%%%%
