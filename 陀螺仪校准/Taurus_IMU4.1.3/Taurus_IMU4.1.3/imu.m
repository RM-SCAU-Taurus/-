filename= 'D:\RM\Taurus_IMU4.1.1\data.csv'
data=csvread(filename);
tao0=0.1;   %��������5ms
[sigma,tau,Err]=avar(data,tao0);
