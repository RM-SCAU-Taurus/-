filename= 'D:\RM\Taurus_IMU4.1.1\data.csv'
data=csvread(filename);
tao0=0.1;   %采样周期5ms
[sigma,tau,Err]=avar(data,tao0);
