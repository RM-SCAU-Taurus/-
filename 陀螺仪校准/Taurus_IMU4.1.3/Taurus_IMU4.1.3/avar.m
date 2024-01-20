function[sigma,tau,Err] = avar(y0,tau0)
N=length(y0);
y=y0;NL=N;
for k=1:16
    sigma(k,1)=sqrt(1/(2*(NL-1))*sum([y(2:NL)-y(1:NL-1)].^2));
    tau(k,1)=2^(k-1)*tau0;
    Err(k,1)=1/sqrt(2*(NL-1));
    NL=floor(NL/2);
    if NL<3
        break;
    end
    y=1/2*(y(1:2:2*NL)+y(2:2:2*NL));
end
% plot(tau0*[1:N],y0,'*');
% grid on
% xlabel('\itt\rm/s');
% ylabel('\ity');
figure;
loglog(tau,sigma,'- +',tau,[sigma.*(1+Err),sigma.*(1-Err)],'r--');
grid on
xlabel('\itt\rm/s');
ylabel('\it\sigma_A\rm(\tau)');
legend('allan方差双对数图','allan方差双对数误差图')
title('allan方差双对数图')
