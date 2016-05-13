%% Calculate state space and create a fitting controller

% %% State-space
% 
% % Parameters
% Ix,Iy,Iz;
% J=diag(Ix,Iy,Iz); %Inertia matrix
% b,d,k,l,m,g; % other params
g=9.81;
m=0.027;
b=1e-3;
k=b;
d=0.1;
Ix = 1.395e-5;
Iy = 1.436e-5;
Iz = 2.173e-5;
% 
% % States needed (all [3])
% xyz;  % pos in global [x,y,z] frame
% v;    % speed in global dot[x,y,z] frame
% rpy;  % roll,pitch,yaw in local frame
% w;    % rotational speed in local frame
% 
% % Transformation matrixes
% R1;   % Rotation used in Newton eq
% R2;   % Rotation used to transform rpy to w
% TorqueThrust=[-1, -1, -1, -1;
%                   0, -d, 0, d;
%                   d, 0, -d, 0;
%                   k/b, -k/b, k/b, -k/b]; % generates Thrust(sum) and torque(1,2,3)
              
% Equations

A=zeros(12);
A(1:3,4:6)=eye(3);
A(7,2)=-g;
A(8,1)=g;
A(10:12,7:9)=eye(3);

B=zeros(12,4);
B(4,2)=1/Ix;
B(5,3)=1/Iy;
B(6,4)=1/Iz;
B(9,1)=1/m;

C=eye(12);

sys=ss(A,B,C,0);


%% Controller design
% Select parameters
Q=eye(12)*100;
Q(10,10)=1;
Q(11,11)=1;
Q(end)=1;
R=eye(4)/10;
K = lqr(sys,Q,R); % K is the feedback vector

%% Kalman design
A=zeros(12);
A(1:3,4:6)=eye(3);
A(7,2)=-g;
A(8,1)=g;
A(10:12,7:9)=eye(3);

B=zeros(12,4);
B(4,2)=1/Ix;
B(5,3)=1/Iy;
B(6,4)=1/Iz;
B(9,1)=1/m;

C=eye(12);
% Values in the quad: 
% roll,pitch,yaw - from sensor fusion (euler deg)
% p,q,r - from gyro (deg/s)
% accelerometer data axis (mG)
% magnetometer axis (Tesla?)

sys=ss(A,B,C,0);
sysd=c2d(sys,0.1);

Qk=eye(12);
Rk=eye(12);
[L, P, E] = lqe(sys, Qk, Rk)

% Test on linear plant

%% pretty outputs

fb=fopen('feedback.txt','w');
%K
for n=1:length(K(:,1))
    fprintf(fb,'{');
    fprintf(fb,'%14.5f',K(n,1));
    for i=K(n,2:end)
        fprintf(fb,',%14.5f',i);
    end
    fprintf(fb,'}\n');
end
fclose(fb);
%A  discrete
sysd=c2d(sys,1/500,'zoh');
Ad=fopen('Ad.txt','w');

for n=1:length(sysd.a(:,1))
    fprintf(Ad,'{');
    fprintf(Ad,'%10.10f',sysd.a(n,1));
    for i=sysd.a(n,2:end)
        fprintf(Ad,',%10.10f',i);
    end
    fprintf(Ad,'}\n');
end
fclose(Ad);
%B  discrete
Bd=fopen('Bd.txt','w');

for n=1:length(sysd.b(:,1))
    fprintf(Bd,'{');
    fprintf(Bd,'%10.10f',sysd.b(n,1));
    for i=sysd.b(n,2:end)
        fprintf(Bd,',%10.10f',i);
    end
    fprintf(Bd,'}\n');
end
fclose(Bd);
%C  discrete
Cd=fopen('Cd.txt','w');

for n=1:length(sysd.c(:,1))
    fprintf(Cd,'{');
    fprintf(Cd,'%10.10f',sysd.c(n,1));
    for i=sysd.c(n,2:end)
        fprintf(Cd,',%10.10f',i);
    end
    fprintf(Cd,'}\n');
end
fclose(Cd);
%D  discrete
Dd=fopen('Dd.txt','w');

for n=1:length(sysd.d(:,1))
    fprintf(Dd,'{');
    fprintf(Dd,'%10.10f',sysd.d(n,1));
    for i=sysd.d(n,2:end)
        fprintf(Dd,',%10.10f',i);
    end
    fprintf(Dd,'}\n');
end
fclose(Dd);
