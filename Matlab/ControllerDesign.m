%% Calculate state space and create a fitting controller

% %% State-space
%
% % Parameters
% Ix,Iy,Iz;
% J=diag(Ix,Iy,Iz); %Inertia matrix
% b,d,k,l,m,g; % other params
g=9.81;
m=0.027;
b=2.75e-11;
k=1e-9;
d=0.05;
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
%%
syms d k b;
 TorqueThrust=[-1, -1, -1, -1;
                   0, -d, 0, d;
                   d, 0, -d, 0;
                   k/b, -k/b, k/b, -k/b]; % generates Thrust(sum) and torque(1,2,3)
               
 Rotz=[ cosd(45) -sind(45) 0;
        sind(45) cosd(45) 0;
        0   0   1];
    
 newTorque=Rotz*TorqueThrust(2:end,:);
 Total=[TorqueThrust(1,:);newTorque];
 Ainv = inv(Total)
%%
% Equations

A=zeros(6);
A(1:3,4:6)=eye(3);
%A(8,7)=1;

B=zeros(6,4);
B(4,2)=1/Ix;
B(5,3)=1/Iy;
B(6,4)=1/Iz;
%B(7,1)=1/m;

C=eye(6);

sys=ss(A,B,C,0);
Ts=1/250; % 250Hz (The example in the quad uses this)
sysd=c2d(sys,Ts);

%% Controller design
% Select parameters
Q=diag([1e4, 1e4, 1e-7,... % r,p,y
    1e4, 1e4, 1e4]); %dz z
R=diag(1*[1, 1, 1, 1]); % thrust,Tx,Ty,Tz
K = lqr(sysd,Q,R) % K is the feedback vector
closed_poles=eig(sysd.a-sysd.b*K)
[K,s,e]=lqi(sysd,[Q,zeros(6);zeros(6),0.09*Q],R);
%% Kalman design
% A=zeros(12);
% A(1:3,4:6)=eye(3);
% A(7,2)=-g;
% A(8,1)=g;
% A(10:12,7:9)=eye(3);
% 
% B=zeros(12,4);
% B(4,2)=1/Ix;
% B(5,3)=1/Iy;
% B(6,4)=1/Iz;
% B(9,1)=1/m;
% 
% C=eye(12);
% C(1,1)=0; % x is not an output
% C(2,2)=0; % y -||-
% C(4,4)=0 % xdot
% C(5,5)=0 % ydot
% % Values in the quad:
% % roll,pitch,yaw - from sensor fusion (euler deg)
% % p,q,r - from gyro (deg/s)
% % accelerometer data axis (mG)
% % magnetometer axis (Tesla?)
% 
% sys=ss(A,B,C,0);
% sysd=c2d(sys,0.1);
% 
% Qk=eye(12);
% Rk=eye(12);
% [L, P, E] = lqe(sys, Qk, Rk)

% Test on linear plant

%% pretty outputs


fb=fopen('feedback.txt','w');

for n=1:length(K(:,1))
    fprintf(fb,'%14.5f',K(n,1));
    for i=K(n,2:end)
        fprintf(fb,',%14.5f',i);
    end
    fprintf(fb,';\n');
end
fclose(fb);

K_c=fopen('feedback_c.txt','w');
for n=1:length(K(:,1))
    fprintf(K_c,'{');
    fprintf(K_c,'%10.10f',K(n,1));
    for i=K(n,2:end)
        fprintf(K_c,',%10.10f',i);
    end
    fprintf(K_c,'},\n');
end
fclose(K_c);

K;