%% Calculate state space and create a fitting controller

%% State-space

% Parameters
Ix,Iy,Iz;
J=diag(Ix,Iy,Iz); %Inertia matrix
b,d,k,l,m,g; % other params


% States needed (all [3])
xyz;  % pos in global [x,y,z] frame
v;    % speed in global dot[x,y,z] frame
rpy;  % roll,pitch,yaw in local frame
w;    % rotational speed in local frame

% Rotational transformation matrixes
R1;   % Used in Newton eq
R2;   % Used to transform rpy to w
