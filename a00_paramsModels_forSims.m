% Vehicle Parameter
clc
% clear

%%
    gRavity=9.81;
    wwFL=464;       % Front Left Vertical Load
    wwFR=426;       % Front Right Vertical Load
    wwF=wwFL+wwFR;
    
    
    wwRL=415;       % Front Right Vertical Load
    wwRR=382;       % Front Right Vertical Load
    wwR=wwRL+wwRR;
    
    CCf=0.171*180/pi;   % Front cornering stiffness coeff ./rad
    CCr=0.181*180/pi;   % Rear cornering stiffness coeff ./rad
    Cf=CCf*wwF*gRavity/2;
    Cr=CCr*wwR*gRavity/2;
    
    
%%

mu=1;                       % road friction coefficient
m_car=1910;                 % mass (Kg) - empty weight 1690
Lf=1.4;                    % distance from C.O.G to the front axle (m)
Lr=2.96-Lf;                    % distance from C.O.G to the rear axle (m)


%Iz=m_car*Lf*Lr;             % moment of inertia around the z-axis (kgm^2)
iyaw=0.992;
Iz = iyaw*m_car*Lf*Lr;		% moment of inertia about z, kg-m^2

% Iz=3667;				  % With full front rear mass - 
						  % unsprung version is 3061, Hue-san 3200 	


L=Lf+Lr;                    % wheel base
Rfactor = Cr*Lr- Cf*Lf;

% Simulation parameter

dt=0.02;   % simulation rate (s)  

  
 
%% State Space form, no disturbance, x_dot=Ax+Bu, y=Cx
% States [y, ydot, psi, psidot]

A=[0, 1, Vx, 0;...
   0, -2*(Cf+Cr)/(m_car*Vx), 0, -Vx-(2*(Lf*Cf-Lr*Cr)/(m_car*Vx));...
   0, 0, 0, 1;...
   0, -2*(Lf*Cf-Lr*Cr)/(Iz*Vx), 0, -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Iz*Vx)];
 
B=[0; 2*Cf/m_car; 0; Lf*2*Cf/Iz];
C=eye(4);
D=zeros(4,1);

% Discrete Time
sysDy = c2d(ss(A, B, C, D),dt); %discrete time state space description.
Ady = sysDy.a;
Bdy = sysDy.b;


% Write error model

%% Error Dynamics Model

Ae=[0, 1, 0, 0,;...
    0, -2*(Cf+Cr)/(m_car*Vx), 2*(Cf+Cr)/m_car, -(2*(Lf*Cf-Lr*Cr)/(m_car*Vx));...
    0, 0, 0, 1;...
    0,-2*(Lf*Cf-Lr*Cr)/(Iz*Vx), 2*(Lf*Cf-Lr*Cr)/(Iz), -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Iz*Vx)];   

Be=[0; 2*Cf/m_car; 0; 2*Lf*Cf/Iz];
Be2=[0; -Vx-2*(Cf*Lf-Cr*Lr)/(m_car*Vx);0 ; -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Iz*Vx)] ; % External Error do to Psi_des_dot
Ce=eye(4);
De=zeros(4,1);

Becom=[Be, Be2];

%% Continous Time State Space
% sysErrCont=ss(Ae, Be, Ce, De);

sysCe=ss(Ae, Be, Ce, De);
sysDe = c2d(ss(Ae, Be, Ce, De),dt); %discrete time state space description.
 
% sysDetz1=absorbDelay(sysDetz)
% 
% sysDe1=ss(sysDetz1);