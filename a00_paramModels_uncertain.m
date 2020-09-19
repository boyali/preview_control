% Vehicle Parameter
clc
% clear
    %VxBound = [5, 10];
    mu=1;                       % road friction coefficient
    m0=1910;                    % mass (Kg) - empty weight 1690
    Lf=1.4;                     % distance from C.O.G to the front axle (m)
    Lr=2.96-Lf;                 % distance from C.O.G to the rear axle (m)


    % Iz=m0*Lf*Lr;             % moment of inertia around the z-axis (kgm^2)
    % Iz=3667;				   % With full front rear mass - 
                               % unsprung version is 3061, Hue-san 3200 	
                               
    iyaw=0.992;
    Iz = iyaw*m0*Lf*Lr;		% moment of inertia about z, kg-m^2                               
                               
    % Iz=3200;                          
    L=Lf+Lr;                   % wheel base


%% Uncertain Parameters
    gRavity=9.81;
    wwFL=464;       % Front Left Vertical Load
    wwFR=426;       % Front Right Vertical Load
    wwF=wwFL+wwFR;
    
    
    wwRL=415;       % Front Right Vertical Load
    wwRR=382;       % Front Right Vertical Load
    wwR=wwRL+wwRR;
    
    CCf=0.171*180/pi;   % Front cornering stiffness coeff ./rad
    CCr=0.181*180/pi;   % Rear cornering stiffness coeff ./rad
    
    cfc=1.2;
    Cf=cfc*CCf*wwF*gRavity/2;
    Cr=cfc*CCr*wwR*gRavity/2;
    
%% 
Vx0as=sum(VxBound)/2;

Cf0=ureal('Cf', Cf, 'Percentage', 30);  % front cornering stiffness (N/rad)
Cr0=ureal('Cr', Cr, 'Percentage', 30);
Cf0.AutoSimplify='full';
Cr0.AutoSimplify='full';

iVx=ureal('iVx0', 1/Vx0as);                   % for 1/Vx velocity (m/s)
iVx.Range=[1/VxBound(2) 1/VxBound(1)];
iVx.AutoSimplify='full';



Vx0b=ureal('Vx0b', Vx0as);                 % velocity (m/s)
Vx0b.Range=[VxBound(1) VxBound(2)];
Vx0b.AutoSimplify='full';

% Simulation parameter

dt=0.02;   % simulation rate (s)  
mu0=1;
  
 
%% State Space form, no disturbance, x_dot=Ax+Bu, y=Cx
% States [y, ydot, psi, psidot]

% A0u=[0, 1, Vx0b, 0;...
%    0, -2*(Cf0+Cr0)*(iVx)/(m0), 0, -Vx0b-(2*(Lf*Cf0-Lr*Cr0)*(iVx)/(m0));...
%    0, 0, 0, 1;...
%    0, -2*(Lf*Cf0-Lr*Cr0)*(iVx)/(Iz), 0, -2*(Cf0*(Lf^2)+Cr0*(Lr^2))*(iVx)/(Iz)];
%  
% B0u=[0; 2*Cf0/m0; 0; Lf*2*Cf0/Iz];
% 
% C0u=zeros(2,4);
% C0u(1,1)=1;
% C0u(2,3)=1;
% 
% D0u=zeros(2,1);

A0u=[0 1 0 0;...
    0 -mu0*2*(Cf0+Cr0)*(iVx)/(m0) mu0*2*(Cf0+Cr0)/m0 -mu0*2*(Cf0*Lf-Cr0*Lr)*(iVx)/(m0);...
    0 0 0 1 ;...
    0 -mu0*2*(Cf0*Lf-Cr0*Lr)*(iVx)/(Iz) mu0*2*(Cf0*Lf-Cr0*Lr)/(Iz) -mu0*2*(Cf0*Lf^2+Cr0*Lr^2)*(iVx)/(Iz)];

Bd10u=[0; -mu0*2*(Lf*Cf0-Lr*Cr0)*(iVx)/(m0); 0; -mu0*2*(Cf0*Lf^2+Cr0*Lr^2)*(iVx)/(Iz)];
Bd20u=[0; -Vx0b; 0; 0];
B0u=[0; mu0*2*Cf0/m0; 0; mu0*Lf*2*Cf0/(Iz)];


C0u=zeros(2,4);
C0u(1,1)=1;
C0u(2,3)=1;

% D0u=zeros(2,1);
D0u=zeros(2,2);


 

%% Uncertain Plant
% uncPlantsys=uss(A0u, B0u, C0u, D0u);

uncPlantsys=uss(A0u, [Bd10u+Bd20u B0u], C0u, D0u);
% uncPlantsys=uss(A0u, B0u, C0u, D0u);

uncPlantsys.StateName={'Lateral Position', 'Lateral Velocity', 'Yaw Angle', 'Yaw Rate'};
uncPlantsys.InputName={'Steering Wheel'};
% uncPlantsys.OutputName={'Lateral Position','Lateral Velocity','Yaw Angle', 'Yaw Rate'};

uncPlantsys.InputGroup.ActualIn=1;
uncPlantsys.OutputGroup.ActualOut=[1,2];

get(uncPlantsys)

%% Robust Toolbox Definitions

[Gnom,Delta,Blkstruct,Normunc]=lftdata(uncPlantsys); 
% [Msys,Delta] = lftdata(uncPlantsys); 
minfo(Gnom)

[Anmt, Bnmt, Cnmt, Dnmt]=ssdata(Gnom);
Gnomsys=pck(Anmt, Bnmt, Cnmt, Dnmt);

% bounds = mussv(Gnomsys,Blkstruct)