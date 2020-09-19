clear
clc

 

%{ 
  Without LFT structure with B1w which is the road disturbance;
%}

%% Load Uncertain System

np =50;                             % Number of Preview Points  
VxBound=[3, 30];                     % Polytope Lower and Upper Bounds 
 
a00_paramModels_uncertain

VxPoly=computePolyUpLow(VxBound);   % Find Vertice Points
N=size(VxPoly,1);                   % Number of Vertices

dt=0.02;                           % sampling time

%% Prepare Road

Aprv = zeros(np,np);                  % Initialize
Aprv(1:end-1,2:end) = eye(np-1);      % Set up the shift register

Bprv0 = zeros(np,1);                   % Input vector for shift register
Bprv0(end)=1;

%% Generalized Plant
 
yp=icsignal(2);

% Define inputs
ref      = icsignal(1);
rcurve   = icsignal(1);
rprv     = icsignal(1);
distunc  = icsignal(2);
cont     = icsignal(1);
noise    = icsignal(1);

M=iconnect;
M.input=[rprv;rcurve;cont];

%% System Definition

ford   = np+4;          % Full order of the system
ncinp  = 1;             % Number of control input

%% Weights 
 
close all

%%
cQ{1}=[0.90 0; 0 0.005]; %(1.01, 0.47) R 0.11
cR{1}=(0.209); 

cQ{2}=[0.75 0; 0 0.003]; %(1.01, 0.47) R 0.11
cR{2}=(0.209); 

cQ{3}=[0.75 0; 0 0.003]; %(1.01, 0.47) R 0.11
cR{3}=(0.209); 
%% Prepare Vertice Models

for i=1:N
       
      vertVal=VxPoly(i,:);
      %% Submit Unkwnowns at the vertices
       
      sysDen1 = usubs(uncPlantsys,'Vx0b', vertVal(1), 'iVx0', vertVal(2),  'Cf', Cf, 'Cr', Cr); 
      dSys    = c2d(sysDen1, dt);
      [Agd, Bgd, Cgd, Dgd]= ssdata(dSys);  
%       Bgd=Bgd(:,2);
      %% Augment System
     
      Aaug  = blkdiag(Agd,Aprv);                % Automatically creates the big matrix          
      Baug  = [Bgd;zeros(np,size(Bgd,2))];       % Input vector for steering angle
      Bprv  = [zeros(4,1);Bprv0];                 % Input vector for road    
      Baug3 = [Bprv Baug];
      
      %% Prepare Output
      
      Czaug = zeros(2,np+4); 
      Czaug(end-1:end,1:6) = [1 0 0 0 -1 0;...
                            0 0 1 0 1*vertVal(2)/(dt) -1*vertVal(2)/(dt)];%set actual values

      Daug=zeros(size(Czaug,1),size(Baug3,2));
      Daug(1:size(Dgd,1), 1:size(Dgd,2))=Dgd;
    
      sysDenM=ss(Aaug, Baug3, Czaug, Daug, dt);

      %% Prepare Generalized Plant
      
%       M.equation{1}=equate(yp, sysDenM*[rprv;rcurve;cont]); 
      M.equation{1}=equate(yp, sysDenM*[rprv;rcurve;cont]); 
      M.output=[cQ{1}*[yp(1);yp(2)];cR{1}*cont];
      
      T = M.System;  
      
      %% Assign Vertice System Matrices
      
     Ad{i}=T.a;  
     Bw{i}=T.b(:,1:end-1)*1;
     
     Bu{i}=T.b(:,end);
     Cz{i}=T.c;
 
     Dw{i}=T.d(:,1:end-1);
     Du{i}=T.d(:,end);
end


%% LMI Solution - Prepare Yalmip

yalmip('clear');
LMIs = [];
etainf=sdpvar(1);

%% Prepare Intstumental Matrices


for i=1:N   % N is number of vertices
    
    P11{i}   = sdpvar(4, 4,'symmetric');
    P12{i} = sdpvar(4, np,'full');
    P22{i} = sdpvar(np, np,'symmetric');
    
%     Pinf{i} = sdpvar(np+4, np+4,'symmetric');
    Pinf{i}=[P11{i}, P12{i};P12{i}', P22{i}];    
    Z11{i} = sdpvar(1,4,'full');
    Z12{i} = sdpvar(1,np,'full');
    Zinf{i} = [Z11{i}, Z12{i}];

    LMIs=[LMIs, Pinf{i}>=0];            % Strict inequalities are not allowed
end

%% Stability Contstaints


%{
    Stability pole constraint rho_s
    Positivity region starts from rho_p
%} 
rho_p=0.3;  
rho_p2=0.95; 
rho_i=0.2;
rho_s=0.2;

%%

for i=1:N
  
        T1im11=Pinf{i};
        T1im12=Ad{i}*Pinf{i}+Bu{i}*Zinf{i};
        T1im13=Bw{i};
        T1im14=zeros(size(Bw{i},1),size(Cz{i},1));
        
        T1im21=T1im12';
        T1im22=Pinf{i};
        T1im23=zeros(size(Bw{i},2), size(Pinf{i},2))';
        T1im24=(Cz{i}*Pinf{i}+Du{i}*Zinf{i})';
        
        T1im31=T1im13';
        T1im32=T1im23';
        T1im33=eye(size(Bw{i},2));
        T1im34=Dw{i}';
        
        T1im41=T1im14';
        T1im42=T1im24';
        T1im43=T1im34';
        T1im44=etainf*eye(size(Cz{i},1));
        
        T1inf=[T1im11 T1im12 T1im13 T1im14;...
               T1im21 T1im22 T1im23 T1im24;...
               T1im31 T1im32 T1im33 T1im34;...
               T1im41 T1im42 T1im43 T1im44]>=0;
           
        % Pole Constraint
        T1im12POLE=Ad{i}(1:4, 1:4)*P11{i}+Bu{i}(1:4)*Z11{i};
        
        T1pos   =(-2*rho_p*P11{i}+T1im12POLE(1:4,1:4)+T1im12POLE(1:4,1:4)')>=0; % Positivity
        T1pos2  =(-2*rho_p2*P11{i}+T1im12POLE(1:4,1:4)+T1im12POLE(1:4,1:4)')<=0; % Positivity
        
        T2stab1 =((1-rho_s)*P11{i}+T1im12POLE)>=0;
        T2stab2 =((1-rho_s)*P11{i}+T1im12POLE')>=0;
       
        T3im1  =(2*rho_i*P11{i}+T1im12POLE-T1im12POLE')>=0; % Positivity
        T3im2  =(2*rho_i*P11{i}-T1im12POLE+T1im12POLE')>=0; % Positivity
       
%         LMIs=[LMIs, T1inf, T1pos, T2stab1, T2stab2, T3im1, T3im2];    
%        LMIs=[LMIs, T1inf];  
       LMIs=[LMIs, T1inf,T1pos];
%         LMIs=[LMIs, T1inf,T1pos,T1pos2];
%         LMIs=[LMIs, T1inf,T1pos, T3im1, T3im2];
%         LMIs=[LMIs, T1inf,T3im1, T3im2];
%        LMIs=[LMIs, T1inf,T2stab1, T2stab2]; 
 
       
end 

%% Solve


options = sdpsettings('solver','mosek');
% options.mosek.MSK_IPAR_INTPNT_MAX_ITERATIONS=12; 

solution = optimize(LMIs,etainf,options)

[pc,dc]=checkset(LMIs) ;
perfMeasure=double(sqrt(etainf))

%% Save and Plot Results

Acl=cell(3,1);
Kc=cell(3,1);
Gc=cell(3,1);
Zc=cell(3,1);

for i=1:N
    Kc{i}=double(Zinf{i})*inv(double(Pinf{i}));
    
    Atrunc{i} = Ad{i}(1:4, 1:4);
    Btrunc{i} = Bu{i}(1:4);
    Ktrunc{i} = Kc{i}(1:4);
    Acl{i}    = Atrunc{i}+Btrunc{i}*Ktrunc{i};
    
    sysPoles{i}=ss(Acl{i},Btrunc{i}, eye(4),0, dt);
    
    Zc{i}=double(Zinf{i});
    Gc{i}=double(Pinf{i});
    
end

%% Plot

% load Kold
% plot(Kold, 'LineWidth', 1.5)
% hold on
% plot(-Kc{1})
% hold on
% plot(-Kc{2})
% plot(-Kc{3})
% legend('Kold','K1','K2','K3')

%%
save('Kv4','Kc', 'perfMeasure', 'Gc', 'Zc') 
% cc=strcat('sol', int2str(nwd), '_delay', int2str(np), 'prw6')
% save(cc)

%% Pole Zero Map
pn=1;
[poles1,zeros1] = pzmap(sysPoles{pn});
figure
pzmap(sysPoles{pn})
zgrid

eig(sysPoles{pn})