
close all
format short g
clc
% clear

%%
Vxrange=[VxBound(1):1:VxBound(2)];

for i=1:numel(Vxrange)
    
    Vx=Vxrange(i);
    alfas=computeAlfaUpLow(Vx, VxBound);

    Gv = alfas(1)*Gc{1}+alfas(2)*Gc{2}+alfas(3)*Gc{3};
    Zv = alfas(1)*Zc{1}+alfas(2)*Zc{2}+alfas(3)*Zc{3};
    Kv4=Zv*inv(Gv);

    % a0_paramsModels
 
    a00_paramsModels_forSims 
     
   [Ade, Bde, Cde, Dde]=ssdata(sysDe);
 
 

     %% Preview Control 
 
    Daug = zeros(np);                  % initialize
    Daug(1:end-1,2:end) = eye(np-1);   % set up the shift register
    Eaug = zeros(np,1);                % input vector for shift register
    Eaug(end) = 1; 

    % Now we need to set up the cost function
    Caug = zeros(2,np+4);                                       % initialize to the right size
    Caug(:,1:6) = [1 0 0 0 -1 0;0 0 1 0 1/(Vx*dt) -1/(Vx*dt)];  % set actual values

    %% Augment Plant
    Aaug = blkdiag(Ade,Daug);                                   % automatically creates the big a matrix
    Baug = [Bde;zeros(size(Eaug))];                             % input vector for steering angle
    B2aug = [zeros(size(Bde));Eaug];                            % input vector for road
    
    %% Assign Preview Coeffs
    Kprv=-Kv4(1:end);
    
    %% Build Road Input Signal
    
    Uroad = [zeros(1,2/dt) linspace(0,3,4/dt) 3*ones(1,10/dt)];
    %lag the reference vector
    Uroad_lagged = [Uroad(1)*ones(1,np) Uroad(1:end-np)];
    sim_steps = length(Uroad);
    t = dt*(1:sim_steps)-dt;
    X0 = zeros(size(Baug));%set initial conditions to zero

    %% Simulate
    Aclprev=Aaug-Baug*Kprv;
    Cdprv=[1 0 0 0 zeros(1, np)];
    Kr_prv = abs(1/(0 +  Cdprv*inv(eye(np+4) - Aclprev)*B2aug));  
    Kr_prv=1;

    [yprv,xprv] = dlsim(Aaug-Baug*Kprv, B2aug*Kr_prv, Kprv, 0, Uroad, X0);
    
    %% Plot Time Response Figure
    figure(1)
    hold on
    plot(t,xprv(:,1)','k')
    
    %% Plot Bode Mag
    Aclprev=Aaug(1:4,1:4)-Baug(1:4)*Kprv(1:4);
    Cout2=[1 0 0 0];
    ssPrevCL=ss(Aclprev, Baug(1:4), Cout2, 0, dt);
    
    figure(2)
    bodemag(ssPrevCL)
    hold on
end
figure(1)
plot(t,Uroad_lagged,'r-.', 'LineWidth', 1.2)

%%
figure(3)
plot(-Kc{1}(5:end))
hold on
plot(-Kc{2}(5:end))
plot(-Kc{3}(5:end))
legend('K1','K2','K3')

figure(4)
margin(ssPrevCL)