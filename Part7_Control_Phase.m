clc

%% Control Phase: PD and PD+Gravity

% PD Controller
Sim_PD=sim('PD_Controller');

%% 
for i=1:5
    figure(i)
    plot(Sim_PD.tout,Sim_PD.R_PD(:,i),'b','LineWidth',1.3)
    hold on
    plot(Sim_PD.tout,Sim_PD.In_PD(:,i),'r--','LineWidth',1.3)
    grid on
    title(['Response Vs Input of Joint#' num2str(i) 'Via PD Controller'])
    legend('Response','Input Signal')
    xlabel('Time')
    ylabel('Response')
end

%%
% PD+Gravity Controller
Sim_PDG=sim('PD_Controller');

%% 
for i=1:5
    figure(i)
    plot(Sim_PDG.tout,Sim_PDG.Results(:,i),'b','LineWidth',1.3)
    hold on
    plot(Sim_PDG.tout,Sim_PDG.Input(:,i),'r--','LineWidth',1.3)
    grid on
    title(['Response Vs Input of Joint#' num2str(i) 'Via PD+G Controller'])
    legend('Response','Input Signal')
    xlabel('Time')
    ylabel('Response')
end

%% Control Phase: Inverse Dynamics

% Computed-Torque-Method "without" uncertanty
Sim_CTM=sim('CTM_Controller');

%% 
for i=1:5
    figure(i)
    plot(Sim_CTM.tout,Sim_CTM.Results(:,i),'b','LineWidth',1.3)
    hold on
    plot(Sim_CTM.tout,Sim_CTM.Input(:,i),'r--','LineWidth',1.3)
    grid on
    title(['Response Vs Input of Joint#' num2str(i) 'Via CTM Controller'])
    legend('Response','Input Signal')
    xlabel('Time')
    ylabel('Response')
end

%%
% Computed-Torque-Method "with" uncertanty
Sim_UCTM=sim('CTM_Uncertainty_Controller');

%% 
for i=1:5
    figure(i)
    plot(Sim_UCTM.tout,Sim_UCTM.Results(:,i),'b','LineWidth',1.3)
    hold on
    plot(Sim_UCTM.tout,Sim_UCTM.Input(:,i),'r--','LineWidth',1.3)
    grid on
    title(['Response Vs Input of Joint#' num2str(i)  'Via UCTM Controller'])
    legend('Response','Input Signal')
    xlabel('Time')
    ylabel('Response')
end
