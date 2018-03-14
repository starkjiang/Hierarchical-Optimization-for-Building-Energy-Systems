% Hierarchical optimization for HVAC system
% Outer loop is the water side HVAC - cooling energy system
% Inner loop is the air side HVAC - AHU - VAV network
% clear
% close all
% clc
%%
tic
% outer iteration number
n = 5000;
% inner iteration number
N = 100;
% chilled water temperature
CWT = [];
CWT_init = 45;
CWT(1:2) = CWT_init;
CWT_con = 42 + (44 - 42)*rand(n,1);
% Supply air temperature
SAT = [];
SAT_init = 60;
SAT(1:2) = SAT_init;
% MAT constant
beta = 0.3;
% mix air temperature 
MAT = [];
MAT_init = 80;
MAT(1:2) = MAT_init;
% Zone Temerature
T = [];
T_init = 63;
T(1:2) = T_init;
% Step size for CWT and SAT_AC
a = 0.085;
b = 0.085;
% Coefficients for Energy function
c = 0.005;
d = 0.155;
OAT(1:2) = 75+2*rand(1); % Outside air temperature
Fw = 0.4; % Flow rate of Water
Fa = 0.35; % Flow rate of Air
SAT_AC(1:2) = 60;
Reheat = [];
Cooling = [];
Energy = [];
SAT_ave = [];
sat_sp = [];
grad_2 = [];
grad_1 = [];
eta = 0.7;
% acceleration constant
% Outer Loop
for i = 2:n
    OAT(i) = 75 + 2*rand(1);
    grad_1(i) = 2*c*(CWT(i)-43) - 200000*exp(-CWT(i)) - 0.0001*rand(1);
    CWT(i+1) = CWT(i)-a*mean(grad_1) + eta*(CWT(i) - CWT(i-1)); % Update for CWT
    MAT(i+1) = beta*OAT(i)+(1-beta)*T(i);
    if mod(i,10) == 0 % Communication period
        % Inner loop
        for ii = 2:N
%             mean_SAT = mean(SAT);
            grad_2(ii) = 2*d*(SAT(ii) - MAT(i)) - 2000*exp(-SAT(ii)) - 0.0001*rand(1);
            SAT(ii+1) = SAT(ii)-b*mean(grad_2) + eta*(SAT(ii) - SAT(ii-1)); % Update for SAT
%             SAT(ii+1) = mean_SAT-b*(2*d*(mean_SAT - MAT(i)) - 2000*exp(-mean_SAT) - 0.0001*rand(1)); % Update for SAT
%             Reheat(ii+1) = b*(SAT(ii+1) - 65)^2;
        end
    end
    SAT_sp = SAT(end);
%     SAT_sp = mean(SAT);
    sat_sp = [sat_sp;SAT_sp];
    % Activating cooling dynamics
    if MAT(i+1)>SAT_sp;
        tai = MAT(i+1); % Inlet air temperature
        twi = CWT(i+1); % Inlet water temperature
        options = simset('SrcWorkspace','current'); % Using Simulink
        sim('test_hie_cool',[],options)
        SAT_AC(i+1) = x.Data(end);
    elseif MAT(i+1)<SAT_sp;
        tai = MAT(i+1);
        twi = 100;
        options = simset('SrcWorkspace','current'); % Using Simulink
        sim('test_hie_hot',[],options)
        SAT_AC(i+1) = x.Data(end);
    else
        SAT_AC(i+1) = MAT(i+1);
    end
        T(i+1) = 0.987*T(i) + 0.05*(SAT_AC(i+1) + 20 - T(i));  % Simple thermal dynamics
        Reheat(i+1) = c*(SAT_AC(i+1) - MAT(i+1))^2 + 2000 * exp(-SAT_AC(i+1)); % Reheat energy
        Cooling(i+1) = d*(CWT(i+1) - 43)^2 + 200000 * exp(-CWT(i+1)); % Cooling energy
        Energy(i+1) = Reheat(i+1) + Cooling(i+1);
        SAT(1:2) = SAT_AC(i+1); % Setting the initialization of supply air temperature as the new actual supply air temperature
end
window_size = 100;
E2_R_T = tsmovavg(Energy(3:end)','s',window_size,1);
% save('E2_R_T.mat','E2_R_T');
% % Plotting
figure
plot(CWT(2:end),'r-','LineWidth',2)
xlabel('Time')
ylabel('Chilled Water Temperature (F)')
% 
figure
plot(SAT_AC(2:end),'k-','LineWidth',2)
hold on
plot(MAT(2:end),'b-','LineWidth',2)
plot(sat_sp(2:end),'g--','LineWidth',2)
legend('Supply Air Temperature','Mixed Air Temperature','Supply Air Temperature Set Point')
xlabel('Time')
ylabel('Temperature (F)')
% 
figure
plot(T,'m-','LineWidth',2)
xlabel('Time')
ylabel('Zone Temperature (F)')
% 
%figure
plot(E2_R_T(3:end),'k-','LineWidth',2)
% xlabel('Time')
% ylabel('Energy Consumption')
% plot(CWT(2:end),'g-','LineWidth',2)
%hold on
%plot(sat_sp(2:end),'b-','LineWidth',2)
toc