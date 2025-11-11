clear all
close all
clc
yalmip('clear')
addpath(genpath('flypath3d'));


%% Parametri razzo
Ixx=330.472;
Iyy=332.721;
Izz=334.931;
m=919.200;
g=3.711;


%% Matrici SYS
%   φ     φ'    θ     θ'    ψ     ψ'    x     x'    y     y'    z     z'
A=[ 0     1     0     0     0     0     0     0     0     0     0     0;
    0     0     0     0     0     0     0     0     0     0     0     0;
    0     0     0     1     0     0     0     0     0     0     0     0;
    0     0     0     0     0     0     0     0     0     0     0     0;
    0     0     0     0     0     1     0     0     0     0     0     0;
    0     0     0     0     0     0     0     0     0     0     0     0;
    0     0     0     0     0     0     0     1     0     0     0     0;
    0     0     0     0     0     0     0     0     0     0     0     0;
    0     0     0     0     0     0     0     0     0     1     0     0;
    0     0     0     0     g     0     0     0     0     0     0     0;
    0     0     0     0     0     0     0     0     0     0     0     1;
    0     0     -g    0     0     0     0     0     0     0     0     0];


B=[0     0     0     0;
   1/Ixx     0     0     0;
   0     0     0     0;
   0     1/Iyy     0     0;
   0     0     0     0;
   0     0     1/Izz     0;
   0     0     0     0;
   0     0     0     1/m;
   0     0     0     0;
   0     0     0     0;
   0     0     0     0;
   0     0     0     0];

C=eye(12);

D=0;

%% Discretizzazione

sys = ss(A,B,C,D);
Ts = 0.01;
sys_d = c2d(sys, Ts, 'zoh');

%% MPC
nx = 12;
nu = 4;
ny = 12;

N = 20;
Nc = 5;
Nu = 1;

%% Definizione dello stato iniziale e finale

x_desired = zeros(12,1); 
% x_desired = [0 0 0 0 0 0 0 0 600 0 2500 0]';

%     φ    φ'   θ         θ'   ψ        ψ'   x       x'     y      y'    z       z'
x0 = [0    0    0.8863    0    -0.49    0    1500    -75    500    40    2000    100]';


%% Dichiarazione delle variabili
u = sdpvar(repmat(nu, 1, N), repmat(1, 1, N));
x = sdpvar(repmat(nx, 1, N + 1), repmat(1, 1, N + 1));
epsS = sdpvar(1);

% Definizione della funzione costo
%   φ     φ'    θ     θ'    ψ     ψ'    x     x'    y     y'    z     z'
Q=[ 5     0     0     0     0     0     0     0     0     0     0     0;
    0     0.5   0     0     0     0     0     0     0     0     0     0;
    0     0     10    0     0     0     0     0     0     0     0     0;
    0     0     0     0.01  0     0     0     0     0     0     0     0;
    0     0     0     0     5     0     0     0     0     0     0     0;
    0     0     0     0     0     0.01  0     0     0     0     0     0;
    0     0     0     0     0     0     10    0     0     0     0     0;
    0     0     0     0     0     0     0     15    0     0     0     0;
    0     0     0     0     0     0     0     0     1     0     0     0;
    0     0     0     0     0     0     0     0     0     1     0     0;
    0     0     0     0     0     0     0     0     0     0     1     0;
    0     0     0     0     0     0     0     0     0     0     0     1];

Q=diag([5 0.5 10 0.01 5 0.01 10 15 1 1 1.8 1.8]);

% Matrice dei pesi ingressi 
R = [0.001 0 0 0;
     0 0.001 0 0;
     0 0 0.001 0;
     0 0 0 1];

R = R * 0.001;
Q = Q * 100;
P = Q * 10;

% Funzione obiettivo
offset = [0 0 0 m*g]';
constr = [];
objective=0;

% objective = objective + (x{N+1} - x_desired)' * P * (x{N+1} - x_desired);
objective = objective + 10 * epsS^2;

for k = 1:N
    objective = objective + (x{k} - x_desired)' * Q * (x{k} - x_desired) + (u{k} - offset)' * R * (u{k} - offset);
    constr = [constr, abs(x{k}(1)) <= 0.5 + epsS];
    constr = [constr, abs(x{k}(3)) <= 0.5 + epsS];
    constr = [constr, abs(x{k}(5)) <= 0.5 + epsS];
    constr = [constr, x{k}(7) >= 0];
end 

% Vincoli
for k = 1:Nc
    constr = [constr, x{k+1}==sys_d.A*x{k} + sys_d.B*(u{k} - offset)];
    constr = [constr,  abs(u{k}(1)) <= 300];
    constr = [constr,  abs(u{k}(2)) <= 300];
    constr = [constr,  abs(u{k}(3)) <= 300];
    constr = [constr,  u{k}(4) <= 15000];
end



% Definizione il controller
controller = optimizer(constr, objective,[], x{1}, [u{:}]);

%% Simulazione
x=x0;

implementedU = [];
implementedX = x;
acceleration = [];

tsim = 300;

progress = waitbar(0, 'Simulazione in corso...');
N = length(1:Ts:tsim); 

for i=1:N
    U = controller{x};    
    x = sys_d.A * x + sys_d.B * (U(:,1) - offset);
    implementedX = [implementedX, x];
    implementedU = [implementedU, U(:,1)];

    if mod(i,100) == 0 || i == N
        waitbar(i/N, progress, sprintf('Iterazione %d di %d', i, N));
    end
end
close(progress)

[acc_x] = gradient(implementedX(8,:)) ./ Ts;
[acc_y] = gradient(implementedX(10,:)) ./ Ts;
[acc_z] = gradient(implementedX(12,:)) ./ Ts;

t = 0:Ts:tsim-1;
t = [t,t(end)+Ts];
t1 = 0:Ts:tsim-1; 

%% plot input
input_fig = figure('Renderer', 'painters', 'Position', [0 70 971 726]);
    removeToolbarExplorationButtons(input_fig)
    grid on
    subplot(2,1,1)
    plot(t1,implementedU(1,:));
    hold on
    plot(t1,implementedU(2,:));
    plot(t1,implementedU(3,:));
    xlabel('Tempo (s)');
    ylabel('Torque (Nm)');
    legend('U_1','U_2','U_3');
    grid on
    subplot(2,1,2)
    plot(t1,implementedU(4,:));
    legend('U_4');
    xlabel('Tempo (s)');
    ylabel('Force (N)');
    grid on;
 sgtitle('Evoluzione ingressi di controllo');
 
figPos = get(input_fig, 'Position');        
figWidthInches = figPos(3) ;
figHeightInches = figPos(4) ;
set(input_fig, 'PaperUnits', 'points');
set(input_fig, 'PaperSize', [figWidthInches, figHeightInches]); % Imposta PaperSize uguale alla figura
set(input_fig, 'PaperPosition', [0, 0, figWidthInches, figHeightInches]); % Imposta PaperPosition per adattarsi
saveas(input_fig,"plots/input.pdf")


%% plot stati
stastes_fig = figure('Renderer', 'painters', 'Position', [0 70 971 726]);
    removeToolbarExplorationButtons(stastes_fig)
    subplot(2,2,1)
    plot(t, implementedX([7 9 11],:));
    xlabel('Tempo (s)');
    ylabel('Posizione (m)');
    grid on;
    legend('x', 'y', 'z');
    subplot(2,2,2)
    plot(t, implementedX([8 10 12],:));
    xlabel('Tempo (s)');
    ylabel('Velocità (m/s)');
    grid on;
    legend('x', 'y', 'z');
    subplot(2,2,3)
    plot(t, implementedX([1 3 5],:));
    xlabel('Tempo (s)');
    ylabel('Posizione angolare (rad)');
    grid on;
    legend('φ', 'θ', 'ψ');
    subplot(2,2,4)
    plot(t, implementedX([2 4 6],:));
    xlabel('Tempo (s)');
    ylabel('Velocità angolare (rad/s)');
    grid on;
    legend('φ', 'θ', 'ψ');
sgtitle('Evoluzione degli stati del sistema');

figPos = get(stastes_fig, 'Position');        
figWidthInches = figPos(3) ;
figHeightInches = figPos(4) ;
set(stastes_fig, 'PaperUnits', 'points');
set(stastes_fig, 'PaperSize', [figWidthInches, figHeightInches]); % Imposta PaperSize uguale alla figura
set(stastes_fig, 'PaperPosition', [0, 0, figWidthInches, figHeightInches]); % Imposta PaperPosition per adattarsi
saveas(stastes_fig,"plots/states.pdf")


%% plot vel
vel_fig = figure('Renderer', 'painters', 'Position', [0 70 971 726]);
    removeToolbarExplorationButtons(vel_fig)
    subplot(2,3,1)
        plot(t,implementedX([8],:));
            title('Vel_X (m/s)');
            xlabel('Tempo (s)');
            ylabel('Velocità (m/s)');
            grid on
    subplot(2,3,2)
        plot(t,implementedX([10],:));
            title('Vel_Y (m/s)');
            xlabel('Tempo (s)');
            ylabel('Velocità (m/s)');
            grid on
    subplot(2,3,3)
        plot(t,implementedX([12],:));
            title('Vel_Z (m/s)');
            xlabel('Tempo (s)');
            ylabel('Velocità (m/s)');
            grid on
    subplot(2,3,4)
        plot(t,acc_x);
            title('Acc_X (m/s^2)');
            xlabel('Tempo (s)');
            ylabel('Accelerazione (m/s^2)');
            grid on
    subplot(2,3,5)
        plot(t,acc_y);
            title('Acc_Y (m/s^2)');
            xlabel('Tempo (s)');
            ylabel('Accelerazione (m/s^2)');
            grid on
    subplot(2,3,6)
        plot(t,acc_z);
            title('Acc_Z (m/s^2)');
            xlabel('Tempo (s)');
            ylabel('Accelerazione (m/s^2)');
            grid on          
sgtitle('Evoluzione accelerazioni e velocità');

figPos = get(vel_fig, 'Position');        
figWidthInches = figPos(3) ;
figHeightInches = figPos(4) ;
set(vel_fig, 'PaperUnits', 'points');
set(vel_fig, 'PaperSize', [figWidthInches, figHeightInches]); % Imposta PaperSize uguale alla figura
set(vel_fig, 'PaperPosition', [0, 0, figWidthInches, figHeightInches]); % Imposta PaperPosition per adattarsi
saveas(vel_fig,"plots/vel_acc.pdf")

pause(2)
close all
pause(2)

%% GRAFICO 3D ANIMATO

x_graph = implementedX(11,:);   
y_graph = implementedX(9,:);    
z_graph = implementedX(7,:);   
x_lim = [min(x_graph)-200 max(x_graph)+200];
y_lim = [min(y_graph)-200 max(y_graph)+200];
z_lim = [min(z_graph)-200 max(z_graph)+200];

pitch_graph = implementedX(3,:)+pi/2;
yaw_graph = implementedX(5,:);
roll_graph = implementedX(1,:);

trajectory = [x_graph; y_graph; z_graph;pitch_graph;yaw_graph;roll_graph]';
new_object('tbm.mat',trajectory,'model','scud.mat','scale',30,'path','on','pathcolor',[.89 .0 .27],'face',[.30 0 0]);

flypath('tbm.mat','animate','on','step',40,'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
'font','Georgia','fontsize',6,'view',[45 8],'window',[1920 900],'xlim',x_lim,'ylim',y_lim,'zlim',z_lim,...
'output','output2.gif','dpi',600);









%% RICCATI



%% GRAFICO 3D VECCHIO

% figure(3)
% axis([0 3500 0 1500 0 1600]);
% curve = animatedline ('LineWidth', 2);
% %set (gca, 'XLim', [0 3500], 'YLim', [0 1500], 'ZLim', [0 1600]);
% set (gca,'GridLineStyle','--');
% xlabel('z','FontSize',16);
% ylabel('y','FontSize',16);
% zlabel('x','FontSize',16);
% view (45,8);
% hold on;
% grid on;
% for i = 1:1:length (x_graph)
%     addpoints (curve, z_graph(i), y_graph(i), x_graph(i)) ;
%     head = scatter3 (z_graph(i), y_graph(i), x_graph(i), 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor','b');
%     drawnow
%     delete (head);
% end



% % SIMULAZIONE tutto sbagliato
% % Numero di passi di simulazione
% num_steps = 500;
% 
% % Inizializza lo stato del sistema con lo stato iniziale
% current_state = x0;
% 
% % Vettori per salvare l'evoluzione dello stato e del controllo
% states_evolution = zeros(nx, num_steps + 1);
% control_evolution = zeros(nu, num_steps);
% 
% % Istanza unica del problema di ottimizzazione
% ctrl = optimizer(constr, cost, sdpsettings('solver', 'quadprog'), x{1}, u{1});
% Loop di simulazione
% for k = 1:num_steps
%     % Risolvi il problema di ottimizzazione per ottenere il controllo ottimale
%     optimal_U = ctrl{current_state}; % Ottieni il controllo ottimale con lo stato attuale
%     
%     
%     % Applica il primo valore di controllo ottimale
%     current_control = optimal_U
%     
%     % Simula l'evoluzione del sistema per un passo di campionamento
%     next_state = sys_d.A * (current_state) + sys_d.B * (current_control);
%     
%     % Salva l'evoluzione dello stato e del controllo
%     states_evolution(:, k) = current_state;
%     control_evolution(:, k) = current_control;
%     
%     % Aggiorna lo stato attuale
%     current_state = next_state;
%     
%      % Aggiorna le variabili di ottimizzazione U e X per il prossimo passo
%     u = {u{2:end}, sdpvar(nu, 1)};
%     x = {x{2:end}, sdpvar(nx, 1)};
% 
% %     % Copia temporanea delle variabili di ottimizzazione
% %     U_temp = u;
% %     X_temp = x;
% %     
% %     % Aggiorna l'orizzonte di predizione delle variabili di ottimizzazione U e X
% %     for i = 1:N-1
% %         u{i} = U_temp{i+1};
% %         x{i} = X_temp{i+1};
% %     end
% %     u{N} = sdpvar(nu, 1);
% %     x{N} = sdpvar(nx, 1);
% %     x{N + 1} = sdpvar(nx, 1);
% %     k
% end
% % Salva anche lo stato finale
% states_evolution(:, num_steps + 1) = current_state;
% 
% 
% % Calcola il tempo della simulazione
% time = 0:1:num_steps;
% time2 = 0:1:(num_steps-1);
% 
% 
% % Plot dell'evoluzione dello stato del sistema
% figure;
% plot(time, states_evolution([7 9 11],:)');
% legend('x','y','z');
% xlabel('Tempo (s)');
% ylabel('Stato del sistema');
% title('Evoluzione dello stato del sistema');
% 
% % Plot dell'evoluzione del controllo applicato
% figure;
% plot(time2, control_evolution');
% legend('Controllo 1', 'Controllo 2', 'Controllo 3', 'Controllo 4');
% xlabel('Tempo (s)');
% ylabel('Controllo applicato');
% title('Evoluzione del controllo applicato');
% 
% figure(1)
% plot(1:1:N, y([7 9 11],:));
% xlabel('Tempo (s)');
% ylabel('Posizione (m)');
% legend('x', 'y', 'z');
% 
% figure(2)
% plot(1:1:N, value(u_optimal));

