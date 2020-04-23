%% Do the control calculations

clear; clc; close all;

% Can change the below parameters, or the goal configurations, as
% appropriate
% Note that the goal configurations have to be at least 2x(T+N)

Kd = [2*sqrt(10) 0; 0 2*sqrt(10)];
Kp = 10*eye(2);

dt = 0.01;
N = 20;         % the length of the horizon for MPC
t = 0:dt:5;
T = size(t, 2);

qGoal1 = [pi/4; pi/8];
qGoal2 = [0;0];
%qGoal = repmat(qGoal1, 1, T+N);
qGoal = [repmat(qGoal1, 1, floor(T/4)) repmat(qGoal2, 1, floor(T/4))...
         repmat(qGoal1, 1, floor(T/4)) repmat(qGoal2, 1, floor(T/4))...
         repmat(qGoal1, 1, T) repmat(qGoal2, 1, T)];

% 1) MPC controller
[ qMPC, qDMPC ] = mpc(qGoal, Kd, Kp, dt, N, T);

% 2) Computed Torque controller
[ qCTC, qDCTC, tau ] = ctc(qGoal, Kd, Kp, dt, T);
    
saveDir = './results/'; mkdir(saveDir);
filename = strcat('mpc_ctc_N_', string(N), '_Time_', datestr(now, 'HH-MM-SS'));
save(strcat(saveDir, filename));

%% Plot graphs

close all;

figure;
plot(t, qCTC(1,:),'b','LineWidth',2);
hold on;
plot(t, qMPC(1,:),'g','LineWidth',2);
hold on;
plot(t, qGoal(1,1:T),'r','LineWidth',2);
xlabel('Time (s)'); ylabel('\Theta (rad)'); title('Link 1');
ylim([0,1]);
legend('\theta_{1} CTC','\theta_{1} MPC','\theta_{1} Goal','Location','best');
hold off;

figure;
plot(t, qCTC(2,:),'b','LineWidth',2);
hold on;
plot(t, qMPC(2,:),'g','LineWidth',2);
hold on;
plot(t, qGoal(2,1:T),'r','LineWidth',2);
xlabel('Time (s)'); ylabel('\Theta (rad)'); title('Link 2');
ylim([0,1]);
legend('\theta_{2} CTC','\theta_{2} MPC','\theta_{2} Goal','Location','best');
hold off;

%% Visualize the manipulator trajectories

% This requires installation of Peter Corke's Robotics Toolbox

close all;
mdl_planar2;
clone = SerialLink(p2,'name','clone');

figure;
set(gcf, 'Position',get(0, 'Screensize'));

for i = 1:size(qCTC,2)
    p2.plot(qCTC(:,2*i)','noname','nowrist','linkcolor','g','fps',100000);
    hold on;
    clone.plot(qMPC(:,2*i)','noname','nowrist','linkcolor','b','fps',100000);
    % custom legend
    h = zeros(2, 1);
    h(1) = plot(NaN,NaN,'g','LineWidth',5);
    h(2) = plot(NaN,NaN,'b','LineWidth',5);
    legend(h, 'CTC','MPC');
    title('Controller Performance with Alternating Goal Angles');
end
