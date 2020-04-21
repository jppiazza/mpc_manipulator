%% Do the control calculations

clear; clc; close all;

% Can change the below parameters, or the goal configurations, as
% appropriate
% Note that the goal configurations have to be at least 2x(T+N)

Kd = [2*sqrt(10) 0; 0 2*sqrt(10)];
Kp = 10*eye(2);

dt = 0.01;
N = 10;
t = 0:dt:5;
T = size(t, 2);

qGoal1 = [pi/4; pi/8];
%qGoal2 = [0;0];
qGoal = repmat(qGoal1, 1, T+N);
%qGoal = [repmat(qGoal1, 1, floor(T/4)) repmat(qGoal2, 1, floor(T/4))...
%         repmat(qGoal1, 1, floor(T/4)) repmat(qGoal2, 1, floor(T/4))...
%         repmat(qGoal1, 1, T) repmat(qGoal2, 1, T)];

% 1) MPC controller
[ qMPC, qDMPC ] = mpc(qGoal, Kd, Kp, dt, N, T);

% 2) Computed Torque controller
[ qCTC, qDCTC, tau ] = ctc(qGoal, Kd, Kp, dt, T);
    
filename = strcat('mpc_ctc_N_', string(N), '_Time_', datestr(now, 'HH-MM-SS'));
save(strcat('results/',filename));

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
for i = 1:size(qCTC,2)
    p2.plot(qCTC(:,2*i)','noname','nowrist','linkcolor','b','fps',10000,'base');
    hold on;
    clone.plot(qMPC(:,2*i)','noname','nowrist','linkcolor','r','fps',10000,'base');
end

