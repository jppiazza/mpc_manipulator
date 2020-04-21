function [ q, qD, tau] = ctc(qGoal, Kd, Kp, dt, T)

qGoal = qGoal(:,1:T);

qGoalD = zeros(2, T);
qGoalDD = zeros(2, T);

q = NaN(2, T);
q(:,1) = [0;0];
qD = NaN(2, T);
qD(:,1) = [0;0];

qDD = NaN(2, T);

tau_p = NaN(2, T);
tau = NaN(2, T);

for i = 1:T
    [ ~, ~, M, V, ~] = twoLinkRobot(Kd, Kp, q(:,i), qD(:,i), [0;0], dt);
    tau_p(:,i) = qGoalDD(:,i) + Kd*(qGoalD(:,i) - qD(:,i)) + Kp*(qGoal(:,i) - q(:,i));
    tau(:,i) = M*tau_p(:,i) + V;
    qDD(:,i) = M\(tau(:,i) - V); 
    if i < T
        qD(:,i+1) = dt*qDD(:,i) + qD(:,i);
        q(:,i+1) = dt*qD(:,i) + q(:,i);
    end
end
    
end



