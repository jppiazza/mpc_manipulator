function [ q, qD ] = mpc(qGoal, Kd, Kp, dt, N, T)

dimState = 4;
dimAction = 2;

q = NaN(2, T);
q(:,1) = [0;0];
qD = NaN(2, T);
qD(:,1) = [0;0];

initGuess = ones(N*(dimState+dimAction), 1);

options = optimoptions(@fmincon,'MaxFunctionEvaluations',100000,'MaxIterations',100000);
alpha = 50;
beta = 1;

for j = 1:T-1
    
    beq = zeros(dimState*N, 1);
    Atemp = zeros(dimState*N, dimState*N);
    Btemp = zeros(dimState*N, dimAction*N);
    
    for i = 1:N
        if i == 1
            [ Ad, Bd, ~, ~, ~ ] = twoLinkRobot(Kd, Kp, q(:,j), qD(:,j), [0;0], dt);
            beq(1:4,1) = Ad*[qD(:,j); q(:,j)];
        else
            [ Ad, Bd, ~, ~, ~ ] = twoLinkRobot(Kd, Kp, q(:,j), qD(:,j), [0;0], dt);
            Atemp(4*i-3:4*i, 4*i-7:4*i-4) = -Ad; 
        end                         
        Atemp(4*i-3:4*i, 4*i-3:4*i) = eye(dimState);
        Btemp(4*i-3:4*i, 2*i-1:2*i) = -Bd;
    
    end
  
    Aeq =  [Btemp Atemp];
    
    if j == 1
        qPrev = initGuess;
    end
    qNew = fmincon(@(qOpt) alpha*norm(qGoal(1,j:j+N-1)' - qOpt((dimAction*N+3):4:end))^2 + alpha*norm(qGoal(2,j:j+N-1)' - qOpt((dimAction*N+4):4:end))^2 ...
            + beta*norm(qOpt((dimAction*N+1):4:end))^2 + beta*norm(qOpt((dimAction*N+2):4:end))^2,...
            qPrev,[],[],Aeq,beq,[],[],[],options);
    
    qPrev = qNew;    
    u = qNew(1:2);
    [ ~, ~, ~, ~, qNext ] = twoLinkRobot(Kd, Kp, q(:,j), qD(:,j), u, dt);
    q(:,j+1) = qNext(3:4);
    qD(:,j+1) = qNext(1:2);

end

end



