function [ Ad, Bd, M, V, qNext ] = twoLinkRobot(Kd, Kp, q, qD, u, dt)

l1 = 1; l2 = 1; m1 = 1; m2 = 1;

m11 = (l2^2)*m2 + 2*l1*l2*m2*cos(q(2)) + (l1^2)*(m1+m2);
m12 = l2*(m2^2) + l1*l2*m2*cos(q(2));
m21 = m12;
m22 = (l2^2)*m2;

M = [m11 m12; m21 m22];

c11 = -2*m2*l1*l2*sin(q(2))*qD(2);
c12 = -m2*l1*l2*sin(q(2))*qD(2);
c21 = m2*l1*l2*sin(q(2))*qD(1);
c22 = 0;

C = [c11 c12; c21 c22];

V = C*qD;

% g = -9.8;
% g1 = m2*l2*g*cos(th1 + th2) + (m1 + m2)*l1*g*cos(th1);
% g2 = m2*l2*g*cos(th1 + th2);
% 
% G = [g1; g2];

A = [M\(-Kd - C) M\(-Kp); eye(2) zeros(2,2)];
B = [M\Kp; zeros(2,2)];

Ad = expm(A*dt);
Bd = (Ad - eye(4))*(A\B);

qNext = Ad*[qD; q] + Bd*u;

end

