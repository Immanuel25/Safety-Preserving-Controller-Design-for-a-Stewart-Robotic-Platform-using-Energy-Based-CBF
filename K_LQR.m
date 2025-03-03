%% Linearized Model
A = [zeros(6) eye(6);
     zeros(6) zeros(6)];
B = [zeros(6); eye(6)];
C = [eye(6) zeros(6)];
D = [zeros(6)];

sys = ss(A,B,C,D); % State-space model

Co = rank(ctrb(sys)); % Check controllability
Ob = rank(obsv(sys)); % Check observability

[Gains.K,P,E] = lqr(sys,Gains.Q,Gains.R); % Determine the LQR Gains