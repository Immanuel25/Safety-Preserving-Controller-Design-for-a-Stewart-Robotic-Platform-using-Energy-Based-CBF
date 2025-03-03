%% =================== Dynamics =======================
%% Control Point
cp = [0; 0; 0];
cp_s = [0 -cp(3) cp(2); cp(3) 0 -cp(1); -cp(2) cp(1) 0];

%% M1
M1_1 = transpose(eye(3)+((lt*n1_skew^2)/s1))*mt*(eye(3)+((lt*n1_skew^2)/s1));
M1_2 = transpose(eye(3)+((lt*n2_skew^2)/s2))*mt*(eye(3)+((lt*n2_skew^2)/s2));
M1_3 = transpose(eye(3)+((lt*n3_skew^2)/s3))*mt*(eye(3)+((lt*n3_skew^2)/s3));
M1_4 = transpose(eye(3)+((lt*n4_skew^2)/s4))*mt*(eye(3)+((lt*n4_skew^2)/s4));
M1_5 = transpose(eye(3)+((lt*n5_skew^2)/s5))*mt*(eye(3)+((lt*n5_skew^2)/s5));
M1_6 = transpose(eye(3)+((lt*n6_skew^2)/s6))*mt*(eye(3)+((lt*n6_skew^2)/s6));

%% M2
M2_1 = ((It+Ib)*transpose(n1_skew)*n1_skew)/(s1^2);
M2_2 = ((It+Ib)*transpose(n2_skew)*n2_skew)/(s2^2);
M2_3 = ((It+Ib)*transpose(n3_skew)*n3_skew)/(s3^2);
M2_4 = ((It+Ib)*transpose(n4_skew)*n4_skew)/(s4^2);
M2_5 = ((It+Ib)*transpose(n5_skew)*n5_skew)/(s5^2);
M2_6 = ((It+Ib)*transpose(n6_skew)*n6_skew)/(s6^2);

%% Qt
Qt1 = (eye(3)+(lt*(n1_skew^2)/s1))*mt*g;
Qt2 = (eye(3)+(lt*(n2_skew^2)/s2))*mt*g;
Qt3 = (eye(3)+(lt*(n3_skew^2)/s3))*mt*g;
Qt4 = (eye(3)+(lt*(n4_skew^2)/s4))*mt*g;
Qt5 = (eye(3)+(lt*(n5_skew^2)/s5))*mt*g;
Qt6 = (eye(3)+(lt*(n6_skew^2)/s6))*mt*g;

%% Qb
Qb1 = ((lb*transpose(n1_skew)*n1_skew)/s1)*mb*g;
Qb2 = ((lb*transpose(n2_skew)*n2_skew)/s2)*mb*g;
Qb3 = ((lb*transpose(n3_skew)*n3_skew)/s3)*mb*g;
Qb4 = ((lb*transpose(n4_skew)*n4_skew)/s4)*mb*g;
Qb5 = ((lb*transpose(n5_skew)*n5_skew)/s5)*mb*g;
Qb6 = ((lb*transpose(n6_skew)*n6_skew)/s6)*mb*g;

%% Mass Matrix
Mp = [mp*eye(3) mp*R*transpose(cp_s)*RT;
      mp*R*cp_s*RT mp*R*cp_s*transpose(cp_s)*RT+R*Ip*RT];
Mq1 = [eye(3); R*p1_s*RT]*(M1_1+M2_1)*[eye(3) R*transpose(p1_s)*RT];
Mq2 = [eye(3); R*p2_s*RT]*(M1_2+M2_2)*[eye(3) R*transpose(p2_s)*RT];
Mq3 = [eye(3); R*p3_s*RT]*(M1_3+M2_3)*[eye(3) R*transpose(p3_s)*RT];
Mq4 = [eye(3); R*p4_s*RT]*(M1_4+M2_4)*[eye(3) R*transpose(p4_s)*RT];
Mq5 = [eye(3); R*p5_s*RT]*(M1_5+M2_5)*[eye(3) R*transpose(p5_s)*RT];
Mq6 = [eye(3); R*p6_s*RT]*(M1_6+M2_6)*[eye(3) R*transpose(p6_s)*RT];
Mq_tot = Mq1+Mq2+Mq3+Mq4+Mq5+Mq6;

%% Coriolis
Cp = [0*eye(3) 0*eye(3); 0*eye(3) W_tilde*R*Ip*RT];
Ca1 = ((mt*lt)/(s1)^2)*(n1*transpose(dqp1)*transpose(n1_skew)*n1_skew+transpose(n1)*dqp1*transpose(n1_skew)*n1_skew+transpose(n1_skew)*n1_skew*dqp1*transpose(n1)) - (mt*s1^2/s1^3)*(transpose(n1)*dqp1*transpose(n1_skew)*n1_skew+transpose(n1_skew)*n1_skew*dqp1*transpose(n1)) - (2*(It+Ib)/s1^3)*(transpose(n1_skew)*n1_skew*dqp1*transpose(n1));
Ca2 = ((mt*lt)/(s2)^2)*(n2*transpose(dqp2)*transpose(n2_skew)*n2_skew+transpose(n2)*dqp2*transpose(n2_skew)*n2_skew+transpose(n2_skew)*n2_skew*dqp2*transpose(n2)) - (mt*s2^2/s2^3)*(transpose(n2)*dqp1*transpose(n2_skew)*n2_skew+transpose(n2_skew)*n2_skew*dqp2*transpose(n2)) - (2*(It+Ib)/s2^3)*(transpose(n2_skew)*n1_skew*dqp2*transpose(n2));
Ca3 = ((mt*lt)/(s3)^2)*(n3*transpose(dqp3)*transpose(n3_skew)*n3_skew+transpose(n3)*dqp3*transpose(n3_skew)*n3_skew+transpose(n3_skew)*n3_skew*dqp3*transpose(n3)) - (mt*s3^2/s3^3)*(transpose(n3)*dqp1*transpose(n3_skew)*n3_skew+transpose(n3_skew)*n3_skew*dqp3*transpose(n3)) - (2*(It+Ib)/s3^3)*(transpose(n3_skew)*n1_skew*dqp3*transpose(n3));
Ca4 = ((mt*lt)/(s4)^2)*(n4*transpose(dqp4)*transpose(n4_skew)*n4_skew+transpose(n4)*dqp4*transpose(n4_skew)*n4_skew+transpose(n4_skew)*n4_skew*dqp4*transpose(n4)) - (mt*s4^2/s4^3)*(transpose(n4)*dqp1*transpose(n4_skew)*n4_skew+transpose(n4_skew)*n4_skew*dqp4*transpose(n4)) - (2*(It+Ib)/s4^3)*(transpose(n4_skew)*n1_skew*dqp4*transpose(n4));
Ca5 = ((mt*lt)/(s5)^2)*(n5*transpose(dqp5)*transpose(n5_skew)*n5_skew+transpose(n5)*dqp5*transpose(n5_skew)*n5_skew+transpose(n5_skew)*n5_skew*dqp5*transpose(n5)) - (mt*s5^2/s5^3)*(transpose(n5)*dqp1*transpose(n5_skew)*n5_skew+transpose(n5_skew)*n5_skew*dqp5*transpose(n5)) - (2*(It+Ib)/s5^3)*(transpose(n5_skew)*n1_skew*dqp5*transpose(n5));
Ca6 = ((mt*lt)/(s6)^2)*(n6*transpose(dqp6)*transpose(n6_skew)*n6_skew+transpose(n6)*dqp6*transpose(n6_skew)*n6_skew+transpose(n6_skew)*n6_skew*dqp6*transpose(n6)) - (mt*s6^2/s6^3)*(transpose(n6)*dqp1*transpose(n6_skew)*n6_skew+transpose(n6_skew)*n6_skew*dqp6*transpose(n6)) - (2*(It+Ib)/s6^3)*(transpose(n6_skew)*n1_skew*dqp6*transpose(n6));

Caq1 = [eye(3); R*p1_s*RT]*Ca1*[eye(3) R*p1_s*RT];
Caq2 = [eye(3); R*p2_s*RT]*Ca2*[eye(3) R*p2_s*RT];
Caq3 = [eye(3); R*p3_s*RT]*Ca3*[eye(3) R*p3_s*RT];
Caq4 = [eye(3); R*p4_s*RT]*Ca4*[eye(3) R*p4_s*RT];
Caq5 = [eye(3); R*p5_s*RT]*Ca5*[eye(3) R*p5_s*RT];
Caq6 = [eye(3); R*p6_s*RT]*Ca6*[eye(3) R*p6_s*RT];
Caq_tot = Caq1+Caq2+Caq3+Caq4+Caq5+Caq6;

C3 = [mp*eye(3); mp*R*transpose(cp_s)*RT]*(W_tilde^2)*R*cp;

CM1 = [eye(3); R*p1_s*RT]*(M1_1+M2_1)*(W_tilde^2)*(R*p1);
CM2 = [eye(3); R*p2_s*RT]*(M1_2+M2_2)*(W_tilde^2)*(R*p2);
CM3 = [eye(3); R*p3_s*RT]*(M1_3+M2_3)*(W_tilde^2)*(R*p3);
CM4 = [eye(3); R*p4_s*RT]*(M1_4+M2_4)*(W_tilde^2)*(R*p4);
CM5 = [eye(3); R*p5_s*RT]*(M1_5+M2_5)*(W_tilde^2)*(R*p5);
CM6 = [eye(3); R*p6_s*RT]*(M1_6+M2_6)*(W_tilde^2)*(R*p6);
CM_tot = CM1+CM2+CM3+CM4+CM5+CM6;

%% Gravity
G1 = ([eye(3); R*p1_s*RT]*(Qt1+Qb1));
G2 = ([eye(3); R*p2_s*RT]*(Qt2+Qb2));
G3 = ([eye(3); R*p3_s*RT]*(Qt3+Qb3));
G4 = ([eye(3); R*p4_s*RT]*(Qt4+Qb4));
G5 = ([eye(3); R*p5_s*RT]*(Qt5+Qb5));
G6 = ([eye(3); R*p6_s*RT]*(Qt6+Qb6));
G_tot = G1+G2+G3+G4+G5+G6;

%% Stewart Platform Dynamics Equation
%Mass Matrix
M = (Mp+Mq_tot);

%Coriolis Vector
C = (Cp)*dq+(Caq_tot)*dq+(C3)+(CM_tot);

%Gravity Vector
G = -[mp*g; mp*R*cp_s*RT*g] - G_tot;

%Inverse Jacobian Matrix
H = [n1 n2 n3 n4 n5 n6;
     R*p1_s*RT*n1 R*p2_s*RT*n2 R*p3_s*RT*n3 R*p4_s*RT*n4 R*p5_s*RT*n5 R*p6_s*RT*n6];