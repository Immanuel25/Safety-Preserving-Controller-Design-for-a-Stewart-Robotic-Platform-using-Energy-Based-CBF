%% Stewart Platform Parameters
rb = 0.2; % base radius (m)
rp = 0.16; % platform radius (m)
sigma = deg2rad(20);

lt = 0.1; % Distance from platform joint to the c.o.g. of the linear actuator (m)
lb = 0.13861; %Distance from platform joint to the c.o.g. of the linear actuator (m)

mp = 0.528; % platform mass (kg), original = 0.528
mt = 0.027; % upper actuator mass (kg), original = 0.027
mb = 0.118; % lower actuator mass (kg), original = 0.118

Ip = [0.03 0.01 0.01; 0.01 0.03 0.01; 0.01 0.01 0.02];
It = [(1/3)*mt*lt^2 0 0; 0 (1/3)*mt*lt^2 0; 0 0 0]; %(1/3)*mt*lt^2
Ib = [(1/3)*mb*lb^2 0 0; 0 (1/3)*mb*lb^2 0; 0 0 0]; %(1/3)*mb*lb^2

g = [ 0;
      0;
    -9.8]; %gravitational acceleration (m/s^2)

%% =================== Kinematics =======================
%% Frame {B}
beta1 = deg2rad(120);
b1_angle = beta1 + sigma;
b6_angle = beta1 - sigma;

beta2 = deg2rad(-120);
b2_angle = beta2 - sigma;
b3_angle = beta2 + sigma;

beta3 = deg2rad(0);
b4_angle = beta3 - sigma;
b5_angle = beta3 + sigma;

b1 = [-rb*sin(b1_angle); rb*cos(b1_angle); 0];
b2 = [-rb*sin(b2_angle); rb*cos(b2_angle); 0];
b3 = [-rb*sin(b3_angle); rb*cos(b3_angle); 0];
b4 = [-rb*sin(b4_angle); rb*cos(b4_angle); 0];
b5 = [-rb*sin(b5_angle); rb*cos(b5_angle); 0];
b6 = [-rb*sin(b6_angle); rb*cos(b6_angle); 0];

%plot bi
xb = [b1(1) b2(1) b3(1) b4(1) b5(1) b6(1) b1(1)];
yb = [b1(2) b2(2) b3(2) b4(2) b5(2) b6(2) b1(2)];
zb = [b1(3) b2(3) b3(3) b4(3) b5(3) b6(3) b1(3)];

%% Frame {P}
gamma1 = deg2rad(180);
p1_angle = gamma1 - sigma;
p2_angle = gamma1 + sigma;

gamma2 = deg2rad(-60);
p3_angle = gamma2 - sigma;
p4_angle = gamma2 + sigma;

gamma3 = deg2rad(60);
p5_angle = gamma3 - sigma;
p6_angle = gamma3 + sigma;

p1 = [-rp*sin(p1_angle); rp*cos(p1_angle); 0];
p2 = [-rp*sin(p2_angle); rp*cos(p2_angle); 0];
p3 = [-rp*sin(p3_angle); rp*cos(p3_angle); 0];
p4 = [-rp*sin(p4_angle); rp*cos(p4_angle); 0];
p5 = [-rp*sin(p5_angle); rp*cos(p5_angle); 0];
p6 = [-rp*sin(p6_angle); rp*cos(p6_angle); 0];

%Skew Symmetric Matrix of P --> For Dynamics
p1_s = [0 -p1(3) p1(2); p1(3) 0 -p1(1); -p1(2) p1(1) 0];
p2_s = [0 -p2(3) p2(2); p2(3) 0 -p2(1); -p2(2) p2(1) 0];
p3_s = [0 -p3(3) p3(2); p3(3) 0 -p3(1); -p3(2) p3(1) 0];
p4_s = [0 -p4(3) p4(2); p4(3) 0 -p4(1); -p4(2) p4(1) 0];
p5_s = [0 -p5(3) p5(2); p5(3) 0 -p5(1); -p5(2) p5(1) 0];
p6_s = [0 -p6(3) p6(2); p6(3) 0 -p6(1); -p6(2) p6(1) 0];

%% Platform Rotation Matrix
Rz = [cos(q2(3)) -sin(q2(3)) 0; sin(q2(3)) cos(q2(3)) 0; 0 0 1]; %yaw
Ry = [cos(q2(2)) 0 sin(q2(2)); 0 1 0; -sin(q2(2)) 0 cos(q2(2))]; %pitch
Rx = [1 0 0; 0 cos(q2(1)) -sin(q2(1)); 0 sin(q2(1)) cos(q2(1))]; %roll

R = Rz*Ry*Rx; %3D Rotation Matrix
RT = R';

%% Angular Velocity
Wp = [1    0        -sin(q2(2));
     0 cos(q2(1))  cos(q2(2))*sin(q2(1));
     0 -sin(q2(1)) cos(q2(2))*cos(q2(1))]*dq2;

Wp_tilde = [ 0    -Wp(3)  Wp(2);
            Wp(3)  0     -Wp(1);
           -Wp(2)  Wp(1)    0];

W_tilde = R*Wp_tilde*RT;

%% li vector value
l1 = q1 + (R*p1) - b1; %vektor kaki ke-1
l2 = q1 + (R*p2) - b2; %vektor kaki ke-2
l3 = q1 + (R*p3) - b3; %vektor kaki ke-3
l4 = q1 + (R*p4) - b4; %vektor kaki ke-4
l5 = q1 + (R*p5) - b5; %vektor kaki ke-5
l6 = q1 + (R*p6) - b6; %vektor kaki ke-6

%% Leg length (Si)
s1 = norm(l1); %panjang kaki ke-1
s2 = norm(l2); %panjang kaki ke-2
s3 = norm(l3); %panjang kaki ke-3
s4 = norm(l4); %panjang kaki ke-4
s5 = norm(l5); %panjang kaki ke-5
s6 = norm(l6); %panjang kaki ke-6

%% Unit Vector
n1 = l1/s1;
n2 = l2/s2;
n3 = l3/s3;
n4 = l4/s4;
n5 = l5/s5;
n6 = l6/s6;

%% Upper Gimbal Point
qp1 = q1 + (R*p1);
qp2 = q1 + (R*p2);
qp3 = q1 + (R*p3);
qp4 = q1 + (R*p4);
qp5 = q1 + (R*p5);
qp6 = q1 + (R*p6);

%% Upper gimbal point velocity
dqp1 = [eye(3) R*transpose(p1_s)*RT]*dq;
dqp2 = [eye(3) R*transpose(p2_s)*RT]*dq;
dqp3 = [eye(3) R*transpose(p3_s)*RT]*dq;
dqp4 = [eye(3) R*transpose(p4_s)*RT]*dq;
dqp5 = [eye(3) R*transpose(p5_s)*RT]*dq;
dqp6 = [eye(3) R*transpose(p6_s)*RT]*dq;

%% Skew Symmetric Matrix of n
n1_skew = [0 -n1(3) n1(2); n1(3) 0 -n1(1); -n1(2) n1(1) 0];
n2_skew = [0 -n2(3) n2(2); n2(3) 0 -n2(1); -n2(2) n2(1) 0];
n3_skew = [0 -n3(3) n3(2); n3(3) 0 -n3(1); -n3(2) n3(1) 0];
n4_skew = [0 -n4(3) n4(2); n4(3) 0 -n4(1); -n4(2) n4(1) 0];
n5_skew = [0 -n5(3) n5(2); n5(3) 0 -n5(1); -n5(2) n5(1) 0];
n6_skew = [0 -n6(3) n6(2); n6(3) 0 -n6(1); -n6(2) n6(1) 0];

%% Legs Velocity Vector
dl1 = transpose(n1)*dqp1;
dl2 = transpose(n2)*dqp1;
dl3 = transpose(n3)*dqp1;
dl4 = transpose(n4)*dqp1;
dl5 = transpose(n5)*dqp1;
dl6 = transpose(n6)*dqp1;

ds1 = norm(dl1);
ds2 = norm(dl2);
ds3 = norm(dl3);
ds4= norm(dl4);
ds5 = norm(dl5);
ds6 = norm(dl6);

%% New platform coordinates
x1 = [b1(1) b1(1)+l1(1)];
x2 = [b2(1) b2(1)+l2(1)];
x3 = [b3(1) b3(1)+l3(1)];
x4 = [b4(1) b4(1)+l4(1)];
x5 = [b5(1) b5(1)+l5(1)];
x6 = [b6(1) b6(1)+l6(1)];

y1 = [b1(2) b1(2)+l1(2)];
y2 = [b2(2) b2(2)+l2(2)];
y3 = [b3(2) b3(2)+l3(2)];
y4 = [b4(2) b4(2)+l4(2)];
y5 = [b5(2) b5(2)+l5(2)];
y6 = [b6(2) b6(2)+l6(2)];

z1 = [b1(3) b1(3)+l1(3)];
z2 = [b2(3) b2(3)+l2(3)];
z3 = [b3(3) b3(3)+l3(3)];
z4 = [b4(3) b4(3)+l4(3)];
z5 = [b5(3) b5(3)+l5(3)];
z6 = [b6(3) b6(3)+l6(3)];

x_new = [b1(1)+l1(1) b2(1)+l2(1) b3(1)+l3(1) b4(1)+l4(1) b5(1)+l5(1) b6(1)+l6(1) b1(1)+l1(1)];
y_new = [b1(2)+l1(2) b2(2)+l2(2) b3(2)+l3(2) b4(2)+l4(2) b5(2)+l5(2) b6(2)+l6(2) b1(2)+l1(2)];
z_new = [b1(3)+l1(3) b2(3)+l2(3) b3(3)+l3(3) b4(3)+l4(3) b5(3)+l5(3) b6(3)+l6(3) b1(3)+l1(3)];
