function dxdt = StewartPlatform(t,Q,Gains,Sims)
    global F 
    
    %   Initialization
    q = Q(1:6);
    dq = Q(7:12);

    x=q(1);
    y=q(2);
    z=q(3);
    phi=q(4);
    theta=q(5);
    psi=q(6);

    dx=dq(1);
    dy=dq(2);
    dz=dq(3);
    dphi=dq(4);
    dtheta=dq(5);
    dpsi=dq(6);

    q1 = [x;y;z];
    q2 = [phi;theta;psi];
    dq1 = [dx;dy;dz];
    dq2 = [dphi;dtheta;dpsi];
    
    Kinematics();
    Dynamics();
    
    %% Compute Nominal Control Input
    u_des = -Gains.K*([q; dq]-Gains.Q_des);

    %% Control Input Mapping
    F = H\(M*u_des + C + G);
    
    [F] = SafetyEnergyClosedMax(t,Q,F,M,G,H,Gains);
    %   Di dalem ada if, kalo Sims.CBF == 'false' maka gaberubah
    
    tau = H*F;

    %% Dynamics Equation
    ddq = M\((tau)-C-G);

    dxdt = [dq; ddq];
    
end