function [F_cbf] = SafetyEnergyClosedMax(t,Q,F,M,G,H,Gains)
    global hD P h Sims

    %% Extract the states
    q = Q(1:6);
    dq = Q(7:12);

    x=q(1);
    y=q(2);
    z=q(3);
    phi=q(4);
    theta=q(5);
    psi=q(6);

    alpha = Gains.alpha;
    alpha_e = Gains.alpha_e;
    x_max = Gains.q_max(1);
    y_max = Gains.q_max(2);
    z_max = Gains.q_max(3);
    phi_max = Gains.q_max(4);
    theta_max = Gains.q_max(5);
    psi_max = Gains.q_max(6);

    %% CBF
    h = [x_max - x;
         y_max - y;
         z_max - z;
         phi_max - phi;
         theta_max - theta;
         psi_max - psi];

    %% Partial Derivative of h
    Jh = -1*eye(6); %dh/dq --> Partial derivatives
    Jh_z = Jh(3,:);
    
    %% Energy-Based CBF
     hD =  -0.5*dq'*M*dq + alpha_e*h

    %% Closed-form solution

    ae = alpha_e*ones(1,6);
    a = alpha*ones(1,6);

%     P = dq'*((ae*Jh)' + G - H*F) + alpha*hD(3);
    P = dq'*(alpha_e*Jh(3,:)' + G - H*F) + alpha*hD(3)
%     alpha_e*Jh'
%     G - H*F
%     dq'*(alpha_e*Jh' + G - H*F)
%     P2 = dq'*(alpha_e*Jh' + G - H*F) + alpha*hD

    % P = dq'*((ae*Jh)' + G - H*F_lqr) + a*hD;
    % P = dq'*(ae*Jh' + G - H*F_lqr) + alpha*hD(3);

    b = H'*dq/(norm(H'*dq)^2);

    if P >= 0 || strcmp(Sims.CBF, 'false')
        F_cbf = F;
    else
        F_cbf = F + b*P;
    end
    % disp(F_cbf)
end