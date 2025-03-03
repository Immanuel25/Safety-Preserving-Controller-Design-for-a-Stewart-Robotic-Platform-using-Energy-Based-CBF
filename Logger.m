function status = Logger(t, Q, flag)
    global Sims F P h hD
    
    if strcmp(flag, 'init')
        Sims.time = [t(1)];
        Sims.F = [F];
        Sims.Q = [Q];
        Sims.P = [P];
        Sims.h = [h];
        Sims.hD = [hD];
        disp('Masuk')
    elseif strcmp(flag, 'done')
        assignin('base', 'logData', Sims);  % Save in workspace at the end
    else
        Sims.time(end+1) = t;
        Sims.F(:,end+1) = F;
        Sims.Q(:,end+1) = Q;
        P
        Sims.P(end+1) = P;
        Sims.h(:,end+1) = h;
        Sims.hD(:,end+1) = hD;
    end
    
    status = 0; % Required by OutputFcn
end