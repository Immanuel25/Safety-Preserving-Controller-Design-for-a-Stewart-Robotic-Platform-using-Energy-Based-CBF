%% Plotting Robot
clf; close all;

v = VideoWriter(['Videos/LQR']);
v.FrameRate=5;
open(v);
fig=figure(20);
set(fig, 'Units', 'normalized', 'Position', [0,0,1,1]);

tslide = 5;
N = length(Sims.time);
idx = 1;

for t = Sims.tspan(1):0.1:Sims.tspan(2)
    idx = find(Sims.time <= t, 1, 'last');
    
    clf;
    figure(fig);
    
    subplot(4, 3, [1,2,4,5,7,8]); % Row 1 spans two columns
    
    q1 = Sims.Q(1:3,idx);
    q2 = Sims.Q(4:6,idx);
    q = [q1;q2];
    dq1 = Sims.Q(7:9,idx);
    dq2 = Sims.Q(10:12,idx);
    dq = [dq1;dq2];
    Kinematics();

    plot3(xb, yb, zb, '-o'); %plot base
    hold on;
    fill3(xb, yb, zb, 'c'); %memberi fill warna platform {B}

    %plot new coordinate
    plot3(x1,y1,z1, '-or', 'LineWidth', 3); %plot kaki ke-1
    plot3(x2,y2,z2, '-og', 'LineWidth', 3); %plot kaki ke-2
    plot3(x3,y3,z3, '-ob', 'LineWidth', 3); %plot kaki ke-3
    plot3(x4,y4,z4, '-oc', 'LineWidth', 3); %plot kaki ke-4
    plot3(x5,y5,z5, '-om', 'LineWidth', 3); %plot kaki ke-5
    plot3(x6,y6,z6, '-ok', 'LineWidth', 3); %plot kaki ke-6

    plot3(x_new, y_new, z_new, '-o'); %plot platform
    fill3(x_new, y_new, z_new, 'm'); %memberi fill warna platform {P}
    hold off;
    grid on;

    title('\textbf{Stewart Platform Simulation}','Interpreter','latex');  %judul plot
    xlabel('$X$-Axis','Interpreter','latex'); %label sumbu Q
    ylabel('$Y$-Axis','Interpreter','latex'); %label sumbu y
    zlabel('$Z$-Axis','Interpreter','latex'); %label sumbu z

    axis([-rb-0.1 rb+0.1, -rb-0.1 rb+0.1, 0 0.6]);
    
    subplot(4, 3, 3); % Row 1 spans two columns
    hold on;
    plot(Sims.time(1:idx), Sims.Q(3,1:idx), '-', 'LineWidth', 2, 'Color', 'b');   
    plot(Sims.time, Gains.q_max(3)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'r');  
    plot(Sims.time, Gains.Q_des(3)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
    hold off;
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$z(m)$', 'FontSize', 12, 'Interpreter', 'latex');
    title('Position $z$', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([0.3 0.6])
    legend('$z$','$z_{max}$','$z_{des}$', 'FontSize', 12, 'Interpreter', 'latex');
    if t>tslide
        xlim([(t-tslide) t]);
    else
        xlim([Sims.tspan(1) tslide]);
    end   
    
    subplot(4, 3, 6); % Row 1 spans two columns
    hold on;
    plot(Sims.time(1:idx), Sims.Q(9,1:idx), '-', 'LineWidth', 2, 'Color', 'b');   
    plot(Sims.time, Gains.dq_max(3)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'r');  
    plot(Sims.time, Gains.Q_des(9)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
    hold off;
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$\dot{z}(m/s)$', 'FontSize', 12, 'Interpreter', 'latex');
    title('Velocity $\dot{z}$', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([0 0.25])
    legend('$\dot{z}$','$\dot{z}_{max}$','$\dot{z}_{des}$', 'FontSize', 12, 'Interpreter', 'latex');
    if t>tslide
        xlim([(t-tslide) t]);
    else
        xlim([Sims.tspan(1) tslide]);
    end   
    
    subplot(4, 3, 9); % Row 1 spans two columns
    hold on;
    plot(Sims.time(1:idx), Sims.Q(1:2,1:idx), '-', 'LineWidth', 2, 'Color', 'b');   
    plot(Sims.time, Gains.q_max(2)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'r');  
    plot(Sims.time, Gains.Q_des(2)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
    hold off;
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$x\&y(m)$', 'FontSize', 12, 'Interpreter', 'latex');
    title('Position $x\&y$', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([-0.025 0.025]);
    h1 = findobj(gca, 'Type', 'Line');
    legend(h1([4,2,1]),{'$x\&y$', '$x\&y_{max}$','$x\&y_{des}$'}, 'FontSize', 12, 'Interpreter', 'latex');
    if t>tslide
        xlim([(t-tslide) t]);
    else
        xlim([Sims.tspan(1) tslide]);
    end  
    
    subplot(4, 3, 12); % Row 1 spans two columns
    hold on;
    plot(Sims.time(1:idx), Sims.Q(4:6,1:idx), '-', 'LineWidth', 2, 'Color', 'b');   
    plot(Sims.time, Gains.q_max(4)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'r');  
    plot(Sims.time, Gains.Q_des(4)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
    hold off;
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$orientation(rad)$', 'FontSize', 12, 'Interpreter', 'latex');
    title('Orientation ($q_2$)', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([-deg2rad(10) deg2rad(10)])
    h1 = findobj(gca, 'Type', 'Line');
    legend(h1([5,2,1]),{'$p_2$', '$P_{2max}$','$P_{2des}$'}, 'FontSize', 12, 'Interpreter', 'latex');
    if t>tslide
        xlim([(t-tslide) t]);
    else
        xlim([Sims.tspan(1) tslide]);
    end  
    
    subplot(4, 3, 10); % Row 1 spans two columns
    plot(Sims.time(1:idx), Sims.F(:,1:idx), '-', 'LineWidth', 2);   
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$F(N)$', 'FontSize', 12, 'Interpreter', 'latex');
    title('Forces', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([-0.8 1.6])
    legend('$F_1$','$F_2$','$F_3$','$F_4$','$F_5$','$F_6$', 'FontSize', 12, 'Interpreter', 'latex');
    if t>tslide
        xlim([(t-tslide) t]);
    else
        xlim([Sims.tspan(1) tslide]);
    end  
    
    subplot(4, 3, 11); % Row 1 spans two columns
    % Identify zero crossings (sign changes)
    signChanges = [false, diff(sign(Sims.P(1:idx))) ~= 0]; % Find points where sign changes
    segments = find(signChanges); % Indices where sign changes occur
    segments = [1, segments, length(Sims.P(1:idx))]; % Include start and end indices
    % Loop through each segment and plot positive/negative separately
    hNeg = [];
    hold on;
    for i = 1:length(segments)-1
        startIdx = segments(i);
        endIdx = segments(i+1);
        if Sims.P(startIdx) < 0
            hNeg = plot(Sims.time(startIdx:endIdx), Sims.P(startIdx:endIdx), 'r-', 'LineWidth', 2); % Red for negative
        else
            hPos = plot(Sims.time(startIdx:endIdx), Sims.P(startIdx:endIdx), 'b-', 'LineWidth', 2); % Blue for positive
        end
    end
    hold off;
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$\Psi$', 'FontSize', 12, 'Interpreter', 'latex');
    title('$\Psi$', 'FontSize', 14, 'Interpreter', 'latex');
    if isempty(hNeg), legend(hPos, {'Not violated'}, 'FontSize', 12); 
    else, legend([hPos,hNeg], {'Not violated', 'Violated'}, 'FontSize', 12); end
    ylim([-0.1 0.2])
    if t>tslide
        xlim([(t-tslide) t]);
    else
        xlim([Sims.tspan(1) tslide]);
    end  
    
    frame = getframe(fig); % Capture the figure
    writeVideo(v, frame);
    
    idx=idx+1;
end

close(v);