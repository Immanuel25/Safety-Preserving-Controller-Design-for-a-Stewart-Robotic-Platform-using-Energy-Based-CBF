POSE_Color(1,:) = [0.8 0.8 0]; % yellow
POSE_Color(2,:) = [0.1 0.1 0]; % yellow
POSE_Color(3,:) = [0.9 0.2 0]; % yellow
POSE_Color(4,:) = [0.2314 0.6745 0.9451];  
POSE_Color(5,:) = [0.8549 0.2431 0.5373];  
POSE_Color(6,:) = [0.1294 0.8431 0.3647];  
% AUV_Color(7,:) = [0.7686 0.6196 0.2196];  
% AUV_Color(8,:) = [0.4157 0.3804 0.8039];  
% AUV_Color(9,:) = [0.9608 0.4863 0.1451];  

LEG_Color(1,:) = [0.0 1.0 1.0];   % Cyan
LEG_Color(2,:) = [1.0 0.0 1.0];   % Magenta
LEG_Color(3,:) = [0.5 0.0 0.5];   % Purple
LEG_Color(4,:) = [0.3 0.3 0.3];   % Dark Gray
LEG_Color(5,:) = [0.2 0.9 0.7];   % Light Gray
LEG_Color(6,:) = [1.0 0.0 0.0];   % Red

N = length(SimsLQR.time);
M = length(SimsCBF.time);

%% Position Z 
fig1 = figure(1);
set(fig1, 'position', [0 0 800 200]); % Adjust height for multiple subplots

hold on;
plot(SimsLQR.time, SimsLQR.Q(3,:), '-', 'LineWidth', 2, 'Color', POSE_Color(3,:));   
plot(SimsCBF.time, SimsCBF.Q(3,:), '-', 'LineWidth', 2, 'Color', POSE_Color(3,:));
plot(SimsLQR.time, Gains.z_max*ones(1,N), '--', 'LineWidth', 2, 'Color', 'b');  
plot(SimsLQR.time, Gains.Q_des(3)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
hold off;

xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Q(m)$', 'FontSize', 12, 'Interpreter', 'latex');
title('Position', 'FontSize', 14, 'Interpreter', 'latex');
    
legend('a','b', 'FontSize', 12, 'Interpreter', 'latex');
ylim([0.3 0.6]);

saveas(fig1, ['Figures/position_x.png']);
savefig(fig1, ['Figures/position_x.fig']); % Save as .fig

%% Force 

fig2 = figure(2);
set(fig2, 'position', [0 0 800 200]); % Adjust height for multiple subplots

hold on;
plot(Sims.time, Sims.F, '-', 'LineWidth', 2, 'Color', POSE_Color(3,:));   
plot(Sims.time, Gains.z_max*ones(1,N), '--', 'LineWidth', 2, 'Color', 'b');  
hold off;

xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Q(m)$', 'FontSize', 12, 'Interpreter', 'latex');
title('Position', 'FontSize', 14, 'Interpreter', 'latex');
    
legend('a','b', 'FontSize', 12, 'Interpreter', 'latex');
% ylim([0.3 0.6]);

saveas(fig1, ['Figures/position_x.png']);
savefig(fig1, ['Figures/position_x.fig']); % Save as .fig


