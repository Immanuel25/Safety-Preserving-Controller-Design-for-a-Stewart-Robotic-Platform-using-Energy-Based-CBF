close all;
clf; 

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

N = length(Sims.time);
names = {'x','y','z','\phi','\theta','\psi'};
names2 = {'x','y','z','phi','theta','psi'};

%% Position 
for i =  1:6
    fig = figure(i);
    set(fig, 'position', [0 0 600 200]); % Adjust height for multiple subplots

    hold on;
    plot(Sims.time, Sims.Q(i,:), '-', 'LineWidth', 2, 'Color', 'b');   
    plot(Sims.time, Gains.q_max(i)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'r');  
    plot(Sims.time, Gains.Q_des(i)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
    hold off;

    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel(sprintf('$%s$',names{i}), 'FontSize', 12, 'Interpreter', 'latex');
    title(sprintf('Position $%s$',names{i}), 'FontSize', 14, 'Interpreter', 'latex');

    legend(sprintf('$%s$',names{i}),sprintf('$%s_{max}$',names{i}),sprintf('$%s_{des}$',names{i}), 'FontSize', 12, 'Interpreter', 'latex');

    saveas(fig, sprintf('Figures/%s.png',names2{i}));
    savefig(fig, sprintf('Figures/%s.fig',names2{i})); % Save as .fig
end

%% Velocity 
for i =  1:6
    fig = figure(i+6);
    set(fig, 'position', [0 0 600 200]); % Adjust height for multiple subplots

    hold on;
    plot(Sims.time, Sims.Q(i+6,:), '-', 'LineWidth', 2, 'Color', 'b');   
    plot(Sims.time, Gains.dq_max(i)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'r');  
    plot(Sims.time, Gains.Q_des(i+6)*ones(1,N), '--', 'LineWidth', 2, 'Color', 'g');  
    hold off;

    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel(sprintf('$\\dot{%s}$',names{i}), 'FontSize', 12, 'Interpreter', 'latex');
    title(sprintf('Velocity $\\dot{%s}$',names{i}), 'FontSize', 14, 'Interpreter', 'latex');

    legend(sprintf('$\\dot{%s}$',names{i}),sprintf('$\\dot{%s}_{max}$',names{i}),sprintf('$\\dot{%s}_{des}$',names{i}), 'FontSize', 12, 'Interpreter', 'latex');

    saveas(fig, sprintf('Figures/d%s.png',names2{i}));
    savefig(fig, sprintf('Figures/d%s.fig',names2{i})); % Save as .fig
end

%% Force 

fig = figure(13);
set(fig, 'position', [0 0 600 200]); % Adjust height for multiple subplots

plot(Sims.time, Sims.F, '-', 'LineWidth', 2);   

xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$F(N)$', 'FontSize', 12, 'Interpreter', 'latex');
title('Force', 'FontSize', 14, 'Interpreter', 'latex');
    
legend('F1','F2','F3','F4','F5','F6', 'FontSize', 12, 'Interpreter', 'latex');
% ylim([0.3 0.6]);

saveas(fig, ['Figures/force.png']);
savefig(fig, ['Figures/force.fig']); % Save as .fig

%% P 

fig = figure(14);
set(fig, 'position', [0 0 600 200]); % Adjust height for multiple subplots
hold on;
t = Sims.time;
P = Sims.P;
% Identify zero crossings (sign changes)
signChanges = [false, diff(sign(P)) ~= 0]; % Find points where sign changes
segments = find(signChanges); % Indices where sign changes occur
segments = [1, segments, length(P)]; % Include start and end indices
% Loop through each segment and plot positive/negative separately
for i = 1:length(segments)-1
    startIdx = segments(i);
    endIdx = segments(i+1);
    if P(startIdx) < 0
        hNeg = plot(t(startIdx:endIdx), P(startIdx:endIdx), 'r-', 'LineWidth', 2); % Red for negative
    else
        hPos = plot(t(startIdx:endIdx), P(startIdx:endIdx), 'b-', 'LineWidth', 2); % Blue for positive
    end
end
hold off;
xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Psi$', 'FontSize', 12, 'Interpreter', 'latex');
title('Psi', 'FontSize', 14, 'Interpreter', 'latex');
legend([hPos, hNeg], {'Not violated', 'Violated'}, 'FontSize', 12);
% ylim([0.3 0.6]);

saveas(fig, ['Figures/force.png']);
savefig(fig, ['Figures/force.fig']); % Save as .fig

%% h 

fig = figure(15);
set(fig, 'position', [0 0 600 200]); % Adjust height for multiple subplots

plot(Sims.time, Sims.h, '-', 'LineWidth', 2);   

xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$h$', 'FontSize', 12, 'Interpreter', 'latex');
title('h', 'FontSize', 14, 'Interpreter', 'latex');
    
legend('h1','h2','h3','h4','h5','h6', 'FontSize', 12, 'Interpreter', 'latex');
% ylim([0.3 0.6]);

saveas(fig, ['Figures/force.png']);
savefig(fig, ['Figures/force.fig']); % Save as .fig

%% hD 

fig = figure(16);
set(fig, 'position', [0 0 600 200]); % Adjust height for multiple subplots

plot(Sims.time, Sims.F, '-', 'LineWidth', 2);   

xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$hD$', 'FontSize', 12, 'Interpreter', 'latex');
title('hD', 'FontSize', 14, 'Interpreter', 'latex');
    
legend('hD1','hD2','hD3','hD4','hD5','hD6', 'FontSize', 12, 'Interpreter', 'latex');
% ylim([0.3 0.6]);

saveas(fig, ['Figures/force.png']);
savefig(fig, ['Figures/force.fig']); % Save as .fig


