
U = {result0.u; result1.u; result2.u; result3.u; result4.u};
N_trajs = length(U);
Y = cell(N_trajs,1);

reset0 = repmat( [0;0;0],1, N_u);
reset_traj_h = repmat([0; pi/2; 0], 1, N_u);
reset_traj = repmat([0; 0; 0], 1, N_u);
reset_traj = [reset_traj_h, reset_traj];

 %% execute trajs
 
for i=1:N_trajs

u = U{i};
y = exetraj(u);
y_rp = (y(2,:)/1000).^2 * 25;
y(2,:) = y_rp;
Y{i} = y;
    
    disp('Execution of traj done. Press keys to reset...')
    pause
    % reset to init position
    exetraj(reset_traj);
    disp('Reseted to initial position. Press keys to continue...')
    pause

end


%% plot records
figure
title('recorded')
subplot(2,1,1)
hold on
for i=1:N_trajs
    plot(t, Y{i}(1,:) )
end
title('position')
legend('1','2','3','4','5')
hold off


subplot(2,1,2)
hold on
for i=1:N_trajs
    plot(t, Y{i}(2,:) )
end
title('rege power')
legend('1','2','3','4','5')
hold off
%% plot comparison
Erec1 = sum(Y{1}(2,:))*0.02;
Erec2 = sum(Y{2}(2,:))*0.02;
Erec3 = sum(Y{3}(2,:))*0.02;
Erec4 = sum(Y{4}(2,:))*0.02;
Erec5 = sum(Y{5}(2,:))*0.02;
Erec = [Erec1,Erec2,Erec3,Erec4,Erec5];

figure
subplot(2,2,1)

hold on

plot(t, result0.x(1,:),'--')
%plot(t, ones(1,N)*target)
plot(t, result1.x(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t, result2.x(1,:),'b-.','LineWidth',1)
plot(t, result3.x(1,:),'k-','LineWidth',1)
plot(t, result4.x(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
legend('C.D.','dynamic', 'regenerative','hybrid','fixed damping')
hold off

% equilibrium position
% subplot(2,3,2)
% hold on
% plot(t, result3.x(3,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
% 
% plot(t, result0.x(3,:),'--')
% plot(t, result1.x(3,:),'r-','LineWidth',1)
% plot(t, result2.x(3,:),'b-.','LineWidth',1)
% hold off
% stiffness


subplot(2,2,3)
hold on
c = categorical({'C.D.','dynamic','regenerative','hybrid','fixed damp'});
E = [result0.E, result1.E, result2.E, result3.E, result4.E];
Erege = [result0.Erege, 0, result2.Erege, result3.Erege, result4.Erege];
Enet = E - Erege;
zeta = Erege./E;
bar(c, E, 'FaceColor',[0.5 0 .5],'EdgeColor',[0.9 0 .9],'LineWidth',1.0)
bar(c, Enet, 'FaceColor',[0 .5 .5],'EdgeColor',[0 .9 .9],'LineWidth',1.0)
hold off

subplot(2,2,2)
hold on

plot(t, Y{1}(1,:),'--')
plot(t, Y{2}(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t, Y{3}(1,:),'b-.','LineWidth',1)
plot(t, Y{4}(1,:),'k-','LineWidth',1)
plot(t, Y{5}(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

hold off
% damping
subplot(2,2,4)
hold on
bar(c, Erec)
hold off

