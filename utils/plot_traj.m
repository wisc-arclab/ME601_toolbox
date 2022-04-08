function [] = plot_traj(f)
% Plot the trajectory contained in f csv file
T = readmatrix(f);
r = T(:,2:4)';
figure(1)
plot3(r(1,:),r(2,:),r(3,:),'--k')
grid on
hold on
end