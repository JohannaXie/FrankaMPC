clear;
close all;
load('.\data\simulation_data\simdata.mat');
rf = simdata.getElement(1);
time = rf.Values.Time;
time = [time(1,:);time(6:1005,:)];
rf = rf.Values.Data;
rf = [rf(1,:);rf(6:1005,:)];
h = simdata.getElement(2);
h = h.Values.Data;
pred = simdata.getElement(3);
pred = pred.Values.Data;
pred = [pred(1,:);pred(6:1005,:)];
states = simdata.getElement(4);
states = states.Values.Data;
states = [states(1,:);states(6:1005,:)];

figure(1);
subplot(3,1,1);
title('MPC Simulation')
hold on;
plot(time,states(:,1),'LineWidth',1);
plot(time+h,pred(:,1),'LineWidth',1);
plot(time,rf(:,1),'-.');
xlabel('Time [sec]')
ylabel('Postion [deg]')
legend('Joint Position','MPC Command','Reference');
subplot(3,1,2);
hold on;
plot(time,states(:,2),'LineWidth',1);
plot(time+h,pred(:,2),'LineWidth',1);
xlabel('Time [sec]')
ylabel('Velocity [deg/s]')
legend('Joint Velocity','MPC Command');
subplot(3,1,3);
hold on;
plot(time,states(:,3),'LineWidth',1);
plot(time+h,pred(:,3),'LineWidth',1);
xlabel('Time [sec]')
ylabel('Acceleration [deg/s^2]')
legend('Joint Acceleration','MPC Command');
savefig('.\data\simulation_data\simfig.fig')

