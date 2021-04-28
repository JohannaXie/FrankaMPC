clear;
close all;
path = "./data/trajectory_result_without_perturbation/";
figure(1);
hold on;
MpcMat = load(path + "MPC_DataRecord.txt");
CtrlMat = load(path + 'Control_DataRecord.txt');
refMat = load(path + "Reference.txt");
MpcMat(:,3:27) = MpcMat(:,3:27)/pi*180;
CtrlMat(:,3:9) = CtrlMat(:,3:9)/pi*180;
plot(CtrlMat(:,1),ones(size(CtrlMat(:,1)))*refMat(4),'LineWidth',1.5);
plot(CtrlMat(:,1),CtrlMat(:,6),'b','LineWidth',1.5);
index = find(MpcMat(:,1)>0);
MpcMat = MpcMat(index,:);
T = 5;
Interval = 1;
N = T/Interval;
for i = 1:N
    x = (i-1)*(Interval/0.01)+1;
    %x = i;
    time = MpcMat(x,1)+MpcMat(x,2):MpcMat(x,2):T;
    plot(time, MpcMat(x,3:27),'LineWidth',2);
end
xlim([0,5]);
legend('Reference','Joint Position','Trajectory @ T = 0.0','Trajectory @ T = 1.0', 'Trajectory @ T = 2.0','Trajectory @ T = 3.0','Trajectory @ T = 4.0'  )
xlabel('Time [sec]')
ylabel('Position [deg]')
title('MPC trajectory on Joint 4');
savefig(path + "MPC_trajectory_J4.fig")