clear;
close all;
path = "./data/result_without_perturbation/";
perturbed = 0; 
PertStart = 0.819;
PertEnd = 0.819+1.355;  
CtrlMat = load(path + "Control_DataRecord.txt");
MpcMat = load(path + "MPC_DataRecord.txt");
refMat = load(path + "Reference.txt");
Gain = [0.935,1.0,1.0,1.0,1.06,1.1,1.12];

for i = 0:6
    figure(i+1);
    subplot(3,1,1);
    title("Joint " + num2str(i+1));    
    hold on;    
    MPC_Pos = plot(MpcMat(:,1)+MpcMat(:,2),MpcMat(:,2+i*3+1)/pi*180,'LineWidth',1);
    Ctrl_Pos = plot(CtrlMat(:,1),CtrlMat(:,2 + i*3+1)/pi*180,'LineWidth',1);
    ref = plot(CtrlMat(:,1),ones(size(CtrlMat(:,1)))*refMat(i+1),'-.','LineWidth',1);
    if (perturbed>0)
        yrange = ylim;
        y = yrange(1):(yrange(2)-yrange(1))/10:yrange(2);
        plot(PertStart*ones(size(y)),y,'-.r')
        plot(PertEnd*ones(size(y)),y,'-.r')
    end        
    xlim([0,5])
    legend([MPC_Pos,Ctrl_Pos,ref],"MPC command","Joint Position","Reference") 
    xlabel("Time [sec]")
    ylabel("Position [deg]")
    subplot(3,1,2);    
    hold on;
    MPC_Vel = plot(MpcMat(:,1)+MpcMat(:,2),MpcMat(:,2+i*3+2)/pi*180 * Gain(i+1),'LineWidth',1);
    Ctrl_Vel = plot(CtrlMat(:,1),CtrlMat(:,2 + i*3+2)/pi*180,'LineWidth',1);    
    if (perturbed>0)
        yrange = ylim;
        y = yrange(1):(yrange(2)-yrange(1))/10:yrange(2);
        plot(PertStart*ones(size(y)),y,'-.r')
        plot(PertEnd*ones(size(y)),y,'-.r')
    end        
    xlim([0,5])    
    legend([MPC_Vel, Ctrl_Vel], "MPC command","Joint Velocity")         
    xlabel("Time [sec]")
    ylabel("Velocity [deg/s]")    
    subplot(3,1,3);    
    hold on;
    MPC_Acc = plot(MpcMat(:,1)+MpcMat(:,2),MpcMat(:,2+i*3+3)/pi*180,'LineWidth',1);
    Ctrl_Acc = plot(CtrlMat(:,1),CtrlMat(:,2 + i*3+3)/pi*180,'LineWidth',1);
    xlabel("Time [sec]")
    ylabel("Acceleration [deg/s^2]")       
    if (perturbed>0)
        yrange = ylim;
        y = yrange(1):(yrange(2)-yrange(1))/10:yrange(2);
        plot(PertStart*ones(size(y)),y,'-.r')
        plot(PertEnd*ones(size(y)),y,'-.r')
    end      
    xlim([0,5])    
    legend([MPC_Acc, Ctrl_Acc], "MPC command","Joint Acceleration");  
    savefig(path+"Joint"+num2str(i+1)+".fig")
end
