%% plot_data.m
% info: m file to plot data generated from LLI outputs

clc, clear all, close all

%% Select plots to show (if data exist)
% 1: on, 0: off

task_plots          = 1;
    trn_des_plot    = 1;
    trn_fbk_plot    = 1;
    trn_err_plot    = 1;
    rot_des_plot    = 1;
    rot_fbk_plot    = 1;
    rot_err_plot    = 1;
    crc_xz_plot     = 1;
    crc_xy_plot     = 1;
    
joint_plots         = 0;
    pos_plot        = 1;
    vel_plot        = 1;
    acc_plot        = 0;
    
sensor_plots        = 0;
    base_for_plot   = 1;
    base_tor_plot   = 1;

torque_plots        = 0;

animate             = 1;
    cad             = 1;
    zoom            = 0;

%% Load data

% addpath('C:\Users\Mertcan\source\print_folder');
    
if exist('print_info.txt', 'file') == 2
    info_table = readtable('print_info.txt');
    print_info(info_table);
end

if exist('q_posDes.txt', 'file') == 2, load q_posDes.txt, end
if exist('q_velDes.txt', 'file') == 2, load q_velDes.txt, end
if exist('q_accDes.txt', 'file') == 2, load q_accDes.txt, end
if exist('q_posFbk.txt', 'file') == 2, load q_posFbk.txt, end
if exist('q_velFbk.txt', 'file') == 2, load q_velFbk.txt, end
if exist('x_posDes.txt', 'file') == 2, load x_posDes.txt, end
if exist('x_velDes.txt', 'file') == 2, load x_velDes.txt, end
if exist('x_accDes.txt', 'file') == 2, load x_accDes.txt, end
if exist('x_posFbk.txt', 'file') == 2, load x_posFbk.txt, end
if exist('x_velFbk.txt', 'file') == 2, load x_velFbk.txt, end
if exist('q_torFbk.txt', 'file') == 2, load q_torFbk.txt, end
if exist('q_torCmd.txt', 'file') == 2, load q_torCmd.txt, end
if exist('q_torDyn.txt', 'file') == 2, load q_torDyn.txt, end
if exist('q_torFrc.txt', 'file') == 2, load q_torFrc.txt, end
if exist('q_torSpr.txt', 'file') == 2, load q_torSpr.txt, end
if exist('q_torGrv.txt', 'file') == 2, load q_torGrv.txt, end
if exist('q_torMot.txt', 'file') == 2, load q_torMot.txt, end
if exist('q_torFor.txt', 'file') == 2, load q_torFor.txt, end
if exist('s_ftSnsr.txt', 'file') == 2, load s_ftSnsr.txt, end
if exist('s_ftSxth.txt', 'file') == 2, load s_ftSxth.txt, end
if exist('s_ftBase.txt', 'file') == 2, load s_ftBase.txt, end
if exist('s_ftUcmp.txt', 'file') == 2, load s_ftUcmp.txt, end
if exist('s_ftWght.txt', 'file') == 2, load s_ftWght.txt, end
if exist('prntTmp1.txt', 'file') == 2, load prntTmp1.txt, end
if exist('prntTmp2.txt', 'file') == 2, load prntTmp2.txt, end
if exist('prntTmp3.txt', 'file') == 2, load prntTmp3.txt, end
if exist('prntTmp4.txt', 'file') == 2, load prntTmp4.txt, end
if exist('prntTmp5.txt', 'file') == 2, load prntTmp5.txt, end
if exist('prntTmp6.txt', 'file') == 2, load prntTmp6.txt, end

%% Initialization

global cycle_time robot_type mount_num n modelstruct

if exist('print_info.txt', 'file') ~= 2
    cycle_time = 0.004;
    robot_type = 1;
    mount_num  = 1;
end

if exist('q_posDes.txt', 'file') == 2
    timeInmtn = (0:cycle_time:(length(q_posDes)-1)*cycle_time)';
    n = length(q_posDes(1,:));
elseif exist('x_posDes.txt', 'file') == 2
    timeInmtn = (0:cycle_time:(length(x_posDes)-1)*cycle_time)';
    n = length(x_posDes(1,:));
end

modelstruct = [];
        
%% Plot data

if task_plots == 1
    
    if trn_des_plot == 1
        figure('Name','Reference Translation Trajectory')
        subplot(3,1,1)    
        if exist('x_posDes.txt', 'file') == 2, plot(timeInmtn,x_posDes(:,1:3)), end
        ylabel('Pos')
        subplot(3,1,2)
        if exist('x_velDes.txt', 'file') == 2, plot(timeInmtn,x_velDes(:,1:3)), end
        ylabel('Vel')
        subplot(3,1,3)
        if exist('x_accDes.txt', 'file') == 2, plot(timeInmtn,x_accDes(:,1:3)), end
        ylabel('Acc')
    end

    if trn_fbk_plot == 1
        figure('Name','Feedback Translation Trajectory')
        subplot(3,1,1)
        if exist('x_posFbk.txt', 'file') == 2, plot(timeInmtn,x_posFbk(:,1:3)), end
        ylabel('Pos')
        subplot(3,1,2)
        if exist('x_velFbk.txt', 'file') == 2, plot(timeInmtn,x_velFbk(:,1:3)), end
        ylabel('Vel')
    end

    if trn_err_plot == 1
        figure('Name','Translation Trajectory Error')
        subplot(3,1,1)
        if exist('x_posDes.txt', 'file') == 2 && exist('x_posFbk.txt', 'file') == 2, plot(timeInmtn,(x_posDes(:,1:3)-x_posFbk(:,1:3))), end
        ylabel('Pos')
        subplot(3,1,2)
        if exist('x_velDes.txt', 'file') == 2 && exist('x_velFbk.txt', 'file') == 2, plot(timeInmtn,(x_velDes(:,1:3)-x_velFbk(:,1:3))), end
        ylabel('Vel')
    end
    
    if rot_des_plot == 1
        figure('Name','Reference Rotation Trajectory')
        subplot(3,1,1)
        if exist('x_posDes.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_posDes(:,4:6))), end
        ylabel('Pos')
        subplot(3,1,2)
        if exist('x_velDes.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_velDes(:,4:6))), end
        ylabel('Vel')
        subplot(3,1,3)
        if exist('x_accDes.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_accDes(:,4:6))), end
        ylabel('Acc')
    end

    if rot_fbk_plot == 1
        figure('Name','Feedback Rotation Trajectory')
        subplot(3,1,1)
        if exist('x_posFbk.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_posFbk(:,4:6))), end
        ylabel('Pos')
        subplot(3,1,2)
        if exist('x_velFbk.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_velFbk(:,4:6))), end
        ylabel('Vel')
    end

    if rot_err_plot == 1
        figure('Name','Rotation Trajectory Error')
        subplot(3,1,1)
        if exist('x_posDes.txt', 'file') == 2 && exist('x_posFbk.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_posDes(:,4:6)-x_posFbk(:,4:6))), end
        ylabel('Pos')
        subplot(3,1,2)
        if exist('x_velDes.txt', 'file') == 2 && exist('x_velFbk.txt', 'file') == 2, plot(timeInmtn,rad2deg(x_velDes(:,4:6)-x_velFbk(:,4:6))), end
        ylabel('Vel')
    end
    
    if crc_xz_plot == 1
        figure('Name','XZ Plane')
        if exist('x_posDes.txt', 'file') == 2 && exist('x_posFbk.txt', 'file') == 2, plot(x_posDes(:,1),x_posDes(:,3),x_posFbk(:,1),x_posFbk(:,3)), end
        xlabel('x ekseni (m)')
        ylabel('z ekseni (m)')
        legend('Referans','Ölçülen')
        daspect([1 1 1])
        axis([min(x_posDes(:,1))-0.01 max(x_posDes(:,1))+0.01 min(x_posDes(:,3))-0.01 max(x_posDes(:,3))+0.01])
    end

    if crc_xy_plot == 1
        figure('Name','XY Plane')
        if exist('x_posDes.txt', 'file') == 2 && exist('x_posFbk.txt', 'file') == 2, plot(x_posDes(:,1),x_posDes(:,2),x_posFbk(:,1),x_posFbk(:,2)), end
        xlabel('x ekseni (m)')
        ylabel('y ekseni (m)')
        legend('Referans','Ölçülen')
        daspect([1 1 1])
        axis([min(x_posDes(:,1))-0.01 max(x_posDes(:,1))+0.01 min(x_posDes(:,2))-0.01 max(x_posDes(:,2))+0.01])
    end
    
end

if joint_plots == 1
    
    if pos_plot == 1
        figure('Name','Joint Position Comparison')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,rad2deg(q_posDes(:,i)),'r',timeInmtn,rad2deg(q_posFbk(:,i)),'b')
            title(['joint ',num2str(i)])
        end
        legend('Reference','Feedback')
    end

    if vel_plot == 1
        figure('Name','Joint Velocity Comparison')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,rad2deg(q_velFbk(:,i)),'b',timeInmtn,rad2deg(q_velDes(:,i)),'r')
            title(['joint ',num2str(i)])
        end
        legend('Feedback','Reference')
    end

    if acc_plot == 1
        figure('Name','Joint Acceleration')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,rad2deg(q_accDes(:,i)))
            title(['joint ',num2str(i)])
        end
    end
end

if torque_plots == 1
    
    if exist('q_torDyn.txt', 'file') == 2
        figure('Name','Torque Dynamic')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,q_torDyn(:,i))
            title(['joint ',num2str(i)])
        end
    end

    if exist('q_torFrc.txt', 'file') == 2
        figure('Name','Torque Friction')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,q_torFrc(:,i))
            title(['joint ',num2str(i)])
        end
    end

    if exist('q_torSpr.txt', 'file') == 2
        figure('Name','Torque Spring')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,q_torSpr(:,i))
            title(['joint ',num2str(i)])
        end
    end
    
    if exist('q_torCmd.txt', 'file') == 2
        figure('Name','Torque Command')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,q_torCmd(:,i))
            title(['joint ',num2str(i)])
        end
    end

    if exist('q_torFbk.txt', 'file') == 2
        figure('Name','Torque Feedback')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,q_torFbk(:,i))
            title(['joint ',num2str(i)])
        end
    end
end

if sensor_plots == 1
    
    if exist('s_ftSnsr.txt', 'file') == 2
        figure('Name','CE trans')
        plot(timeInmtn,s_ftSnsr(:,1:3))

        figure('Name','CE rot')
        plot(timeInmtn,s_ftSnsr(:,4:6))
    end

    if exist('s_ftSxth.txt', 'file') == 2
        figure('Name','E trans')
        plot(timeInmtn,s_ftSxth(:,1:3))

        figure('Name','E rot')
        plot(timeInmtn,s_ftSxth(:,4:6))
    end
        
    if exist('s_ftUcmp.txt', 'file') == 2
        figure('Name','u')
        for i = 1:n
            subplot(3,2,i)
            plot(timeInmtn,s_ftUcmp(:,i))
            title(['joint ',num2str(i)])
        end
    end
    
    if exist('s_ftBase.txt', 'file') == 2
        if base_for_plot == 1
            figure('Name','Sensor Force Base')
            plot(timeInmtn,s_ftBase(:,1:3))
            legend('Fx','Fy','Fz')
            xlabel('zaman (saniye)')
            ylabel('kuvvet (N)')
        end

        if base_tor_plot == 1
            figure('Name','Sensor Torque Base')
            plot(timeInmtn,s_ftBase(:,4:6))
            xlabel('zaman (saniye)')
            ylabel('moment (N.m)')
            legend('Mx','My','Mz')
        end
        
        if exist('s_ftWght', 'file') == 2
            figure('Name','Sensor Force Fy')
            plot(timeInmtn,s_ftBase(:,2),timeInmtn,s_ftWght(:,1))
            xlabel('zaman (saniye)')
            ylabel('kuvvet (N)')
            legend('Fy','connection')
        end
    end
                
%     figure('Name','Uncompensated Force Sensor')
%         plot(timeInmtn,s_ftUcmp(:,1:3))
% 
%     figure('Name','Uncompensated Torque Sensor')
%         plot(timeInmtn,s_ftUcmp(:,4:6))
%         
%     figure('Name','Weight Force Sensor')
%         plot(timeInmtn,s_ftWght(:,1:3))
% 
%     figure('Name','Weight Torque Sensor')
%         plot(timeInmtn,s_ftWght(:,4:6))
end

if exist('prntTmp1.txt', 'file') == 2
    figure('Name','Lambda')
    for i = 1:n
        subplot(3,2,i)
        plot(timeInmtn,prntTmp1(:,i))
        title(['Lambda ',num2str(i)])
    end
end

if exist('prntTmp2.txt', 'file') == 2
    figure('Name','B')
    for i = 1:n
        subplot(3,2,i)
        plot(timeInmtn,prntTmp2(:,i))
        title(['B ',num2str(i)])
    end
end

if exist('prntTmp3.txt', 'file') == 2
    figure('Name','K')
    for i = 1:n
        subplot(3,2,i)
        plot(timeInmtn,prntTmp3(:,i))
        title(['K ',num2str(i)])
    end
end

if exist('prntTmp4.txt', 'file') == 2
    figure('Name','E')
    for i = 1:n
        subplot(3,2,i)
        plot(timeInmtn,prntTmp4(:,i))
        title(['E ',num2str(i)])
    end
end

if exist('prntTmp5.txt', 'file') == 2
    figure('Name','CE')
    for i = 1:n
        subplot(3,2,i)
        plot(timeInmtn,prntTmp5(:,i))
        title(['CE ',num2str(i)])
    end
end

if exist('prntTmp6.txt', 'file') == 2
    figure('Name','alpha')
    for i = 1:n
        subplot(3,2,i)
        plot(timeInmtn,prntTmp6(:,i))
        title(['alpha ',num2str(i)])
    end
end
    
%% Animate robot

if animate == 1

    % limiting dimensions
    if zoom == 1
        limDim(1,1) = min(min(x_posDes(:,1)),min(x_posFbk(:,1)))-0.01;
        limDim(1,2) = max(max(x_posDes(:,1)),max(x_posFbk(:,1)))+0.01;
        limDim(2,1) = min(min(x_posDes(:,2)),min(x_posFbk(:,2)))-0.01;
        limDim(2,2) = max(max(x_posDes(:,2)),max(x_posFbk(:,2)))+0.01;
        limDim(3,1) = min(min(x_posDes(:,3)),min(x_posFbk(:,3)))-0.01;
        limDim(3,2) = max(max(x_posDes(:,3)),max(x_posFbk(:,3)))+0.01;
    else
        limDim = zeros(3,2);
    end

    % Generate animation figure
    animFig = figure('Name','Animation Plot');
    animPlot = axes('Parent',animFig);
    if cad == 0
        animate_line(q_posDes',q_posFbk',timeInmtn,animPlot,zoom,limDim)
    else
        animate_cad(q_posDes',q_posFbk',timeInmtn,animPlot,zoom,limDim)
    end
    
end