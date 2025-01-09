function animate_line(thetaDes,thetaFbk,time,animPlot,zoom,limDim)

    global cycle_time robot_type mount_num n
    
    DH_val = 0; % 0: Real links, 1: DH vectors
    
    if cycle_time == 0.004
        time_factor = 15;
    else
        time_factor = 1;
    end
    
    base = [0 0;0 0;-0.6 0];

    sensor_length = 0.033;
    tool_length = 0.144;
    
    if zoom == 0
        ee_axes_length = 0.1;
    else
        ee_axes_length = 0.02;
    end
    
    mount_length = 0;
    if mount_num > 0
        mount_length = mount_length+sensor_length;
        if mount_num > 1
            mount_length = mount_length+tool_length;
        end
    end
    
    [d,d_plus,~,theta_plus,a,alpha] = robotLinks(robot_type,zeros(6,1));
    
    if zoom == 0
        maxHor = d_plus(2)+d_plus(4)+d_plus(6)+d_plus(8)+mount_length+ee_axes_length;
        maxVer = d_plus(1)+d_plus(4)+d_plus(6)+d_plus(8)+mount_length+ee_axes_length;

        minVer = base(3,1);

        limDim(1,1) = -maxHor;
        limDim(1,2) = +maxHor;
        limDim(2,1) = -maxHor;
        limDim(2,2) = +maxHor;
        limDim(3,1) = minVer;
        limDim(3,2) = maxVer;
    end

    rot_z45 = [	cos(pi/4)	-sin(pi/4)	0
                sin(pi/4)	cos(pi/4)	0
                0           0           1   ];
    
    P_des = zeros(3,length(time));
    for t = 1:time_factor:length(time)
        [~,~,theta_des,~,~,~] = robotLinks(robot_type,thetaDes(:,t));
        [Tref, ~] = Transformation(d,d_plus,theta_des,theta_plus,a,alpha,0);
        
        A_Des    = Tref(1:3,1:3,end);
        
        if mount_num == 0
            mountDim = 0;
        elseif mount_num == 1
            mountDim = A_Des*[0;0;sensor_length];
        else
            mountDim = A_Des*[0;0;sensor_length+tool_length];
        end
        
        P_des(:,t)  = Tref(1:3,4,end)+mountDim;
    end
    
    P_fbk = zeros(3,length(time));
    for t = 1:time_factor:length(time)

        if DH_val == 1
            [Ti_E_0_fbk,~] = Transformation(d,d_plus,thetaFbk(:,t),theta_plus,a,alpha,0);

            DH_vec = zeros(3,n+1);
            for j = 1:n+1
                DH_vec(:,j) = Ti_E_0_fbk(1:3,4,j);
            end
        else
        end

        [~,~,theta_fbk,~,~,~] = robotLinks(robot_type,thetaFbk(:,t));
        [Tlink, ~] = Transformation(d,d_plus,theta_fbk,theta_plus,a,alpha,0);
        
        A_fbk       = Tlink(1:3,1:3,end);
        
        linkDim = Tlink(1:3,4,:);

        if mount_num > 0
            A_fbk = A_fbk*rot_z45;
            
            mountDim = A_fbk*[0;0;mount_length];
            linkDim(:,:,10) = Tlink(1:3,4,end)+mountDim;
            if mount_num > 1
                mountDim = A_fbk*[0;0;mount_length];
                linkDim(:,:,11) = Tlink(1:3,4,end)+mountDim;
            end
        end
        
        P_fbk(:,t)  = Tlink(1:3,4,end)+mountDim;
        
        six = [ Tlink(1:3,4,end)+mountDim Tlink(1:3,4,end)+mountDim+ee_axes_length*A_fbk(:,1)];
        siy = [ Tlink(1:3,4,end)+mountDim Tlink(1:3,4,end)+mountDim+ee_axes_length*A_fbk(:,2)];
        siz = [ Tlink(1:3,4,end)+mountDim Tlink(1:3,4,end)+mountDim+ee_axes_length*A_fbk(:,3)];

        if DH_val == 1
            plot3(animPlot,DH_vec(1,:),DH_vec(2,:),DH_vec(3,:),'c')
        else
            plot3(animPlot,linkDim(1,1:8),linkDim(2,1:8),linkDim(3,1:8),'Color',[1 0.6471 0])
        end
        hold(animPlot,'on')
        if DH_val == 1
        else
            plot3(animPlot,linkDim(1,8:9),linkDim(2,8:9),linkDim(3,8:9),'Color',[0.75 0.75 0.75])
            if mount_num > 0
                plot3(animPlot,linkDim(1,9:10),linkDim(2,9:10),linkDim(3,9:10),'Color',[0.5 0.5 0.5],'LineWidth',2) % sensor
                if mount_num > 1
                    plot3(animPlot,linkDim(1,10:11),linkDim(2,10:11),linkDim(3,10:11),'Color',[0.5 0.5 0.5]) % tool
                end
            end
        end
        plot3(animPlot,base(1,:),base(2,:),base(3,:),'Color',[0.5 0.5 0.5],'LineWidth',2)
        plot3(animPlot,six(1,:),six(2,:),six(3,:),'r')
        plot3(animPlot,siy(1,:),siy(2,:),siy(3,:),'g')
        plot3(animPlot,siz(1,:),siz(2,:),siz(3,:),'b')
        plot3(animPlot,P_des(1,1:time_factor:end),P_des(2,1:time_factor:end),P_des(3,1:time_factor:end),':c')
        plot3(animPlot,P_fbk(1,1:time_factor:t),P_fbk(2,1:time_factor:t),P_fbk(3,1:time_factor:t),'m')
        hold(animPlot,'off')

        xlabel(animPlot,'x')
        ylabel(animPlot,'y')
        zlabel(animPlot,'z')
        grid(animPlot,'on')
        
        set(animPlot,	'Projection','perspective', ...
                        'PlotBoxAspectRatio',[1 1 1], ...
                        'DataAspectRatio',[1 1 1], ...
                        'XLim',[limDim(1,1) limDim(1,2)], ...
                        'YLim',[limDim(2,1) limDim(2,2)], ...
                        'ZLim',[limDim(3,1) limDim(3,2)]);

        drawnow;
    end
    
    function [T0_h,A_h] = Transformation(d,d_plus,theta,theta_plus,a,alpha,MDH_val)

        k = length(d);

    %     if MDH_val == 0
    %         % A_i = % Trn_(z_(i-1),d_i)*Rot_(z_(i-1),theta_i)*Trn_(x_i,a_i)*Rot_(x_i,alpha_i)
    %         n = length(d);
    %     else
    %         % A_i = % Rot_(x_(i-1),alpha_(i-1)*Trn_(x_(i-1),a_(i-1)*Rot_(z_i,theta_i)*Trn_(z_i,d_i)
    %         n = length(d)+1;
    %     end

        trnZ_x      = zeros(4,4,k);
        rotZ_x      = zeros(4,4,k);
        trnX_i      = zeros(4,4,k);
        rotX_i      = zeros(4,4,k);

        A_h         = zeros(4,4,k+1);
        A_h(:,:,1)  = eye(4);

        for i = 1:k

            % Trans_z_x (x = i if DH, x = h if MDH)
            trnZ_x(:,:,i)	= [ 1 0 0 0;
                                0 1 0 0
                                0 0 1 d(i)+d_plus(i)
                                0 0 0 1 ];

            % Rot_z_x (x = i if DH, x = h if MDH)
            rotZ_x(:,:,i)   = [ cos(theta(i)+theta_plus(i))	-sin(theta(i)+theta_plus(i))	0	0
                                sin(theta(i)+theta_plus(i))	 cos(theta(i)+theta_plus(i))	0	0
                                0                          	 0                          	1	0
                                0                          	 0                           	0	1 ];

            % Trans_x_i
            trnX_i(:,:,i)   = [ 1 0 0 a(i)
                                0 1 0 0
                                0 0 1 0
                                0 0 0 1 ];

            % Rot_x_i
            rotX_i(:,:,i)   = [ 1	0               0               0
                                0	cos(alpha(i))  -sin(alpha(i))	0
                                0	sin(alpha(i))	cos(alpha(i))	0
                                0	0               0               1 ];

            if MDH_val == 0
                trnZ_h = trnZ_x;
                rotZ_h = rotZ_x;

                % A_i = % Trn_(z_(i-1),d_i)*Rot_(z_(i-1),theta_i)*Trn_(x_i,a_i)*Rot_(x_i,alpha_i)
                A_h(:,:,i+1) = trnZ_h(:,:,i)*rotZ_h(:,:,i)*trnX_i(:,:,i)*rotX_i(:,:,i);
            else
                trnZ_i = trnZ_x;
                rotZ_i = rotZ_x;

                % A_i = % Rot_(x_(i-1),alpha_(i-1)*Trn_(x_(i-1),a_(i-1)*Rot_(z_i,theta_i)*Trn_(z_i,d_i)
                A_h(:,:,i+1) = rotX_i(:,:,i)*trnX_i(:,:,i)*rotZ_i(:,:,i)*trnZ_i(:,:,i);
            end

        end

        % Transformation n to 0
        T0_h(:,:,1) = A_h(:,:,1);
        for i = 1:k
            T0_h(:,:,i+1) = T0_h(:,:,i)*A_h(:,:,i+1);
        end

    end

    function [d,d_plus,theta,theta_plus,a,alpha] = robotLinks(v,q)

        switch (v)
            case 0
                % Stäubli RX90
                theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
                a           = zeros(8,1);
                d           = zeros(8,1);
                alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
                d_plus    	= [0.478;0.05;0.196;0.425;-0.146;0.425;0;0.100];
                theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
            case 1
                % Stäubli RX160
                theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
                a           = zeros(8,1);
                d           = zeros(8,1);
                alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
                d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.625;0;0.110];
                theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
            case 2
                % Stäubli RX160L
                theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
                a           = zeros(8,1);
                d           = zeros(8,1);
                alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
                d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.925;0;0.110];
                theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
            case 3
                % Stäubli RX160+sl
                theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
                a           = zeros(8,1);
                d           = zeros(8,1);
                alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
                d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.625;0;0.110+0.033];
                theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
            case 4
                % Stäubli RX160+sl+gl
                theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
                a           = zeros(8,1);
                d           = zeros(8,1);
                alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
                d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.625;0;0.110+0.033+0.144];
                theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
            otherwise
                theta       = zeros(6,1);
                a           = zeros(6,1);
                d           = zeros(6,1);
                alpha       = zeros(6,1);
                d_plus    	= zeros(6,1);
                theta_plus  = zeros(6,1);
        end

    %     sensor_length = 0.033;
    %     tool_length = 0.144;
    end

end