function animate_cad(thetaDes,thetaFbk,time,animPlot,zoom,limDim)

    global cycle_time robot_type mount_num n modelstruct
    
    load('SBAS00168A.mat'); s1 = modelstruct;
    load('SBAS00169A.mat'); s2 = modelstruct;
    load('SBAS00170A.mat'); s3 = modelstruct;
    load('SBAS00171A.mat'); s4 = modelstruct;
    load('SBAS00172A.mat'); s5 = modelstruct;
    load('SBAS00173A.mat'); s6 = modelstruct;
    load('SBAS00174A.mat'); s7 = modelstruct;

    p1 = patch(animPlot,'Faces', s1.F, 'Vertices' ,s1.V.*0.001);
    set(p1, 'facec', 'flat');            % Set the face color flat
    set(p1, 'FaceVertexCData', s1.C);    % Set the color (from file)
    set(p1, 'EdgeColor','none');         % Set the edge color
    p2 = patch(animPlot,'Faces', s2.F, 'Vertices' ,s2.V.*0.001);
    set(p2, 'facec', 'flat');            % Set the face color flat
    set(p2, 'FaceVertexCData', s2.C);    % Set the color (from file)
    set(p2, 'EdgeColor','none');         % Set the edge color
    p3 = patch(animPlot,'Faces', s3.F, 'Vertices' ,s3.V.*0.001);
    set(p3, 'facec', 'flat');            % Set the face color flat
    set(p3, 'FaceVertexCData', s3.C);    % Set the color (from file)
    set(p3, 'EdgeColor','none');         % Set the edge color
    p4 = patch(animPlot,'Faces', s4.F, 'Vertices' ,s4.V.*0.001);
    set(p4, 'facec', 'flat');            % Set the face color flat
    set(p4, 'FaceVertexCData', s4.C);    % Set the color (from file)
    set(p4, 'EdgeColor','none');         % Set the edge color
    p5 = patch(animPlot,'Faces', s5.F, 'Vertices' ,s5.V.*0.001);
    set(p5, 'facec', 'flat');            % Set the face color flat
    set(p5, 'FaceVertexCData', s5.C);    % Set the color (from file)
    set(p5, 'EdgeColor','none');         % Set the edge color
    p6 = patch(animPlot,'Faces', s6.F, 'Vertices' ,s6.V.*0.001);
    set(p6, 'facec', 'flat');            % Set the face color flat
    set(p6, 'FaceVertexCData', s6.C);    % Set the color (from file)
    set(p6, 'EdgeColor','none');         % Set the edge color
    p7 = patch(animPlot,'Faces', s7.F, 'Vertices' ,s7.V.*0.001);
    set(p7, 'facec', 'flat');            % Set the face color flat
    set(p7, 'FaceVertexCData', s7.C);    % Set the color (from file)
    set(p7, 'EdgeColor','none');         % Set the edge color
    
    % DH parameters
    a = [0.150;0.825;0;0;0;0];            
    alpha = [-pi/2;0;pi/2;-pi/2;pi/2;0];
    d = [0;0;0;0;0;0];
    d_plus = [0.550;0;0;0.625;0;0.110];
    theta_plus = [0;-pi/2;pi/2;0;0;0];

    V1 = [s1.V';ones(1,length(s1.V(:,1)))];
    V2 = [s2.V';ones(1,length(s2.V(:,1)))];
    V3 = [s3.V';ones(1,length(s3.V(:,1)))];
    V4 = [s4.V';ones(1,length(s4.V(:,1)))];
    V5 = [s5.V';ones(1,length(s5.V(:,1)))];
    V6 = [s6.V';ones(1,length(s6.V(:,1)))];
    V7 = [s7.V';ones(1,length(s7.V(:,1)))];

    light                               % add a default light
%     daspect([1 1 1])                    % Setting the aspect ratio
    view(3)                             % Isometric view
    
    xlabel(animPlot,'x')
    ylabel(animPlot,'y')
    zlabel(animPlot,'z')
    grid(animPlot,'on')
        
    if cycle_time == 0.004
        time_factor = 100;
    else
        time_factor = 1;
    end
    
%     base = [0 0;0 0;-0.6 0];

    sensor_length = 0.033;
    tool_length = 0.144;
    
%     if zoom == 0
%         ee_axes_length = 0.1;
%     else
%         ee_axes_length = 0.02;
%     end
    
    mount_length = 0;
    if mount_num > 0
        mount_length = mount_length+sensor_length;
        if mount_num > 1
            mount_length = mount_length+tool_length;
        end
    end
    
    if zoom == 0
        maxX = 0.3;
        minX = -0.3;
        maxY = 0.3;
        minY = -0.3;
        maxZ = 0;
        minZ = 0;
        m = n+mount_num;
        for j = 1:m
            for t = 1:time_factor:length(time)
                T0_h = Transformation(d,d_plus,thetaFbk(:,t),theta_plus,a,alpha);
                
                if T0_h(1,4,j) > maxX, maxX = T0_h(1,4,j); end
                if T0_h(1,4,j) < minX, minX = T0_h(1,4,j); end
                if T0_h(2,4,j) > maxY, maxY = T0_h(2,4,j); end
                if T0_h(2,4,j) < minY, minY = T0_h(2,4,j); end
                if T0_h(3,4,j) > maxZ, maxZ = T0_h(3,4,j); end
            end
        end

        limDim(1,1) = minX-0.2;
        limDim(1,2) = maxX+0.2;
        limDim(2,1) = minY-0.2;
        limDim(2,2) = maxY+0.2;
        limDim(3,1) = minZ;
        limDim(3,2) = maxZ+0.2;
    end
    
    rot_z45 = [	cos(pi/4)	-sin(pi/4)	0
                sin(pi/4)	cos(pi/4)	0
                0           0           1   ];
    
    P_des = zeros(3,length(time));
    for t = 1:time_factor:length(time)
        Tref = Transformation(d,d_plus,thetaDes(:,t),theta_plus,a,alpha);
        
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

        T0_h = Transformation(d,d_plus,thetaFbk(:,t),theta_plus,a,alpha);
        
        A_fbk = T0_h(1:3,1:3,end);
        
        linkDim = T0_h(1:3,4,:);

        if mount_num > 0
            A_fbk = A_fbk*rot_z45;
            
            mountDim = A_fbk*[0;0;mount_length];
            linkDim(:,:,8) = T0_h(1:3,4,end)+mountDim;
            if mount_num > 1
                mountDim = A_fbk*[0;0;mount_length];
                linkDim(:,:,9) = T0_h(1:3,4,end)+mountDim;
            end
        end
        
        P_fbk(:,t)  = T0_h(1:3,4,end)+mountDim;
        
%         six = [ T0_h(1:3,4,end)+mountDim T0_h(1:3,4,end)+mountDim+ee_axes_length*A_fbk(:,1)];
%         siy = [ T0_h(1:3,4,end)+mountDim T0_h(1:3,4,end)+mountDim+ee_axes_length*A_fbk(:,2)];
%         siz = [ T0_h(1:3,4,end)+mountDim T0_h(1:3,4,end)+mountDim+ee_axes_length*A_fbk(:,3)];

        hold(animPlot,'on')
        plot3(animPlot,P_des(1,1:time_factor:end),P_des(2,1:time_factor:end),P_des(3,1:time_factor:end),':c')
        plot3(animPlot,P_fbk(1,1:time_factor:t),P_fbk(2,1:time_factor:t),P_fbk(3,1:time_factor:t),'m')
        
        nv1 = T0_h(:,:,1)*V1;
        nv2 = T0_h(:,:,2)*V2;
        nv3 = T0_h(:,:,3)*V3;
        nv4 = T0_h(:,:,4)*V4;
        nv5 = T0_h(:,:,5)*V5;
        nv6 = T0_h(:,:,6)*V6;
        nv7 = T0_h(:,:,7)*V7;

        set(p1,'Vertices',nv1(1:3,:)')
        set(p2,'Vertices',nv2(1:3,:)')
        set(p3,'Vertices',nv3(1:3,:)')
        set(p4,'Vertices',nv4(1:3,:)')
        set(p5,'Vertices',nv5(1:3,:)')
        set(p6,'Vertices',nv6(1:3,:)')
        set(p7,'Vertices',nv7(1:3,:)')
                
        hold(animPlot,'off')
        
        set(animPlot,	'Projection','perspective', ...
                        'PlotBoxAspectRatio',[1 1 1], ...
                        'DataAspectRatio',[1 1 1], ...
                        'XLim',[limDim(1,1) limDim(1,2)], ...
                        'YLim',[limDim(2,1) limDim(2,2)], ...
                        'ZLim',[limDim(3,1) limDim(3,2)]);

        drawnow;
    end
    
    function T0_h = Transformation(d,d_plus,theta,theta_plus,a,alpha)

        trnZ_x      = zeros(4,4,n);
        rotZ_x      = zeros(4,4,n);
        trnX_i      = zeros(4,4,n);
        rotX_i      = zeros(4,4,n);

        A_h         = zeros(4,4,n+1);
        A_h(:,:,1)  = eye(4);

        for i = 1:n

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

            trnZ_h = trnZ_x;
            rotZ_h = rotZ_x;

            % A_i = % Trn_(z_(i-1),d_i)*Rot_(z_(i-1),theta_i)*Trn_(x_i,a_i)*Rot_(x_i,alpha_i)
            A_h(:,:,i+1) = trnZ_h(:,:,i)*rotZ_h(:,:,i)*trnX_i(:,:,i)*rotX_i(:,:,i);        
        end

        % Transformation n to 0
        T0_h(:,:,1) = A_h(:,:,1);
        for i = 1:n
            T0_h(:,:,i+1) = T0_h(:,:,i)*A_h(:,:,i+1);
        end

    end
end