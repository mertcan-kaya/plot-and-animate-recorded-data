function print_info(info_table)

    global cycle_time robot_type mount_num
    
    cycle_time  = sum((info_table{:,2}.*contains(info_table{:,1},'cycle_time')));
    robot_type  = sum((info_table{:,2}.*contains(info_table{:,1},'robot_type')));
    mount_num   = sum((info_table{:,2}.*contains(info_table{:,1},'mount_num')));
    getAlgo     = sum((info_table{:,2}.*contains(info_table{:,1},'algo_num')));
    wrkSpc      = sum((info_table{:,2}.*contains(info_table{:,1},'work_space')));
	if wrkSpc == 1
		eulerSet = sum((info_table{:,2}.*contains(info_table{:,1},'euler_set')));
	end
    trajectory  = sum((info_table{:,2}.*contains(info_table{:,1},'trj_type')));
    sub_trj     = sum((info_table{:,2}.*contains(info_table{:,1},'trj_pnt2pnt')));
    custom      = sum((info_table{:,2}.*contains(info_table{:,1},'custom_trj')));
    prcntVel    = sum((info_table{:,2}.*contains(info_table{:,1},'prcnt_vel')));
    prcntAcc    = sum((info_table{:,2}.*contains(info_table{:,1},'prcnt_acc')));
    recType 	= sum((info_table{:,2}.*contains(info_table{:,1},'rec_type')));
    kalman_v 	= sum((info_table{:,2}.*contains(info_table{:,1},'kalman_v')));
    q_v         = sum((info_table{:,2}.*contains(info_table{:,1},'kalman_qv')));
    r_v         = sum((info_table{:,2}.*contains(info_table{:,1},'kalman_rv')));
    shoulder    = sum((info_table{:,2}.*contains(info_table{:,1},'kin_shldr')));
    elbow       = sum((info_table{:,2}.*contains(info_table{:,1},'kin_elbow')));
    wrist       = sum((info_table{:,2}.*contains(info_table{:,1},'kin_wrist')));
    start_delay = sum((info_table{:,2}.*contains(info_table{:,1},'start_delay')));
    stop_delay 	= sum((info_table{:,2}.*contains(info_table{:,1},'stop_delay')));
    if getAlgo == 2
        ctrlMthd	= sum((info_table{:,2}.*contains(info_table{:,1},'ctrl_mthd')));        
        fuzzy_joint = sum((info_table{:,2}.*contains(info_table{:,1},'fuzzy_joint')));
        fuzzy_task  = sum((info_table{:,2}.*contains(info_table{:,1},'fuzzy_task')));
        fuzzy_force = sum((info_table{:,2}.*contains(info_table{:,1},'fuzzy_force')));
        if ctrlMthd ~= 3
            if fuzzy_joint == 1 || fuzzy_task == 1 || fuzzy_force == 1
                fis_type    = sum((info_table{:,2}.*contains(info_table{:,1},'fis_type')));
                GE(1)       = sum((info_table{:,2}.*contains(info_table{:,1},'GE1')));
                GE(2)       = sum((info_table{:,2}.*contains(info_table{:,1},'GE2')));
                GE(3)       = sum((info_table{:,2}.*contains(info_table{:,1},'GE3')));
                GE(4)       = sum((info_table{:,2}.*contains(info_table{:,1},'GE4')));
                GE(5)       = sum((info_table{:,2}.*contains(info_table{:,1},'GE5')));
                GE(6)       = sum((info_table{:,2}.*contains(info_table{:,1},'GE6')));
                GCE(1)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCE1')));
                GCE(2)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCE2')));
                GCE(3)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCE3')));
                GCE(4)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCE4')));
                GCE(5)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCE5')));
                GCE(6)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCE6')));
                GU(1)       = sum((info_table{:,2}.*contains(info_table{:,1},'GU1')));
                GU(2)       = sum((info_table{:,2}.*contains(info_table{:,1},'GU2')));
                GU(3)       = sum((info_table{:,2}.*contains(info_table{:,1},'GU3')));
                GU(4)       = sum((info_table{:,2}.*contains(info_table{:,1},'GU4')));
                GU(5)       = sum((info_table{:,2}.*contains(info_table{:,1},'GU5')));
                GU(6)       = sum((info_table{:,2}.*contains(info_table{:,1},'GU6')));
                GCU(1)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCU1')));
                GCU(2)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCU2')));
                GCU(3)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCU3')));
                GCU(4)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCU4')));
                GCU(5)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCU5')));
                GCU(6)      = sum((info_table{:,2}.*contains(info_table{:,1},'GCU6')));
            else
                if ctrlMthd ~= 1
                    Kp(1)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_motion1')));
                    Kp(2)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_motion2')));
                    Kp(3)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_motion3')));
                    Kp(4)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_motion4')));
                    Kp(5)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_motion5')));
                    Kp(6)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_motion6')));
                    Ki(1)       = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_motion1')));
                    Ki(2)       = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_motion2')));
                    Ki(3)       = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_motion3')));
                    Ki(4)       = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_motion4')));
                    Ki(5)       = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_motion5')));
                    Ki(6)       = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_motion6')));
                    Kd(1)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_motion1')));
                    Kd(2)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_motion2')));
                    Kd(3)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_motion3')));
                    Kd(4)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_motion4')));
                    Kd(5)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_motion5')));
                    Kd(6)       = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_motion6')));
                end
                if ctrlMthd ~= 0
                    Kp_ft(1)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_sensor1')));
                    Kp_ft(2)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_sensor2')));
                    Kp_ft(3)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_sensor3')));
                    Kp_ft(4)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_sensor4')));
                    Kp_ft(5)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_sensor5')));
                    Kp_ft(6)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kp_sensor6')));
                    Ki_ft(1)    = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_sensor1')));
                    Ki_ft(2)    = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_sensor2')));
                    Ki_ft(3)    = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_sensor3')));
                    Ki_ft(4)    = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_sensor4')));
                    Ki_ft(5)    = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_sensor5')));
                    Ki_ft(6)    = sum((info_table{:,2}.*contains(info_table{:,1},'Ki_sensor6')));
                    Kd_ft(1)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_sensor1')));
                    Kd_ft(2)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_sensor2')));
                    Kd_ft(3)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_sensor3')));
                    Kd_ft(4)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_sensor4')));
                    Kd_ft(5)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_sensor5')));
                    Kd_ft(6)    = sum((info_table{:,2}.*contains(info_table{:,1},'Kd_sensor6'))); 
                    
                    switch_cond = sum((info_table{:,2}.*contains(info_table{:,1},'switch_cond')));
                    ft_bDes(1)  = sum((info_table{:,2}.*contains(info_table{:,1},'ft_base_des1')));
                    ft_bDes(2)  = sum((info_table{:,2}.*contains(info_table{:,1},'ft_base_des2')));
                    ft_bDes(3)  = sum((info_table{:,2}.*contains(info_table{:,1},'ft_base_des3')));
                    ft_bDes(4)  = sum((info_table{:,2}.*contains(info_table{:,1},'ft_base_des4')));
                    ft_bDes(5)  = sum((info_table{:,2}.*contains(info_table{:,1},'ft_base_des5')));
                    ft_bDes(6)  = sum((info_table{:,2}.*contains(info_table{:,1},'ft_base_des6')));
                    force_ff    = sum((info_table{:,2}.*contains(info_table{:,1},'force_ff')));
                    force_tVel  = sum((info_table{:,2}.*contains(info_table{:,1},'force_tsk_vel')));
                end
            end
        else
            Lambda(1)   = sum((info_table{:,2}.*contains(info_table{:,1},'Lambda1')));
            Lambda(2)   = sum((info_table{:,2}.*contains(info_table{:,1},'Lambda2')));
            Lambda(3)   = sum((info_table{:,2}.*contains(info_table{:,1},'Lambda3')));
            Lambda(4)   = sum((info_table{:,2}.*contains(info_table{:,1},'Lambda4')));
            Lambda(5)   = sum((info_table{:,2}.*contains(info_table{:,1},'Lambda5')));
            Lambda(6)   = sum((info_table{:,2}.*contains(info_table{:,1},'Lambda6')));
            B(1)        = sum((info_table{:,2}.*contains(info_table{:,1},'B1')));
            B(2)        = sum((info_table{:,2}.*contains(info_table{:,1},'B2')));
            B(3)        = sum((info_table{:,2}.*contains(info_table{:,1},'B3')));
            B(4)        = sum((info_table{:,2}.*contains(info_table{:,1},'B4')));
            B(5)        = sum((info_table{:,2}.*contains(info_table{:,1},'B5')));
            B(6)        = sum((info_table{:,2}.*contains(info_table{:,1},'B6')));
            K(1)        = sum((info_table{:,2}.*contains(info_table{:,1},'K1')));
            K(2)        = sum((info_table{:,2}.*contains(info_table{:,1},'K2')));
            K(3)        = sum((info_table{:,2}.*contains(info_table{:,1},'K3')));
            K(4)        = sum((info_table{:,2}.*contains(info_table{:,1},'K4')));
            K(5)        = sum((info_table{:,2}.*contains(info_table{:,1},'K5')));
            K(6)        = sum((info_table{:,2}.*contains(info_table{:,1},'K6')));
        end
        f_vec       = sum((info_table{:,2}.*contains(info_table{:,1},'f_vector')));
        g_vec       = sum((info_table{:,2}.*contains(info_table{:,1},'g_vector')));
        s_vec       = sum((info_table{:,2}.*contains(info_table{:,1},'s_vector')));
        ctrlSpc 	= sum((info_table{:,2}.*contains(info_table{:,1},'ctrl_spc')));
        ctrlType 	= sum((info_table{:,2}.*contains(info_table{:,1},'ctrl_type')));
        predictive 	= sum((info_table{:,2}.*contains(info_table{:,1},'predictive')));
        prdct_f 	= sum((info_table{:,2}.*contains(info_table{:,1},'prdctv_f')));
        prdct_g 	= sum((info_table{:,2}.*contains(info_table{:,1},'prdctv_g')));
        prdct_s     = sum((info_table{:,2}.*contains(info_table{:,1},'prdctv_s')));
        prdct_ne 	= sum((info_table{:,2}.*contains(info_table{:,1},'prdctv_ne')));
        if f_vec == 1
            f_mod       = sum((info_table{:,2}.*contains(info_table{:,1},'frctn_mod')));
        end
		if s_vec == 1
			s_mod       = sum((info_table{:,2}.*contains(info_table{:,1},'spring_mod')));
		end
    end    
    tool_mass   = sum((info_table{:,2}.*contains(info_table{:,1},'tool_mass')));
    tool_CoM_x 	= sum((info_table{:,2}.*contains(info_table{:,1},'tool_com1')));
    tool_CoM_y 	= sum((info_table{:,2}.*contains(info_table{:,1},'tool_com2')));
    tool_CoM_z 	= sum((info_table{:,2}.*contains(info_table{:,1},'tool_com3')));
    
    fprintf('Robot Type\t: ')
    if robot_type == 0
        fprintf('TX90\n');
    elseif robot_type == 1
        fprintf('RX160\n');
    elseif robot_type == 2
        fprintf('RX160L\n');
    else
        fprintf('ERROR!');
    end
    
    if cycle_time >= 0.1
        fprintf('Cycle time\t: %.1f\n\n', cycle_time);
    else
        fprintf('Cycle time\t: %.3f\n\n', cycle_time);
    end

    fprintf('Algorithm\t: ')
    if getAlgo == 1 
        disp('Position-Velocity Mode')
    else
        disp('Torque Mode')
    end

    fprintf('Work space\t: ')
    if wrkSpc == 0 
        disp('Joint space')
    else
        disp('Task space')
    end

    fprintf('Trajectory\t: ')
    switch trajectory
        case 0
            disp('Point-to-Point')

            fprintf('P2P type\t: ')
            switch sub_trj
                case 0
                    disp('Linear')
                case 1
                    disp('Cubic')
                case 2
                    disp('Quintic')
                case 3
                    disp('Bang-bang')
                case 4
                    disp('Trapezoidal')
                case 5
                    disp('Smooth Trapezoidal')
                otherwise
                    disp('ERROR: p2p trajectory')
            end
        case 1
            disp('Via Points')

            if custom == 1
                fprintf('Custom trajectory\t: ')
                disp('1')
            end
        case 2
            disp('Sinusoidal')
        case 3
            disp('Parametric')

            fprintf('Parametric trj type\t: ')
            switch sub_trj
                case 0
                    disp('Circle XY')
                case 1
                    disp('Circle XZ')
                case 2
                    disp('EIGHT XY')
                case 3
                    disp('EIGHT XZ')
                otherwise
                    disp('ERROR: parametric trajectory')
            end
        otherwise
            disp('ERROR: trajectory')
    end

    fprintf('\n')
    fprintf('Percent vel\t: %.2f\n', prcntVel)
    fprintf('Percent acc\t: %.2f\n', prcntAcc)

    if wrkSpc == 1 
		fprintf('Euler set\t: ')	
		if eulerSet == 0
			disp('ZXZ')
		elseif eulerSet == 1
			disp('ZYZ')
		elseif eulerSet == 2
			disp('ZYX')
		elseif eulerSet == 3
			disp('XYZ')
		else
			disp('ERROR')
		end
    end

    fprintf('\nRecord Type\t: ')
    if recType == 0
        fprintf('INMTN\n')
    else
        fprintf('ENBLD\n')
    end
    
    if getAlgo == 2
        fprintf('\n')
        fprintf('Friction Model\t: ')
		if f_vec == 1
			if f_mod == 0
				fprintf('SERHAT\n')
			elseif f_mod == 1
				fprintf('OMER\n')
			elseif f_mod == 2
				fprintf('PSO\n')
			elseif f_mod == 3
				fprintf('LSE\n')
			else
				fprintf('ERROR\n')
			end
		else
			fprintf('NONE\n')
		end

        fprintf('Spring Model\t: ')		
		if s_vec == 1
			if s_mod == 0
				fprintf('SERHAT\n')
			else
				fprintf('OMER\n')
			end
		else
			fprintf('NONE\n')
		end
    end
    
    fprintf('\n')
    fprintf('Sensor Mounted\t: ')
    if mount_num >= 1
        fprintf('ON\n')        
    else
        fprintf('OFF\n')
    end
    
    fprintf('Tool Mounted\t: ')
    if mount_num >= 2
        fprintf('ON\n')
    else
        fprintf('OFF\n')
    end
        
    fprintf('Load Mounted\t: ')
    if mount_num >= 3
        fprintf('ON\n')
    else
        fprintf('OFF\n')
    end
    
    if mount_num > 1
        fprintf('\n')
        fprintf('Tool Mass\t: %f\n', tool_mass)
        fprintf('Tool CoM\t: %f, %f, %f\n', tool_CoM_x, tool_CoM_y, tool_CoM_z)
    end
    
    if getAlgo == 2
        
        fprintf('\nDynamic factors\n')
        if f_vec == 0
            fprintf('\tFriction\t\t: OFF\n')
        else
            fprintf('\tFriction\t\t: ON\n')
        end

        if g_vec == 0
            fprintf('\tGravity\t\t\t: OFF\n')
        else
            fprintf('\tGravity\t\t\t: ON\n')
        end

        if s_vec == 0
            fprintf('\tSpring\t\t\t: OFF\n')
        else
            fprintf('\tSpring\t\t\t: ON\n')
        end

        fprintf('\nControl parameters\n')
        if ctrlMthd == 0
            fprintf('\tControl Method\t: Motion Control\n')
        elseif ctrlMthd == 1
            fprintf('\tControl Method\t: Force Control\n')
        elseif ctrlMthd == 2
            fprintf('\tControl Method\t: Hybrid Control\n')
        elseif ctrlMthd == 3
            fprintf('\tControl Method\t: Impedance Control\n')
        end

        if ctrlSpc == 0
            fprintf('\tControl Space\t: Joint Space\n')
        else
            fprintf('\tControl Space\t: Task Space\n')
        end

        if ctrlType == 0
            fprintf('\tControl Type\t: PID\n')
        else
            fprintf('\tControl Type\t: CTM\n')
        end

        if predictive == 0
            fprintf('\tPredictive\t\t: OFF\n')
        else
            fprintf('\tPredictive\t\t: ON\n')

            if prdct_f == 0
                fprintf('\tFriction\t\t: OFF\n')
            else
                fprintf('\tFriction\t\t: ON\n')
            end

            if ctrlType == 0
                if prdct_g == 0
                    fprintf('\tGravity\t\t: OFF\n')
                else
                    fprintf('\tGravity\t\t: ON\n')
                end
            else
                fprintf('\tGravity\t\t: INVALID\n')
            end

            if prdct_s == 0
                fprintf('\tSpring\t\t: OFF\n')
            else
                fprintf('\tSpring\t\t: ON\n')
            end

            if ctrlType == 1
                if prdct_ne == 0
                    fprintf('\tN-E\t\t: OFF\n')
                else
                    fprintf('\tN-E\t\t: ON\n')
                end
            else
                fprintf('\tN-E\t\t: INVALID\n')
            end
        end

        fprintf('\nFilters\n')
        if kalman_v == 0
            fprintf('\tKalman (Velocity)\t: OFF\n')
        else
            fprintf('\tKalman (Velocity)\t: ON\n')
            fprintf('\t\tq_v: %f\n', q_v)
            fprintf('\t\tr_v: %f\n', r_v)
        end

        fprintf('\nRobot configs\n')
        fprintf('\tShoulder\t: ')
        switch shoulder
            case 0
                fprintf('SSAME\n')
            case 1
                fprintf('LEFTY\n')
            case 2
                fprintf('RIGHTY\n')
            case 3
                fprintf('SFREE\n')
            otherwise
        end

        fprintf('\tElbow\t\t: ')
        switch elbow
            case 0
                fprintf('PNSAME\n')
            case 1
                fprintf('POSITIVE\n')
            case 2
                fprintf('NEGATIVE\n')
            case 3
                fprintf('PNFREE\n')
            otherwise
        end

        fprintf('\tWrist\t\t: ')
        switch wrist
            case 0
                fprintf('PNSAME\n')
            case 1
                fprintf('POSITIVE\n')
            case 2
                fprintf('NEGATIVE\n')
            case 3
                fprintf('PNFREE\n')
            otherwise
        end

        fprintf('\n')
        fprintf('Start delay\t: %f\n',start_delay)
        fprintf('Stop delay\t: %f\n',stop_delay)
        if ctrlMthd ~= 3
            if fuzzy_joint == 1 || fuzzy_task == 1 || fuzzy_force == 1
                fprintf('\n')
                if fuzzy_joint == 1
                    fprintf('Fuzzy joint\t\t\t: ON\n')
                end
                if fuzzy_task == 1
                    fprintf('Fuzzy task\t\t\t: ON\n')
                end
                if fuzzy_force == 1
                    fprintf('Fuzzy force\t\t\t: ON\n')
                end

                if fis_type == 0
                    fprintf('FIS type\t\t\t: Linear\n')
                elseif fis_type == 1
                    fprintf('FIS type\t\t\t: Nonlinear1\n')
                elseif fis_type == 2
                    fprintf('FIS type\t\t\t: Nonlinear2\n')
                elseif fis_type == 3
                    fprintf('FIS type\t\t\t: SEVENMF\n')
                end

                fprintf('\nFuzzy gains:')
                fprintf('\nGE\t\tGCE\t\tGU\t\tGCU\n')
                disp('-------------------------------')
                for i = 1:6
                    fprintf('%.3f\t%.3f\t%.3f\t%.3f\n',GE(i),GCE(i),GU(i),GCU(i))
                end
            else
                if ctrlMthd ~= 1
                    fprintf('\nMotion gains:')
                    fprintf('\nKp\t\t\tKi\t\t\tKd\n')
                    disp('-------------------------------')
                    for i = 1:6
                        if Kp(i) < 1000
                            fprintf('%.3f\t\t%.3f\t\t%.3f\n',Kp(i),Ki(i),Kd(i))
                        else
                            fprintf('%.3f\t%.3f\t\t%.3f\n',Kp(i),Ki(i),Kd(i))
                        end
                    end
                end

                if ctrlMthd ~= 0
                    fprintf('\nForce/Torque gains:')
                    fprintf('\nKp\t\t\tKi\t\t\tKd\n')
                    disp('-------------------------------')
                    for i = 1:6
                        if Kp_ft(i) < 1000
                            fprintf('%.3f\t\t%.3f\t\t%.3f\n',Kp_ft(i),Ki_ft(i),Kd_ft(i))
                        else
                            fprintf('%.3f\t%.3f\t\t%.3f\n',Kp_ft(i),Ki_ft(i),Kd_ft(i))
                        end
                    end
                    
                    fprintf('\n')
                    fprintf('Desired Force\t: ')
                    for i = 1:6
                        if abs(ft_bDes(i)) > 0
                            f_des = ft_bDes(i);
                            if i == 1
                                fprintf('Fx: %f\n', ft_bDes(i))
                            end
                            if i == 2
                                fprintf('Fy: %f\n', ft_bDes(i))
                            end
                            if i == 3
                                fprintf('Fz: %f\n', ft_bDes(i))
                            end
                        end
                    end

                    fprintf('\n')
                    fprintf('Switch cond\t: %f\n', switch_cond)
                end                
            end
        else
            fprintf('\nImpedance gains:')
            fprintf('\nLambda\t\tB\t\t\tK\n')
            disp('-------------------------------')
            for i = 1:6
%                 if K(i) < 1000
%                     fprintf('%.3f\t\t%.3f\t\t%.3f\n',Lambda(i),B(i),K(i))
%                 else
                    fprintf('%.3f\t%.3f\t\t%.3f\n',Lambda(i),B(i),K(i))
%                 end
            end
        end

    end
    
end