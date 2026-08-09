%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //           版本|   日 期    |开发人员| 改版描述
% // 当前版本：V1.0| 2024.3.15  | 朱安源 | 生成装车路径

function [feasible_WP4, WayPoints, WayJoints, tArray, time]  = GenWP(debug_mode, excavator_param, loadingFlag,unload_biasy, truck_array, attitude_angle, start_joint, unload_point, ...
    unload_joint, middleUP, WJS_vel, feasible)

WayPoints = zeros(3,6);
WayJoints = zeros(4,6);
tArray = [ 0; 1; 2; 3; 4; 5];
time = zeros(1000, 1);
feasible_WP4 = 0;
DEBUG_PRINT = debug_mode;  % ← 代码生成前改为 false

if loadingFlag == 1 && feasible == 1
    if DEBUG_PRINT
        fprintf('\n[TP] === GenWP called ===\n');
        fprintf('[TP] loadingFlag=%.0f  feasible=%.0f\n', loadingFlag, feasible);
        fprintf('[TP] start_joint =[sw=%.2f, bm=%.2f, ar=%.2f, bk=%.2f]\n', start_joint(1), start_joint(2), start_joint(3), start_joint(4));
        fprintf('[TP] unload_point=[%.3f, %.3f, %.3f]\n', unload_point(1), unload_point(2), unload_point(3));
        fprintf('[TP] unload_joint=[sw=%.2f, bm=%.2f, ar=%.2f, bk=%.2f]\n', unload_joint(1), unload_joint(2), unload_joint(3), unload_joint(4));
        fprintf('[TP] middleUP=[%.3f, %.3f]\n', middleUP(1,1), middleUP(1,2));
    end
    if WJS_vel(2) == 0 | WJS_vel(4) == 0 | WJS_vel(5) == 0 | WJS_vel(6) == 0 | WJS_vel(7) == 0
        WJS_vel = [1.5; 7.5; 25; 14; 14; 15; 50; 2.5; 2; 15; 0.125;0.5;0.5;100;10;15;6;10;100;0.5;2;0.5;0.5;6];
        fprintf('default loading velocities!')
    end
    WP2_Z1            = WJS_vel(1); %WP2提动臂，铲斗铰点到参考高度 ,         G2_z, 1.5m
    WP2_vel_boom      = WJS_vel(2); %WP2提升动臂速度，°/s,                   H2_x, 7.5 °/s
    WP3_swingJoint    = WJS_vel(3); %WP3回转到矿卡车框边位置的角度临界值，>=15°,   H2_y, 15°
    WP3_vel_swing     = WJS_vel(4); %WP3回转速度，°/s,                       H2_z，14 °/s 
    WP4_vel_swing     = WJS_vel(5); %WP4回转速度，°/s,                       I2_x, 14 °/s
    WP4_vel_arm       = WJS_vel(6); %WP4斗杆卸载速度，°/s,                   I2_y，15°/s
    WP5_vel_arm       = WJS_vel(10);%WP5斗杆卸载速度，°/s,                  G3_x，15°/s
    WP5_vel_bkt       = WJS_vel(7); %WP5铲斗卸载速度，°/s,                   I2_z, 50 °/s 尽量使铲斗卸载速度加大
    WP4_heightBias    = WJS_vel(8); %WP4铲斗铰点距矿车平面高度               I1_y, 2.5m
    WP3_Z1            = WJS_vel(9); %WP3回转过程中提动臂需要提到的高度       I1_z, 2m
    WP3_vel_swing_cof = WJS_vel(11);%回转前半段回转速度计算参考系数
    P3_boomCof        = WJS_vel(12);%P2-P3阶段动臂百分比，值越小，动臂提升越早。0表示P3时直接提升到P4高度；1时表示P2-P3阶段不提升动臂  G4_x, 0.5
    P3_armCof         = WJS_vel(13);%P2-P3阶段斗杆百分比，值越小，斗杆卸载越早。0表示P3时直接卸载到P4位置；1时表示P2-P3阶段不进行斗杆卸载  G4_y, 0.5
    P3maxSwing        = WJS_vel(14);%P2-P3阶段用于映射回转速度的最大回转角度 G4_z, 100°
    P3minSwing        = WJS_vel(15);%P2-P3阶段用于映射回转速度的最小回转角度  H4_x, 10°
    maxP3_SwingVel    = WJS_vel(16);%P2-P3阶段在最大回转角度时对应的最大回转速度  H4_y, 15°/s
    minP3_SwingVel    = WJS_vel(17);%P2-P3阶段在最小回转角度时对应的最小回转速度   H4_z, 6°/s
    P3minZ0_Swing     = WJS_vel(18);%挖掘结束后计算映射最小提升高度时对应的回转角 I4_x，10°
    P3maxZ0_Swing     = WJS_vel(19);%挖掘结束后计算映射最大提升高度时对应的回转角I4_y，100°
    max_boomLift24    = WJS_vel(20);%最小提升高度相对于挖掘终止点动臂角度差值  I4_z，30°
    truckPlane_bias   = WJS_vel(21);%最大提升高度相较于矿卡平面高度差     I5_x，2m
    P5_boomCof        = WJS_vel(22);%P5到车框内部时动臂角度系数百分比，0-1，值越小，越靠近卸载点角度，为0时直接到卸载角度  I5_y, 0.5
    P5_armCof         = WJS_vel(23);%P5到车框内部时斗杆角度系数百分比，0-1，值越小，越靠近卸载点角度，为0时直接到卸载角度  T5_z, 0.5
    WP5_vel_boom      = WJS_vel(24);%P5阶段动臂参考速度  H5_x, 6
    
    truck_center = truck_array(1:3);
    truck_size = truck_array(4:6);
    headingAngle = truck_array(7);
    truckPlane = truck_center(3);
    time = zeros(1000,1);
    boom_limit= [-45.44+2, 55.28-2];
    arm_limit = [-138.69+2, -37.35-2];
    bucket_limit = [-133.48+3, 30.35-3];
    
    % if calculate_angle_diff(start_joint(1), unload_joint(1), 1) <= calculate_angle_diff(start_joint(1), unload_joint(1), -1)%卸载时顺时针回转角度小于逆时针回转时，回转方向为顺时针
    %     swing_direction = 1 * swing_direction;
    % else
    %     swing_direction = -1 * swing_direction;
    % end
    
    %%
    % WP1轨迹起点，时间0，挖掘终止点
    swingAngle1 = start_joint(1);
    boomAngle1 = start_joint(2);
    armAngle1 = start_joint(3);
    bucketAngle1 = start_joint(4);
    if boomAngle1 + armAngle1 + bucketAngle1 >= attitude_angle
        bucketAngle1 = attitude_angle - boomAngle1 - armAngle1;
    end
    WJ1 = [swingAngle1, boomAngle1, armAngle1, bucketAngle1];
    excavator_param_WP1 = excavator_param; excavator_param_WP1(3) = 0; 
    [X0, Y0, Z0, ~] = Kinematic_forward_solution(WJ1, excavator_param_WP1);
    t1 = 0;
    WP1 = [X0, Y0, Z0];
    
    %%
    xe = middleUP(1, 1);
    ye = middleUP(1, 2);

    height_bias = WP4_heightBias;
    WP4 = [xe, ye, unload_point(3)+height_bias];
    excavator_param_WP2 = excavator_param; excavator_param_WP2(3) = 0;
    [feasible_WP4, jointAngle] = Inverse_kinematics_solution_jointbucket(WP4, excavator_param_WP2, 25);
    wp4_tag = 'initial';

    if feasible_WP4 == 0
        % 策略1：放宽铲斗关节角约束（不改变空间位置，物理代价最小）
        bkt_try = [20, 15, 10];
        for k_bkt = 1:3
            [feasible_WP4, jointAngle] = Inverse_kinematics_solution_jointbucket(WP4, excavator_param_WP2, bkt_try(k_bkt));
            if feasible_WP4 == 1
                wp4_tag = 'bkt_relax';
                break;
            end
        end
    end

    if feasible_WP4 == 0
        % 策略2：XY 向 unloadPoint 方向收缩（该点已被候选点 IK 验证可达）
        %   每次朝目标移动 25%，最多 3 步（到 75% 处）
        for k_xy = 1:3
            ratio = 0.25 * k_xy;
            WP4(1) = xe + ratio * (unload_point(1) - xe);
            WP4(2) = ye + ratio * (unload_point(2) - ye);
            [feasible_WP4, jointAngle] = Inverse_kinematics_solution_jointbucket(WP4, excavator_param_WP2, 25);
            if feasible_WP4 == 1
                wp4_tag = 'xy_shrink';
                break;
            end
        end
    end

    swingAngle4 = jointAngle(1);
    boomAngle4 = jointAngle(2);
    armAngle4 = jointAngle(3);

    if DEBUG_PRINT
        fprintf('[TP] WP4 IK: feasible=%.0f  tag=%s  WP4=[%.3f, %.3f, %.3f]\n', feasible_WP4, wp4_tag, WP4(1), WP4(2), WP4(3));
        fprintf('[TP]   jointAngle4=[sw=%.2f, bm=%.2f, ar=%.2f, bk=%.2f]\n', jointAngle(1), jointAngle(2), jointAngle(3), jointAngle(4));
        if feasible_WP4 == 0
            fprintf('[TP] WARNING: WP4 infeasible after all fallbacks, using default joints!\n');
        end
    end
    
    % WP3 swing：最短路径弧中点（处理 0°/360° 跨界）
    % 将 swingAngle4 展开到 WJ1(1) 的同一周期，保证 |sw4_unwrapped - sw1| <= 180°
    swing_diff_14 = signed_shortest_diff_deg(WJ1(1), swingAngle4);  % sw1→sw4 最短有向弧
    sw4_unwrapped = WJ1(1) + swing_diff_14;
    dir_14        = sign(swing_diff_14);   % 回转方向（+1=CCW, -1=CW, 0=重合）
    half_arc      = swing_diff_14 / 2;

    if abs(half_arc) <= WP3_swingJoint
        % 中点距 WP4 过近（半弧 <= 最小步长）→ 从 WP4 朝起点方向退一步
        swingAngle3 = sw4_unwrapped - dir_14 * WP3_swingJoint;
    else
        swingAngle3 = WJ1(1) + half_arc;
    end
    % 归一化到 [0, 360)
    swingAngle3 = mod(swingAngle3, 360);
    
    %% 
    swingAngle2 = swingAngle1;

    % 预计算最短路径回转差（供后续 boom 映射、速度映射、时间计算使用）
    swing_diff_23 = abs(signed_shortest_diff_deg(swingAngle2, swingAngle3));  % |WP2→WP3|
    swing_diff_34 = abs(signed_shortest_diff_deg(swingAngle3, swingAngle4));  % |WP3→WP4|

    if DEBUG_PRINT
        fprintf('[TP] WP3 swing: sw1=%.2f -> sw3=%.2f -> sw4=%.2f\n', swingAngle1, swingAngle3, swingAngle4);
        fprintf('[TP]   swing_diff_14=%.2f  dir=%.0f  swing_diff_23=%.2f  swing_diff_34=%.2f\n', ...
            swing_diff_14, dir_14, swing_diff_23, swing_diff_34);
    end

    boomAngle2_low = max(boomAngle4 - max_boomLift24, boomAngle1);  
    boomAngle2_high = boomAngle4;

    if swing_diff_23 <= P3minZ0_Swing %P3minZ0_Swing为P2-P3阶段回转设定的最小范围阈值，假定为10°，在P2-P3回转角度小于10°时，P2的参考位置由矿卡高度位置决定，目的是直接提升到高于矿卡
        boomAngle2 = boomAngle2_high;
    elseif swing_diff_23 >= P3maxZ0_Swing
        boomAngle2 = boomAngle2_low;
    else
        boomAngle2 = interp1([P3maxZ0_Swing, P3minZ0_Swing], [boomAngle2_low, boomAngle2_high], swing_diff_23);
    end

    armAngle2   = jointAngle(3);
    if boomAngle2 <= WJ1(2)
        boomAngle2 = WJ1(2) + 1;
    end
    armAngle2 = armAngle1 + (boomAngle1 - boomAngle2);
    
    if armAngle2 <= WJ1(3)
        armAngle2 = WJ1(3);
    end
    
    if WJ1(4) + boomAngle2 + armAngle2 >= attitude_angle 
        bktangle2  = attitude_angle - boomAngle2- armAngle2;
        bktAngle2  = jointInScope(bktangle2, bucket_limit);
    else
        bktAngle2 = WJ1(4);
    end
    
    WJ2 = [swingAngle2, boomAngle2, armAngle2, bktAngle2];
    excavator_param_WP2 = excavator_param; excavator_param_WP2(3) = 0;
    [X2, Y2, Z2, ~] = Kinematic_forward_solution(WJ2, excavator_param_WP2);
    WP2 = [X2, Y2, Z2];
    
    t2 = abs(boomAngle2 - start_joint(2)) / WP2_vel_boom;
    t2 = t2 + t1;
    %%
    boomAngle3 = P3_boomCof*boomAngle2 + (1-P3_boomCof)*boomAngle4;% P3_boomCof=0.5。表示在P3位置动臂提升百分比，值越小，动臂提升越早。0表示P3时直接提升到P4高度；1时表示P2-P3阶段不提升动臂
    if boomAngle3 < boomAngle2
        boomAngle3 = boomAngle2;
    end
    armAngle3 = P3_armCof*armAngle2 + (1-P3_armCof)*armAngle4;%P3_armCof=0.5。表示在P3位置斗杆卸载百分比，值越小，斗杆卸载越早
    if armAngle3 < armAngle2
        armAngle3 = armAngle2;%取消斗杆内收动作
    end

    if bktAngle2 + boomAngle3 + armAngle3 >= attitude_angle%取消多余铲斗内收
        bktangle3   = attitude_angle - boomAngle3- armAngle3;
        bktAngle3   = jointInScope(bktangle3, bucket_limit);
    else
        bktAngle3 = bktAngle2;
    end

    WJ3 = [swingAngle3, boomAngle3, armAngle3, bktAngle3];
    excavator_param_WP3 = excavator_param; excavator_param_WP3(3) = 0;
    [X3, Y3, Z3, ~] = Kinematic_forward_solution(WJ3, excavator_param_WP3);
    WP3 = [X3, Y3, Z3];

    if swing_diff_23 >= P3maxSwing
        WP3_vel_swing = maxP3_SwingVel;
    elseif swing_diff_23 <= P3minSwing
        WP3_vel_swing = minP3_SwingVel;
    else
        WP3_vel_swing = interp1([P3maxSwing, P3minSwing], [maxP3_SwingVel, minP3_SwingVel], swing_diff_23);
    end

    ts3 = swing_diff_23 / WP3_vel_swing;
    tb3 = abs(boomAngle3 - boomAngle2) / WP2_vel_boom;
    t3 = max(ts3, tb3);
    t3 = t3 + t2;
    %%
    if bktAngle3 + boomAngle4 + armAngle4 >= attitude_angle
        bktangle4   = attitude_angle - boomAngle4- armAngle4;
        bktAngle4   = jointInScope(bktangle4, bucket_limit);
    else
        bktAngle4 = WJ3(4);
    end
    
    WJ4 = [swingAngle4, boomAngle4, armAngle4, bktAngle4];
    ts4 = swing_diff_34 / WP4_vel_swing; 
    ta4 = abs(armAngle4 - armAngle3) / WP4_vel_arm; 
    t4 = max(ts4, ta4);
    t4 = t4 + t3;
    
    %%
    % WP5除铲斗外其他工作装置到达卸载状态，时间t5由swing boom arm共同确定
    swingAngle5 = unload_joint(1);
    boomAngle5 = P5_boomCof*boomAngle4 + (1-P5_boomCof)*unload_joint(2);
    armAngle5 = P5_armCof*armAngle4 + (1-P5_armCof)*unload_joint(3);
    
    if WJ4(4)+boomAngle5+armAngle5 >= attitude_angle
        bktangle5   = attitude_angle - boomAngle5- armAngle5;
        bktAngle5   = jointInScope(bktangle5, bucket_limit);
    else
        bktAngle5 = WJ4(4);
    end
    if bktAngle5 <= WJ4(4)
        bktAngle5 = WJ4(4);
    end

    WJ5 = [swingAngle5, boomAngle5, armAngle5, bktAngle5];
    excavator_param_WP5 = excavator_param; excavator_param_WP5(3) = 0;
    [X5, Y5, Z5, ~] = Kinematic_forward_solution(WJ5, excavator_param_WP5);
    WP5 = [X5, Y5, Z5];
    ts5 = abs(WJ5(1) - WJ4(1)) / WP4_vel_swing;
    tb5 = abs(WJ5(2) - WJ4(2)) / WP5_vel_boom;
    ta5 = abs(WJ5(3) - WJ4(3)) / WP5_vel_arm;
    t5 = max(max(ts5, tb5), ta5);
    t5 = t5 + t4;

    %%
    % WP6为轨迹终点
    WP6 = unload_point;
    WJ6 = [unload_joint(1), unload_joint(2), unload_joint(3), unload_joint(4)];
    t6 = abs(unload_joint(4) - WJ5(4)) / WP5_vel_bkt;
    t6 = t6 + t5;
    
    WayPoints = [WP1; WP2; WP3; WP4; WP5; WP6]';
    WayJoints = [WJ1; WJ2; WJ3; WJ4; WJ5; WJ6]';
    tArray = [ t1; t2; t3; t4; t5; t6];

    if DEBUG_PRINT
        fprintf('[TP] --- Waypoints (XYZ) ---\n');
        fprintf('[TP] WP1=[%.3f, %.3f, %.3f]  t=%.2fs\n', WP1(1), WP1(2), WP1(3), t1);
        fprintf('[TP] WP2=[%.3f, %.3f, %.3f]  t=%.2fs\n', WP2(1), WP2(2), WP2(3), t2);
        fprintf('[TP] WP3=[%.3f, %.3f, %.3f]  t=%.2fs\n', WP3(1), WP3(2), WP3(3), t3);
        fprintf('[TP] WP4=[%.3f, %.3f, %.3f]  t=%.2fs\n', WP4(1), WP4(2), WP4(3), t4);
        fprintf('[TP] WP5=[%.3f, %.3f, %.3f]  t=%.2fs\n', WP5(1), WP5(2), WP5(3), t5);
        fprintf('[TP] WP6=[%.3f, %.3f, %.3f]  t=%.2fs\n', WP6(1), WP6(2), WP6(3), t6);
        fprintf('[TP] --- WayJoints (sw/bm/ar/bk deg) ---\n');
        fprintf('[TP] WJ1=[%.2f, %.2f, %.2f, %.2f]\n', WJ1(1), WJ1(2), WJ1(3), WJ1(4));
        fprintf('[TP] WJ2=[%.2f, %.2f, %.2f, %.2f]\n', WJ2(1), WJ2(2), WJ2(3), WJ2(4));
        fprintf('[TP] WJ3=[%.2f, %.2f, %.2f, %.2f]\n', WJ3(1), WJ3(2), WJ3(3), WJ3(4));
        fprintf('[TP] WJ4=[%.2f, %.2f, %.2f, %.2f]\n', WJ4(1), WJ4(2), WJ4(3), WJ4(4));
        fprintf('[TP] WJ5=[%.2f, %.2f, %.2f, %.2f]\n', WJ5(1), WJ5(2), WJ5(3), WJ5(4));
        fprintf('[TP] WJ6=[%.2f, %.2f, %.2f, %.2f]\n', WJ6(1), WJ6(2), WJ6(3), WJ6(4));
        fprintf('[TP] total time=%.2fs  feasible_WP4=%.0f\n', t6, feasible_WP4);
        fprintf('[TP] === GenWP done ===\n');
    end
    
    len = (t6 - t1) / 0.1 + 1;
    for i = 2 : round(len)
        time(i) = time(i-1) + 0.1;
    end
    
end
end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：王宵
% //	改版履历
% //           版本 |   日 期   |开发人员| 改版描述
% // 当前版本：V1.0| 2023.3.15  |  王宵  | 运动学正解程序
function [X, Y, Z, Alpha] = Kinematic_forward_solution(joint_angle,excavator_param)
% 输入参数 
% joint_angle 回转、动臂、斗杆、铲斗关节角度，为1*4的向量
% bias_x 回转中心到动臂铰点在x轴上的偏移量
% bias_z 回转中心到动臂铰点在Z轴上的偏移量
% length_boom 动臂长度
% length_arm 斗杆长度
% length_bucket 铲斗长度

% 输出参数
% X 铲斗齿尖位置X
% Y 铲斗齿尖位置Y
% Z 铲斗齿尖位置Z
% Alpha 铲斗齿尖位姿
length_boom = excavator_param(1); length_arm = excavator_param(2); length_bucket = excavator_param(3);
bias_x = excavator_param(4); bias_y = excavator_param(5); bias_z = excavator_param(6);
safe_bias = 2;
angleLimits = reshape((excavator_param(7:12)), 3,2);
boom_limit = [angleLimits(1,1) + safe_bias, angleLimits(1,2) - safe_bias]; 
arm_limit = [angleLimits(2,1) + safe_bias, angleLimits(2,2)- safe_bias]; 
bucket_limit = [angleLimits(3,1) + safe_bias, angleLimits(3,2)- safe_bias];

swing_angle = joint_angle(1); boom_angle = joint_angle(2); arm_angle = joint_angle(3); bucket_angle = joint_angle(4);
% if swing_angle > 360 || swing_angle < 0
%     fprintf("The swing angle is out of limit.")
% elseif boom_angle > boom_limit(2) || boom_angle < boom_limit(1)
%     fprintf("The boom angle is out of limit.")
% elseif arm_angle > arm_limit(2) || arm_angle < arm_limit(1)
%     fprintf("The arm_angle is out of limit.")
% elseif bucket_angle < bucket_limit(1) || bucket_angle > bucket_limit(2)
%     fprintf("The bucket_angle is out of limit.")
% end

X = cosd(swing_angle-180) * (length_bucket * cosd(boom_angle+arm_angle+bucket_angle) + ...
    length_arm * cosd(boom_angle+arm_angle) + length_boom * cosd(boom_angle) + bias_x);
Y = sind(swing_angle-180) * (length_bucket * cosd(boom_angle+arm_angle+bucket_angle) + ...
    length_arm * cosd(boom_angle+arm_angle) + length_boom * cosd(boom_angle) + bias_x);
Z = length_bucket * sind(boom_angle+arm_angle+bucket_angle) + length_arm * sind(boom_angle+arm_angle) ...
    +length_boom * sind(boom_angle) + bias_z;
Alpha = boom_angle + arm_angle + bucket_angle;

end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：王宵
% //	改版履历
% //          版本 |    日 期  |开发人员| 改版描述
% // 当前版本：V1.1| 2024.3.15  | 朱安源 | 识别当铲斗关节角大于0时的异常情况计算；添加关节安全保护阈值
% // 当前版本：V1.0| 2023.3.15  |  王宵  | 运动学逆解程序，根据齿尖位姿(空间坐标+铲斗关节角)解算关节角

function [feasibility, joint_angle] = Inverse_kinematics_solution_jointbucket(point, excavator_param, bucket_angle)

length_boom = excavator_param(1); length_arm = excavator_param(2); length_bucket = excavator_param(3);
bias_x = excavator_param(4); bias_y = excavator_param(5); bias_z = excavator_param(6);
safe_bias = 0.5;
angleLimits = reshape((excavator_param(7:12)), 3,2);
boom_limit = [angleLimits(1,1) + safe_bias, angleLimits(1,2) - safe_bias]; %角度制
arm_limit = [angleLimits(2,1) + safe_bias, angleLimits(2,2)- safe_bias]; 
bucket_limit = [angleLimits(3,1) + safe_bias, angleLimits(3,2)- safe_bias];
feasibility = 1;
joint_angle = [180, boom_limit(2), arm_limit(2), bucket_limit(2)];
X = point(1); Y = point(2); Z = point(3);
swing_angle = atan2d(Y, X);

edge_bucket_X = X - bias_x * cosd(swing_angle);
edge_bucket_Y = Y - bias_x * sind(swing_angle);
edge_bucket_Z = Z - bias_z;

bucket_angle = deg2rad(bucket_angle);
if bucket_angle <= 0
    theta_FQV = pi + bucket_angle;
else
    theta_FQV = pi - bucket_angle;
end

FV = (length_arm^2 + length_bucket^2 - 2 * cos(theta_FQV) * length_arm * length_bucket)^0.5;
CV = (edge_bucket_X^2 + edge_bucket_Y^2 + edge_bucket_Z^2)^0.5;
cos_FCV = (length_boom^2 + (edge_bucket_X^2 + edge_bucket_Y^2 + edge_bucket_Z^2) - FV^2)/(2 * length_boom * (edge_bucket_X^2 + edge_bucket_Y^2 + edge_bucket_Z^2)^0.5);
cos_CFV = (length_boom^2 + FV^2 - CV^2)/(2 * length_boom * FV);

if -1 <= cos_FCV && cos_FCV <= 1 && -1 <= cos_CFV && cos_CFV <= 1
    theta_FCV = acos(cos_FCV);
    theta_CFV = acos(cos_CFV);
else
    feasibility = 0;
    return
end

boom_angle = theta_FCV + atan2(edge_bucket_Z, (edge_bucket_X^2 + +edge_bucket_Y^2)^0.5);
theta_VFQ  = acos((FV^2 + length_arm^2 - length_bucket^2)/(2 * FV * length_arm));
if bucket_angle <= 0
    arm_angle  = theta_CFV + theta_VFQ - pi;
else
    arm_angle  = theta_CFV - theta_VFQ - pi;
end

if swing_angle < 0
    swing_angle = swing_angle + 360;
end

swing_angle = swing_angle + 180;

if swing_angle > 360
    swing_angle = swing_angle - 360;
end

if ~(rad2deg(boom_angle) >= boom_limit(1) && rad2deg(boom_angle) <= boom_limit(2) && ...
        rad2deg(arm_angle) >= arm_limit(1) && rad2deg(arm_angle) <= arm_limit(2) && ...
        rad2deg(bucket_angle) >= bucket_limit(1) && rad2deg(bucket_angle) <= bucket_limit(2))

    feasibility = 0;
    fprintf('return2')
    return; 
end

boom_angle = rad2deg(boom_angle);
arm_angle = rad2deg(arm_angle);
bucket_angle = rad2deg(bucket_angle);

joint_angle = [swing_angle, boom_angle, arm_angle, bucket_angle];
 
end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //          版本 |    日 期  |开发人员| 改版描述
% // 当前版本：V1.1| 2024.3.15  | 朱安源 | 限制输入数值在限定范围内
function [joint_angle] = jointInScope(joint_angle_, angle_limit)

if joint_angle_ < angle_limit(1)
    joint_angle = angle_limit(1);
elseif joint_angle_ > angle_limit(2)
    joint_angle = angle_limit(2);
else
    joint_angle = joint_angle_;
end

end
%% //*********************
% // Copyright (c) 2025, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //          版本 |    日 期  |开发人员| 改版描述
% // 当前版本：V1.1| 2025.12.4  | 朱安源 | 根据不同回转方向，计算中间角度和中间角度到目标角度的偏差，0-360范围内
function Mid_angle = calculate_mid_angle(A, B, direction)
% 计算回转中间角度和中间角度到目标角度的角度差
% 输入:
%   A - 起始角度(度)
%   B - 目标角度(度)
%   direction - 回转方向: 1=左回转(逆时针), 0=右回转(顺时针)
% 输出:
%   Mid_angle - 中间角度(度)

% 计算顺时针角度差(从A到B)
diff_cw = mod(B - A, 360);
if diff_cw < 0
    diff_cw = diff_cw + 360;
end

% 计算逆时针角度差
diff_ccw = 360 - diff_cw;

% 根据回转方向计算中间角度
if direction == 1  % 左回转(逆时针)
    Mid_angle = mod(A - diff_ccw/2, 360);
else  % 右回转(顺时针)
    Mid_angle = mod(A + diff_cw/2, 360);
end

% 确保M在0-360范围内
if Mid_angle < 0
    Mid_angle = Mid_angle + 360;
elseif Mid_angle >= 360
    Mid_angle = Mid_angle - 360;
end

end

%% //*********************
% // Copyright (c) 2025, 技术研究院智能化研究所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //          版本 |    日 期  |开发人员| 改版描述
% // 当前版本：V1.1| 2025.12.4  | 朱安源 | 计算角度A和角度B之间的偏差，0-360度范围内

function angle_diff = calculate_angle_diff(A, B, direction)

    A = mod(A, 360);
    B = mod(B, 360);
    if direction == 1
        if B <= A
            angle_diff = A - B;
        else
            angle_diff = A + (360 - B);
        end
    else
        if B >= A
            angle_diff = B - A;
        else
            angle_diff = (360 - A) + B;
        end
    end
end

%% //*********************
% // 开发人员：朱安源
% // 当前版本：V1.0| 2025.7.30  | 朱安源 | 最短有向角差，处理 0°/360° 跨界
function diff = signed_shortest_diff_deg(from_deg, to_deg)
% 返回从 from_deg 到 to_deg 的最短有向角差，范围 (-180, 180]
% 正值 = 逆时针（CCW），负值 = 顺时针（CW）
diff = mod(to_deg - from_deg, 360);
if diff > 180
    diff = diff - 360;
end
end
