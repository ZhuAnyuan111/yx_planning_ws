% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //           版本 |    日 期    |开发人员| 改版描述
% // 当前版本：V1.0| 2024.3.15  | 朱安源 | 判断矿卡停靠是否合适；选取候选卸载点；判断卸载点是否装满
% // 当前版本：V1.1| 2024.4.18  | 朱安源 | 卸载点选取修改为基于矿车框坐标系(模块自行定义)，后续根据车体中心点进行与挖机位置换算，解决挖机后退动态选点问题
% // 当前版本：V2.0| 2025.7.30  | 朱安源 | 移除满载判断逻辑，改为基于load_num三段式选点（远/中/近）

function [TruckFeasible, unloadPoint, unloadJoint, middle_UP] = ...
    CandidatePointsGeneration(debug_mode, excavator_param, loadingFlag, load_numParam, unload_point_far, unload_point_near, ...
                              load_num, bucket_bias, bucket_angle, start_joint, truck_array, R_coff, rot_dir, WJS_vel)

num_CandidatePoints = 21;
DEBUG_PRINT = debug_mode;  % ← 代码生成前改为 false

TruckFeasible = 0;
unloadPoint = reshape(unload_point_far, 1, 3);  % 强制 [1x3] 行向量
unloadJoint = [180, 30, -45, 25];
middle_UP = zeros(1, 2);  % 初始化 [1x2]，防止早期 return 时未赋值

if DEBUG_PRINT
    fprintf('\n[UP] === CandidatePointsGeneration called ===\n');
    fprintf('[UP] loadingFlag=%.1f  load_num=%.0f\n', loadingFlag, load_num);
    fprintf('[UP] unload_point_far =[%.3f, %.3f, %.3f]\n', unload_point_far(1), unload_point_far(2), unload_point_far(3));
    fprintf('[UP] unload_point_near=[%.3f, %.3f, %.3f]\n', unload_point_near(1), unload_point_near(2), unload_point_near(3));
end

% ===== 1. 生成候选卸载点（由近到远，远端更密） =====
CandidateUnloadPoints = Calculate_candidate_points(unload_point_far, unload_point_near, num_CandidatePoints);

% ===== 2. IK 可达性检测 =====
feasibility = zeros(num_CandidatePoints, 1);
joint_angles = repmat([180, 20, -70, 20], num_CandidatePoints, 1);

if loadingFlag > 0.57
    for i = 1:num_CandidatePoints
        [feasibility(i), joint_angles(i,:)] = Inverse_kinematics_solution( ...
            CandidateUnloadPoints(i,:), bucket_angle, excavator_param, bucket_bias);
        if feasibility(i) == 0
            [feasibility(i), joint_angles(i,:)] = Inverse_kinematics_solution_jointbucket( ...
                CandidateUnloadPoints(i,:), 30, excavator_param, bucket_bias);
        end
    end
else
    if DEBUG_PRINT
        fprintf('[UP] loadingFlag=%.2f <= 0.57, IK skipped\n', loadingFlag);
    end
end

% ===== 3. 提取可达点索引（代码生成兼容：避免 find 变长数组） =====
num_feasible = 0;
idx_near = 0;    % 最近可达索引
idx_far = 0;     % 最远可达索引
idx_mid = 0;     % 中间可达索引

for i = 1:num_CandidatePoints
    if feasibility(i) == 1
        num_feasible = num_feasible + 1;
        if idx_near == 0
            idx_near = i;   % 第一个可达 = 最近
        end
        idx_far = i;        % 持续更新 = 最终为最远
    end
end

if DEBUG_PRINT
    fprintf('[UP] IK result: %.0f/%.0f feasible | idx_near=%.0f  idx_far=%.0f\n', ...
        num_feasible, num_CandidatePoints, idx_near, idx_far);
end

if num_feasible == 0
    TruckFeasible = 0;
    if DEBUG_PRINT
        fprintf('[UP] ERROR: no feasible point, TruckFeasible=0, return\n');
    end
    return;
end
TruckFeasible = 1;

% ===== 4. 划分三个卸载点：最远可达 / 中间 / 最近可达 =====
% 计算中间可达索引：第 ceil(num_feasible/2) 个可达点
mid_count = 0;
mid_target = round(num_feasible / 2);
if mid_target < 1
    mid_target = 1;
end
for i = idx_near:num_CandidatePoints
    if feasibility(i) == 1
        mid_count = mid_count + 1;
        if mid_count == mid_target
            idx_mid = i;
            break;
        end
    end
end

unloadPoint_near_reachable = CandidateUnloadPoints(idx_near, :);
unloadPoint_mid            = CandidateUnloadPoints(idx_mid, :);
unloadPoint_far_reachable  = CandidateUnloadPoints(idx_far, :);

if DEBUG_PRINT
    fprintf('[UP] 3-point division: idx_near=%.0f  idx_mid=%.0f  idx_far=%.0f\n', idx_near, idx_mid, idx_far);
    fprintf('[UP]   far  =[%.3f, %.3f, %.3f]\n', unloadPoint_far_reachable(1), unloadPoint_far_reachable(2), unloadPoint_far_reachable(3));
    fprintf('[UP]   mid  =[%.3f, %.3f, %.3f]\n', unloadPoint_mid(1),            unloadPoint_mid(2),            unloadPoint_mid(3));
    fprintf('[UP]   near =[%.3f, %.3f, %.3f]\n', unloadPoint_near_reachable(1), unloadPoint_near_reachable(2), unloadPoint_near_reachable(3));
end

% ===== 5. 基于 load_num 三段式选点，直接输出 =====
if load_num < load_numParam(1)
    unloadPoint = unloadPoint_far_reachable;
    unloadJoint = joint_angles(idx_far, :);
    sel_tag = 'FAR(<3)';
elseif load_num < load_numParam(2)
    unloadPoint = unloadPoint_mid;
    unloadJoint = joint_angles(idx_mid, :);
    sel_tag = 'MID(3-5)';
elseif load_num < load_numParam(3)
    unloadPoint = unloadPoint_near_reachable;
    unloadJoint = joint_angles(idx_near, :);
    sel_tag = 'NEAR(6-8)';
else
    unloadPoint = unloadPoint_mid;
    unloadJoint = joint_angles(idx_mid, :);
    sel_tag = 'MID(>=9,fallback)';
end

if DEBUG_PRINT
    fprintf('[UP] load_num=%.0f -> select %s\n', load_num, sel_tag);
    fprintf('[UP]   unloadPoint=[%.3f, %.3f, %.3f]\n', unloadPoint(1), unloadPoint(2), unloadPoint(3));
    fprintf('[UP]   unloadJoint=[sw=%.2f, bm=%.2f, ar=%.2f, bk=%.2f]\n', ...
        unloadJoint(1), unloadJoint(2), unloadJoint(3), unloadJoint(4));
end

% calculate entry point
excavator_param(3) = 0;
[digX, digY, ~, ~] = Kinematic_forward_solution(start_joint, excavator_param);
dig_end = [digX, digY]';
truck_center = truck_array(1:2,:);
truck_size = truck_array(4:5,:)';
truck_yaw = truck_array(7,:);
[flag, p_entry] = calc_entry_point_circle_strategy(dig_end, unloadPoint, R_coff, truck_center, truck_yaw, truck_size, rot_dir);

middle_UP = zeros(1, 2);
middle_UP(1) = p_entry(1);
middle_UP(2) = p_entry(2);

if DEBUG_PRINT
    fprintf('[UP] entry point: flag=%.0f  middle_UP=[%.3f, %.3f]\n', flag, middle_UP(1), middle_UP(2));
    if flag == 0
        fprintf('[UP] WARNING: entry point calc failed (R too small or degenerate)\n');
    end
    fprintf('[UP] === done ===\n');
end

end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //           版本 |    日 期    |开发人员| 改版描述
% // 当前版本：V1.0| 2024.3.15  | 朱安源 | 生成候选卸载点
function cand_points = Calculate_candidate_points(unload_point_far, unload_point_near, N_CAND)

% truck_center = reshape(truck_center,1,3);
% 
% CandidateUnloadPoints = repmat(truck_center(1:3), length(PresetTruckUnloadPoint), 1);
% CandidateUnloadPoints(:, 1) = CandidateUnloadPoints(:, 1) + cos( truck_headingAngle)  .* PresetTruckUnloadPoint;
% CandidateUnloadPoints(:, 2) = CandidateUnloadPoints(:, 2) + sin( truck_headingAngle)  .* PresetTruckUnloadPoint;
% CandidateUnloadPoints(:, 3) = truck_center(3) + truck_size(3)/2 + unloadPoint_HeightBias;
%%
% cand_points = zeros(N_CAND, 3);
% for i = 1:N_CAND
%     t = (i - 1) / (N_CAND - 1);  % 0, 1/9, 2/9, ..., 1
%     cand_points(i, 1) = unload_point_near(1) + t * (unload_point_far(1) - unload_point_near(1));
%     cand_points(i, 2) = unload_point_near(2) + t * (unload_point_far(2) - unload_point_near(2));
%     cand_points(i, 3) = unload_point_near(3) + t * (unload_point_far(3) - unload_point_near(3));
% end
%%
cand_points = zeros(N_CAND, 3);
for i = 1:N_CAND
    t_raw = (i - 1) / (N_CAND - 1);   % 0 → 1
    t = 1 - (1 - t_raw)^2.5;          % ✅ 远端密集

    cand_points(i,1) = unload_point_near(1) + t * (unload_point_far(1) - unload_point_near(1));
    cand_points(i,2) = unload_point_near(2) + t * (unload_point_far(2) - unload_point_near(2));
    cand_points(i,3) = unload_point_near(3);

end

end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：王宵
% //	改版履历
% //           版本 |    日 期    |开发人员| 改版描述
% // 当前版本：V1.1| 2024.3.15  | 朱安源 | 添加关节安全保护阈值
% // 当前版本：V1.0| 2023.3.15  |  王宵  | 运动学逆解程序，根据齿尖位姿(空间坐标+铲斗姿态角)解算关节角
function [feasibility, joint_angle] = Inverse_kinematics_solution(point, bucket_attitude_angle, excavator_param, bucket_bias)

length_boom = excavator_param(1); length_arm = excavator_param(2); length_bucket = excavator_param(3);
bias_x = excavator_param(4); bias_y = excavator_param(5); bias_z = excavator_param(6); 
safe_bias = 2;
length_bucket = length_bucket * bucket_bias;
x = point(1); y = point(2); z = point(3);
angleLimits = reshape((excavator_param(7:12)), 3,2);%角度制
boom_limit = [angleLimits(1,1) + safe_bias, angleLimits(1,2) - safe_bias]; 
arm_limit = [angleLimits(2,1) + safe_bias, angleLimits(2,2)- safe_bias]; 
bucket_limit = [angleLimits(3,1) + safe_bias, angleLimits(3,2)- safe_bias];
feasibility = 1;
joint_angle = [180, boom_limit(2), arm_limit(2), bucket_limit(2)];
swing_angle = atan2d(y, x);

if swing_angle == 90
    x_1 = y - bias_x * sind(swing_angle);
elseif swing_angle == -90
    x_1 = -(y - bias_x * sind(swing_angle));
else
    x_1 = (x - bias_x * cosd(swing_angle)) / cosd(swing_angle);
end
z_1 = z - bias_z;

x3 = x_1 - length_bucket * cosd(bucket_attitude_angle);
z3 = z_1 - length_bucket * sind(bucket_attitude_angle);

l5 = sqrt(x_1^2+z_1^2);
l6 = sqrt(x3^2+z3^2);

cos_beta = (l5^2 + l6^2 - length_bucket^2)/(2 * l5 * l6);
cos_alpha = (length_boom^2 + l6^2 - length_arm^2)/(2 * length_boom * l6);
if -1<=cos_beta && cos_beta<=1 && -1<=cos_alpha && cos_alpha<=1
    r = atan2d(z_1, x_1);
    b = acosd(cos_beta);
    a = acosd(cos_alpha);
else
    feasibility = 0;
    return
end

boom_angle = r + b + a;
arm_angle = acosd((length_boom^2 + length_arm^2 - l6^2)/(2 * length_boom * length_arm)) - 180;
bucket_angle = bucket_attitude_angle - boom_angle - arm_angle;

if swing_angle < 0
    swing_angle = swing_angle + 360;
end

swing_angle = swing_angle + 180;

if swing_angle > 360
    swing_angle = swing_angle - 360;
end

joint_angle = [swing_angle, boom_angle, arm_angle, bucket_angle];

if ~(boom_angle >= boom_limit(1) && boom_angle <= boom_limit(2) && ...
     arm_angle >= arm_limit(1) && arm_angle <= arm_limit(2) && ...
     bucket_angle >= bucket_limit(1) && bucket_angle <= bucket_limit(2))

    feasibility = 0; 
    return; 

end

end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：王宵
% //	改版履历
% //          版本 |    日 期  |开发人员| 改版描述
% // 当前版本：V1.1| 2024.3.15  | 朱安源 | 识别当铲斗关节角大于0时的异常情况计算；添加关节安全保护阈值
% // 当前版本：V1.0| 2023.3.15  |  王宵  | 运动学逆解程序，根据齿尖位姿(空间坐标+铲斗关节角)解算关节角
function [feasibility, joint_angle] = Inverse_kinematics_solution_jointbucket(point, bucket_angle, excavator_param, bucket_bias)

length_boom = excavator_param(1); length_arm = excavator_param(2); length_bucket = excavator_param(3);
bias_x = excavator_param(4); bias_y = excavator_param(5); bias_z = excavator_param(6);

length_bucket = length_bucket * bucket_bias;
safe_bias = 2;
angleLimits = reshape((excavator_param(7:12)), 3,2);%角度制
boom_limit = [angleLimits(1,1) + safe_bias, angleLimits(1,2) - safe_bias]; 
arm_limit = [angleLimits(2,1) + safe_bias, angleLimits(2,2)- safe_bias]; 
bucket_limit = [angleLimits(3,1) + safe_bias, angleLimits(3,2)- safe_bias];
feasibility = 1;
joint_angle = [180, boom_limit(2), arm_limit(2), bucket_limit(2)];
X = point(1);
Y = point(2);
Z = point(3);

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
    return;
end

boom_angle = rad2deg(boom_angle);
arm_angle = rad2deg(arm_angle);
bucket_angle = rad2deg(bucket_angle);

joint_angle = [swing_angle, boom_angle, arm_angle, bucket_angle];
 
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
% //	开发人员：朱安源
% //	改版履历
% //           版本 |   日 期   |开发人员| 改版描述
% // 当前版本：V1.0| 2026.1.8  |  朱安源  | 求解装车轨迹入厢点
function [entry_flag, p_entry] = calc_entry_point_circle_strategy(dig_end, unload_point, R_coff, truck_center, truck_yaw, truck_size, rot_dir)

%% ================= 1. 回转圆 =================
O = [0;0];

dx1 = dig_end(1) - O(1);
dy1 = dig_end(2) - O(2);
R1  = sqrt(dx1*dx1 + dy1*dy1);
dx2 = unload_point(1) - O(1);
dy2 = unload_point(2) - O(2);
R2  = sqrt(dx2*dx2 + dy2*dy2);
R = R_coff * R1 + (1-R_coff) * R2;

p_entry = zeros(2,1);

if R < 1e-6
    entry_flag = 0;
    return;
end

%% ================= 2. 坐标变换 =================
L = truck_size(1);
W = truck_size(2);

xmin = -L*0.5; xmax = L*0.5;
ymin = -W*0.5; ymax = W*0.5;

cy = cos(truck_yaw);
sy = sin(truck_yaw);

R_WT = [ cy -sy;
         sy  cy ];

O_local = R_WT' * (O - truck_center);
p_end_local = R_WT' * (dig_end - truck_center);

%% ================= 3. 圆-矩形交点 =================
intersections = zeros(2,8);
n_inter = 0;

% --- x = xmin
b = O_local(1) - xmin;
D2 = R*R - b*b;
if D2 >= 0
    s = sqrt(D2);
    y1 = O_local(2) + s;
    y2 = O_local(2) - s;
    if y1 >= ymin && y1 <= ymax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [xmin; y1];
    end
    if y2 >= ymin && y2 <= ymax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [xmin; y2];
    end
end

% --- x = xmax
b = O_local(1) - xmax;
D2 = R*R - b*b;
if D2 >= 0
    s = sqrt(D2);
    y1 = O_local(2) + s;
    y2 = O_local(2) - s;
    if y1 >= ymin && y1 <= ymax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [xmax; y1];
    end
    if y2 >= ymin && y2 <= ymax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [xmax; y2];
    end
end

% --- y = ymin
d = O_local(2) - ymin;
D2 = R*R - d*d;
if D2 >= 0
    s = sqrt(D2);
    x1 = O_local(1) + s;
    x2 = O_local(1) - s;
    if x1 >= xmin && x1 <= xmax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [x1; ymin];
    end
    if x2 >= xmin && x2 <= xmax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [x2; ymin];
    end
end

% --- y = ymax
d = O_local(2) - ymax;
D2 = R*R - d*d;
if D2 >= 0
    s = sqrt(D2);
    x1 = O_local(1) + s;
    x2 = O_local(1) - s;
    if x1 >= xmin && x1 <= xmax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [x1; ymax];
    end
    if x2 >= xmin && x2 <= xmax
        n_inter = n_inter + 1;
        intersections(:,n_inter) = [x2; ymax];
    end
end

%% ================= 4. 有交点 =================
entry_flag = 0;
if n_inter > 0
    p_end_swing = calculate_swing(dig_end(1), dig_end(2));  % 修复：第二参数应为 dig_end(2)
    angleDiff_min = 360;
    for i = 1:n_inter  % 修复：只遍历实际交点数，而非全部 8 列
        if (intersections(1,i)*intersections(1,i) + intersections(2,i)*intersections(2,i)) > 0
            interP = R_WT * intersections(:, i) + truck_center;
            swing_angle = calculate_swing(interP(1), interP(2));
            angleDiff = angle_diff(p_end_swing, swing_angle, rot_dir);
            if angleDiff < angleDiff_min
                angleDiff_min = angleDiff;
                p_entry = [interP(1); interP(2)];
                entry_flag = 1;  % 修复：找到有效交点才置 1
            end
        end
    end
    return;
end

%% ================= 5. fallback ================= 
best_d2 = 1e12;
nearest = [0;0];

% --- 四条边显式展开
[nearest, best_d2] = seg_check(O_local, [xmin;ymin], [xmin;ymax], nearest, best_d2);
[nearest, best_d2] = seg_check(O_local, [xmax;ymin], [xmax;ymax], nearest, best_d2);
[nearest, best_d2] = seg_check(O_local, [xmin;ymin], [xmax;ymin], nearest, best_d2);
[nearest, best_d2] = seg_check(O_local, [xmin;ymax], [xmax;ymax], nearest, best_d2);

vx = nearest(1) - O_local(1);
vy = nearest(2) - O_local(2);
vn = sqrt(vx*vx + vy*vy);
if vn < 1e-6
    entry_flag = 0;
    return;
end

p_entry_local = O_local + R * [vx; vy] / vn;
p_entry = R_WT * p_entry_local + truck_center;
entry_flag = 2;

end

%% ========== 辅助函数（Simulink 可用） ==========
function [nearest, best_d2] = seg_check(P, A, B, nearest, best_d2)
AB = B - A;
AP = P - A;

den = AB(1)*AB(1) + AB(2)*AB(2);
if den < 1e-9
    Q = A;
else
    t = (AP(1)*AB(1) + AP(2)*AB(2)) / den;
    if t < 0
        t = 0;
    elseif t > 1
        t = 1;
    end
    Q = A + t * AB;
end

dx = Q(1) - P(1);
dy = Q(2) - P(2);
d2 = dx*dx + dy*dy;

if d2 < best_d2
    best_d2 = d2;
    nearest = Q;
end
end

function angle_diff = angle_diff(A, B, rot_dir)
theta_start = mod(A, 360);
theta_end   = mod(B,   360);

delta_ccw = mod(theta_end - theta_start, 360);

delta_cw  = mod(theta_start - theta_end, 360);

if rot_dir > 0
    angle_diff = delta_ccw;
else
    angle_diff = delta_cw;
end
end

function swing_angle = calculate_swing(x, y)
swing_angle = atan2d(y,x);
if swing_angle < 0
    swing_angle = swing_angle + 360;
end
swing_angle = swing_angle + 180;
if swing_angle > 360
    swing_angle = swing_angle -360;
end
end

