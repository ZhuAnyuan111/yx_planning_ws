function [selected_points, selected_joints, num_feasible, UP_feasible] = ...
    multi_unload_selector(unload_point_far, unload_point_near, ...
                          bucket_angle, excavator_param, bucket_bias)
% MULTI_UNLOAD_SELECTOR 多卸载点候选插值与选择
%
% 功能：
%   1. 在 unload_point_far（最远）和 unload_point_near（最近）之间
%      均匀插值生成 N_CAND=10 个候选卸载点
%   2. 从远到近依次进行 IK 可达性检测
%   3. 从可达候选点中选出 3 个卸载点（远/中/近）
%   4. 每个卸载点执行 3 次卸载（由上层调度循环使用）
%
% Simulink 代码生成兼容：
%   - 所有数组固定大小
%   - 无 cell / 无动态分配
%   - 循环边界为编译期常量
%
% 输入：
%   unload_point_far  - 最远卸载点 [x, y, z] (m)，对应原 unload_point2
%   unload_point_near - 最近卸载点 [x, y, z] (m)，对应原 unload_point1
%   bucket_angle      - 铲斗角度 (deg)
%   excavator_param   - 挖掘机参数结构体
%   bucket_bias       - 铲斗偏置 (deg)
%
% 输出：
%   selected_points - 选中的 3 个卸载点 [3x3]，每行 [x,y,z]
%                     按远→中→近排列；不足 3 个时剩余行填 0
%   selected_joints - 对应的关节角 [3x4]，每行 [swing, boom, arm, bucket] (deg)
%   num_feasible    - 可达候选点总数 (0~10)
%   UP_feasible     - 是否有足够可达点 (1=成功选出3个, 0=不足)
%
% 使用示例（上层调度）：
%   for k = 1:3  % 3 个卸载点
%       for j = 1:3  % 每点卸载 3 次
%           execute_unload(selected_points(k,:), selected_joints(k,:));
%       end
%   end

%% ===================== 常量定义 =====================
N_CAND = 10;       % 候选点数量
N_SELECT = 3;      % 需选出的卸载点数量
N_DOF = 4;         % 关节自由度 (swing, boom, arm, bucket)

%% ===================== 输出初始化 =====================
selected_points = zeros(N_SELECT, 3);
selected_joints = zeros(N_SELECT, N_DOF);
num_feasible = 0;
UP_feasible = 0;

%% ===================== 1. 生成候选点（远→近） =====================
% t=0 对应 far，t=1 对应 near；从远到近排列
cand_points = zeros(N_CAND, 3);
for i = 1:N_CAND
    t = (i - 1) / (N_CAND - 1);  % 0, 1/9, 2/9, ..., 1
    cand_points(i, 1) = unload_point_far(1) + t * (unload_point_near(1) - unload_point_far(1));
    cand_points(i, 2) = unload_point_far(2) + t * (unload_point_near(2) - unload_point_far(2));
    cand_points(i, 3) = unload_point_far(3) + t * (unload_point_near(3) - unload_point_far(3));
end

%% ===================== 2. 从远到近 IK 可达性检测 =====================
% 固定大小缓存：最多 N_CAND 个可达点
feasible_points = zeros(N_CAND, 3);
feasible_joints = zeros(N_CAND, N_DOF);
feasible_count = 0;

for i = 1:N_CAND
    pt = cand_points(i, :);

    % 第一次尝试：原始铲斗角度 IK
    [feasibility, unload_joint] = Inverse_kinematics_solution( ...
        pt, bucket_angle, excavator_param, bucket_bias);

    % 第二次尝试：若失败，使用固定铲斗角度 25° 的关节-铲斗联合 IK
    if feasibility == 0
        [feasibility, unload_joint] = Inverse_kinematics_solution_jointbucket( ...
            pt, 25, excavator_param, bucket_bias);
    end

    if feasibility == 1
        feasible_count = feasible_count + 1;
        feasible_points(feasible_count, :) = pt;
        feasible_joints(feasible_count, :) = unload_joint;
    end
end

num_feasible = feasible_count;

%% ===================== 3. 从可达点中选出 3 个（远/中/近） =====================
if feasible_count >= N_SELECT
    % 选取策略：第一个（最远）、中间、最后一个（最近）
    idx_far = 1;
    idx_mid = floor((feasible_count + 1) / 2);  % 中间索引
    idx_near = feasible_count;

    % 确保三个索引不重复（当 feasible_count == 3 时自然为 1,2,3）
    if idx_mid == idx_far
        idx_mid = idx_far + 1;
    end
    if idx_mid == idx_near
        idx_mid = idx_near - 1;
    end
    % 安全 clamp
    if idx_mid < 1
        idx_mid = 1;
    end
    if idx_mid > feasible_count
        idx_mid = feasible_count;
    end

    selected_points(1, :) = feasible_points(idx_far, :);
    selected_joints(1, :) = feasible_joints(idx_far, :);

    selected_points(2, :) = feasible_points(idx_mid, :);
    selected_joints(2, :) = feasible_joints(idx_mid, :);

    selected_points(3, :) = feasible_points(idx_near, :);
    selected_joints(3, :) = feasible_joints(idx_near, :);

    UP_feasible = 1;

elseif feasible_count > 0
    % 可达点不足 3 个：有几个用几个，重复填充
    for k = 1:N_SELECT
        idx = mod(k - 1, feasible_count) + 1;  % 循环复用已有点
        selected_points(k, :) = feasible_points(idx, :);
        selected_joints(k, :) = feasible_joints(idx, :);
    end
    UP_feasible = 1;  % 至少有可达点，允许执行

else
    % 无任何可达点
    UP_feasible = 0;
end

end
