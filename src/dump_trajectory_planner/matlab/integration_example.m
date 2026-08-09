%% integration_example.m
% 展示如何将 multi_unload_selector 集成到现有 Simulink 模型中
% 替换原有的 "if loadingFlag == 1 ... end" 逻辑
%
% ======================== 原代码（被替换） ========================
% if loadingFlag == 1
%     unload_points = {unload_point2, unload_point};
%     point_names   = {'unload_point2', 'unload_point'};
%     for i = 1:numel(unload_points)
%         pt = unload_points{i};
%         [feasibility, unload_joint] = Inverse_kinematics_solution( ...
%             pt, bucket_angle, excavator_param, bucket_bias);
%         if feasibility == 0
%             [feasibility, unload_joint] = Inverse_kinematics_solution_jointbucket( ...
%                 pt, 25, excavator_param, bucket_bias);
%         end
%         if feasibility == 1
%             UP_feasible = 1;
%             break;
%         end
%     end
%     if feasibility == 0
%         UP_feasible = 0;
%     end
% end
%
% ======================== 新代码（替换为） ========================
%
% 方案 A：纯组合逻辑（仅选点，执行调度由外部 Stateflow 管理）
% -------------------------------------------------------------------
% 适用场景：Stateflow 已有卸载/复位状态机，只需替换选点逻辑
%
%   if loadingFlag == 1
%       [selected_points, selected_joints, num_feasible, UP_feasible] = ...
%           multi_unload_selector(unload_point2, unload_point, ...
%                                 bucket_angle, excavator_param, bucket_bias);
%   end
%
%   % 然后 Stateflow 根据 selected_points / selected_joints 执行：
%   %   点1卸3次 → 点2卸3次 → 点3卸3次
%
%
% 方案 B：完整状态机调度（MATLAB Function Block 自管理）
% -------------------------------------------------------------------
% 适用场景：无 Stateflow，用 MATLAB Function Block + persistent 管理全流程
%
%   [UP_feasible, selected_points, selected_joints, ...
%    unload_exec_count, current_point_idx, current_dump_idx] = ...
%       multi_unload_scheduler(loadingFlag, unload_point2, unload_point, ...
%                              bucket_angle, excavator_param, bucket_bias, ...
%                              dump_done_flag, reset_done_flag);
%
%   % 输出 current_point_idx / current_dump_idx 驱动轨迹规划：
%   %   当前目标点 = selected_points(current_point_idx, :)
%   %   当前目标关节 = selected_joints(current_point_idx, :)
%
%
% ======================== Simulink 接线说明 ========================
%
%  [MATLAB Function Block: multi_unload_scheduler]
%
%  输入端口：
%    loadingFlag       ← 上层决策 (Double, scalar)
%    unload_point2     ← 卡车包络计算 (Bus/Vector, [1x3])
%    unload_point      ← 卡车包络计算 (Bus/Vector, [1x3])
%    bucket_angle      ← 传感器/常量 (Double, scalar)
%    excavator_param   ← 参数结构体 (Bus)
%    bucket_bias       ← 参数常量 (Double, scalar)
%    dump_done_flag    ← 卸载轨迹完成反馈 (Double, 0/1)
%    reset_done_flag   ← 复位轨迹完成反馈 (Double, 0/1)
%
%  输出端口：
%    UP_feasible        → 允许卸载标志 (Double, 0/1)
%    selected_points    → 3个卸载点 (Vector, [3x3])
%    selected_joints    → 3组关节角 (Vector, [3x4])
%    unload_exec_count  → 已完成卸载次数 (Double, 0~9)
%    current_point_idx  → 当前点索引 (Double, 0~3)
%    current_dump_idx   → 当前次第几次 (Double, 0~3)
%
%
% ======================== 代码生成注意事项 ========================
%
% 1. Inverse_kinematics_solution / Inverse_kinematics_solution_jointbucket
%    必须也支持代码生成（无 fprintf、无 cell、固定输出尺寸）
%
% 2. excavator_param 建议定义为 Simulink.Bus 类型：
%    busObj = Simulink.Bus;
%    busObj.Elements = {
%      Simulink.BusElement('boom_length', 'double', 1), ...
%      Simulink.BusElement('arm_length', 'double', 1), ...
%      Simulink.BusElement('bucket_length', 'double', 1), ...
%      % ... 其余字段
%    };
%
% 3. 若使用 persistent 变量，Simulink 会自动映射为静态局部变量
%
% 4. 所有数组维度在编译期确定：
%    - cand_points: [10x3]
%    - feasible_points: [10x3]
%    - selected_points: [3x3]
%    - selected_joints: [3x4]
%
% 5. 不支持代码生成的函数（需替换或移除）：
%    - fprintf → 移除或用 Simulink 日志
%    - cell {} → 用矩阵
%    - numel → 用常量
%    - 动态索引变量名 → 固定索引
