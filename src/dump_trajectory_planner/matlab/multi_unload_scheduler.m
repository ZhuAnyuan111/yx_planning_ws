function [UP_feasible, selected_points, selected_joints, ...
          unload_exec_count, current_point_idx, current_dump_idx] = ...
    multi_unload_scheduler(loadingFlag, unload_point2, unload_point, ...
                           bucket_angle, excavator_param, bucket_bias, ...
                           dump_done_flag, reset_done_flag)
% MULTI_UNLOAD_SCHEDULER 多卸载点调度器（状态机驱动，Simulink 兼容）
%
% 架构说明：
%   本函数为 Simulink 状态机调度层，每次 step 调用一次。
%   内部维护执行进度（当前卸载点索引 + 当前卸载次数），
%   外部通过 dump_done_flag / reset_done_flag 反馈单次动作完成。
%
% 执行流程：
%   loadingFlag==1 触发 →
%   候选点插值 + IK 筛选 → 选出 3 个卸载点 →
%   从远到近，每点执行 3 次卸载（dump → reset → dump → reset → ...）→
%   全部完成后 UP_feasible 保持 1，unload_exec_count = 9
%
% Simulink 代码生成兼容：
%   - 固定大小数组，无动态分配
%   - 状态通过 persistent 变量或外部 Stateflow 维护
%   - 本函数设计为 MATLAB Function Block，persistent 保持状态
%
% 输入：
%   loadingFlag       - 装载标志 (1=触发卸载选择)
%   unload_point2     - 最远卸载点 [x,y,z] (m)
%   unload_point      - 最近卸载点 [x,y,z] (m)（即 unload_point1）
%   bucket_angle      - 铲斗角度 (deg)
%   excavator_param   - 挖掘机参数结构体
%   bucket_bias       - 铲斗偏置 (deg)
%   dump_done_flag    - 单次卸载完成反馈 (1=本次卸载到位)
%   reset_done_flag   - 单次复位完成反馈 (1=复位到位，可进入下一次)
%
% 输出：
%   UP_feasible        - 卸载点可用标志 (1=有可行方案)
%   selected_points    - 选中的 3 个卸载点 [3x3]
%   selected_joints    - 对应关节角 [3x4]
%   unload_exec_count  - 已完成的卸载次数 (0~9)
%   current_point_idx  - 当前卸载点索引 (1~3, 0=未开始)
%   current_dump_idx   - 当前点的第几次卸载 (1~3, 0=未开始)

%% ===================== 持久状态（跨 step 保持） =====================
persistent state_initialized;
persistent sched_state;         % 0=空闲, 1=已选点待执行, 2=执行中, 3=全部完成
persistent sel_points;
persistent sel_joints;
persistent n_feasible;
persistent pt_idx;              % 当前卸载点索引 (1~3)
persistent dump_idx;            % 当前点卸载次数 (1~3)
persistent exec_count;          % 已完成总卸载次数
persistent waiting_reset;       % 是否正在等待复位完成

if isempty(state_initialized)
    state_initialized = true;
    sched_state = 0;
    sel_points = zeros(3, 3);
    sel_joints = zeros(3, 4);
    n_feasible = 0;
    pt_idx = 0;
    dump_idx = 0;
    exec_count = 0;
    waiting_reset = false;
end

%% ===================== 默认输出 =====================
UP_feasible = 0;
selected_points = sel_points;
selected_joints = sel_joints;
unload_exec_count = exec_count;
current_point_idx = pt_idx;
current_dump_idx = dump_idx;

%% ===================== 状态机 =====================
switch sched_state
    case 0  % ---- 空闲：等待 loadingFlag 触发 ----
        if loadingFlag == 1
            % 调用候选点选择
            [sel_points, sel_joints, n_feasible, UP_feasible] = ...
                multi_unload_selector(unload_point2, unload_point, ...
                                      bucket_angle, excavator_param, bucket_bias);

            if UP_feasible == 1
                sched_state = 1;
                pt_idx = 1;
                dump_idx = 1;
                exec_count = 0;
                waiting_reset = false;
            else
                sched_state = 3;  % 无可行点，直接结束
            end

            selected_points = sel_points;
            selected_joints = sel_joints;
        end

    case 1  % ---- 已选点，准备执行第一个卸载 ----
        UP_feasible = 1;
        sched_state = 2;

    case 2  % ---- 执行中 ----
        UP_feasible = 1;

        if waiting_reset
            % 等待复位完成后进入下一次卸载
            if reset_done_flag == 1
                waiting_reset = false;
                dump_idx = dump_idx + 1;

                if dump_idx > 3
                    % 当前点 3 次卸载完成，切换到下一个点
                    dump_idx = 1;
                    pt_idx = pt_idx + 1;

                    if pt_idx > 3
                        % 全部 3 个点 × 3 次 = 9 次完成
                        sched_state = 3;
                    end
                end
            end
        else
            % 正在执行卸载，等待 dump_done_flag
            if dump_done_flag == 1
                exec_count = exec_count + 1;
                waiting_reset = true;  % 卸载完成，等待复位
            end
        end

        current_point_idx = pt_idx;
        current_dump_idx = dump_idx;
        unload_exec_count = exec_count;

    case 3  % ---- 全部完成 ----
        UP_feasible = 1;
        current_point_idx = pt_idx;
        current_dump_idx = dump_idx;
        unload_exec_count = exec_count;

        % 重置（下次 loadingFlag 上升沿重新触发）
        if loadingFlag == 0
            sched_state = 0;
            pt_idx = 0;
            dump_idx = 0;
            exec_count = 0;
            waiting_reset = false;
        end
end

end
