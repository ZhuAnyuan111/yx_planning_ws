%% test_reachability.m
% 验证 unload_point_far / unload_point_near 的 IK 可达性

% ---- 挖掘机参数（来自 dump_trajectory.yaml）----
length_boom   = 6.987;
length_arm    = 2.797;
length_bucket = 2.359;
bias_x        = 0.09;
bias_y        = 0.03;
bias_z        = 0.0;
boom_limit    = [-25, 50];
arm_limit     = [-150, -30];
bucket_limit  = [-135, 30];
excavator_param = [length_boom, length_arm, length_bucket, bias_x, bias_y, bias_z, ...
                   boom_limit(1), boom_limit(2), ...
                   arm_limit(1), arm_limit(2), ...
                   bucket_limit(1), bucket_limit(2)];

bucket_bias  = 1.0;       % 铲斗偏置系数（按实际值修改）
bucket_angle = -90;       % 铲斗姿态角 (deg)

% ---- 待验证点 ----
unload_point_far  = [7.400, -5.272, 0.118];
unload_point_near = [6.975, -5.535, 0.118];

fprintf('=== IK Reachability Test ===\n');
fprintf('excavator_param = [%s]\n\n', num2str(excavator_param));

points = {unload_point_far, unload_point_near};
names  = {'unload_point_far ', 'unload_point_near'};

for idx = 1:2
    pt = points{idx};
    fprintf('--- %s = [%.3f, %.3f, %.3f] ---\n', names{idx}, pt(1), pt(2), pt(3));
    
    % 水平距离
    r = sqrt(pt(1)^2 + pt(2)^2);
    fprintf('  horizontal distance = %.3f m\n', r);
    
    % 方法1: Inverse_kinematics_solution (bucket_attitude_angle)
    [f1, j1] = Inverse_kinematics_solution(pt, bucket_angle, excavator_param, bucket_bias);
    fprintf('  IK_solution(bkt_att=%.0f): feasible=%.0f  joints=[%.2f, %.2f, %.2f, %.2f]\n', ...
        bucket_angle, f1, j1(1), j1(2), j1(3), j1(4));
    
    % 方法2: Inverse_kinematics_solution_jointbucket (bucket_joint_angle=25)
    [f2, j2] = Inverse_kinematics_solution_jointbucket(pt, 25, excavator_param, bucket_bias);
    fprintf('  IK_jointbucket(bkt=25):   feasible=%.0f  joints=[%.2f, %.2f, %.2f, %.2f]\n', ...
        f2, j2(1), j2(2), j2(3), j2(4));
    
    % 方法3: 放宽铲斗角 (20, 15, 10)
    for bk = [20, 15, 10]
        [f3, j3] = Inverse_kinematics_solution_jointbucket(pt, bk, excavator_param, bucket_bias);
        fprintf('  IK_jointbucket(bkt=%d):   feasible=%.0f  joints=[%.2f, %.2f, %.2f, %.2f]\n', ...
            bk, f3, j3(1), j3(2), j3(3), j3(4));
    end
    fprintf('\n');
end
