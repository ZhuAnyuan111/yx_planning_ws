%% //*********************
% // Copyright (c) 2026, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //           版本 |    日 期  |开发人员| 改版描述
% // 当前版本：V1.0| 2026.8.13 | 朱安源 | 根据斗杆/铲斗关节角与齿尖触地z高度反解动臂关节角
function [feasibility, boom_angle] = solve_boom_ground_contact(arm_angle, bucket_angle, z_target, excavator_param, reference_boom)
% 输入参数
% arm_angle       斗杆关节角 (deg)
% bucket_angle    铲斗关节角 (deg)
% z_target        齿尖触地目标高度 (m, 与FK输出的Z同一全局基准)
% excavator_param 机构参数向量，定义与 Inverse_kinematics_solution 一致：
%                 (1)动臂长 (2)斗杆长 (3)铲斗长 (4)bias_x (5)bias_y (6)bias_z (7:12)关节限位[3x2, deg]
% reference_boom  可选，参考动臂角 (deg)：二解选择时取与之角度差最小的解；
%                 缺省时取较小动臂角（更低、更接近触地姿态）
% 输出参数
% feasibility     1 可行 / 0 不可行
% boom_angle      动臂关节角 (deg)
%
% 原理：FK 中齿尖 z 与 swing 无关：
%   z = bias_z + L1*sin(θb) + L2*sin(θb+θa) + L3*sin(θb+θa+θk)
% 已知 θa(arm)、θk(bucket) 与目标 z，化为 A*sin(θb)+B*cos(θb)=C 解析求解，
% 得到肘上/肘下两个解，经关节限位过滤后按策略选择。

length_boom = excavator_param(1);
length_arm = excavator_param(2);
length_bucket = excavator_param(3);
bias_z = excavator_param(6);

safe_bias = 2;
angleLimits = reshape((excavator_param(7:12)), 3, 2); % 角度制
boom_limit = [angleLimits(1,1) + safe_bias, angleLimits(1,2) - safe_bias];

feasibility = 1;
boom_angle = boom_limit(1);

arm_rad = deg2rad(arm_angle);
bucket_rad = deg2rad(bucket_angle);

% A*sin(θb) + B*cos(θb) = C
A = length_boom + length_arm * cos(arm_rad) + length_bucket * cos(arm_rad + bucket_rad);
B = length_arm * sin(arm_rad) + length_bucket * sin(arm_rad + bucket_rad);
C = z_target - bias_z;
R = sqrt(A^2 + B^2);

% 可达性检查：|C| > R 时目标高度超出可达范围，无解
if abs(C) > R
    feasibility = 0;
    return;
end

phi = atan2d(B, A);
tmp = asind(min(max(C / R, -1), 1)); % clamp 防浮点误差越界

boom1 = mod(tmp - phi + 180, 360) - 180;        % 解1（归一化到[-180,180)）
boom2 = mod(180 - tmp - phi + 180, 360) - 180;  % 解2（肘上/肘下另一构型）

valid1 = (boom1 >= boom_limit(1)) && (boom1 <= boom_limit(2));
valid2 = (boom2 >= boom_limit(1)) && (boom2 <= boom_limit(2));

if ~valid1 && ~valid2
    feasibility = 0;
    return;
end

if nargin >= 5 && ~isempty(reference_boom)
    % 有参考角：选与之角度差最小的解
    d1 = abs(mod(boom1 - reference_boom + 180, 360) - 180);
    d2 = abs(mod(boom2 - reference_boom + 180, 360) - 180);
    if valid1 && (~valid2 || d1 <= d2)
        boom_angle = boom1;
    else
        boom_angle = boom2;
    end
else
    % 缺省：选较小动臂角（更低、更接近触地姿态）
    if valid1 && valid2
        boom_angle = min(boom1, boom2);
    elseif valid1
        boom_angle = boom1;
    else
        boom_angle = boom2;
    end
end

end
