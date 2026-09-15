% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //           版本 |    日 期    |开发人员| 改版描述
% // 当前版本：V1.0| 2024.3.15  | 朱安源 | 判断矿卡停靠是否合适；选取候选卸载点；判断卸载点是否装满
% // 当前版本：V1.1| 2024.4.18  | 朱安源 | 卸载点选取修改为基于矿车框坐标系(模块自行定义)，后续根据车体中心点进行与挖机位置换算，解决挖机后退动态选点问题

function [TruckFeasible, feasibility, TruckFullFlag, CandidateUnloadPoints, joint_angles, full_MeanHeight, unloadPoint, unloadJoint, middleUP, SoilFullInPoint]  =...
    CandidatePointsGeneration(excavator_param, loadingFlag, LoadNums, MaxLoadNum, MinLoadNum, ptCloudBias, bucket_bias, truck_array, JudgeFullBias, full_Height_bias, endpoint_to_truckHead,...
    startpoint_to_truckend, bucket_angle, select_soil_radius, point_cloud_soil_truck, num_soilPoint, unloadPoint_HeightBias, SoilFullInPoint_)

num_CandidatePoints = 21;
truck_center       = truck_array(1:3);
truck_size         = truck_array(4:6); 
truck_headingAngle  = truck_array(7);
TruckFeasible = 0;
unloadPoint = [truck_array(1), truck_array(2), truck_array(3)+truck_array(6)];
middleUP = [truck_array(1), truck_array(2), truck_array(3)+truck_array(6)];
unloadJoint = [180, 30, -45, 25];
TruckFullFlag = 0;
case_feasible = 1;

truckPlane = truck_center(3) + truck_size(3) / 2;
full_MeanHeight   = truckPlane + full_Height_bias;

endpoint_to_truckHeadlength_bias = truck_size(1)/2 + endpoint_to_truckHead;
startpoint_to_truckHeadlength_bias = truck_size(1)/2 + startpoint_to_truckend;
PresetTruckUnloadPoint = linspace(-startpoint_to_truckHeadlength_bias, endpoint_to_truckHeadlength_bias, num_CandidatePoints)'; 

% *****根据PresetTruckUnloadPoint进行矿卡坐标系下卸载点状态判断 *****
CandidateUnloadPoints = Calculate_candidate_points(truck_center, truck_size, truck_headingAngle, PresetTruckUnloadPoint, unloadPoint_HeightBias) ;%将矿卡坐标系下的候选卸载点转到相对于挖机的位置
%判断候选卸载点是否可达;feasibility  (21*2) ，第一列代表可达状态 0:不可达 1：可达，第二列代表满载状态 0：未满  1：已满 
feasibility = zeros(num_CandidatePoints, 2);
joint_angles = repmat([180, 20, -70, 20],num_CandidatePoints,1);
%重置卸载点计算情况
if loadingFlag == 2
    SoilFullInPoint = zeros(num_CandidatePoints,1);
    fprintf("error1!!!")
    return
else
    SoilFullInPoint = SoilFullInPoint_;
end
%激活计算下一次卸载点位置
if loadingFlag == 1
    for i = 1 : num_CandidatePoints
        [feasibility(i,1), joint_angles(i,:)] = Inverse_kinematics_solution (CandidateUnloadPoints(i,:), bucket_angle, excavator_param, bucket_bias); 
        if feasibility(i,1) == 0
            [feasibility(i,1), joint_angles(i,:)] = Inverse_kinematics_solution_jointbucket (CandidateUnloadPoints(i,:), 30, excavator_param, bucket_bias); 
        end
    end
    case_feasible = find(feasibility(:,1) == 1);
end

%判断矿卡是否可达卸载
index1 = find(SoilFullInPoint_, 1);%正序查找第一个未装满候选点的索引
index2 = find(feasibility(:,1), 1, 'last');%查找最远可达位置索引
TruckFeasible = 0;
if index2 > 0.5*num_CandidatePoints % 第一步判断，若最远可达位置大于设定阈值
    TruckFeasible = 1;%矿卡可继续卸载
elseif index2 >= 1 & index2 <= 0.5*num_CandidatePoints
    %若最远可达位置不能满足设定阈值，判断设定从当前最远可达到设定阈值处是否已装满
    if SoilFullInPoint(round(0.5*num_CandidatePoints)) == 1%可达阈值处已装满，则矿卡可继续卸载
        TruckFeasible = 1;
    end
else
    fprintf("error2!!!")
    return
end
if TruckFeasible == 1
    fprintf('good truck');
else
    fprintf('unreachable truck')
end

%对可达候选卸载点进行满载判断，由远及近，找到第一个可达未满的位置即选为下一次卸载位置
unload_capacity = zeros(num_CandidatePoints, 1);
ptCloud_truck = point_cloud_soil_truck(point_cloud_soil_truck(:,3) <= truckPlane + ptCloudBias,:);
for i = num_CandidatePoints : -1 : 1
    
    if feasibility(i,1) == 0%不可达位置
        feasibility(i,2) = 0;%判断为未满不可达
    else%对可达位置进行装满判断
        if SoilFullInPoint_(i) == 1%若可达位置上次已判断装满，则本次直接继承为装满
            SoilFullInPoint(i) = 1;
            feasibility(i,2) = 2; %feasibility=2
        else
            [SoilFullInPoint(i), unload_capacity(i)] = JudgeSoilFullInPoint(CandidateUnloadPoints(i,:), select_soil_radius,...
            full_MeanHeight, ptCloud_truck, num_soilPoint, truck_array, JudgeFullBias);%上次记录的可达位置未满的情况下，进行本次判断

            if SoilFullInPoint(i) == 0 %可达的位置未装满，选择当前位置为下一次卸载点
                unloadPoint = CandidateUnloadPoints(i,:);
                unloadJoint = joint_angles(i,:);
                feasibility(i,2) = 1;
                break
            end
        end

    end
    if i == 1%若判断到最近位置都没找到未装满的点，判断此时矿卡整体装满
        TruckFullFlag = 1;
    end
end


%最大最小斗数限制
middleFeasibleIndex = round(length(case_feasible) / 2);
middleUP = CandidateUnloadPoints(case_feasible(middleFeasibleIndex), :);
if LoadNums <= MaxLoadNum 
    if isempty(find(SoilFullInPoint == 0,1)) %如果候选点全部判断装满isempty(find(SoilFullInPoint == 0))
        if LoadNums <= MinLoadNum
            unloadJoint = joint_angles(case_feasible(middleFeasibleIndex), :);
            unloadPoint = middleUP;
            TruckFullFlag = 0;
            fprintf('LoadNums is less than MinLoadNum but SoilFullInPoint == 1! \n')
            return
        end
    
        fprintf('TruckFull! \n');
        TruckFullFlag = 1;
        unloadJoint = joint_angles(case_feasible(middleFeasibleIndex), :);
        unloadPoint = middleUP;
        return
    end
else
    % TruckFullFlag = 1;
    TruckFullFlag = 0;
    fprintf('LoadNums is more than MaxLoadNum! \n')
    unloadJoint = joint_angles(case_feasible(middleFeasibleIndex), :);
    unloadPoint = middleUP;
    return
end
formatSpec = '\n unload joint is \n swingAngle = %4.2f; \n boomAngle = %4.2f; \n armAngle = %4.2f; \n bucketAngle = %4.2f; \n ';
fprintf(formatSpec, unloadJoint(1), unloadJoint(2), unloadJoint(3), unloadJoint(4));

end

%% //*********************
% // Copyright (c) 2024, 智能化研究院决控技术所, All rights reserved.
% //	开发人员：朱安源
% //	改版履历
% //           版本 |    日 期    |开发人员| 改版描述
% // 当前版本：V1.0| 2024.3.15  | 朱安源 | 生成候选卸载点
function CandidateUnloadPoints = Calculate_candidate_points(truck_center, truck_size, truck_headingAngle, PresetTruckUnloadPoint, unloadPoint_HeightBias)

truck_center = reshape(truck_center,1,3);

CandidateUnloadPoints = repmat(truck_center(1:3), length(PresetTruckUnloadPoint), 1);
CandidateUnloadPoints(:, 1) = CandidateUnloadPoints(:, 1) + cos( truck_headingAngle)  .* PresetTruckUnloadPoint;
CandidateUnloadPoints(:, 2) = CandidateUnloadPoints(:, 2) + sin( truck_headingAngle)  .* PresetTruckUnloadPoint;
CandidateUnloadPoints(:, 3) = truck_center(3) + truck_size(3)/2 + unloadPoint_HeightBias;

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
% //	开发人员：朱安源
% //	改版履历
% //           版本 |   日 期   |开发人员| 改版描述
% // 当前版本：V1.0| 2024.3.15  | 朱安源 | 判断当前位置是否装满
function [SoilFullInPoint, unload_capacity] = JudgeSoilFullInPoint(UnloadPoint, selectsoilradius, full_AverageHeight, point_cloud_soil_truck, num_soilPoint, truck_array, JudgeBias)

judgePoint = UnloadPoint;
judgePoint(1) = UnloadPoint(1) + JudgeBias * cos(truck_array(7));
judgePoint(2) = UnloadPoint(2) + JudgeBias * sin(truck_array(7));
soilPointNum = length(point_cloud_soil_truck(vecnorm(point_cloud_soil_truck(:, 1:2) - judgePoint(:,1:2), 2, 2) < selectsoilradius));
unload_capacity = 0.;
if soilPointNum > num_soilPoint
    unload_capacity = double(mean(point_cloud_soil_truck(vecnorm(point_cloud_soil_truck(:, 1:2) - judgePoint(:,1:2), 2, 2) < selectsoilradius, 3)));
    
    if unload_capacity <= full_AverageHeight
       SoilFullInPoint = 0;
    else
       SoilFullInPoint = 1; 
    end
    
else 
    SoilFullInPoint = 0;
end

end
