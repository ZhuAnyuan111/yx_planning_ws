function [V_dig_final,dig_point_final] = fcn(x_start_init, x_end_init, start_z_offset, end_z_offset, V_ref_0, interpX, DP_interpZ, DP_groundZ, DP_depth, DP_pitch, l_boom, l_arm, l_bkt, bucket_angle)

[x_start, x_end_list] = fcn1(x_start_init, x_end_init);
[V_dig_set_init,dig_point_set_init] = fcn2();
interpZ_filtered = fcn3(DP_interpZ)';
interpZ = interpZ_filtered;
x_end_list_len = 5;

init_Error = 0;
V_dig_set_update = inf(5,1);
dig_point_set_update = zeros(10,3,5);
for n = 1:1:x_end_list_len
    state_V_dig_matrix_update = inf(9,16);
    state_cost_matrix_update = inf(9,16);
    Traj_matrix_update = inf(9,16);
    x_end = x_end_list(n);
    Volume_Boom_Down_diff = 2;
    step = 9;
    [Start_Point, End_Point, z_start, x_end_update, Flag_Error_1, Flag_Error_2]...
        = init_point_gen(x_start, x_end, interpX, interpZ_filtered, start_z_offset, end_z_offset,l_boom, l_arm, l_bkt, bucket_angle);
    [Boom_Up_range,List_Boom] = List_Boom_gen(Start_Point, End_Point, Volume_Boom_Down_diff);
    [List_Arm,List_Bucket] = List_Arm_Bucket_gen(Start_Point, End_Point);
    [state_cost_matrix_init,state_V_dig_matrix_init,Traj_matrix_init] = state_matrix_init();

    if n == 1
        V_dig_set = V_dig_set_init;
        dig_point_set = dig_point_set_init;
    else
        V_dig_set = V_dig_set_update;
        dig_point_set = dig_point_set_update;
    end



    for i = 2:step

        if i == 2
            state_V_dig_matrix = state_V_dig_matrix_init;
            state_cost_matrix = state_cost_matrix_init;
            Traj_matrix = Traj_matrix_init;
        else
            state_V_dig_matrix = state_V_dig_matrix_update;
            state_cost_matrix = state_cost_matrix_update;
            Traj_matrix = Traj_matrix_update;
        end
        [point_before_list,point_after_list] = point_list_gen(i,List_Boom,List_Arm,List_Bucket);
        action_V_dig_matrix = V_dig_cal(point_before_list,point_after_list,interpX,interpZ,l_boom, l_arm, l_bkt);
        temp_V_dig_matrix = V_dig_add(i,state_V_dig_matrix,action_V_dig_matrix);
        action_cost_matrix = constraint(i,point_before_list,point_after_list,action_V_dig_matrix,x_start,x_end,interpX,...
            interpZ_filtered, V_ref_0,List_Bucket,Boom_Up_range,z_start, DP_groundZ, DP_depth,DP_pitch,l_boom, l_arm, l_bkt);
        temp_cost_matrix = action_cost_add(i,state_cost_matrix,action_cost_matrix);
        [state_V_dig_matrix_update,state_cost_matrix_update,Traj_matrix_update] = fcn4(i,...
            state_V_dig_matrix,state_cost_matrix,Traj_matrix,temp_cost_matrix,temp_V_dig_matrix);

    end

    [Traj,dig_point,min_cost,V_dig] = ...
        fcn5(state_V_dig_matrix_update,state_cost_matrix_update,Traj_matrix_update,List_Boom,List_Arm,List_Bucket,End_Point);
    [V_dig_set_update,dig_point_set_update] = fcn6(n,init_Error,min_cost,V_dig_set,dig_point_set,dig_point,V_dig);

end

index = 1;
if max(V_dig_set_update) < V_ref_0
    [~,index] = max(V_dig_set_update);
else
    for i = 5:-1:1
        if V_dig_set_update(i) > V_ref_0
            index = i;
            break;
        end
    end
end

V_dig_final = V_dig_set_update(index);
dig_point_final = dig_point_set_update(:,:,index);

end


%%
function [x_start, x_end_list] = fcn1(x_start_init, x_end_init)

x_start = x_start_init;

x_end_1 = x_end_init;
if x_start - x_end_init > 3
    x_end_2 = x_start - 3;
else
    x_end_2 = x_end_1;
end

x_end_list = linspace(x_end_1, x_end_2, 5);
end
%%
function [V_dig_set,dig_point_set] = fcn2()

V_dig_set = inf(5,1);

dig_point_set = zeros(10,3,5);
dig_point_set(:,1,:) = 30;
dig_point_set(:,2,:) = -50;
dig_point_set(:,3,:) = 10;
end
%%
function interpZ_filtered = fcn3(interpZ)


len = length(interpZ);
interpZ_filtered = zeros(201,1);
for i = 1:1:len
    if i == 1 || i == len
        interpZ_filtered(i) = interpZ(i);
    elseif i == 2 || i == len - 1
        interpZ_filtered(i) = mean([interpZ(i-1),interpZ(i),interpZ(i+1)]);
    elseif i == 3 || i == len - 2
        interpZ_filtered(i) = mean([interpZ(i-2),interpZ(i-1),interpZ(i),...
            interpZ(i+1),interpZ(i+2)]);
    else
        interpZ_filtered(i) = mean([interpZ(i-3),interpZ(i-2),interpZ(i-1),...
            interpZ(i),interpZ(i+1),interpZ(i+2),interpZ(i+3)]);
    end
end
end
%%

function [state_V_dig_matrix_update,state_cost_matrix_update,Traj_matrix_update] = fcn4(i,...
    state_V_dig_matrix,state_cost_matrix,Traj_matrix,temp_cost_matrix,temp_V_dig_matrix)

state_set_len = 16;
[state_cost_matrix(i,:),Traj_matrix(i,:)] = min((temp_cost_matrix'));
state_cost_matrix_update = state_cost_matrix;
Traj_matrix_update = Traj_matrix;

for j = 1:1:state_set_len
    state_V_dig_matrix(i,j) = temp_V_dig_matrix(j,Traj_matrix_update(i,j));
end
state_V_dig_matrix_update = state_V_dig_matrix;
end

%%
function [Traj,dig_point,min_cost,V_dig] = fcn5(state_V_dig_matrix_update,state_cost_matrix_update,Traj_matrix_update,List_Boom,List_Arm,List_Bucket,End_Point)

Volume_Step = 9;
Traj = zeros(1,9);
[min_cost, index] = min(state_cost_matrix_update(Volume_Step,:));
V_dig = state_V_dig_matrix_update(Volume_Step,index);


Traj(1) = Traj_matrix_update(Volume_Step,index);
for i = 2:1:Volume_Step
    Traj(i) = Traj_matrix_update(Volume_Step - i + 2,Traj(i-1));
end
Traj = fliplr(Traj);

dig_point = zeros(Volume_Step + 1,3);
for i = Volume_Step + 1:-1:1
    if i == Volume_Step + 1
        dig_point(i,:) = End_Point;
    else
        dig_point(i,:) = [List_Boom(Traj(i)),List_Arm(i),List_Bucket(i)];
    end
end
end

%%
function [V_dig_set_update,dig_point_set_update] = fcn6(n,init_Error,min_cost,V_dig_set,dig_point_set,dig_point,V_dig)

if init_Error == 1 || min_cost == inf
    V_dig_set(n) = inf;
    dig_point_set(:,1:end,n) = ones(10,1) * [30,-50,10];
else
    V_dig_set(n) = V_dig;
    dig_point_set(:,:,n) = dig_point;
end

V_dig_set_update = V_dig_set;
dig_point_set_update = dig_point_set;

end
%%

function [Start_Point, End_Point, z_start, x_end_update, Flag_Error_1, Flag_Error_2] = init_point_gen(x_start, x_end, interpX, interpZ_filtered, start_z_offset, end_z_offset,l_boom, l_arm, l_bkt, bucket_angle)

start_index = floor((x_start - interpX(1))/0.05 + 1);
[Start_Point, Flag_Error_1] = inverse_start(interpX(start_index), interpZ_filtered(start_index) + start_z_offset,l_boom, l_arm, l_bkt, bucket_angle);
while Flag_Error_1 == 1 && x_start >= 7
    x_start = x_start - 0.5;
    start_index = floor((x_start - interpX(1))/0.05 + 1);
    [Start_Point, Flag_Error_1] = inverse_start(interpX(start_index), interpZ_filtered(start_index) + start_z_offset,l_boom, l_arm, l_bkt, bucket_angle);
end
[~,z_start] = trans(Start_Point,l_boom, l_arm, l_bkt);
end_index = floor((x_end - interpX(1))/0.05 + 1);
[End_Point, Flag_Error_2] = inverse_end(interpX(end_index), interpZ_filtered(end_index) + end_z_offset,l_boom, l_arm, l_bkt);

if End_Point(1) <= Start_Point(1) + 5
    End_Point(1) = Start_Point(1) + 5;
end
[x_end_update,~] = trans(End_Point,l_boom, l_arm, l_bkt);

end


%%
function [theta, flag_error_1] = inverse_start(X_Start,Z_Start,l_boom, l_arm, l_bkt, bucket_angle)

% bias_z = 1.048; bias_x = 0.18; 
bias_z = 0; bias_x = 0; 
length_boom = l_boom;
length_arm = l_arm;
length_bucket = l_bkt;
% 关节限位
boom_limit = deg2rad([-45.44, 55.28]);
arm_limit = deg2rad([-138.69, -37.35]);
bucket_limit = deg2rad([-133.48, 30.53]);
flag_error_1 = 0;
theta = [boom_limit(2), arm_limit(2), bucket_limit(2)];
X = X_Start;
Z = Z_Start;


edge_bucket_X = X - bias_x;
edge_bucket_Z = Z - bias_z;

bucket_angle = deg2rad(bucket_angle);
if bucket_angle <= 0
    theta_FQV = pi + bucket_angle;
else
    theta_FQV = pi - bucket_angle;
end

FV = (length_arm^2 + length_bucket^2 - 2 * cos(theta_FQV) * length_arm * length_bucket)^0.5;
CV = (edge_bucket_X^2 + edge_bucket_Z^2)^0.5;
cos_FCV = (length_boom^2 + (edge_bucket_X^2 + edge_bucket_Z^2) - FV^2)/(2 * length_boom * (edge_bucket_X^2 + edge_bucket_Z^2)^0.5);
cos_CFV = (length_boom^2 + FV^2 - CV^2)/(2 * length_boom * FV);
% mustBeInRange(cos_FCV,-1,1)
if -1 <= cos_FCV && cos_FCV <= 1 && -1 <= cos_CFV && cos_CFV <= 1
    theta_FCV = acos(cos_FCV);
    theta_CFV = acos(cos_CFV);
else
    flag_error_1 = 1;
    return
end

boom_angle = theta_FCV + atan2(edge_bucket_Z, (edge_bucket_X^2)^0.5);
theta_VFQ  = acos((FV^2 + length_arm^2 - length_bucket^2)/(2 * FV * length_arm));
if bucket_angle <= 0
    arm_angle  = theta_CFV + theta_VFQ - pi;
else
    arm_angle  = theta_CFV - theta_VFQ - pi;
end

% rad2deg(boom_angle)
% rad2deg(arm_angle)
% rad2deg(bucket_angle)
% 判断逆解是否可行
if ~(boom_angle >= boom_limit(1) && boom_angle <= boom_limit(2) && ...
        arm_angle >= arm_limit(1) && arm_angle <= arm_limit(2) && ...
        bucket_angle >= bucket_limit(1) && bucket_angle <= bucket_limit(2))

    flag_error_1 = 1; % 如果无解，轨迹生成失败，可行性设为1
    return; % 结束循环
end

boom_angle = rad2deg(boom_angle);
arm_angle = rad2deg(arm_angle);
bucket_angle = rad2deg(bucket_angle);

theta = [boom_angle, arm_angle, bucket_angle];

end

%% Inverse_Transfer_End

function [theta,flag_error_2] = inverse_end(X_End,Z_End,l_boom, l_arm, l_bkt)

% SY750
% l_boom = 7; l_arm = 2.59965; 
% l_QV = 2.05747;

theta_end = -180;

flag_error_2 = 0;
theta = zeros(1,3);

l_CV = sqrt(X_End^2 + Z_End^2);

theta_dig = atand(-Z_End / X_End);
theta_CVQ = -theta_end - theta_dig;

l_CQ = sqrt(l_CV^2 + l_bkt^2 - 2 * l_CV * l_bkt * cosd(theta_CVQ));

if l_boom + l_arm - l_CQ <= 0 || l_CV + l_CQ - l_bkt  <= 0 ...
        || l_boom + l_CQ - l_arm <= 0 || l_arm + l_CQ - l_boom <= 0 ...
        || l_CQ + l_arm - l_boom <= 0
    flag_error_2 = 1;
    theta = [30 -50 10];
else

    theta_CFQ = acosd((l_boom^2 + l_arm^2 - l_CQ^2) / (2 * l_boom * l_arm));
    theta_2 = theta_CFQ - 180;
    theta_VCQ = acosd((l_CV^2 + l_CQ^2 - l_bkt^2) / (2 * l_CQ * l_CV));
    theta_FCQ = acosd((l_boom^2 + l_CQ^2 - l_arm^2) / (2 * l_boom * l_CQ));

    theta_1 = theta_VCQ + theta_FCQ + (90 - theta_dig) - 90;

    theta(1) = theta_1;
    theta(2) = theta_2;
    theta(3) = theta_end - theta_2 - theta_1;

    if theta(1) < -45.44 || theta(1) > 55.28 ...
            || theta(2) < -138.69 || theta(2) > -37.35 ...
            || theta(3) < -133.48 || theta(3) > 30.53
        flag_error_2 = 1;
    end
end
end

function [X,Z] = trans(point,l_boom, l_arm, l_bkt)
% SY870
% l_boom = 7; l_arm = 2.59965; 
% l_QV = 2.05747;

x = point(:, 1);
y = point(:, 2);
z = point(:, 3);

X = l_boom * cosd(x) + l_arm * cosd(x + y) + l_bkt * cosd(x + y + z);
Z = l_boom * sind(x) + l_arm * sind(x + y) + l_bkt * sind(x + y + z);
end

%%

function [List_Arm,List_Bucket] = List_Arm_Bucket_gen(Start_Point, End_Point)

Arm_Start = Start_Point(2);
Arm_End = End_Point(2);
Bucket_Start = Start_Point(3);
Bucket_End = End_Point(3);

List_Arm = List_Arm_generation(Arm_Start,Arm_End);
List_Bucket = List_Bkt_generation(Bucket_Start,Bucket_End);

end


function output_ = custom_Array(Array_range, Section_ratio, Section_length)

len = length(Section_length);
temp = 0;
for i = 1:1:len
    temp = temp + Section_ratio(i) * Section_length(i);
end
total_diff = Array_range(2) - Array_range(1);
unit = total_diff/temp;
cur_index = 1;
output_ = zeros(10,1);
output_(1) = Array_range(1);
for i = 1:1:len
    for j = 1:1:Section_length(i)
        cur_index = cur_index + 1;
        output_(cur_index) = output_(cur_index-1) + unit * Section_ratio(i);
    end
end
end


%% List_Arm_generation

function List_Arm = List_Arm_generation(Arm_Start,Arm_End)
    a1 = 0.0432;
    b1 = 2.4619;
    c1 = 4.4965;
    a2 = 0.0140;
    b2 = 8.1076;
    c2 = 1.7438;
    a3 = 0.9812;
    b3 = -5.1543;
    c3 = 36.8919;

    x = linspace(0,80,10)';

    List_Arm = (a1.*exp(-((x-b1)./c1).^2) + a2.*exp(-((x-b2)./c2).^2) + ...
            a3.*exp(-((x-b3)./c3).^2)) * (Arm_Start - Arm_End) + Arm_End;
    
    List_Arm(end) = Arm_End;

    for i = length(List_Arm):-1:2
        if List_Arm(i) > List_Arm(i-1)
            List_Arm(i-1) = List_Arm(i);
        end
    end
    List_Arm(1) = Arm_Start;
end



%% List_Bkt_generation

function List_Bucket = List_Bkt_generation(Bucket_Start,Bucket_End)
    a = -0.0004;
    b = 0.7769;
    c = 0.9965;
    d = 0.0015;

    x = 1:10;

    List_Bucket = (a.*exp(b.*x) + c.*exp(d.*x)).*...
            (Bucket_Start - Bucket_End) + Bucket_End;


    List_Bucket(end) = Bucket_End;
    for i = length(List_Bucket):-1:2
        if List_Bucket(i) > List_Bucket(i-1)
            List_Bucket(i-1) = List_Bucket(i);
        end
    end
    List_Bucket(1) = Bucket_Start;

    
    for i = 1:1:length(List_Bucket)-1
        List_Bucket(i) = (List_Bucket(i) - List_Bucket(1)) + List_Bucket(1);
    end
end


%%
function [Boom_Up_range,List_Boom] = List_Boom_gen(Start_Point, End_Point, Volume_Boom_Down_diff)

Boom_Start = Start_Point(1);
Boom_End = End_Point(1);
Boom_Up_range = Boom_End - Boom_Start;
Volume_Boom_Up = 12;
Volume_Boom_Down = 3;

List_Boom_Down = linspace(Boom_Start - Volume_Boom_Down*Volume_Boom_Down_diff,...
    Boom_Start,Volume_Boom_Down + 1);
List_Boom_Up = linspace(Boom_Start,Boom_End,Volume_Boom_Up + 1);
List_Boom = [List_Boom_Down(1:end - 1), List_Boom_Up]';

% Boom_Start = Start_Point(1);
% Boom_End = End_Point(1);
% 
% Boom_Up_range = floor(Boom_End - Boom_Start);
% if Boom_Up_range < Volume_Boom_Up
%     Volume_Boom_Up = Boom_Up_range;
% end
% state_set_len = Volume_Boom_Down + Volume_Boom_Up + 1;
% List_Boom_Down = linspace(Boom_Start - Volume_Boom_Down*Volume_Boom_Down_diff,...
%     Boom_Start,Volume_Boom_Down + 1);
% List_Boom_Up = linspace(Boom_Start,Boom_Start + Boom_Up_range,Volume_Boom_Up + 1);
% List_Boom = [List_Boom_Down(1:end - 1), List_Boom_Up]';

end

%%
function [point_before_list,point_after_list] = point_list_gen(i,List_Boom,List_Arm,List_Bucket)

% 生成前一个状态与后一个状态的两组点（关节空间）
state_set_len = 16;
point_before_list = zeros(state_set_len,3);
point_after_list = zeros(state_set_len,3);

point_before_list(:,1) = List_Boom;
point_before_list(1:end,2) = List_Arm(i-1);
point_before_list(1:end,3) = List_Bucket(i-1);
point_after_list(:,1) = List_Boom;
point_after_list(1:end,2) = List_Arm(i);
point_after_list(1:end,3) = List_Bucket(i);

end

%%
function [state_cost_matrix_init,state_V_dig_matrix_init,Traj_matrix_init] = state_matrix_init()

step = 9;
Volume_Boom_Down = 3;
state_set_len = 16;

state_cost_matrix_init = inf(step,state_set_len);
state_cost_matrix_init(1,Volume_Boom_Down + 1) = 0;

state_V_dig_matrix_init = inf(step,state_set_len);
state_V_dig_matrix_init(1,Volume_Boom_Down + 1) = 0;

Traj_matrix_init = zeros(step,state_set_len);
Traj_matrix_init(1, 1:state_set_len) = Volume_Boom_Down + 1;

end

%%
function temp_V_dig_matrix = V_dig_add(i,state_V_dig_matrix,action_V_dig_matrix)  

% 将单次转移挖掘的土方量矩阵与上一状态的历史土方量相加
% 得到基于上一状态下，下一状态的所有可能的挖掘土方量总和

state_set_len = 16;
temp_V_dig_matrix = action_V_dig_matrix + ones(state_set_len,1) * state_V_dig_matrix(i-1,:);

end

%%

function action_V_dig_matrix = V_dig_cal(point_before_list,point_after_list,interpX,interpZ,l_boom, l_arm, l_bkt)

% 计算并保存前后两个状态之间单次所有可能转移的挖掘土方量（每次循环时重置）

state_set_len = 16;
action_V_dig_matrix = inf(state_set_len,state_set_len);     % 单次转移土方量的临时储存矩阵
for j = 1:1:state_set_len
    for k = 1:1:state_set_len
        point_before = point_before_list(k,:);
        point_after = point_after_list(j,:);
        action_V_dig_matrix(j,k) = single_V_dig_cal(point_before,point_after,interpX,interpZ,l_boom, l_arm, l_bkt);
    end
end
end


function single_V_dig = single_V_dig_cal(point_before,point_after,interpX,interpZ,l_boom, l_arm, l_bkt)

% 计算单次转移的挖掘土方量
single_V_dig = 0;
Interval = 2;                                                               % 增加分辨率，提高计算准确度
point_Boom = linspace(point_before(1),point_after(1),Interval);
point_Arm = linspace(point_before(2),point_after(2),Interval);
point_Bucket = linspace(point_before(3),point_after(3),Interval);

point_before_temp_i = [point_Boom; point_Arm; point_Bucket]';
[x_temp_i, z_temp_i] = trans(point_before_temp_i,l_boom, l_arm, l_bkt);
x_terrain_temp_i = floor((x_temp_i - interpX(1)) / 0.05 + 1);               % 为了得到地形高度，确定地形数据的下标



for i = 1:1:Interval-1

    if x_terrain_temp_i(i) >= 201 || x_terrain_temp_i(i) <= 1 || isnan(x_terrain_temp_i(i)) || x_terrain_temp_i(i + 1) >= 201 || x_terrain_temp_i(i + 1) <= 1
        single_V_dig = inf;                                                 % 超出地形数据边界时，设挖掘土方量为无穷大，归为异常
        return
    else

        if z_temp_i(i) > interpZ(x_terrain_temp_i(i)) || z_temp_i(i + 1) > interpZ(x_terrain_temp_i(i+1))
            d_v_temp = -0.1;                                                   % 当用于计算的两个点其中一个点高于地形，设这两点间的挖掘土方量为0
        else
            % 计算土方量
            d_v_temp = polyarea([x_temp_i(i),x_temp_i(i+1),x_temp_i(i+1),x_temp_i(i)],...
                [z_temp_i(i),z_temp_i(i+1),interpZ(x_terrain_temp_i(i+1)),interpZ(x_terrain_temp_i(i))]);
            if x_terrain_temp_i(i) < x_terrain_temp_i(i+1)
                d_v_temp = 0;                                       % 当齿尖从近往远处移动时，土方量未负
            end
        end
        single_V_dig = single_V_dig + d_v_temp;
    end
end
end


%%

function action_cost_matrix = constraint(i,point_before_list,point_after_list,action_all_V_dig_matrix,X_Start,X_End,interpX,...
    interpZ_filtered, V_ref_0,List_Bucket,Boom_Up_range,Z_start, DP_groundZ, DP_depth,DP_pitch,l_boom, l_arm, l_bkt)

state_set_len = 16;
action_cost_matrix = inf(state_set_len,state_set_len);

for j = 1:1:state_set_len
    for k = 1:1:state_set_len
        point_before = point_before_list(k,:);
        point_after = point_after_list(j,:);
        single_V_dig = action_all_V_dig_matrix(j,k);
        action_cost_matrix(j,k) = single_cost_cal(i,point_before,point_after,single_V_dig,X_Start,X_End,interpX,...
            interpZ_filtered,V_ref_0,List_Bucket,Boom_Up_range,Z_start,DP_groundZ,DP_depth,DP_pitch,l_boom, l_arm, l_bkt);
    end
end
end


function single_cost = single_cost_cal(i,point_before,point_after,single_V_dig,...
    X_Start,X_End,interpX,interpZ_filtered,V_ref_0,List_Bucket,Boom_Up_range,Z_start,DP_groundZ,DP_depth,DP_pitch,l_boom, l_arm, l_bkt)

[x_after,z_after] = trans(point_after,l_boom, l_arm, l_bkt);

x_terrain_after = floor((x_after- interpX(1)) / 0.05 + 1);

if x_terrain_after < 1 || x_terrain_after > 201
    single_cost = inf;
    return;
end

z_terrain_after = interpZ_filtered(x_terrain_after);

X_terrain_End = floor((X_End- interpX(1)) / 0.05 + 1);
Z_terrain_End = interpZ_filtered(X_terrain_End);

X_Diff = X_Start - X_End;
Z_Diff = Z_terrain_End - Z_start;

% Cost 1: V_Dig
% V_ref =  abs(X_Start - x_after) / X_Diff * V_ref_0;
% cost_1 = abs(V_ref - V_dig);

% pitch < 0：前倾 pitch > 0：后仰

V_adjust = (X_Start - x_after) * (X_Start - x_after) * sind(-DP_pitch) / 2;

V_ref = (V_ref_0 - X_Diff * Z_Diff / 2) / X_Diff * (X_Start - x_after)...
    + Z_Diff / X_Diff * ((X_Start - x_after)^2) / 2 + V_adjust;


cost_1 = abs(V_ref - single_V_dig);


% Cost 2: forbit Boom_Up at the beginning
if (i >= 2 || i <= 8) && z_after > z_terrain_after
    cost_2 = 1000;
else
    cost_2 = 0;
end

% Cost 3: limit the variation of boom
% 根据动臂上升角度区间大小决定动臂提升限制
limit_boom = max(1,floor(Boom_Up_range/2));
if abs(point_after(1) - point_before(1)) > limit_boom ...
        && point_before(3) ~= List_Bucket(end)
    cost_3 = inf;
else
    cost_3 = 0;
end

% Cost 4: forbit 3-componud action(except Boom_Down)
if point_after(1) > point_before(1) && point_after(2) ~= point_before(2)...
        && point_after(3) ~= point_before(3)
    cost_4 = inf;
else
    cost_4 = 0;
end

% Cost 5: forbit Boom_Down before arriving the gravity-point
if point_before(1) + point_before(2) > -90 && point_after(1) < point_before(1)
    cost_5 = inf;
else
    cost_5 = 0;
end

% Cost 6: forbit Boom_Down during Bucket_Up
if point_after(1) < point_before(1) && point_after(2) == point_before(2)
    cost_6 = inf;
else
    cost_6 = 0;
end

% Cost 7: forbit back_step except the last 2 steps
if i <= 2 && point_after(1) > point_before(1)
    cost_7 = inf;
else
    cost_7 = 0;
end

% Cost 8: 
if i >= 2 && point_after(1) == point_before(1) && point_after(3) == point_before(3)
    cost_8 = inf;
else
    cost_8 = 0;
end

% Cost 9: 
if i >= 3 && point_after(1) == point_before(1) && point_after(2) == point_before(2)
    cost_9 = inf;
else
    cost_9 = 0;
end

% Cost 10:
if z_after < DP_groundZ(x_terrain_after) + DP_depth
    cost_10 = 1000;
else
    cost_10 = 0;
end

% Cost 11: depth
if i >= 1 && i <= 6
    cost_11 = abs(z_after - Z_start);
else
    cost_11 = 0;
end

a = 2;

single_cost =  cost_1 + cost_2 + cost_3 + cost_5 + cost_6 + cost_10;
% single_cost =  cost_1 + cost_2;
end


%%
function temp_cost_matrix = action_cost_add(i,state_cost_matrix,action_cost_matrix)  

state_set_len = 16;
temp_cost_matrix = action_cost_matrix + ones(state_set_len,1) * state_cost_matrix(i-1,:);
end