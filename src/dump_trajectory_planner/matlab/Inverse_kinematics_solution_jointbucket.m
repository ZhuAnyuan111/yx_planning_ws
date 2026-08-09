function [feasibility, joint_angle] = Inverse_kinematics_solution_jointbucket(point, bucket_angle, excavator_param, bucket_bias)

length_boom = excavator_param(1); length_arm = excavator_param(2); length_bucket = excavator_param(3);
bias_x = excavator_param(4); bias_y = excavator_param(5); bias_z = excavator_param(6);

length_bucket = length_bucket * bucket_bias;
safe_bias = 2;
angleLimits = reshape((excavator_param(7:12)), 3,2);
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
