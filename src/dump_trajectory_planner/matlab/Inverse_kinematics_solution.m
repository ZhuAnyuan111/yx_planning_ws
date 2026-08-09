function [feasibility, joint_angle] = Inverse_kinematics_solution(point, bucket_attitude_angle, excavator_param, bucket_bias)

length_boom = excavator_param(1); length_arm = excavator_param(2); length_bucket = excavator_param(3);
bias_x = excavator_param(4); bias_y = excavator_param(5); bias_z = excavator_param(6); 
safe_bias = 2;
length_bucket = length_bucket * bucket_bias;
x = point(1); y = point(2); z = point(3);
angleLimits = reshape((excavator_param(7:12)), 3,2);
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
