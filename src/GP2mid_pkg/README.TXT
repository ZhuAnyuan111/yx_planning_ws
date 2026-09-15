1.0.0
基于GP1 23-3-3

1.0.1
选点至少和边界一样远

1.0.2
若栅格地图有异常点，停止选择对应方向

1.0.3
更改打印警告的时机，仅当MPC_SWITCH=1时警告地图异常
发送一个向量topic，包含GUI参数：track_prot, N_fans, ground_crash_prot, max_depth, ground_setting,
                            angle_ub_setting, angle_lb_setting, truck_crash_prot, target_ub, target_lb,
                            edge_extend, extend_angle, expnt, l_QV, truckpos_default_param[0],
                            truckpos_default_param[1], is_receive_truck
修正了float64array msg的发送

1.1.0
加入了对铲斗角的规划（精确到0.1度），目标尽量使得动臂关节、铲斗关节、铲斗齿尖在一条直线上
方向上土方量估算时，最大深度不超过铲斗长度
为栅格地图加入俯仰角误差矫正1.5度

1.1.1
比较t与t+1时刻相同方向上的土量变化，可以判断是否空挖，再将可挖掘土方量估计相应调整(E1.Z)。输出debug topic: /GPdbg_EmptyVolm
修正通讯机制，检测到flag下降沿再计算
GUI参数输出：track_prot,       N_fans,      ground_crash_prot,      max_depth,      ground_setting,
            angle_ub_setting, angle_lb_setting, truck_crash_prot,   target_ub,      target_lb,
            edge_extend,        extend_angle,   expnt,              decr_threshold, l_QV, 
            truckpos_default_param[0], truckpos_default_param[1], is_receive_truck

1.1.2
增加调试模式，持续算点（每秒1次）
修正通讯机制，用signal表示，最大时间或下降沿触发

1.1.3
加入斗杆碰地判断约束
GPlow时，可视化点为红色

1.1.4
GPlow时，仍然选择土量最大方向，而不是默认值

1.1.5
加入工作装置的可视化
计算成功后，gplow_flag=1

1.1.6
工作装置的可视化默认为白色，gplow时为红色
每车第一斗重置vol modifier

-- exhaustive --
1.1.7
使用穷举法，直接比较全部方向上所有点

1.1.8
利用挖掘初始点所在位置及方向，预测挖掘影响范围，最大化包含高于地面>30CM部分的停靠范围影响面积(为了简化运算，一个方向上所有点算一个值)
用最近5个边界点计算边界距离

1.1.8b
补全接收F1的topic
GUI参数输出：track_prot,       N_fans,      ground_crash_prot,      max_depth,      ground_setting,
            angle_ub_setting, angle_lb_setting, truck_crash_prot,   target_ub,      target_lb,
            edge_extend,        extend_angle,   expnt,              decr_threshold, l_QV, 
            truckpos_default_param[0], truckpos_default_param[1], is_receive_truck, parking_lr, parking_forw

1.1.9
加入挖掘范围的可视化
计算terrain值取临近方向平均值
extend_angle异常时，输出履带保护默认值4
将部分参数移动到头文件内，方便移植
在寻找土方量满足的最优点时，找pareto最优

1.2.0
当挖掘影响范围过小时（<1），采用最大化土方量策略
修复履带保护分类计算的错误bug
分段预测轨迹，前半为指数，后半为线性，分段点可调(F1.Z)
计算挖掘影响范围时，最近处改为预测轨迹的分段点（程工数据为6m）
GUI参数输出：track_prot,       N_fans,      ground_crash_prot,      max_depth,      ground_setting,
            angle_ub_setting, angle_lb_setting, truck_crash_prot,   target_ub,      target_lb,
            edge_extend,        extend_angle,   expnt,              decr_threshold, l_QV, 
            truckpos_default_param[0], truckpos_default_param[1], is_receive_truck, parking_lr, parking_forw
            turn_point

1.2.1
实时输出铲斗齿尖位置可视化
输出预测挖掘轨迹可视化
更改debug模式计算时机

1.2.2
补全注释

1.2.3
解决地面平移参数不起效的bug
履带保护不起效的bug

1.2.4
加入参考轨迹可视化

1.2.5
令预测轨迹全程不低于起始点高度
小于高度阈值的土量认为无法挖掘,加入对应可视化为紫色

1.2.5b
新决策交互
reset msg时不重置数据，GPfinish改为float64。0：默认，1：成功，-1：失败，-2：Gplow，数列长度为804
在矿卡离场后挖掘，利用markers长度判断挖掘范围左右

1.2.5c
修复finishflag的bug

1.2.5d
修复地形传输错误重置的bug
修改了无矿卡的默认位置

1.2.5e
减小了触发计算的动臂角度阈值（改为16度）

1.2.6
铲斗角度话题补发flag
4m以内terrain改为履带下表面高度
可视化地形
修复无矿卡时挖掘范围设置bug

1.2.7
输出计算时间戳

1.2.8
输出异常检测topic
修改挖一铲等待逻辑，挖最外侧30度

GPmid 1.0.0
修复了逆运动学因精度导致的无解bug

1.0.1
修复了话题名错误bug

1.0.2
读取地图时同时读取尺寸参数
去除角度倾斜矫正

1.0.3
下车回转中心高度参数修正为0.7
nan值地形方向不选
将正北方向设为180度

1.0.4
计算时机改为等待3s

1.0.5
回调函数读取地图使用移动语义

1.0.6
挖掘范围角度根据东北天设置

1.0.7
X轴方向设为可变，F0: Z
解决了因为逆运动学无解导致的角度错误输出
标定角度更新

1.0.8
加入最近挖掘限制 F1:X

###1.0.0
整理了一些dig变量

###fan2parts1.0.0
将扇区分为两端不连续的扇区

###fan2parts1.0.1
加入反向的角度随机偏置

###truck_avoid1.0.0
加入卡车随机位置，避开卡车区域搜索

###truck_avoid1.0.1
加入矿卡中心点角度，简化修改判断矿卡占据角度范围逻辑

###truck_avoid and work_space1.0.0
在遍历搜索时避让卡车角度，添加显示卡车角度范围

###truck_avoid and work_space1.0.1
理清卡车角度显示范围，在避让卡车角度函数中将卡车占据角度分解成了点集，方便之后的显示

###truckRangeDisplay1.0.2
将卡车占据角度范围的工作区间id剔除出去，创建safe_fan_ids向量接收无卡车占据的id,方便之后的分区检索等

###truckRangeDisplay1.0.3
将target_ub定义为足够好的土方量，达到该土方量则跳出检索循环，提高搜索速度，避免挖掘点过深导致撑车

###truckRange&target_ub1.0.1
加入矿卡车厢四点坐标和矿卡中心点坐标的回调函数，让truck_angle_front等全局变量接收对应坐标值，实现矿卡范围的随机

###truckRange&target_ub1.0.2
修改坐标系之间的错误，发现挖机坐标系和挖掘坐标系之间存在180度偏差，angle挖掘 = angle挖机 + 180度
修改最小遍历距离digx_min由4m到6m，因为过小会导致斗杆过度内收，无法以合适的角度进入挖掘

###truckRange&target_ub1.0.3
改正truckVertices_callback()函数，增加讨论角度横跨0/2M_PI的情况
修改矿卡显示效果

###truckRange&target_ub1.0.4
栅格地图坐标系改动为与挖机坐标系同样高度，所以改动boom_pos位置索引
增加显示挖掘土方量在target_ub和target_lb之间的显示

###GPmid_2digArea_1.0.0
更改工作区逻辑，选择地形建模区域中的两个工作区，由angle_near_ub_setting1等定义
对planFlag_callback()函数增加注释

###huzhou870_1.0.0
将矿卡的显示角度从isdangerangle()函数中分离出来，单独作为一个函数；
将两个挖掘区间增加显示

###huzhou870_1.0.1
支持扫描角度输入大于360度（如350~390）的格式：
在isInDangerZone()中对输入angle做[0,2π)归一化，避免卡车避让角度判断误差；
在onestep_choose()最终赋值处对dig_swing做[0,2π)归一化，保证输出角度不越界

###huzhou870_1.0.2
对最终的dig_swing加入正负20度随机角度偏置
修改GP.h中的参数，将保护的地面参数设置为4.7m，增加地面的第一个元素的显示

###huzhou870_1.0.1
添加越界保护

###huzhou870_randomOffset_yaml_1.0.0
将gui发布的话题参数改为yaml配置文件读取
启动方式变为launch文件启动
对msg_write()函数进行修改，对读取dig_terrain[i]时增加长度保护

###huzhou870_rd_yaml_1.0.1
修改map_construct()函数，使用elevmat的实际行列数避免尺寸不匹配导致的索引越界

###huzhou870_rd_yaml_1.0.2
修改了msg_write()函数：增加尺寸检查，避免数组越界，修复地形/轨迹可视化的 Eigen 断言（尺寸不匹配时用 NaN 兜底 + 节流 WARN）
修改Algrthm.cpp()函数，对safe_fan_id为空时进行处理

###huzhou870_rd_yaml_1.0.3
修改safe_fan_id为空时的处理逻辑，给出warning提示,给出nan值解
修改矿卡显示效果
修改随机偏置逻辑，计算挖掘点回转角度与最近矿卡保护角度的最近距离与方向，
在该方向上的随机偏置不超过最近矿卡角，另一方向上不超过20度

###huzhou870_rd_yaml_1.0.4
将地面保护参数ground_def改成yaml文件读取

###huzhou870_1.0.5
修改随机偏置逻辑，挖掘点回转角度到最近矿卡保护角度的方向上的偏置角度也不超过20度

###huzhou870_1.0.6
boom_pitch由boom_pitch_topic改为joints_angle话题接收
将挖掘深度改为yaml文件可配置项

###huzhou870_1.0.7
修改随机偏置逻辑，得出挖掘点回转角度到最近矿卡保护角度的方向，
计算挖掘点回转角度反方向到达矿卡另一边界的角度距离，取20度和该角度的最小值作为反向偏置角度范围

###huzhou870_1.0.8
增加扫描区间和两挖掘子区间的输入保护，
当扫描区间的ub < lb 时，将ub增加2派，并确保两挖掘子区间在扫描区间的角度范围内

###huzhou870_1.0.9
增加心跳安全检测，心跳频率为10Hz
FAULT_0：gridmap过多Nan值
FAULT_1：gridmap高度值超出范围
FAULT_2：gridmap标签值超出范围
FAULT_3：算法规划结果无效/无解 

###huzhou870_1.0.10
规范化输入地形区域，可挖掘区域，
现在可以直接输入0到360内的数值，lb为起点，ub为终点,逆时针方向
增添挖掘逻辑，选择剔除矿卡后最大的连续安全区域进行挖掘

###huzhou870_1.0.11
增加挖机参数yaml文件读取

###huzhou870_1.0.12
增加随机偏置后的回转角度必须在设置挖掘区域之间的限制

###huzhou870_1.0.13
将随机偏置的可选回转角度限制放宽，并且这个可选范围会在rviz中显示出来
在两挖掘区前后各自增加可配置的角度范围，以在超调情况下依旧能挖到挖掘区域的边界
这个角度范围设置默认设置为10度

###huzhou870_1.0.14
放宽随机偏置的范围，现在可以进入矿卡区域内10度，
且这个10度可以通过danger_zone_margin_deg参数进行配置，当为0时则不进入矿卡区域

###huzhou870_1.0.15
将挖机尺寸通过其他yaml文件读取
FAULT_3：算法规划结果无效/无解的判断条件有所改动，改为dig_flag == -3
此时请求接管

###huzhou870_1.0.16
修改yaml文件部分参数

###huzhou870_1.0.17
将挖机尺寸的读取又改回由gp_params.yaml文件读取

###huzhou870_1.0.18
将下地面的保护距离改为也可以通过输入下地面海拔高来获取，并在yaml文件中设置模式开关use_rtk_ground_def
增添打印保护下地面与动臂铰点的距离ground_def

###huzhou870_1.0.19
将rtk海拔模式的下地面的保护距离限制在base2ground+boom_pos[2]和rtk_ground_def_max之间，
并添加超出限制的打印信息


###huzhou870_1.0.20
修改rtk海拔模式的下地面保护距离限制逻辑，
改为设置一个参考值rtk_ground_def_ref，在这个参考值一定范围内波动则使用计算出的rtk_ground_def，否则使用参考值

###huzhou870_1.0.21
记录扣除矿卡危险区的block，并在终端打印出来它们的角度范围

###huzhou870_1.0.22
将计算安全blocks的逻辑封装成一个函数

###huzhou870_1.0.23
接收/truck_pose_topic话题，如果接收到则覆盖yaml文件中设置的ground_protect_alt
新增truck_pose_ground_protect_alt_offset：truck上rtk到地面的高度

###huzhou870_1.0.24
显示回转中心的海拔高，在msgprocess.cpp中调整了一下变量显示顺序

###huzhou870_1.0.25
将r_limit也即地形扫描半径范围修改成yaml文件可配置项

###huzhou870_1.0.26
将结构参数读取改为从params_sy870.yaml文件优先读取

###huzhou870_1.0.27
将矿卡原始区域也增添显示

###huzhou870_1.0.28
修改搜索深度逻辑，深度搜索低于保护地面值时不再跳出，而是使用此时地形值到保护地面值之间的差值作为挖掘深度

###huzhou870_1.0.29
修复矿卡保护角过大时，矿卡占据区域范围越界bug
增添dig_swing在未偏置和偏置后的对比打印输出

###huzhou870_1.0.30
将base2ground改为从params_sy870.yaml文件优先读取

###huzhou870_1.0.31
将挖掘区的设置自动化：接收重新设置挖掘区的话题，通过yaml文件中的参数设置改动范围，改动完成后重新写入yaml文件

###huzhou870_1.0.32
更改自动设置挖掘区的触发逻辑，仅在/unman_params_set话题的position.y由1到2到0的情况下触发

###huzhou870_1.0.33
增添自动设置挖掘区的触发逻辑，只在/Mpc_Switch为false时可以设置

###huzhou870_1.0.34
增添挖机正前方车道角front_lane_deg和挖机后方线缆角rear_cable_deg
随着自动设置挖掘取的逻辑触发一并更新

###huzhou870_1.0.35
自动设置挖掘范围时，接收的挖机正前方航向角由rtk_can_msg话题改为Swing_topic话题接收
因为Swing_topic才是挖机坐标系下的航向角度

###huzhou870_1.0.36
自动设置挖掘区域后，将挖机正前方车道角front_lane_deg和挖机后方线缆角rear_cable_deg以话题形式发布100次
话题名为/dig_region_reference_angles

###huzhou870_1.0.37
增添开关mpc2_switch == true 时，停止挖掘点计算；进行清台时卸载点计算，发布卸载点；
mpc2_switch == false 时，停止计算和发布卸载点，进行挖掘点的计算和发布；
并修改打印输出，卸载模式输出卸载点，挖掘模式输出挖掘点

###huzhou870_1.0.38
话题/Sys_RGp_Unload.z增添发布卸载点的高度
修复无挖掘点信息时的msg_write()错误

###huzhou870_1.0.39
话题名/Mpc2_Switch改为/Mpc_Switch2

###huzhou870_1.0.40
添加卸载模式打印测试

###huzhou870_1.0.41
修复bug:mpc2_switch触发时，不再依赖calculate()进行触发计算卸载点

###huzhou870_1.0.42
增添发布/GP_dig_joints话题，包含挖掘时的关节角度信息(弧度角)

###huzhou870_1.0.43
修复bug:在调用 computeUnloadPlan() 前，如果地形/地面/角度数据未就绪，
就先执行一遍从 anormaly_check 到 ground_calculation 的预处理链

###huzhou870_1.0.44
修改/GP_dig_joints话题消息类型，由sensor_msgs/JointState改为geometry_msgs/Quaternion，
其x/y/z/w分量分别填充挖掘点的swing/boom/arm/bucket角度值(单位:度)