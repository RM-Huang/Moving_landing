std_msgs/Header header

uint16 week                                                 # gps 周
float64 second                                              # gps 周内秒
float64 latitude                                            # 纬度(deg)
float64 longitude                                           # 经度(deg)
float32 altitude                                            # 海拔(m)
float32[3] position_stdev                                   # 位置标准差 [0]lat [1]lon [2]alt   (m)
float32 undulation                                          # 高程异常

float32 roll                                                # 翻滚角(deg)
float32 pitch                                               # 俯仰角(deg)
float32 yaw                                                 # 航向角(deg)  车体坐标系下双天线航向角，取值范围 [-180, +180]，遵循右手定则，逆时针为正。
float32[3] euler_stdev                                      # 欧拉角标准差 [0]roll [1]pitch [2]yaw   (deg)

float32 speed                                               # 地面速度
float32 heading                                             # 航迹角  为车体坐标系下速度航向角，也称航迹角。取值范围 [0, 360] ，顺时针为正。
float32 heading2                                            # 航向角  为车体坐标系下双天线航向角，取值范围 [0, 360] ，顺时针为正。

geometry_msgs/Vector3 enu_velocity                          # 东北天速度 [0]e 东 [1]n 北 [2]u 天 (m/s)
float32[3] enu_velocity_stdev                               # 东北天速度标准差 [0]e 东 [1]n 北 [2]u 天 (m/s)

geometry_msgs/Vector3 vehicle_angular_velocity              # 车辆 xyz 角速度 (deg/s)
geometry_msgs/Vector3 vehicle_linear_velocity               # 车辆 xyz 速度 (m/s)
geometry_msgs/Vector3 vehicle_linear_acceleration           # 车辆 xyz 加速度 有重力 (m/s2)
geometry_msgs/Vector3 vehicle_linear_acceleration_without_g # 车辆 xyz 加速度，无重力

geometry_msgs/Vector3 raw_angular_velocity                  # 设备原始 xyz 角速度 (deg/s)
geometry_msgs/Vector3 raw_acceleration                      # 设备原始 xyz 加速度 有重力

# 状态 
# stat[0] 组合状态 
#           0-初始化
#           1-卫导模式
#           2-组合导航模式
#           3-纯惯导模式    
# stat[1] GNSS状态
#           0-不定位不定向 
#           1-单点定位定向
#           2-伪距差分定位定向
#           3-组合推算
#           4-RTK 稳定解定位定向
#           5-RTK浮点解定位定向
#           6-单点定位不定向
#           7-伪距差分定位不定向
#           8-RTK稳定解定位不定向
#           9-RTK浮点解定位不定向
uint8[2] stat
float32 age                                                 # 差分龄期 s

uint16 ns                                                    # 主天线可见卫星颗数
uint16 ns2                                                   # 辅天线可见卫星颗数
uint16 leaps                                                 # 闰秒 s

float32 hdop                                                # 水平精度因子
float32 pdop                                                # 位置精度因子
float32 vdop                                                # 垂直精度因子
float32 tdop                                                # 种差精度因子
float32 gdop                                                # 几何精度因子

geometry_msgs/Vector3 ins2gnss_vector                       # 设备到天线位置，车辆坐标系下，x/y/z 轴杆臂，m 
geometry_msgs/Vector3 ins2body_angle                        # 车辆坐标系到设备坐标系旋转欧拉角，Z-X-Y顺序旋转，deg 
geometry_msgs/Vector3 gnss2body_vector                      # 天线到后轮中心位置，车辆坐标系下，x/y/z 轴杆臂。m
float32 gnss2body_angle_z                                   # 车辆朝向到GNSS定向方向旋转角，沿车辆坐标系Z轴旋转，deg


uint16 warning                                               # 异常标识
uint16 sensor_used                                           # 传感器使用标识
uint8[16] receiver                                           # 预留