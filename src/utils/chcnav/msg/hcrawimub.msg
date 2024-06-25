std_msgs/Header header

uint16 week                                             # gps 周
float64 second                                          # gps 周内秒

geometry_msgs/Vector3 angular_velocity                  # 设备原始 xyz 角速度 (deg/s)/Vector3 vehicle_angular_velocity              # 车辆 xyz 角速度 (deg/s)
geometry_msgs/Vector3 angular_acceleration              # 设备原始 xyz 角加速度 有重力
float32 temp                                            # IMU温度（摄氏度）
int8 err_status                                         # 状态异常标识
int16 yaw                                               # Z轴陀螺积分航向, 180~180 系数 0.01
int8 receiver                                           # 预留