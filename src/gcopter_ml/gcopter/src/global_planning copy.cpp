#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

namespace gcopter {
//从外部获取模型的参数
struct Config
{
    int setpointTag; //1:setpoint,0:traj_follow
    std::string mapTopic;//地图话题
    std::string poseTopic;
    std::string ctrlTopic;
    std::string cmdTopic;
    std::string targetTopic;//起点和终点目标话题
    double dilateRadius;//膨胀半径.物理意义是啥？
    double voxelWidth;//体素宽度.物理意义是啥？
    std::vector<double> mapBound;//地图边界
    double timeoutRRT;//一种经典路径规划算法
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    double x_wide;
    double y_wide;
    double z_wide;
    double z_min;

    //从yaml文件获取变量取值
    Config(const ros::NodeHandle &nh)
    {
        nh.getParam("SetpointTag",setpointTag);
        nh.getParam("MapTopic", mapTopic);//voxel_map
        nh.getParam("PoseTopic",poseTopic);
        nh.getParam("CtrlTopic",ctrlTopic);
        nh.getParam("CmdTopic",cmdTopic);
        nh.getParam("TargetTopic", targetTopic);///move_base_simple/goal
        nh.getParam("DilateRadius", dilateRadius);//膨胀半径0.5
        nh.getParam("VoxelWidth", voxelWidth);//体素宽度
        nh.getParam("MapBound", mapBound);
        nh.getParam("TimeoutRRT", timeoutRRT);
        nh.getParam("MaxVelMag", maxVelMag);
        nh.getParam("MaxBdrMag", maxBdrMag);//
        nh.getParam("MaxTiltAngle", maxTiltAngle);
        nh.getParam("MinThrust", minThrust);
        nh.getParam("MaxThrust", maxThrust);
        nh.getParam("VehicleMass", vehicleMass);
        nh.getParam("GravAcc", gravAcc);
        nh.getParam("HorizDrag", horizDrag);
        nh.getParam("VertDrag", vertDrag);
        nh.getParam("ParasDrag", parasDrag);
        nh.getParam("SpeedEps", speedEps);
        nh.getParam("WeightT", weightT);
        nh.getParam("ChiVec", chiVec);
        nh.getParam("SmoothingEps", smoothingEps);
        nh.getParam("IntegralIntervs", integralIntervs);
        nh.getParam("RelCostTol", relCostTol);
        nh.getParam("X_Wide",x_wide);
        nh.getParam("Y_Wide",y_wide);
        nh.getParam("Z_Wide",z_wide);
        nh.getParam("Z_min",z_min);
    }
};

class TypeTransform
{
public:
    TypeTransform();
    ~TypeTransform();
    template<typename T>
    static inline Eigen::Vector3d RosMsg2Eigen(const T& p)
    {
        Eigen::Vector3d ev3(p.x, p.y, p.z);
        return ev3;
    }
    static inline void Eigen2RosMsg(const Eigen::Vector3d& ev3, geometry_msgs::Vector3& gv3)
    {
        gv3.x = ev3[0], gv3.y = ev3[1], gv3.z = ev3[2];
    }
private:
};

//全局规划器
class GlobalPlanner
{
private:
    int setpointTag; //1:setpoint,0:traj_follow
    std::string mapTopic;//地图话题
    std::string poseTopic;
    std::string ctrlTopic;
    std::string cmdTopic;
    std::string targetTopic;//起点和终点目标话题
    double dilateRadius;//膨胀半径.物理意义是啥？
    double voxelWidth;//体素宽度.物理意义是啥？
    std::vector<double> mapBound;//地图边界
    double timeoutRRT;//一种经典路径规划算法
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    double x_wide;
    double y_wide;
    double z_wide;
    double z_min;
    // Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;    //订阅地图
    ros::Subscriber poseSub;   
    ros::Subscriber ctrlSub;
    ros::Subscriber targetSub; //订阅终点

    ros::Publisher cmdPub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap; //体素地图
    Visualizer visualizer;
    geometry_msgs::PoseStamped current_pose;
    mavros_msgs::AttitudeTarget ctrl_last;
    std::vector<Eigen::Vector3d> startGoal; //起点+终点
    
    Trajectory<5> traj; //5条子轨迹
    double trajStamp;   //轨迹的时间戳

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        nh.getParam("SetpointTag",setpointTag);
        nh.getParam("MapTopic", mapTopic);//voxel_map
        nh.getParam("PoseTopic",poseTopic);
        nh.getParam("CtrlTopic",ctrlTopic);
        nh.getParam("CmdTopic",cmdTopic);
        nh.getParam("TargetTopic", targetTopic);///move_base_simple/goal
        nh.getParam("DilateRadius", dilateRadius);//膨胀半径0.5
        nh.getParam("VoxelWidth", voxelWidth);//体素宽度
        nh.getParam("MapBound", mapBound);
        nh.getParam("TimeoutRRT", timeoutRRT);
        nh.getParam("MaxVelMag", maxVelMag);
        nh.getParam("MaxBdrMag", maxBdrMag);//
        nh.getParam("MaxTiltAngle", maxTiltAngle);
        nh.getParam("MinThrust", minThrust);
        nh.getParam("MaxThrust", maxThrust);
        nh.getParam("VehicleMass", vehicleMass);
        nh.getParam("GravAcc", gravAcc);
        nh.getParam("HorizDrag", horizDrag);
        nh.getParam("VertDrag", vertDrag);
        nh.getParam("ParasDrag", parasDrag);
        nh.getParam("SpeedEps", speedEps);
        nh.getParam("WeightT", weightT);
        nh.getParam("ChiVec", chiVec);
        nh.getParam("SmoothingEps", smoothingEps);
        nh.getParam("IntegralIntervs", integralIntervs);
        nh.getParam("RelCostTol", relCostTol);
        nh.getParam("X_Wide",x_wide);
        nh.getParam("Y_Wide",y_wide);
        nh.getParam("Z_Wide",z_wide);
        nh.getParam("Z_min",z_min);

        // std::cout<<"setpointTag = "<< config.setpointTag <<std::endl;
        if(config.setpointTag)
        {
            mapInit();

            mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

            targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                    ros::TransportHints().tcpNoDelay());
        }
        else if (config.setpointTag == 0)
        {
            targetSetting();
        }
    }

    inline void mapInit()
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth); //根据地图的长宽高、偏置，构建体素地图
    }

    //加载地图
    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)//地图初始化
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;//点云中实际点的个数?
            float *fdata = (float *)(&msg->data[0]);//点云数据
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;//当前点再数组中的序号

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))//该点为NAN或INF
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));//实际上是将每组点的坐标输入到函数
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    //轨迹最优化计算
    inline void plan()
    {
        std::vector<Eigen::Vector3d> route;//默认三维列向量
        if (startGoal.size() == 2 && config.setpointTag)//满足两个点且模式为1
        {
            //前端路径
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route); //根据起点和终点，地图信息计算粗路径
            //飞行走廊生成
            std::vector<Eigen::MatrixX4d> hPolys;//声明凸多面体序列，即飞行走廊
            std::vector<Eigen::Vector3d> pc;  //声明三维向量点集      
            voxelMap.getSurf(pc);//障碍物膨胀之后最外面的一层

            // std::cout<< "1111111111111" <<std::endl;  
            // for (size_t i = 0; i < pc.size(); ++i) 
            // for (size_t j = 0; j < pc[0].size(); ++j)         
            //    std::cout<< pc[i][j]<<"," ;
            //输入：粗路径，障碍物膨胀之后最外面的一层，体素地图的原点和最远点
            //输出：飞行走廊凸多面体序列，半平面表示的凸多面体
            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);//飞行走廊
            sfc_gen::shortCut(hPolys);//对飞行走廊进行短路操作

            //最优化计算轨迹
            if (route.size() > 1)//已经获得粗路径
            {
                visualizer.visualizePolytope(hPolys);//显示飞行走廊

                Eigen::Matrix3d iniState;//元素类型为double大小为3*3的矩阵变量
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();//[粗路径的起点，0,0]
                finState << route.back(),  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();//[粗路径的终点，0,0]

                gcopter::GCOPTER_PolytopeSFC gcopter;//继承凸多面体飞行走廊类

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penvisualizealtyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                //初始化约束参数
                Eigen::VectorXd magnitudeBounds(5);//物理参数限制
                Eigen::VectorXd penaltyWeights(5);//惩罚项权重
                Eigen::VectorXd physicalParams(6);//物理参数

                magnitudeBounds(0) = config.maxVelMag;//最大速度
                magnitudeBounds(1) = config.maxBdrMag;//最大机体角速率
                magnitudeBounds(2) = config.maxTiltAngle;//最大倾斜角
                magnitudeBounds(3) = config.minThrust;//最小推力
                magnitudeBounds(4) = config.maxThrust;//最大推力

                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];

                physicalParams(0) = config.vehicleMass;//质量
                physicalParams(1) = config.gravAcc;    //重力加速度
                physicalParams(2) = config.horizDrag;//水平阻力系数
                physicalParams(3) = config.vertDrag;//垂直阻力系数
                physicalParams(4) = config.parasDrag;//附加阻力系数
                physicalParams(5) = config.speedEps;//速度平滑因子
                const int quadratureRes = config.integralIntervs;//数值积分分辨率

                traj.clear(); //删除轨迹


                //轨迹优化参数设置
                //设置：设置时间正则系数，初始状态，终止状态，凸多面体序列，空，平滑系数，数值积分分辨率，物理参数极限，惩罚权重，物理参数
                if (!gcopter.setup_setpoints(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                //计算最优化的轨迹
                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }               

                //带时间戳的轨迹显示
                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route);//可视化轨迹和路径
                }
            }
        }
        else if (config.setpointTag == 0)
        {
            // std::cout<<"traj_fol plan entry"<<std::endl;

            Eigen::Matrix3Xd route_M;
            Eigen::Matrix3d iniState;
            Eigen::Matrix3d finState;//元素类型为double大小为3*3的矩阵变量

            double x_h = config.x_wide / 2;
            double y_h = config.y_wide / 2;
            double z_min = config.z_min;
            double z_wide = config.z_wide;

            route_M.resize(3,9);
            // route_M.col(0) = Eigen::Vector3d(0,0,0);
            route_M.col(0) = TypeTransform::RosMsg2Eigen(current_pose.pose.position);

            std::cout<<"position = "<<std::endl;
            std::cout<<current_pose.pose.position<<std::endl;

            route_M.col(1) = Eigen::Vector3d(x_h, y_h, z_min);
            route_M.col(2) = Eigen::Vector3d(x_h, -y_h, z_min + z_wide / 8);
            route_M.col(3) = Eigen::Vector3d(-x_h, y_h, z_min + z_wide / 4);
            route_M.col(4) = Eigen::Vector3d(-x_h, -y_h, z_min + 3 * z_wide / 8);

            route_M.col(5) = Eigen::Vector3d(x_h, y_h, z_min + z_wide / 2);
            route_M.col(6) = Eigen::Vector3d(x_h, -y_h, z_min + 5 * z_wide / 8);
            route_M.col(7) = Eigen::Vector3d(-x_h, y_h, z_min + 6 * z_wide / 8);
            route_M.col(8) = Eigen::Vector3d(-x_h, -y_h, z_min + 7 * z_wide / 8);
            
            // std::cout<<"setpoint suc"<<std::endl;

            iniState << route_M.leftCols(1), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();//[粗路径的起点，0,0]
            finState << route_M.rightCols(1),  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();//[粗路径的终点，0,0]

            gcopter::GCOPTER_PolytopeSFC gcopter;

            // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
            // penvisualizealtyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
            // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
            //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
            // initialize some constraint parameters
            //初始化约束参数
            Eigen::VectorXd magnitudeBounds(5);//物理参数限制
            Eigen::VectorXd penaltyWeights(5);//惩罚项权重
            Eigen::VectorXd physicalParams(6);//物理参数

            magnitudeBounds(0) = config.maxVelMag;//最大速度
            magnitudeBounds(1) = config.maxBdrMag;//最大机体角速率
            magnitudeBounds(2) = config.maxTiltAngle;//最大倾斜角
            magnitudeBounds(3) = config.minThrust;//最小推力
            magnitudeBounds(4) = config.maxThrust;//最大推力

            penaltyWeights(0) = (config.chiVec)[0];
            penaltyWeights(1) = (config.chiVec)[1];
            penaltyWeights(2) = (config.chiVec)[2];
            penaltyWeights(3) = (config.chiVec)[3];
            penaltyWeights(4) = (config.chiVec)[4];

            physicalParams(0) = config.vehicleMass;//质量
            physicalParams(1) = config.gravAcc;    //重力加速度
            physicalParams(2) = config.horizDrag;//水平阻力系数
            physicalParams(3) = config.vertDrag;//垂直阻力系数
            physicalParams(4) = config.parasDrag;//附加阻力系数
            physicalParams(5) = config.speedEps;//速度平滑因子
            const int quadratureRes = config.integralIntervs;//数值积分分辨率

            traj.clear(); //删除轨迹


            //轨迹优化参数设置
            //设置：设置时间正则系数，路径，初始状态，最终状态，空，平滑系数，数值积分分辨率，物理参数极限，惩罚权重，物理参数
            if (!gcopter.setup(config.weightT,
                               route_M,iniState,
                               finState,INFINITY,
                               config.smoothingEps,
                               quadratureRes,
                               magnitudeBounds,
                               penaltyWeights,
                               physicalParams))
            {
                return;
            }

            //计算最优化的轨迹
            if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
            {
                return;
            }               

            // //带时间戳的轨迹显示
            // if (traj.getPieceNum() > 0)
            // {
            //     for (int i = 0; i < route_M.cols(); i++)
            //         route[i] = route_M.col(i);
                
            trajStamp = ros::Time::now().toSec();
                visualizer.visualize(traj, route);//可视化轨迹和路径
            // }
        }
    }

    inline void stateCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose = *msg;
    }

    inline void ctrlCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
    {
        ctrl_last = *msg;
        // trajStamp = ctrl_last.header.stamp;
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }

            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius); //目标点的高度
            // const double zGoal = msg->pose.position.z; //traj following exc

            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);//目标点的坐标

            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());//可视化显示
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan(); //轨迹规划
        }
        return;
    }

    inline void targetSetting()
    {
        poseSub = nh.subscribe(config.poseTopic, 1, &GlobalPlanner::stateCallBack, this,
                               ros::TransportHints().tcpNoDelay());
            
        ctrlSub = nh.subscribe(config.ctrlTopic, 1, &GlobalPlanner::ctrlCallback, this,
                                   ros::TransportHints().tcpNoDelay());

        cmdPub = nh.advertise<quadrotor_msgs::PositionCommand>(config.cmdTopic,10);

        plan(); //轨迹规划
    }

    inline void cmdPublish(const Eigen::Vector4d &quat, const Eigen::Vector3d &omg, const Eigen::Vector3d &pos,
                            const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, const Eigen::Vector3d &jer)
    {
        quadrotor_msgs::PositionCommand cmdMsg;
        cmdMsg.position.x = pos(0);
        cmdMsg.position.y = pos(1);
        cmdMsg.position.z = pos(2);
        cmdMsg.velocity.x = vel(0);
        cmdMsg.velocity.y = vel(1);
        cmdMsg.velocity.z = vel(2);
        cmdMsg.acceleration.x = acc(0);
        cmdMsg.acceleration.y = acc(1);
        cmdMsg.acceleration.z = acc(2);
        cmdMsg.jerk.x = jer(0);
        cmdMsg.jerk.y = jer(1);
        cmdMsg.jerk.z = jer(2);
        cmdMsg.yaw = atan2(2.0*(quat(2)*quat(3) + quat(0)*quat(1)), quat(0)*quat(0) - quat(1)*quat(1) - quat(2)*quat(2) + quat(3)*quat(3));
        cmdMsg.yaw_dot = 0;

        cmdPub.publish(cmdMsg);

        // std::cout<<"quat = "<<quat.transpose()<<std::endl;
        // std::cout<<"yaw = "<<cmdMsg.yaw<<std::endl;
        // std::cout<<"omg.x = "<<omg(0)<<std::endl;
        // std::cout<<"omg.y = "<<omg(1)<<std::endl;
        // std::cout<<"omg.z = "<<omg(2)<<std::endl;
    }

    //计算总推力，姿态四元数，机体角速率
    inline void process()
    {
        // std::cout<<"planning process entry"<<std::endl;
        
        Eigen::VectorXd physicalParams(6); //物理参数
        physicalParams(0) = config.vehicleMass;//质量
        physicalParams(1) = config.gravAcc;//重力加速度
        physicalParams(2) = config.horizDrag;//水平阻力系数
        physicalParams(3) = config.vertDrag;//垂直阻力系数
        physicalParams(4) = config.parasDrag;//附加阻力系数
        physicalParams(5) = config.speedEps;//速度平滑因子

        flatness::FlatnessMap flatmap;//微分平坦
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));//将物理参数赋值给微分平坦的私有变量

        if (traj.getPieceNum() > 0)
        {
            // std::cout<<"flap count start"<<std::endl;

            const double delta = ros::Time::now().toSec() - trajStamp;//delta=当前时科-上一次规划的时刻
            if (delta > 0.0 && delta < traj.getTotalDuration())//delta小于轨迹的总时长
            {
                double thr;//总推力
                Eigen::Vector4d quat;//四元数
                Eigen::Vector3d omg;//角速率
                Eigen::Vector3d pos;
                Eigen::Vector3d vel;
                Eigen::Vector3d acc;
                Eigen::Vector3d jer;

                pos = traj.getPos(delta);
                vel = traj.getVel(delta);
                acc = traj.getAcc(delta);
                jer = traj.getJer(delta);

                flatmap.forward(vel,
                                acc,
                                jer,
                                0.0, 0.0,
                                thr, quat, omg); //利用微分平坦特性计算出总推力，姿态四元数，机体角速率

                cmdPublish(quat, omg, pos, vel, acc, jer);

                double speed = traj.getVel(delta).norm(); //航点的速度二范数
                double bodyratemag = omg.norm();//姿态角的二范数
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2))); //倾斜角

                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;//速度
                thrMsg.data = thr;//总推力
                tiltMsg.data = tiltangle;//倾斜角
                bdrMsg.data = bodyratemag;//机体角速率

                visualizer.speedPub.publish(speedMsg);//显示速度
                visualizer.thrPub.publish(thrMsg);//显示总推力
                visualizer.tiltPub.publish(tiltMsg);//显示倾斜角
                visualizer.bdrPub.publish(bdrMsg);//显示机体角速率
                visualizer.visualizeSphere(traj.getPos(delta), config.dilateRadius);//显示轨迹
            }
        }
    }
};

//主运行函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_); //全局规划器

    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process(); //根据微分平坦特性，计算总推力，姿态四元数，机体角速率
        ros::spinOnce();//调用ROS主题
        lr.sleep();//睡眠1s
    }

    return 0;
}
} //namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);
