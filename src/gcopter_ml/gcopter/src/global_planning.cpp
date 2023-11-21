#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrajctrlTrigger.h>
#include <quadrotor_msgs/TrajcurDesire.h>

#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <thread>

namespace gcopter {

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
class GlobalPlanner : public nodelet::Nodelet
{
private:
    std::thread initThread_;
    ros::Timer process_timer;

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
    ros::Subscriber ctrltriSub;
    ros::Subscriber targetSub; //订阅终点

    ros::Publisher cmdPub;
    ros::Publisher desPub;

    /* visualize topic */
    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher meshPub;
    ros::Publisher edgePub;
    ros::Publisher spherePub;
    ros::Publisher speedPub;
    ros::Publisher thrPub;
    ros::Publisher tiltPub;
    ros::Publisher bdrPub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap; //体素地图
    Visualizer visualizer;
    geometry_msgs::PoseStamped current_pose;
    quadrotor_msgs::TrajctrlTrigger ctrl_start_trigger;
    std::vector<Eigen::Vector3d> startGoal; //起点+终点
    
    Trajectory<5> traj; //5条子轨迹
    double trajStamp;   //轨迹的时间戳
    double traj_start_stamp;

public:

    inline void mapInit()
    {
        const Eigen::Vector3i xyz((mapBound[1] - mapBound[0]) / voxelWidth,
                                  (mapBound[3] - mapBound[2]) / voxelWidth,
                                  (mapBound[5] - mapBound[4]) / voxelWidth);

        const Eigen::Vector3d offset(mapBound[0], mapBound[2], mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, voxelWidth); //根据地图的长宽高、偏置，构建体素地图
    }

    //加载地图
    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)//地图初始化
        {
            std::size_t cur = 0;
            const std::size_t total = msg->data.size() / msg->point_step;//点云中实际点的个数?
            float *fdata = (float *)(&msg->data[0]);//点云数据
            for (std::size_t i = 0; i < total; i++)
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

            voxelMap.dilate(std::ceil(dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    //轨迹最优化计算
    inline void plan()
    {
        ROS_INFO("start planning!");
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;
        std::vector<Eigen::Vector3d> route;//默认三维列向量
        if (startGoal.size() == 2 && setpointTag)//满足两个点且模式为1
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
            // for (std::size_t i = 0; i < pc.size(); ++i) 
            // for (std::size_t j = 0; j < pc[0].size(); ++j)         
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
                visualization_msgs::Marker meshMarker, edgeMarker;
                visualizer.visualizePolytope(hPolys, meshMarker, edgeMarker);//显示飞行走廊
                meshPub.publish(meshMarker);
                edgePub.publish(edgeMarker);

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

                magnitudeBounds(0) = maxVelMag;//最大速度
                magnitudeBounds(1) = maxBdrMag;//最大机体角速率
                magnitudeBounds(2) = maxTiltAngle;//最大倾斜角
                magnitudeBounds(3) = minThrust;//最小推力
                magnitudeBounds(4) = maxThrust;//最大推力

                penaltyWeights(0) = (chiVec)[0];
                penaltyWeights(1) = (chiVec)[1];
                penaltyWeights(2) = (chiVec)[2];
                penaltyWeights(3) = (chiVec)[3];
                penaltyWeights(4) = (chiVec)[4];

                physicalParams(0) = vehicleMass;//质量
                physicalParams(1) = gravAcc;    //重力加速度
                physicalParams(2) = horizDrag;//水平阻力系数
                physicalParams(3) = vertDrag;//垂直阻力系数
                physicalParams(4) = parasDrag;//附加阻力系数
                physicalParams(5) = speedEps;//速度平滑因子
                const int quadratureRes = integralIntervs;//数值积分分辨率

                traj.clear(); //删除轨迹


                //轨迹优化参数设置
                //设置：设置时间正则系数，初始状态，终止状态，凸多面体序列，空，平滑系数，数值积分分辨率，物理参数极限，惩罚权重，物理参数
                if (!gcopter.setup_setpoints(weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                //计算最优化的轨迹
                if (std::isinf(gcopter.optimize(traj, relCostTol)))
                {
                    return;
                }               

                //带时间戳的轨迹显示
                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route, routeMarker, wayPointsMarker, trajMarker);//可视化轨迹和路径
                }
            }
        }
        else if (setpointTag == 0)
        {
            // std::cout<<"traj_fol plan entry"<<std::endl;

            Eigen::Matrix3Xd route_M;
            Eigen::Matrix3d iniState;
            Eigen::Matrix3d finState;//元素类型为double大小为3*3的矩阵变量

            double x_h = x_wide / 2;
            double y_h = y_wide / 2;

            route_M.resize(3,9);
            // route_M.col(0) = Eigen::Vector3d(0,0,0);
            route_M.col(0) = TypeTransform::RosMsg2Eigen(current_pose.pose.position);

            // std::cout<<"position = "<<std::endl;
            // std::cout<<current_pose.pose.position<<std::endl;

            // route_M.col(1) = Eigen::Vector3d(x_h, y_h, z_min);
            // route_M.col(2) = Eigen::Vector3d(-x_h, y_h, z_min + z_wide / 8);
            // route_M.col(3) = Eigen::Vector3d(x_h, -y_h, z_min + z_wide / 4);
            // route_M.col(4) = Eigen::Vector3d(-x_h, -y_h, z_min + 3 * z_wide / 8);

            // route_M.col(5) = Eigen::Vector3d(x_h, y_h, z_min + z_wide / 2);
            // route_M.col(6) = Eigen::Vector3d(-x_h, y_h, z_min + 5 * z_wide / 8);
            // route_M.col(7) = Eigen::Vector3d(x_h, -y_h, z_min + 6 * z_wide / 8);
            // route_M.col(8) = Eigen::Vector3d(-x_h, -y_h, z_min + 7 * z_wide / 8);

            route_M.col(1) = Eigen::Vector3d(current_pose.pose.position.x+0.3, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(2) = Eigen::Vector3d(current_pose.pose.position.x+0.6, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(3) = Eigen::Vector3d(current_pose.pose.position.x+0.9, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(4) = Eigen::Vector3d(current_pose.pose.position.x+1.2, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(5) = Eigen::Vector3d(current_pose.pose.position.x+1.5, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(6) = Eigen::Vector3d(current_pose.pose.position.x+1.8, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(7) = Eigen::Vector3d(current_pose.pose.position.x+2.1, current_pose.pose.position.y, current_pose.pose.position.z);
            route_M.col(8) = Eigen::Vector3d(current_pose.pose.position.x+2.4, current_pose.pose.position.y, current_pose.pose.position.z);
            
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

            magnitudeBounds(0) = maxVelMag;//最大速度
            magnitudeBounds(1) = maxBdrMag;//最大机体角速率
            magnitudeBounds(2) = maxTiltAngle;//最大倾斜角
            magnitudeBounds(3) = minThrust;//最小推力
            magnitudeBounds(4) = maxThrust;//最大推力

            penaltyWeights(0) = (chiVec)[0];
            penaltyWeights(1) = (chiVec)[1];
            penaltyWeights(2) = (chiVec)[2];
            penaltyWeights(3) = (chiVec)[3];
            penaltyWeights(4) = (chiVec)[4];

            physicalParams(0) = vehicleMass;//质量
            physicalParams(1) = gravAcc;    //重力加速度
            physicalParams(2) = horizDrag;//水平阻力系数
            physicalParams(3) = vertDrag;//垂直阻力系数
            physicalParams(4) = parasDrag;//附加阻力系数
            physicalParams(5) = speedEps;//速度平滑因子
            const int quadratureRes = integralIntervs;//数值积分分辨率

            traj.clear(); //删除轨迹


            //轨迹优化参数设置
            //设置：设置时间正则系数，路径，初始状态，最终状态，空，平滑系数，数值积分分辨率，物理参数极限，惩罚权重，物理参数
            if (!gcopter.setup(weightT,
                               route_M,iniState,
                               finState,INFINITY,
                               smoothingEps,
                               quadratureRes,
                               magnitudeBounds,
                               penaltyWeights,
                               physicalParams))
            {
                return;
            }

            //计算最优化的轨迹
            if (std::isinf(gcopter.optimize(traj, relCostTol)))
            {
                return;
            }        
            ROS_INFO("planning succeed! Total time duration: %f",traj.getTotalDuration());       

            // //带时间戳的轨迹显示
            // if (traj.getPieceNum() > 0)
            // {
            //     for (int i = 0; i < route_M.cols(); i++)
            //         route[i] = route_M.col(i);
                
            trajStamp = ros::Time::now().toSec();
            visualizer.visualize(traj, route, routeMarker, wayPointsMarker, trajMarker);//可视化轨迹和路径
            // }
        }
        routePub.publish(routeMarker);
        wayPointsPub.publish(wayPointsMarker);
        trajectoryPub.publish(trajMarker);
    }

    inline void stateCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose = *msg;
        std::cout<<"start_pose = "<<current_pose.pose.position.x<<" "<<current_pose.pose.position.y<<" "<<current_pose.pose.position.z<<std::endl;
    }

    inline void ctrltriCallback(const quadrotor_msgs::TrajctrlTrigger::ConstPtr &msg)
    {
        if (!ctrl_start_trigger.trigger && msg->trigger)
        {
            ctrl_start_trigger.trigger = msg->trigger;
            traj_start_stamp = msg->header.stamp.toSec();
            ROS_INFO("Traj_follow: ctrl trigger recive!");
        }
        else
        {
            ROS_INFO("Traj_follow: ctrl trigger haven't recived, please checkout px4 node");
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }

            const double zGoal = mapBound[4] + dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (mapBound[5] - mapBound[4] - 2 * dilateRadius); //目标点的高度
            // const double zGoal = msg->pose.position.z; //traj following exc

            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);//目标点的坐标

            if (voxelMap.query(goal) == 0)
            {
                visualization_msgs::Marker sphereMarkers, sphereDeleter;
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size(), sphereMarkers, sphereDeleter);//可视化显示
                spherePub.publish(sphereDeleter);
                spherePub.publish(sphereMarkers);
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
        poseSub = nh.subscribe(poseTopic, 1, &GlobalPlanner::stateCallBack, this,
                               ros::TransportHints().tcpNoDelay());
            
        ctrltriSub = nh.subscribe(ctrlTopic, 1, &GlobalPlanner::ctrltriCallback, this,
                                   ros::TransportHints().tcpNoDelay());

        cmdPub = nh.advertise<quadrotor_msgs::PositionCommand>(cmdTopic,10);

        desPub = nh.advertise<quadrotor_msgs::TrajcurDesire>("/desire_pose_current_traj", 100);

        ROS_WARN("please enter 'y' to start planning");
        while( !(std::getchar() == 'y') )
        {
            ROS_WARN("enter error,please retry");
        }

        plan(); //轨迹规划
    }

    inline void cmdPublish(const Eigen::Vector4d &quat, const Eigen::Vector3d &omg, const Eigen::Vector3d &pos,
                            const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, const Eigen::Vector3d &jer)
    {
        quadrotor_msgs::PositionCommandPtr cmdMsg(new quadrotor_msgs::PositionCommand());
        cmdMsg->position.x = pos(0);
        cmdMsg->position.y = pos(1);
        cmdMsg->position.z = pos(2);
        cmdMsg->velocity.x = vel(0);
        cmdMsg->velocity.y = vel(1);
        cmdMsg->velocity.z = vel(2);
        cmdMsg->acceleration.x = acc(0);
        cmdMsg->acceleration.y = acc(1);
        cmdMsg->acceleration.z = acc(2);
        cmdMsg->jerk.x = jer(0);
        cmdMsg->jerk.y = jer(1);
        cmdMsg->jerk.z = jer(2);
        cmdMsg->yaw = atan2(2.0*(quat(1)*quat(2) + quat(0)*quat(3)), 1.0 - 2.0 * (quat(2) * quat(2) + quat(3) * quat(3)));
        cmdMsg->yaw_dot = 0;

        cmdPub.publish(cmdMsg);

        // std::cout<<"quat = "<<quat.transpose()<<std::endl;
        // std::cout<<"yaw = "<<cmdMsg->yaw<<std::endl;
        // std::cout<<"cmdTopic: "<<cmdTopic<<std::endl;
        // std::cout<<"omg.x = "<<omg(0)<<std::endl;
        // std::cout<<"omg.y = "<<omg(1)<<std::endl;
        // std::cout<<"omg.z = "<<omg(2)<<std::endl;
    }

    inline void desPublish(const Eigen::Vector4d &quat, const Eigen::Vector3d &pos, const ros::Time &current_time)
    {
        quadrotor_msgs::TrajcurDesirePtr desMsg(new quadrotor_msgs::TrajcurDesire());
        desMsg->header.stamp = current_time;
        desMsg->pos.orientation.w = quat(0);
        desMsg->pos.orientation.x = quat(1);
        desMsg->pos.orientation.y = quat(2);
        desMsg->pos.orientation.z = quat(3);
        desMsg->pos.position.x = pos(0);
        desMsg->pos.position.y = pos(1);
        desMsg->pos.position.z = pos(2);

        desPub.publish(desMsg);
    }

    //计算总推力，姿态四元数，机体角速率
    void process(const ros::TimerEvent& time_event)
    {
        // std::cout<<"planning process entry"<<std::endl;
        
        Eigen::VectorXd physicalParams(6); //物理参数
        physicalParams(0) = vehicleMass;//质量
        physicalParams(1) = gravAcc;//重力加速度
        physicalParams(2) = horizDrag;//水平阻力系数
        physicalParams(3) = vertDrag;//垂直阻力系数
        physicalParams(4) = parasDrag;//附加阻力系数
        physicalParams(5) = speedEps;//速度平滑因子

        flatness::FlatnessMap flatmap;//微分平坦
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));//将物理参数赋值给微分平坦的私有变量

        ros::Time current_time = ros::Time::now();

        if (traj.getPieceNum() > 0)
        {
            // std::cout<<"flap count start"<<std::endl;

            const double delta = current_time.toSec() - trajStamp;//delta=当前时科-上一次规划的时刻
            // traj_start_stamp = trajStamp; //trajals node test
            if (delta > 0.0 && delta < traj.getTotalDuration())//delta小于轨迹的总时长
            {
                visualization_msgs::Marker sphereMarkers, sphereDeleter;
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

                cmdPublish(quat, omg, pos, vel, acc, jer); // quat 还得检查一下！！

                double speed = traj.getVel(delta).norm(); //航点的速度二范数
                double bodyratemag = omg.norm();//姿态角的二范数
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2))); //倾斜角

                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;//速度
                thrMsg.data = thr;//总推力
                tiltMsg.data = tiltangle;//倾斜角
                bdrMsg.data = bodyratemag;//机体角速率

                speedPub.publish(speedMsg);//显示速度
                thrPub.publish(thrMsg);//显示总推力
                tiltPub.publish(tiltMsg);//显示倾斜角
                bdrPub.publish(bdrMsg);//显示机体角速率
                visualizer.visualizeSphere(traj.getPos(delta), dilateRadius, sphereMarkers, sphereDeleter);//显示轨迹
                spherePub.publish(sphereDeleter);
                spherePub.publish(sphereMarkers);
            }
            if (ctrl_start_trigger.trigger)
            {
                const double delta_from_start = current_time.toSec() - traj_start_stamp;
                if (delta_from_start > 0.0 && delta_from_start < traj.getTotalDuration())
                {
                    double thr;//总推力
                    Eigen::Vector4d quat;//四元数
                    Eigen::Vector3d omg;//角速率
                    Eigen::Vector3d pos;
                    Eigen::Vector3d vel;
                    Eigen::Vector3d acc;
                    Eigen::Vector3d jer;

                    pos = traj.getPos(delta_from_start);
                    vel = traj.getVel(delta_from_start);
                    acc = traj.getAcc(delta_from_start);
                    jer = traj.getJer(delta_from_start);

                    flatmap.forward(vel,
                                    acc,
                                    jer,
                                    0.0, 0.0,
                                    thr, quat, omg); //利用微分平坦特性计算出总推力，姿态四元数，机体角速率
                    
                    desPublish(quat, pos, current_time);
                }
            }
        }
    }

    void init(ros::NodeHandle &nh)
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

        ctrl_start_trigger.trigger = false;
        // ctrl_start_trigger.trigger = true;
        mapInitialized = false;

        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
        spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);

        // std::cout<<"setpointTag = "<< setpointTag <<std::endl;
        if(setpointTag == 1)
        {
            mapInit();

            mapSub = nh.subscribe(mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

            targetSub = nh.subscribe(targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                    ros::TransportHints().tcpNoDelay());
        }
        else if (setpointTag == 0)
        {
            targetSetting();
        }
        process_timer = nh.createTimer(ros::Duration(0.008), &GlobalPlanner::process, this);
    }

    void onInit(void) 
    {
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&GlobalPlanner::init, this, nh)); //在单独的线程中运行
    }
};

} //namespace gcopter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gcopter::GlobalPlanner, nodelet::Nodelet);
