/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace gcopter
{

    class GCOPTER_PolytopeSFC
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;//顶点表示的凸多面体
        typedef Eigen::MatrixX4d PolyhedronH;//半空间表示的凸多面体
        typedef std::vector<PolyhedronV> PolyhedraV;//顶点表示的凸多面体向量
        typedef std::vector<PolyhedronH> PolyhedraH;//半空间表示的凸多面体向量

    private:
        minco::MINCO_S3NU minco;//轨迹类
        flatness::FlatnessMap flatmap;//微分平坦

        double rho;//时间正则项系数
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;

        PolyhedraV vPolytopes;//顶点表示的凸多面体向量
        PolyhedraH hPolytopes;//半空间表示的凸多面体向量
        Eigen::Matrix3Xd shortPath;//飞行走廊内的最短路径

        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;//顶点表示
        Eigen::VectorXi hPolyIdx;

        int polyN;//航点数量
        int pieceN;//多项式曲线的数量
        int spatialDim;//空间决策变量的维度
        int temporalDim;//时间决策变量的维度

        double smoothEps;//光滑因子
        int integralRes;//积分分辨率
        Eigen::VectorXd magnitudeBd;//物理参数的幅值边界
        Eigen::VectorXd penaltyWt;//等距积分规则选择权重
        Eigen::VectorXd physicalPm;//物理参数
        double allocSpeed;

        lbfgs::lbfgs_parameter_t lbfgs_params;
        Eigen::Matrix3Xd points;
        Eigen::VectorXd times;//时长
        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;//能量泛函对空间参数的偏导数
        Eigen::VectorXd partialGradByTimes;//能量泛函对时间参数的偏导数

    private:
        
        //功能：消除时间流形约束的变换
        //输入：时刻tau
        //输出：时长Ti
        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);//((0.5t+1)*t+1)或1/((0.5t+1)*t+1)
            }
            return;
        }

        //功能：消除时间流行约束的变换
        //输入：总时长Ti
        //输出：时刻tau
        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));//(sqrt(2*Ti-1)-1)或(1-sqrt(2/Ti-1))
            }

            return;
        }

        //J对欧式空间t的梯度计算
        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);//J对欧式空间t的梯度=J对T的梯度*(t+1)
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;//((0.5t+1)*t+1)
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);//J对欧式空间t的梯度=J对T的梯度*(1-t)/((0.5t+1)*t+1)^2
                }
            }
            return;
        }

        //消除几何约束的变换.
        //输入：自变量xi，vIdx，顶点表征的凸多面体vPolys
        //输出：N维球表示的凸多面体P
        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
        {
            const int sizeP = vIdx.size();//
            P.resize(3, sizeP);

            Eigen::VectorXd q;//声明q
            //j += k表示从第一列头元素跳到下一列头元素
            //
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)//循环向量容器
            {
                l = vIdx(i);//凸多面体的序号
                k = vPolys[l].cols();//数组，凸多面体的顶点数量
                q = xi.segment(j, k).normalized().head(k - 1);//取xi的第j到第j+k个元素,归一化,提取前k-1个元素；即[x]
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) + vPolys[l].col(0);//l凸多面体的前k-1个顶点*q点乘q + v0; 即V*[x]^2+v0
            }
            return;
        }


        //小规模非线性最小二乘法，输出函数的梯度 
        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();//二范数
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;//
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        //采用小规模非线性最小二乘法计算凸多面体的代理变量
        //输入：P，vIdx，vPolys
        //输出：凸多面体的代理变量xi
        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;//损失值

            lbfgs::lbfgs_parameter_t tiny_nls_params;//优化参数
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();//第l个凸多面体的顶点数量

                ovPoly.resize(3, k + 1);//存储空间
                ovPoly.col(0) = P.col(i);//P的第i列元素
                ovPoly.rightCols(k) = vPolys[l];//第l个凸多面体

                Eigen::VectorXd x(k);//决策变量
                x.setConstant(sqrt(1.0 / k));//设置为给定值
                lbfgs::lbfgs_optimize(x,
                                      minSqrD,
                                      &GCOPTER_PolytopeSFC::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);

                xi.segment(j, k) = x;//取j到j+k的元素
            }

            return;
        }

        //J对欧式空间代理变量的梯度
        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;//声明q
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();//

                q = xi.segment(j, k);//
                normInv = 1.0 / q.norm();//
                unitQ = q * normInv;

                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;

                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        //范数检索层
        //输入：x，vIdx，顶点 表示的凸多面体
        //输出：花费，梯度
        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               double &cost,
                                               EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;

            Eigen::VectorXd q;//声明q
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();//顶点表示的凸多面体的顶点的列数

                //下面的代码块重复用
                q = xi.segment(j, k);//获得q

                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

       //光滑近似函数f,磨光因子mu
        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df)
        {
            if (x < 0.0)
            {
                return false;
            }
            else if (x > mu)
            {
                f = x - 0.5 * mu;//光滑近似函数
                df = 1.0;//一阶导数
                return true;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;//光滑近似函数
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);//一阶导数
                return true;
            }
        }

        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        //惩罚泛函项
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                //    const Eigen::VectorXi &hIdx,
                                                //    const PolyhedraH &hPolys,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   flatness::FlatnessMap &flatMap,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC)
        {
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);//速度平方的违反约束幅值上界
            const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);//机体角速率平方的违反约束的幅值上界
            const double thetaMax = magnitudeBounds(2);//偏航角的违反约束的幅值上界
            const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
            const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
            const double thrustSqrRadi = thrustRadi * thrustRadi;//总推力平方的违反约束的幅值上界

            const double weightPos = penaltyWeights(0);//位置约束的惩罚权重
            const double weightVel = penaltyWeights(1);//速度约束的惩罚权重
            const double weightOmg = penaltyWeights(2);//机体角速率约束的惩罚权重
            const double weightTheta = penaltyWeights(3);//偏航角约束的惩罚权重
            const double weightThrust = penaltyWeights(4);//总推力约束的惩罚权重

            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;
            double thr, cos_theta;
            Eigen::Vector4d quat;
            Eigen::Vector3d omg;
            double gradThr;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradPos, gradVel, gradOmg;

            double step, alpha;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;
            int K, L;
            double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
            double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
            double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
            double node, pena;

            const int pieceNum = T.size();//多项式曲线的数量
            const double integralFrac = 1.0 / integralResolution;//数值积分的分辨率 = 1/数值积分的精度
            for (int i = 0; i < pieceNum; i++)//多项式样条曲线大循环
            {
                const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);//块大小为<6,3>，从矩阵的(i,j)位置开始，第i段多项式的系数矩阵
                step = T(i) * integralFrac;//Ti/数值积分的精度

                for (int j = 0; j <= integralResolution; j++)//积分的精度
                {
                    s1 = j * step;//
                    s2 = s1 * s1;//S1^2
                    s3 = s2 * s1;//S1^3
                    s4 = s2 * s2;//S1^4
                    s5 = s4 * s1;//S1^5
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    pos = c.transpose() * beta0;//位置     =(1，s1, s1^2， s1^3,  s1^4，  s1^5)
                    vel = c.transpose() * beta1;//速度     =(0, 1, 2*s1,3*s1^2, 4*s1^3,  5*s1^4)
                    acc = c.transpose() * beta2;//加速度   =(0, 0, 2,    6*s1,  12*s1^2, 20*s1^3)
                    jer = c.transpose() * beta3;//加加速度  =(0, 0,  0,    6,    24*s1,  60*s1^2)
                    sna = c.transpose() * beta4;//加加加速度=(0, 0, 0,     0,    24,     120*s1)
 
                    //微分平坦输出的前向；输入：速度，加速度，加加速度；
                    //输出：总推力，姿态四元数，机体角速率
                    flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

                    violaVel = vel.squaredNorm() - velSqrMax;//速度违反约束=速度的二范数-速度上界的平方
                    violaOmg = omg.squaredNorm() - omgSqrMax;//机体角速度违反约束                  
                    cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
                    violaTheta = acos(cos_theta) - thetaMax;//偏航角违反约束？
                    violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;//推力违反约束

                    gradThr = 0.0;
                    gradQuat.setZero();
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();//初始化为0
                    pena = 0.0;//


                    // L = hIdx(i);//
                    // K = hPolys[L].rows();//凸多面体L的行数
                    // for (int k = 0; k < K; k++)
                    // {
                    //     outerNormal = hPolys[L].block<1, 3>(k, 0);//凸多面体L的1*3块
                    //     violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                    //     if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))//位置的平滑函数
                    //     {
                    //         gradPos += weightPos * violaPosPenaD * outerNormal;//位置的梯度
                    //         pena += weightPos * violaPosPena;//位置的惩罚项
                    //     }
                    // }

                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))//速度平滑函数
                    {
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;//速度的梯度
                        pena += weightVel * violaVelPena;//速度的惩罚项
                    }

                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))//机体角速率平滑函数
                    {
                        gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;//机体角速率的梯度
                        pena += weightOmg * violaOmgPena;//机体角速率的惩罚项
                    }

                    if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))//偏航角平滑函数
                    {
                        gradQuat += weightTheta * violaThetaPenaD /
                                    sqrt(1.0 - cos_theta * cos_theta) * 4.0 * Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);//偏航角的梯度
                        pena += weightTheta * violaThetaPena;//偏航角的惩罚项
                    }

                    if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))//总推力的平滑函数
                    {
                        gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);//总推力的梯度
                        pena += weightThrust * violaThrustPena;//总推力的惩罚项
                    }

                    //微分平坦输出的后向；输入：位置梯度，速度梯度，推力梯度，姿态四元数梯度，机体角速率梯度；
                    //输出：位置总梯度，速度总梯度，加速度总梯度，jerk总梯度，偏航角总梯度
                    flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                                     totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                     totalGradPsi, totalGradPsiD);
                    
                    //
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;//数值积分的节点，0.5/1
                    alpha = j * integralFrac;//j*数值积分的分辨率 
                    gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                    beta1 * totalGradVel.transpose() +
                                                    beta2 * totalGradAcc.transpose() +
                                                    beta3 * totalGradJer.transpose()) *
                                                   node * step;
                    gradT(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;//

                    cost += node * step * pena;//增加关于时间积分的惩罚泛函(数值积分的节点*积分间距*惩罚项)
                }
            }

            return;
        }

        //定义的损失函数
        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
            const int dimTau = obj.temporalDim;//时间维度
            const int dimXi = obj.spatialDim;  //空间维度
            const double weightT = obj.rho;    //时间正则项系数

            //声明
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);//时刻
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);//平坦空间变量
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);//花费函数对欧拉空间t的梯度
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);//花费函数对欧拉空间航点的梯度
            
            //进入欧拉空间处理决策变量
            forwardT(tau, obj.times);//消除时间约束的变换，欧拉空间t
            // forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);//消除几何约束的变换，欧拉空间的航点

            double cost;//损失
            obj.minco.setParameters(obj.points, obj.times);//参数设置
            obj.minco.getEnergy(cost);//损失函数=弯曲能

            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);//能量泛函对空间参数的偏导数
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);  //能量泛函对时间参数的偏导数
            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    // obj.hPolyIdx, obj.hPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.flatmap,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);//损失函数=弯曲能+惩罚泛函项

            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);//增加新的惩罚项
            cost += weightT * obj.times.sum(); //损失函数=弯曲能+惩罚泛函项+时间正则项
            obj.gradByTimes.array() += weightT;//时间正则项系数

           //得到平坦空间的花费和梯度
            backwardGradT(tau, obj.gradByTimes, gradTau);//任务泛函对T的梯度
            // backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);//任务泛函对航点的梯度
            // normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);//计算任务泛函的花费和梯度

            return cost;//损失函数
        }

        //梯度计算公式
        //输入：起点状态，终止状态，顶点表示的凸多面体序列，平滑因子；决策变量
        //输出：目标函数对决策变量的梯度
        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));//光滑因子
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));//初始状态
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));//终止状态
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));//凸多面体序列飞行走廊

            double cost = 0.0;//默认为零
            const int overlaps = vPolys.size() / 2;//重叠数量

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);//对P的梯度是三维，路径中间点的数量
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;//起点状态
                if (i < overlaps)//小循环
                {
                    k = vPolys[2 * i + 1].cols();//序号奇数多面体序列的列数
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);//存储q
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);//奇数序列凸多面体的右边k-1元素*r^2+v0
                }
                else
                {
                    b = fin;//终止状态
                }

                d = b - a;//方向向量
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;//花费=sqrt(d所有元素的平方和+光滑因子)

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;//梯度的第i列=(b-a)/sqrt(d所有元素的平方和+光滑因子)
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;//梯度的第i列=-(b-a)/sqrt(d所有元素的平方和+光滑因子)
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)//小循环
            {
                k = vPolys[2 * i + 1].cols();//序号奇数多面体序列的列数
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);//存储q
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);//存储gradQ

                sqrNormQ = q.squaredNorm();//所有元素的平方和

                invNormQ = 1.0 / sqrt(sqrNormQ);//1/sqrt(d所有元素的平方和)
                unitQ = q * invNormQ;//
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;//奇数序列凸多面体的右边k-1元素*gradP(i)*q*1/sqrt(d所有元素的平方和)*2
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;//()*1/sqrt(d所有元素的平方和)
                
                //代码块
                sqrNormViolation = sqrNormQ - 1.0;//所有元素的平方和-1
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;//(所有元素的平方-1)^2
                    dc = 3.0 * c;//3*(所有元素的平方-1)^2
                    c *= sqrNormViolation;//c=c*(所有元素的平方-1)
                    cost += c;//+c*(所有元素的平方-1)
                    gradQ += dc * 2.0 * q;//3*(所有元素的平方-1)^2*2*q
                }
            }
            return cost;
        }


        //确定飞行走廊之后，直接优化得到精确的最短路径。如果用rrt迭代计算时间太久.
        //rrt输出粗路径，用于构建飞行走廊；在飞行走廊寻找最短路径；对路径优化得到轨迹.
        //输入：起点状态，终止状态，顶点表示的飞行走廊凸多面体序列，平滑因子
        //输出：最短路径
        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;//凸多面体序列尺寸的一半
            Eigen::VectorXi vSizes(overlaps);//
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();//序号奇数顶点凸多面体的列数
            }

            Eigen::VectorXd xi(vSizes.sum());//级数和
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));//决策变量
                j += vSizes(i);
            }


            double minDistance;//要计算出损失值
            void *dataPtrs[4];//数据指针
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);

            lbfgs::lbfgs_parameter_t shortest_path_params;//参数
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            //决策变量xi
            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GCOPTER_PolytopeSFC::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(3, overlaps + 2);//三维，元素数量是凸多面体序列尺寸的一般+2
            path.leftCols<1>() = ini;//左侧第一个元素是起点状态
            path.rightCols<1>() = fin;//右侧第一个元素是终止状态
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();//序号奇数顶点表示的凸多面体的列数
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);//存储为q
                r = q.normalized().head(k - 1);//取归一化，前k-1元素
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);// 奇数序列凸多面体的右边k-1元素*r^2+v0
            }

            return;
        }

        //处理飞行走廊函数
        //输入：以半平面表示的凸多面体
        //输出：以顶点表示的凸多面体
        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;//凸多面体的数量-1

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);//顶点表示的凸多面体

            int nv;
            PolyhedronH curIH;//用半空间表示的凸多面体
            PolyhedronV curIV, curIOB;//顶点表示的凸多面体

            for (int i = 0; i < sizeCorridor; i++)//每一个凸多面体均执行
            {
                //---- 1 hPs[i] ---
                if (!geo_utils::enumerateVs(hPs[i], curIV))//得到顶点表示的凸多面体
                {
                    return false;
                }
                nv = curIV.cols();//顶点凸多面体的列数
                curIOB.resize(3, nv);//
                curIOB.col(0) = curIV.col(0);//
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);//用顶点表示的凸多面体
                
                //---- 2 curIH ---
                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);//用半空间表示的凸多面体
                curIH.topRows(hPs[i].rows()) = hPs[i];//用半空间表示的凸多面体
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];//用半空间表示的凸多面体

                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);//用顶点表示的凸多面体
            }

            //--- 3 hPs的终点凸多面体 ---
            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);//用顶点表示的凸多面体

            return true;
        }

        //设置初始化函数
        //输入：获得飞行走廊内的最短路径,速度极限*3，
        //输出：内点，时长
        static inline void setInitial_setpoints(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();//c节点数量
            const int sizeN = intervalNs.sum();//
            innerPoints.resize(3, sizeN - 1);//
            timeAlloc.resize(sizeN);//时长

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);//最短路径的第i列
                b = path.col(i + 1);//最短路径的第i+1列
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);//第j到j+l块=c范数/速度极限*3
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;//内点的每一列=a+(b-a)/l*m;
                    }
                }
            }
        }

        //设置初始化函数
        //输入：获得飞行走廊内的最短路径,速度极限*3，
        //输出：内点，时长
        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();//c节点数量
            const int sizeN = intervalNs.sum();//
            // innerPoints.resize(3, sizeN - 1);//
            innerPoints = path;
            timeAlloc.resize(sizeN);//时长

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);//最短路径的第i列
                b = path.col(i + 1);//最短路径的第i+1列
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);//第j到j+l块=c范数/速度极限*3
                j += l;
                // for (int m = 0; m < l; m++)
                // {
                //     if (i > 0 || m > 0)
                //     {
                //         innerPoints.col(k++) = a + c * m;//内点的每一列=a+(b-a)/l*m;
                //         std::cout<<"a + c * m = "<<(a + c * m).transpose()<<std::endl;
                //         std::cout<<"innerPoints.col("<<k<<") = "<<innerPoints.col(k).transpose()<<std::endl;
                //     }
                // }
            }
            // std::cout<<"innerPoints = "<<innerPoints<<std::endl;
        }

    public:
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        //优化参数设置
        //时间权重
        //设置参数：路径元素,光滑因子，积分分辨率，物理参数的极限，惩罚权重，物理参数
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix3Xd &setPoint,
                          const Eigen::Matrix3d &initialState,
                          const Eigen::Matrix3d &finalState,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams)
        {   
            rho = timeWeight;     ////时间正则项系数
            shortPath = setPoint; //路径元素，三维数据
            headPVA = initialState;
            tailPVA = finalState;

            // polyN = hPolytopes.size();//凸多面体的数量
            polyN = shortPath.cols() - 1;

            smoothEps = smoothingFactor;//光滑因子
            integralRes = integralResolution;//积分分辨率
            magnitudeBd = magnitudeBounds;//物理参数的幅值边界
            penaltyWt = penaltyWeights;//惩罚权重
            physicalPm = physicalParams;//物理参数
            allocSpeed = magnitudeBd(0) * 3.0;//速度极限*3


            // getShortestPath(headPVA.col(0), tailPVA.col(0), vPolytopes, smoothEps, shortPath);//获得飞行走廊内的最短路径

            const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);//三维中间点，航点的数量=凸多面体的数量
            pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();//向量列的二范数/每个子段长度,取整
            pieceIdx.array() += 1;//数组检索序号要+1
            pieceN = pieceIdx.sum();//维度求和

            temporalDim = pieceN;//时间维度
            spatialDim = 0;//空间维度
            vPolyIdx.resize(pieceN - 1);//
            hPolyIdx.resize(pieceN);

            // for (int i = 0, j = 0, k; i < polyN; i++)
            // {
            //     k = pieceIdx(i);
            //     for (int l = 0; l < k; l++, j++)
            //     {
            //         if (l < k - 1)
            //         {
            //             vPolyIdx(j) = 2 * i;
            //             spatialDim += vPolytopes[2 * i].cols();//空间维度
            //         }
            //         else if (i < polyN - 1)
            //         {
            //             vPolyIdx(j) = 2 * i + 1;
            //             spatialDim += vPolytopes[2 * i + 1].cols();//空间维度
            //         }
            //         hPolyIdx(j) = i;
            //     }
            // }

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVA, tailPVA, pieceN); //路径起点元素三维数据，路径终点元素三维数据，多项式数量

            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                          physicalPm(3), physicalPm(4), physicalPm(5));//微分平坦计算用到的飞机物理参数

            // Allocate temp variables
            points.resize(3, pieceN - 1);//航点=3*多项式的数量
            points.setZero(); 
            times.resize(pieceN);//多项式的时长

            gradByPoints.resize(3, pieceN - 1);//对航点的导数
            gradByTimes.resize(pieceN);//对时长的导数

            partialGradByCoeffs.resize(6 * pieceN, 3);//任务泛函对空间参数的偏导数
            partialGradByTimes.resize(pieceN);//任务泛函对时长参数的偏导数
            return true;
        }

        inline bool setup_setpoints(const double &timeWeight,
                          const Eigen::Matrix3d &initialPVA,
                          const Eigen::Matrix3d &terminalPVA,
                          const PolyhedraH &safeCorridor,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams)
        {
            rho = timeWeight;     ////时间正则项系数
            headPVA = initialPVA; //路径起点元素，三维数据
            tailPVA = terminalPVA;//路径终点元素，三维数据

            hPolytopes = safeCorridor;//采用半平面表示的飞行走廊凸多面体序列

            for (size_t i = 0; i < hPolytopes.size(); i++)//凸多面体序列的长度
            {
                const Eigen::ArrayXd norms = hPolytopes[i].leftCols<3>().rowwise().norm();//每一个凸多面体的左3列的每行求范数
                hPolytopes[i].array().colwise() /= norms;//飞行走廊凸多面体的每一列进行单独运算
            }

            //处理飞行走廊函数
            //输入：以半平面表示的凸多面体
            //输出：以顶点表示的凸多面体
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            polyN = hPolytopes.size();//凸多面体的数量

            smoothEps = smoothingFactor;//光滑因子
            integralRes = integralResolution;//积分分辨率
            magnitudeBd = magnitudeBounds;//物理参数的幅值边界
            penaltyWt = penaltyWeights;//惩罚权重
            physicalPm = physicalParams;//物理参数
            allocSpeed = magnitudeBd(0) * 3.0;//速度极限*3


            getShortestPath(headPVA.col(0), tailPVA.col(0), vPolytopes, smoothEps, shortPath);//获得飞行走廊内的最短路径

            const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);//三维中间点，航点的数量=凸多面体的数量
            pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();//向量列的二范数/每个子段长度,取整
            pieceIdx.array() += 1;//数组检索序号要+1
            pieceN = pieceIdx.sum();//维度求和

            temporalDim = pieceN;//时间维度
            spatialDim = 0;//空间维度
            vPolyIdx.resize(pieceN - 1);//
            hPolyIdx.resize(pieceN);

            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {
                    if (l < k - 1)
                    {
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();//空间维度
                    }
                    else if (i < polyN - 1)
                    {
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();//空间维度
                    }
                    hPolyIdx(j) = i;
                }
            }

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVA, tailPVA, pieceN); //路径起点元素三维数据，路径终点元素三维数据，多项式数量

            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                          physicalPm(3), physicalPm(4), physicalPm(5));//微分平坦计算用到的飞机物理参数

            // Allocate temp variables
            points.resize(3, pieceN - 1);//航点=3*多项式的数量
            times.resize(pieceN);//多项式的时长

            gradByPoints.resize(3, pieceN - 1);//对航点的导数
            gradByTimes.resize(pieceN);//对时长的导数

            partialGradByCoeffs.resize(6 * pieceN, 3);//任务泛函对空间参数的偏导数
            partialGradByTimes.resize(pieceN);//任务泛函对时长参数的偏导数
            return true;
        }

        //优化计算
        inline double optimize(Trajectory<5> &traj,
                               const double &relCostTol)
        {
            Eigen::VectorXd x(temporalDim + spatialDim);//声明x(时间+空间维度)
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);//映射x现有数据数组的前时间维度元素
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);//映射x现有数据数组的前后面的元素

            //初始化
            setInitial(shortPath, allocSpeed, pieceIdx, points, times);

            backwardT(times, tau);//将时间向量times变换到欧式空间的时间向量代理变量tau
            // backwardP(points, vPolyIdx, vPolytopes, xi);//将航点向量points变换到欧式空间的航点向量代理变量xi

            //最优化算法的参数设置
            double minCostFunctional;//输出结果
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER_PolytopeSFC::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);//将欧式空间的时间向量代理变量tau变换到平坦空间times
                // forwardP(xi, vPolyIdx, vPolytopes, points);//将欧式空间的航点向量代理变量xi变换到平坦空间points

                minco.setParameters(points, times);//欧式空间(表征的(P，T)
                minco.getTrajectory(traj);//轨迹
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            //--------------------------------------------
            return minCostFunctional;
        }
    };

}

#endif
