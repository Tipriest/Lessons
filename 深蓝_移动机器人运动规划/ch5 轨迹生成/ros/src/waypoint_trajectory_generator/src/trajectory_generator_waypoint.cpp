#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

MatrixXd TrajectoryGeneratorWaypoint::getM(int n_seg, int n_order, const Eigen::VectorXd& ts){
    MatrixXd M = MatrixXd::Zero(0, 0); // 初始空矩阵

    for (int k = 0; k < n_seg; ++k) {
        int m_k_rows = n_order + 1;
        int m_k_cols = n_order + 1;
        MatrixXd M_k = MatrixXd::Zero(m_k_rows, m_k_cols); // 创建当前段的 M_k 矩阵

        // 计算 M_k 矩阵
        for (int l = 0; l <= (n_order - 1) / 2; ++l) { // l代表起始点物理量的阶数
            for (int i = l; i <= n_order; ++i) {      // i对应所有系数对应的权重
                int x_id = l;
                int y_id = i;
                M_k(x_id, y_id) = Factorial(i) / Factorial(i - l) * pow(0, i - l);
                M_k(x_id + (n_order - 1) / 2 + 1, y_id) = Factorial(i) / Factorial(i - l) * pow(ts(k), i - l);
            }
        }

        // 将 M_k 加入到 M 中 (块对角矩阵)
        if (M.size() == 0) {
            M = M_k;
        } else {
            MatrixXd temp(M.rows() + M_k.rows(), M.cols() + M_k.cols());
            temp << M, MatrixXd::Zero(M.rows(), M_k.cols()),
                    MatrixXd::Zero(M_k.rows(), M.cols()), M_k;
            M = temp;
        }
    }

    return M;
}

MatrixXd TrajectoryGeneratorWaypoint::getCt(int n_seg, int n_order){
    int rows = n_seg * (n_order + 1);
    int cols = (n_order + 1) + (n_seg - 1) * (n_order + 1) / 2;
    MatrixXd Ct = MatrixXd::Zero(rows, cols); // 初始化 Ct 为零矩阵

    int n_constrain = (n_order + 1) + (n_seg - 1); // 约束条件数
    int n_free = (n_order + 1) / 2 * (n_seg + 1) - n_constrain;

    // 处理起始点的 (n_order + 1)/2 个参数
    for (int i = 0; i < (n_order + 1) / 2; ++i) {
        Ct(i, i) = 1;
    }

    // 处理最后一段终点的 (n_order + 1)/2 个参数
    for (int i = 0; i < (n_order + 1) / 2; ++i) {
        int id_x = n_seg * (n_order + 1) - (n_order + 1) / 2 + i;
        int id_y = (n_order + 1) / 2 + (n_seg - 1) + i;
        Ct(id_x, id_y) = 1;
    }

    // 处理中间 n_seg - 1 个段的参数
    for (int i = 0; i < n_seg - 1; ++i) {
        for (int j = 0; j < (n_order + 1) / 2; ++j) { // 对重复的部分进行处理
            int id_x = (n_order + 1) / 2 + (n_order + 1) * i + j;

            int id_y;
            if (j == 0) {
                id_y = (n_order + 1) / 2 + i;
            } else {
                id_y = n_constrain + ((n_order + 1) / 2 - 1) * i + (j - 1);
            }

            Ct(id_x, id_y) = 1;
            Ct(id_x + (n_order + 1) / 2, id_y) = 1;
        }
    }

    return Ct;
}

MatrixXd TrajectoryGeneratorWaypoint::getQ(int n_seg, int n_order, const Eigen::VectorXd& ts){
    MatrixXd Q = MatrixXd::Zero(0, 0); // 初始化 Q 为空矩阵

    for (int k = 0; k < n_seg; ++k) {
        MatrixXd Q_k = MatrixXd::Zero(n_order + 1, n_order + 1); // 初始化每段的 Q_k 矩阵

        // 计算 Q_k 矩阵
        for (int i = 4; i <= n_order; ++i) {
            for (int l = 4; l <= n_order; ++l) {
                Q_k(i, l) = (i * (i - 1) * (i - 2) * (i - 3) * 
                             l * (l - 1) * (l - 2) * (l - 3) /
                             (i + l - 7)) * pow(ts(k), i + l - 7);
                // 若需要使用阶乘版本，替换为以下公式
                // Q_k(i, l) = factorial(i) / factorial(i - 4) *
                //             factorial(l) / factorial(l - 4) /
                //             (i + l - 7) * pow(ts(k), i + l - 7);
            }
        }

        // 将 Q_k 加入到 Q 中 (块对角矩阵)
        if (Q.size() == 0) {
            Q = Q_k;
        } else {
            MatrixXd temp(Q.rows() + Q_k.rows(), Q.cols() + Q_k.cols());
            temp << Q, MatrixXd::Zero(Q.rows(), Q_k.cols()),
                    MatrixXd::Zero(Q_k.rows(), Q.cols()), Q_k;
            Q = temp;
        }
    }
    return Q;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(n_seg, 3 * n_num1d);   // position(x,y,z), so we need (3 * n_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int n_order   = 2 * d_order - 1;              // the order of polynomial
    int n_num1d   = n_order + 1;                  // the number of variables in each segment

    int n_seg = Time.size();                          // the number of segments
    Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(n_seg, 3 * n_num1d);           // position(x,y,z), so we need (3 * n_num1d) coefficients
    Eigen::VectorXd Px(n_num1d * n_seg), Py(n_num1d * n_seg), Pz(n_num1d * n_seg);

    for(int axis = 0; axis < 3; axis ++){
        /*   Produce Mapping Matrix M to the entire trajectory, M is a mapping matrix that maps polynomial coefficients to derivatives.   */
        Eigen::MatrixXd M  = getM(n_seg, n_order, Time);
        Eigen::MatrixXd Ct = getCt(n_seg, n_order);
        Eigen::MatrixXd Q  = getQ(n_seg, n_order, Time);
        Eigen::MatrixXd C  = Ct.transpose();
        Eigen::MatrixXd R  = C * M.inverse().transpose() * Q * M.inverse() * Ct;
        
        // cout<<"M:"<<endl<<M<<endl;
        // cout<<"Ct:"<<endl<<Ct<<endl;
        // cout<<"Q:"<<endl<<Q<<endl;
        // cout<<"R:"<<endl<<R<<endl;
        // 计算 n_constrain 和 n_free
        int n_constrain = (n_order + 1) + (n_seg - 1);
        int n_free = (n_order+1)/2*(n_seg+1) - n_constrain;
        // cout<<"1"<<endl;
        // 将 R 矩阵分块
        // R 的分块：Mat2Cell 等价操作
        Eigen::MatrixXd R_ff = R.topLeftCorner(n_constrain, n_constrain);
        Eigen::MatrixXd R_fp = R.topRightCorner(n_constrain, n_free);
        Eigen::MatrixXd R_pf = R.bottomLeftCorner(n_free, n_constrain);
        Eigen::MatrixXd R_pp = R.bottomRightCorner(n_free, n_free);   // R_cell{2, 2}
        
        Eigen::VectorXd df = Eigen::VectorXd::Zero(n_constrain);
        Eigen::VectorXd start_cond = Eigen::VectorXd::Zero(d_order);
        Eigen::VectorXd end_cond   = Eigen::VectorXd::Zero(d_order);
        
        start_cond(0) = Path(0, axis), start_cond(1) = Vel(0, axis),
        start_cond(2) = Acc(0, axis);
        end_cond(0) = Path(n_seg, axis), end_cond(1) = Vel(1, axis),
        end_cond(2) = Acc(1, axis);
        
        for(int i = 0; i < d_order; i++){
            df(i) = start_cond(i);
        }
        for(int i = 0; i < n_seg-1; i++){
            df(d_order+i) = Path(i+1, axis);
        }
        for(int i = 0; i < d_order; i++){
            df(n_seg-1 + d_order + i) = end_cond(i);
        }
        // cout<<"df: "<<df<<endl;
        Eigen::VectorXd dp = - R_pp.inverse() * R_fp.transpose() * df;
        Eigen::VectorXd dall(df.size() + dp.size());
        // cout<<"2"<<endl;
        dall << df, dp;
        Eigen::VectorXd polycoeff = M.inverse() * Ct * dall;
        Eigen::MatrixXd polycoeff_reshaped(n_seg, n_order+1);
        for(int i = 0; i < n_seg; i++){
            for(int j = 0; j < n_order+1; j++){
                polycoeff_reshaped(i, j) = polycoeff(i*(n_order+1) + j);
            }
        }
        // cout<<"polycoeff: "         <<polycoeff<<endl;
        // cout<<"polycoeff_reshaped: "<<polycoeff_reshaped<<endl;

        // cout<<"polycoeff.rows(): "<<polycoeff.rows()<<"polycoeff.cols(): "<<polycoeff.cols()<<endl;
        // cout<<"PolyCoeff.rows(): "<<PolyCoeff.rows()<<"PolyCoeff.cols(): "<<PolyCoeff.cols()<<endl;
        // cout<<'3'<<endl;
        PolyCoeff.block(0, axis*(n_order+1), polycoeff_reshaped.rows(), polycoeff_reshaped.cols()) = polycoeff_reshaped;
        
    }
    return PolyCoeff;
}
