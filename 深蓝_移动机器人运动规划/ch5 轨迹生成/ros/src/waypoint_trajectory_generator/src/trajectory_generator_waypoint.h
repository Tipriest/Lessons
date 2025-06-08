#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;
    public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);
        
        int Factorial(int x);
        Eigen::MatrixXd getM(int n_seg, int n_order, const Eigen::VectorXd& ts);
        Eigen::MatrixXd getCt(int n_seg, int n_order);
        Eigen::MatrixXd getQ(int n_seg, int n_order, const Eigen::VectorXd& ts);
};      
        

#endif
