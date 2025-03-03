#include "mpc.h"

MPC::MPC(double initMpcFreq, double initN) : mpc_freq(initMpcFreq), N(initN)
{
    std::cout << "MPC FREQ: " << mpc_freq << "Hz" << std::endl;
    std::cout << "MPC HORIZON: " << N / mpc_freq << "s" << std::endl;
    std::cout << "MPC CLASS IS SUCCESSFULLY CONSTRUCTED" << std::endl;
}

void MPC::cartTableModel(double T, double h)
{
    
}

void MPC::cartTableModelMPC(double T, double h)
{
    
}

void MPC::ComTrajectoryGenerator(Eigen::VectorXd &zx_ref, Eigen::VectorXd &zy_ref, Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double comHeight)
{

}
