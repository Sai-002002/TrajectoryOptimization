#include <iostream>
#include <vector>
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <thread>
#include <chrono>

using namespace Eigen;
using namespace qpOASES;

// ===============================
// Vehicle parameters
// ===============================
const double L = 2.5;          // Wheelbase [m]
const double dt = 0.1;         // Sampling time [s]
const int N_horizon = 20;      // MPC horizon
const double delta_max = 0.4;  // Steering limits [rad]

// ===============================
// Generate reference path
// ===============================
void generateReferencePath(double x_start, double x_end, double y_ref,
                           int N_points,
                           std::vector<double>& X_ref,
                           std::vector<double>& Y_ref)
{
    X_ref.clear(); Y_ref.clear();
    for (int i = 0; i <= N_points; i++) {
        double t = i / static_cast<double>(N_points);
        double x = x_start + t*(x_end-x_start);
        X_ref.push_back(x);
        Y_ref.push_back(y_ref);
    }
}

// ===============================
// Linearized discrete-time model
// State: y, psi
// Input: delta
// ===============================
void getLinearizedModel(double v, Matrix2d &A, Vector2d &B)
{
    A << 1.0, v*dt,
         0.0, 1.0;
    B << 0.0,
         v*dt / L;
}

// ===============================
// Solve QP MPC for lane change
// ===============================
void solveMPC(const std::vector<double>& Y_ref_vec,
              double y0, double psi0,
              int N_steps,
              std::vector<double>& y_trajectory)
{
    int n_x = 2; // state: y, psi
    int n_u = 1; // input: delta

    double v = 15.0; // constant speed [m/s]

    Matrix2d A;
    Vector2d B;
    getLinearizedModel(v, A, B);

    // Build prediction matrices
    MatrixXd Phi(2*N_steps, 2);
    MatrixXd Gamma(2*N_steps, N_steps);
    Phi.setZero(); Gamma.setZero();

    Matrix2d A_power = Matrix2d::Identity();

    for (int i=0;i<N_steps;i++)
    {
        A_power = A_power * A;
        Phi.block<2,2>(2*i,0) = A_power;
        for (int j=0;j<=i;j++)
        {
            Matrix2d A_j = Matrix2d::Identity();
            for (int k=j+1;k<=i;k++)
                A_j *= A;
            Gamma.block<2,1>(2*i,j) = A_j*B;
        }
    }

    // Cost matrices
    MatrixXd Q = MatrixXd::Zero(2*N_steps,2*N_steps);
    MatrixXd R = MatrixXd::Identity(N_steps,N_steps)*1.0;

    for (int i=0;i<N_steps;i++)
        Q(2*i,2*i) = 10.0; // weight on lateral error y

    // Reference vector
    VectorXd Y_ref = VectorXd::Zero(2*N_steps);
    for (int i=0;i<N_steps;i++)
        Y_ref(2*i) = Y_ref_vec[i];

    // QP matrices
    MatrixXd H = Gamma.transpose()*Q*Gamma + R;
    VectorXd f = Gamma.transpose()*Q*(Phi*Vector2d(y0,psi0) - Y_ref);

    // Constraints
    VectorXd lb = VectorXd::Constant(N_steps,-delta_max);
    VectorXd ub = VectorXd::Constant(N_steps, delta_max);

    // qpOASES solver
    QProblem qp(N_steps,0);
    Options options;
    options.printLevel = PL_NONE;
    qp.setOptions(options);

    int nWSR = 100;
    qp.init(H.data(), f.data(), nullptr, lb.data(), ub.data(), nullptr, nullptr, nWSR);

    VectorXd U(N_steps);
    qp.getPrimalSolution(U.data());

    // Compute trajectory
    Vector2d xk(y0,psi0);
    y_trajectory.clear();
    for (int k=0;k<N_steps;k++)
    {
        double delta = U(k);
        xk = A*xk + B*delta;
        y_trajectory.push_back(xk(0));
    }
}

// ===============================
// Animate trajectory
// ===============================
void animateTrajectory(const std::vector<double>& X_ref,
                       const std::vector<double>& Y_ref,
                       const std::vector<double>& Y_ego)
{
    FILE* gp = popen("gnuplot", "w");
    if (!gp){ std::cerr<<"Cannot start gnuplot\n"; return; }

    fprintf(gp,"set terminal qt\n");
    fprintf(gp,"set title 'QP MPC Lane Change'\n");
    fprintf(gp,"set xlabel 'X (m)'\n");
    fprintf(gp,"set ylabel 'Y (m)'\n");
    fprintf(gp,"set yrange [0:10]\n");
    fflush(gp);

    size_t N = Y_ego.size();
    for (size_t i=1;i<=N;i++)
    {
        fprintf(gp,"plot '-' using 1:2 with lines lw 2 lc rgb 'blue' title 'Reference',");
        fprintf(gp,"'-' using 1:2 with lines lw 3 lc rgb 'green' title 'Ego Vehicle'\n");
        fflush(gp);

        // Reference path
        for (size_t j=0;j<X_ref.size();j++)
            fprintf(gp,"%f %f\n", X_ref[j], Y_ref[j]);
        fprintf(gp,"e\n");
        fflush(gp);

        // Ego trajectory up to current step
        for (size_t j=0;j<i;j++)
            fprintf(gp,"%f %f\n", X_ref[j], Y_ego[j]);
        fprintf(gp,"e\n");
        fflush(gp);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    fprintf(gp,"pause -1\n");
    fflush(gp);
    pclose(gp);
}

// ===============================
// Main
// ===============================
int main()
{
    // Reference path
    double x_start=0.0, x_end=200.0, y_ref=2.0;
    int N_points=50;
    std::vector<double> X_ref, Y_ref;
    generateReferencePath(x_start,x_end,y_ref,N_points,X_ref,Y_ref);

    // Ego starting offset
    double y0 = 6.0; // start 4 m above reference
    double psi0 = 0.0;

    // MPC
    std::vector<double> Y_ego;
    solveMPC(Y_ref, y0, psi0, N_points, Y_ego);

    // Animate
    animateTrajectory(X_ref, Y_ref, Y_ego);

    return 0;
}
