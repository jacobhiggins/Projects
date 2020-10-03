#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <map>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Define augment vector to include time int step
struct aug_vec{
    VectorXd state;
    int t;

    bool operator<(const aug_vec &w) const{
        return (this->t < w.t);
    }
};

const int x_num_discrete = 10; // Discretize each state values
const int u_num_discrete = 10; // Discretize control inputs
const int x_dim = 1; // Number of dimensions of state vector
const int u_dim = 1; // Number of dimensions for control vector
const int Tf_steps = 2; // Final time, in num of discrete time steps

std::vector<double> x_llim(x_dim);
std::vector<double> x_ulim(x_dim);
std::vector<double> u_llim(u_dim);
std::vector<double> u_ulim(u_dim);

MatrixXd A(x_dim,x_dim);
MatrixXd B(u_dim,u_dim);
MatrixXd Q(x_dim,x_dim); // state weighting matrix
MatrixXd R(u_dim,u_dim); // input weighting matrix

double opt_costs[x_dim][Tf_steps];
double opt_us[x_dim][Tf_steps][u_dim];

VectorXd step(MatrixXd A, MatrixXd B, VectorXd x, VectorXd u); // One time step
double cost_to_go(MatrixXd Q, MatrixXd R, MatrixXd A, MatrixXd B, VectorXd x0, VectorXd u);
double interp_costs(VectorXd x);
void dp_solve();
void print_costs(int T_step);

int main(){
    A(0,0) = -0.5;
    B(0,0) = 1;
    Q(0,0) = 4;
    R(0,0) = 1;
    VectorXd u(u_dim);
    VectorXd x(x_dim);
    u(0) = 1.0;
    x(0) = 1.0;
    x_llim[0] = -0.2;
    x_ulim[0] = 0.2;
    u_llim[0] = -0.1;
    u_ulim[0] = 0.1;
    std::cout << "A: \n" << A << std::endl;
    std::cout << "B: \n" << B << std::endl;
    std::cout << "x: \n" << x << std::endl;
    std::cout << "u: \n" << u << std::endl;
    std::cout << "Output with u = 1\n" << step(A,B,x,u) << std::endl;
    std::cout << "Cost to go:\n" << cost_to_go(Q,R,A,B,x,u) << std::endl;

    dp_solve();
    print_costs(Tf_steps);
}

VectorXd step(MatrixXd A, MatrixXd B, VectorXd x, VectorXd u){
    VectorXd x_plus(x_dim);
    x_plus = A*x + B*u;
    return x_plus;
}

double cost_to_go(MatrixXd Q, MatrixXd R, MatrixXd A, MatrixXd B, VectorXd x0, VectorXd u){
    VectorXd x1(x_dim);
    VectorXd state_cost(1);
    VectorXd input_cost(1);
    x1 = step(A, B, x0, u);
    double cost;
    state_cost = x0.transpose()*Q*x0 + x1.transpose()*Q*x1;
    input_cost = u.transpose()*R*u;
    cost = state_cost[0] + input_cost[0];
    return cost;
}

double interp_costs(VectorXd x_vec, int t){
    // Get lower and upper x indices
    double x_scaled;
    double x = x_vec[0];
    x_scaled = (x - )
}

void dp_solve(){
    VectorXd delx(x_dim);
    std::vector<double> delxs(x_dim);
    VectorXd delu(u_dim);
    std::vector<double> delus(u_dim);
    VectorXd x(x_dim);
    VectorXd u(u_dim);
    // First, get delxs
    for (int i = 0; i<x_dim; i++){
        delxs[i] = (x_ulim[i] - x_llim[i])/x_num_discrete;
    }
    // Get delus
    for (int i = 0; i<u_dim; i++){
        delus[i] = (u_ulim[i]-u_llim[i])/u_num_discrete;
    }
    // First, solve final cost
    for (int i = 0; i < x_num_discrete; i++){ // For each x position
        for (int j = 0; j < x_dim; j++){
            x[j] = x_llim[j] + i*delxs[j];
        }
        for (int j=0;j<u_dim;j++){
            u[j] = 0.0;
        }
        double cost;
        cost = cost_to_go(Q, R, A, B, x, u);
        opt_costs[i][Tf_steps-1] = cost;
        std::cout << cost << std::endl;
    }

    // Next, back up and solve for other costs
    for (int t = Tf_steps-1; t > 0; t--) // For each time step backwards
    {
        for (int i = 0; i < x_num_discrete; i++){ // For each x position
            // Create x
            for (int k = 0; k < x_dim; k++){
                x[k] = x_llim[k] * i*delxs[k];
            }
            // Create u_opt
            double opt_u[u_dim];
            for (int j = 0; j < u_num_discrete; j++){ // For each u command
                // Create u
                for (int k = 0; k < u_dim; k++){
                    u[k] = u_llim[k] + j*delus[k];
                }
                x_plus = step(A,B,x,u);
                c2g = cost_to_go(Q, R, A, B, x, u);
                // Use interpolation to find total optimal cost

            }
        }
    }
}

void print_costs(int T_step){
    std::ofstream file;
    file.open("costs.csv");
    for (int i = 0; i < x_num_discrete; i++){
        file << opt_costs[i][T_step-1] << ",";
    }
    file.close();
}