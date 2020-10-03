#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <map>
#include <math.h>
#include <string.h>

const int x_num_discrete = 30; // Discretize each state values
const int u_num_discrete = 20; // Discretize control inputs
const int x_dim = 1; // Number of dimensions of state vector
const int u_dim = 1; // Number of dimensions for control vector
const int Tf_steps = 2; // Final time, in num of discrete time steps
const int x_space= int(pow(x_num_discrete+1,x_dim)); // Size of discretized x space
const int u_space= int(pow(u_num_discrete+1,u_dim)); // Size of discretized u space

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Define augment vector to include time int step
struct aug_vec{
    VectorXd state; // state vector
    int t; // time index

    // This 
    bool operator<(const aug_vec &that) const{
        double pow10;
        double numthis;
        double numthat;
        pow10 = 0;
        numthis = this->t*pow(10,pow10);
        numthat = that.t*pow(10,pow10);
        for (int i = 0; i < x_dim; i++){
            pow10++;
            numthis += this->state[i]*pow(10,pow10);
            numthat += that.state[i]*pow(10,pow10);
        }
        return (numthis < numthat);
    }
};

std::vector<double> x_llim(x_dim);
std::vector<double> x_ulim(x_dim);
std::vector<double> u_llim(u_dim);
std::vector<double> u_ulim(u_dim);

MatrixXd A(x_dim,x_dim);
MatrixXd B(u_dim,u_dim);
MatrixXd Q(x_dim,x_dim); // state weighting matrix
MatrixXd R(u_dim,u_dim); // input weighting matrix

VectorXd discrete_xs[x_space];
VectorXd discrete_us[u_space]; 

std::map<aug_vec,double> opt_costs;
std::map<aug_vec,VectorXd> opt_us;

std::ofstream debug_file;
bool debug = false;

VectorXd step(VectorXd x, VectorXd u); // One time step
double cost(VectorXd x, VectorXd u); // One cost calculations
double cost_to_go(VectorXd x0, VectorXd u); // Calculate cost to go (includes previous state cost)
double interp_opt_cost(VectorXd x, int t);
VectorXd interp_opt_u(VectorXd x, int t);
void discretize();
void dp_solve(); // Create array that takes state+time to optimal cost and input
void print_opts();
void example_traj(); // Testing the solver on a sample trajectory

int main(){
    A(0,0) = 0.5;
    B(0,0) = 1;
    Q(0,0) = 4;
    R(0,0) = 1;
    VectorXd x1(x_dim);
    VectorXd x2(x_dim);
    x1(0) = 1.0;
    x2(0) = 1.0;
    x_llim[0] = -0.2;
    x_ulim[0] = 0.2;
    u_llim[0] = -0.1;
    u_ulim[0] = 0.1;
    if (debug){
        debug_file.open("debug.csv");
    }

    discretize();
    dp_solve();

    // print_opts();
    example_traj();

    if (debug){
        debug_file.close();
    }

    return 0;
}

VectorXd step(VectorXd x, VectorXd u){
    VectorXd x_plus(x_dim);
    x_plus = A*x + B*u;
    return x_plus;
}

double cost_final(VectorXd x){
    
    VectorXd cost_vec(1);
    double cost;
    cost_vec = x.transpose()*Q*x;
    // std::cout << "hello" << std::endl;
    cost = cost_vec(0);
    return cost;
}

double cost_to_go(VectorXd x0, VectorXd u){
    VectorXd x1(x_dim);
    VectorXd state_cost(1);
    VectorXd input_cost(1);
    x1 = step(x0, u);
    double cost;
    state_cost = x1.transpose()*Q*x1;
    input_cost = u.transpose()*R*u;
    cost = state_cost[0] + input_cost[0];
    return cost;
}

double interp_opt_cost(VectorXd x, int t){
    /*
    x, t = state, time for the optimal cost to be estimated

    TODO
    For input state x, find closest states and interpolate
    optimal costs

    For now, just use closest state
    */
   double state_dist;
   double lowest_state_dist = 100000; // Initialize to very big value
   aug_vec aug_x;
   aug_x.t = t;
   for (auto x_d: discrete_xs){
       state_dist = (x_d - x).transpose()*(x_d - x);
       if (state_dist < lowest_state_dist){
           aug_x.state = x_d;
           lowest_state_dist = state_dist;
       }       
   }
   return opt_costs[aug_x];
}

VectorXd interp_opt_u(VectorXd x, int t){
    double state_dist;
    double lowest_state_dist = 100000; // Initialize to very big value
    aug_vec aug_x;
    aug_x.t = t;
    for (auto x_d: discrete_xs){
       state_dist = (x_d - x).transpose()*(x_d - x);
       if (state_dist < lowest_state_dist){
           aug_x.state = x_d;
           lowest_state_dist = state_dist;
       }       
    }
   return opt_us[aug_x];
}

void discretize(){
    VectorXd delx(x_dim);
    std::vector<double> delxs(x_dim);
    VectorXd delu(u_dim);
    std::vector<double> delus(u_dim);
    VectorXd x(x_dim);
    VectorXd u(u_dim);
    // Fet delxs
    for (int i = 0; i<x_dim; i++){
        delxs[i] = (x_ulim[i] - x_llim[i])/x_num_discrete;
    }
    // Get delus
    for (int i = 0; i<u_dim; i++){
        delus[i] = (u_ulim[i]-u_llim[i])/u_num_discrete;
    }

    for (int i = 0; i < int(pow(x_num_discrete+1,x_dim)); i++){ // Incremement by one
        int quot, remain, number;
        number = i;
        for (int j = x_dim-1; j >= 0; j--){ // For each x dimension
            // std::cout << "(i,j): " << i << "," <<j << std::endl;
            quot = number/int(pow(x_num_discrete+1,j));
            remain = number%int(pow(x_num_discrete+1,j));
            // std::cout << "quot,remain: " << quot << remain << std::endl;
            x[j] = x_llim[j] + quot*delxs[j];
            number = remain;
        }
        // std::cout << "after j for loop" << std::endl;
        // std::cout << "size of array" << sizeof(discrete_xs)/sizeof(discrete_xs[0]) << std::endl;
        discrete_xs[i] = x;
        // std::cout << "after discrete x assignment" << std::endl;
        // std::cout << x << std::endl;
        // std::cout << "...." << std::endl;
    }
    
    // Discretize for u space
    for (int i = 0; i < int(pow(u_num_discrete+1,u_dim)); i++){
        int quot, remain, number;
        number = i;
        for (int j = u_dim-1; j>=0; j--){
            quot = number/int(pow(u_num_discrete+1,j));
            remain = number%int(pow(u_num_discrete+1,j));
            u[j] = u_llim[j] + quot*delus[j];
            number = remain;
        }
        discrete_us[i] = u;
    }
}

void dp_solve(){
    for (int t = Tf_steps; t>=0; t--){ // For each time, working backwards from end 
        aug_vec aug_x;
        aug_x.t = t;
        if (debug){
            std::string header;
            header = header + "time,";
            header += "x,";
            header += "x+,";
            header += "input,";
            header += "carrive,";
            header += "c2g,";
            header += "ctotal\n";
            debug_file << header;
        }
        if (t==Tf_steps){ // Calculate final costs
            for (auto x : discrete_xs){ // Iterate through all discretized x values
                double cost;
                aug_x.state = x;
                cost = cost_final(x);
                opt_costs[aug_x] = cost;
                if (debug){
                    std::string line;
                    line += std::to_string(t);
                    line += std::to_string(x(0));
                    line += std::to_string(x(0));
                    line += std::to_string(0.0);
                    line += std::to_string(1.0);
                }
            }
        }else{ // if not final costs, include command in cost
            for (auto x : discrete_xs){ // For each x
                aug_x.state = x;
                double c2g, c_arrive;
                double opt_cost = 10000;
                VectorXd x_plus(x_dim);
                for (auto u: discrete_us){ // For each us
                    x_plus = step(x,u);
                    c2g = cost_to_go(x,u); // To get to state
                    c_arrive = interp_opt_cost(x_plus,t+1); // optimal cost of arrived state
                    if(debug){
                        std::string line;
                    }
                    // std::cout << "Time = " << t << std::endl;
                    // std::cout << "State = " << aug_x.state << std::endl;
                    // std::cout << "Input = \n" << u << std::endl;
                    // std::cout << "Cost = " << c_arrive+c2g << std::endl;
                    // std::cout << "....." << std::endl;
                    if ((c2g+c_arrive)<opt_cost){
                        opt_us[aug_x] = u;
                        opt_cost = c2g+c_arrive;
                        opt_costs[aug_x] = c2g+c_arrive;
                        
                    }
                }
            }
        }
    }
}

void print_opts(){
    aug_vec aug_x;
    for (auto x: discrete_xs){
        aug_x.state = x;
        for (int t = 0; t < Tf_steps; t++){
            aug_x.t = t;
            std::cout << "Time = " << t << std::endl;
            std::cout << "State = " << aug_x.state << std::endl;
            std::cout << "Optimal Input = \n" << opt_us[aug_x] << std::endl;
            std::cout << "Optimal Cost = " << opt_costs[aug_x] << std::endl;
            std::cout << "....." << std::endl;
        }
    }
}

void example_traj(){
    aug_vec aug_x;
    double cost;
    VectorXd x0(1);
    VectorXd u;
    VectorXd x_plus;
    x0[0] =  0.2;
    aug_x.state = x0;

    for (int t=0; t<Tf_steps; t++){
        aug_x.t = t;
        u = interp_opt_u(aug_x.state,aug_x.t);
        cost = interp_opt_cost(aug_x.state,aug_x.t);
        x_plus = step(aug_x.state,u);
        std::cout << "state: \n" << aug_x.state << std::endl;
        std::cout << "optimal input u: \n" << u << std::endl;
        std::cout << "optimal cost: " << cost << std::endl;
        std::cout << "next state: \n" <<  x_plus << std::endl << ".....\n";
        aug_x.state = x_plus;
    }
}