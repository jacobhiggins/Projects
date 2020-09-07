#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <map>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct aug_vec{
    VectorXd state;
    int t;

    bool operator<(const aug_vec &w){
        return true;
    }
};
bool operator<(const aug_vec &w, const aug_vec &v){
    return true;
}

int main(){
    std::map<aug_vec,double> costs;
    VectorXd v(2);
    VectorXd w(2);
    v[0] = 0;
    v[1] = 1;
    w[0] = 1;
    w[1] = 0;
    aug_vec a, b;
    a.state = v;
    a.t = 0;
    b.state = w;
    b.t = 0;
    costs[a] = 1;
    costs[b] = -1;

    std::cout << costs[a] << std::endl;
}