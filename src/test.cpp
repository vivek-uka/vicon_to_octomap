#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

using namespace Eigen;

bool in_hull(const MatrixXd& points, const VectorXd& x) {
    int n_points = points.cols();
    int n_dim = x.size();
    
    VectorXd c = VectorXd::Zero(n_points);
    MatrixXd A(n_points + 1, n_dim);
    VectorXd b(n_points + 1);
    
    A << points.transpose(), VectorXd::Ones(n_points);
    b << x, 1.0;
    
    VectorXd lp_result = VectorXd::Zero(n_dim);
    
    VectorXd result = A.partialPivLu().solve(b);
    lp_result = result.head(n_dim);

    return lp_result.minCoeff() >= -1e-8; // Adjust the tolerance as needed
}

int main() {
    int n_points = 4;
    int n_dim = 3;
    
    MatrixXd Z(n_dim, n_points);
    Z << 1, -1, -1, 1,
         1, 1, -1, -1,
         0, 0, 0, 0;
    
    VectorXd x(n_dim);
    x << 0, 0, 0;

    VectorXd x2(n_dim);
    x2 << 0, 0, 0.1;
    
    std::cout << std::boolalpha << in_hull(Z, x) << std::endl;
    std::cout << std::boolalpha << in_hull(Z, x2) << std::endl;
    return 0;
}
