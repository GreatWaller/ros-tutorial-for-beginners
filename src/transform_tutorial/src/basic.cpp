#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
int main(int argc, char const *argv[])
{
    Eigen::Quaterniond q(0.35, 0.2, 0.3, 0.1);
    q.normalize();
    Eigen::Vector3d t(0.3, 0.1, 0.1);

    Eigen::Vector3d p_c(0.5, 0, 0.2);
    // 也可 auto p_c = q * p_c + t;
    auto p_o = q.matrix() * p_c + t;
    std::cout << p_o << std::endl;

    return 0;
}