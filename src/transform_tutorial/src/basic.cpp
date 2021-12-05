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

    Eigen::Matrix3d R;
    R << 0.9999702331747733, 0.004739171650665555, 0.00608876148778472,

        -0.004808490243971323, 0.9999232173912479, 0.01142093444114481,

        -0.006034168208066053, -0.01144987222639725, 0.9999162411122423;
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
    std::cout << euler << std::endl;
    // 3.13678
    // 3.13556
    // 3.13014

    return 0;
}