#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <iostream>

int main(int argc, char const *argv[])
{
  Eigen::Matrix3Xf ref(3, 3);

  ref << 1, 1, 1, 2, 2, 2, 3, 3, 3;

  Eigen::VectorXf f = ref.colwise().stableNorm();

  std::cout << f << std::endl;
  return 0;
}
