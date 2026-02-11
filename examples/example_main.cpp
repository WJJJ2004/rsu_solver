#include <iostream>
#include "rsu_solver/rsu_solver.hpp"

int main() {
  rsu::RSUParams p;

  // 너의 값 (unit: mm)
  p.a_W[0] = Eigen::Vector3d(0, 0,  44);
  p.a_W[1] = Eigen::Vector3d(0, 0, -44);

  p.b_F[0] = Eigen::Vector3d(-40.35, 40.0, -20.0);
  p.b_F[1] = Eigen::Vector3d( 19.65, 40.0, -20.0);

  p.c[0] = 30;  p.c[1] = 30;
  p.r[0] = 190; p.r[1] = 102;

  p.psi[0] = 0; p.psi[1] = 0;

  rsu::RSUSolver solver(p);

  // roll, pitch (rad)
  const double roll  = 0.1;
  const double pitch = -0.05;

  std::optional<std::array<double,2>> prev = std::array<double,2>{0.0, 0.0};
  auto res = solver.solve(roll, pitch, prev);

  std::cout << "feasible: " << res.feasible << "\n";
  std::cout << "alpha: [" << res.alpha[0] << ", " << res.alpha[1] << "] rad\n";
  std::cout << "branch: [" << res.branch[0] << ", " << res.branch[1] << "]\n";
  std::cout << "residual: [" << res.residual[0] << ", " << res.residual[1] << "]\n";
  return 0;
}
