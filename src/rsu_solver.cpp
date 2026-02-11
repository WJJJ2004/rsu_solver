#include "rsu_solver/rsu_solver.hpp"
#include <cmath>
#include <limits>

namespace rsu {

RSUSolver::RSUSolver(RSUParams params) : params_(std::move(params)) {}

Eigen::Matrix3d RSUSolver::Rx(double a) {
  const double c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d R;
  R << 1, 0, 0,
       0, c,-s,
       0, s, c;
  return R;
}

Eigen::Matrix3d RSUSolver::Ry(double a) {
  const double c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d R;
  R <<  c, 0, s,
        0, 1, 0,
       -s, 0, c;
  return R;
}

Eigen::Matrix3d RSUSolver::Rz(double a) {
  const double c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d R;
  R << c,-s, 0,
       s, c, 0,
       0, 0, 1;
  return R;
}

double RSUSolver::wrapToPi(double x) {
  // Wrap to (-pi, pi]
  const double two_pi = 2.0 * M_PI;
  x = std::fmod(x + M_PI, two_pi);
  if (x < 0) x += two_pi;
  return x - M_PI;
}

double RSUSolver::clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Motor axis = +Y.
// We end up with equation: dx*cos(alpha) + dz*sin(alpha) = k
// where dx, dz are components of d_hat in motor-aligned coordinates.
// Represent as rho*sin(alpha + varphi) = k
// varphi = atan2(dx, dz), rho = sqrt(dx^2 + dz^2)
void RSUSolver::computeAlphaCandidates_Yaxis(
    double dx, double dz, double k,
    double eps,
    std::array<double,2>& alpha_cand,
    std::array<bool,2>& cand_valid,
    double& rho_out,
    double& arg_out
) {
  const double rho = std::sqrt(dx*dx + dz*dz);
  rho_out = rho;

  if (rho <= eps) {
    cand_valid = {false, false};
    arg_out = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const double arg = k / rho;
  // clamp for numerical stability
  const double arg_c = clamp(arg, -1.0, 1.0);
  arg_out = arg_c;

  // If arg is outside [-1,1] beyond tolerance, mark infeasible.
  // (still keep clamped candidates for "best effort" debug)
  if (std::abs(arg) > 1.0 + 1e-12) {
    cand_valid = {false, false};
    // still compute something meaningful:
    const double varphi = std::atan2(dx, dz);
    const double as = std::asin(arg_c);
    alpha_cand[0] = wrapToPi(-varphi + as);
    alpha_cand[1] = wrapToPi(-varphi + M_PI - as);
    return;
  }

  const double varphi = std::atan2(dx, dz);
  const double as = std::asin(arg_c);

  alpha_cand[0] = wrapToPi(-varphi + as);
  alpha_cand[1] = wrapToPi(-varphi + M_PI - as);
  cand_valid = {true, true};
}

SolveResult RSUSolver::solve(double roll, double pitch,
                            const std::optional<std::array<double,2>>& prev_alpha) const {
  SolveResult out;

  // Foot orientation: R = Ry(pitch) Rx(roll) (yaw=0)
  const Eigen::Matrix3d R_WF = Ry(pitch) * Rx(roll);

  // per-leg candidates
  std::array<std::array<double,2>, 2> alpha_cands{};
  std::array<std::array<bool,2>,   2> valid_cands{};
  std::array<double,2> rho{};
  std::array<double,2> arg{};

  bool all_feasible = true;

  for (int i = 0; i < 2; ++i) {
    const Eigen::Vector3d& a = params_.a_W[i];
    const Eigen::Vector3d& b = params_.b_F[i];

    // d_i = a_i - R b_i
    out.d[i] = a - R_WF * b;
    const double dn = out.d[i].norm();
    if (dn <= params_.eps) {
      all_feasible = false;
      valid_cands[i] = {false, false};
      continue;
    }

    out.d_hat[i] = out.d[i] / dn;

    const double c = params_.c[i];
    const double r = params_.r[i];
    if (c <= params_.eps) {
      all_feasible = false;
      valid_cands[i] = {false, false};
      continue;
    }

    // k_i
    const double k = (r*r - c*c - dn*dn) / (2.0 * c * dn);
    out.k[i] = k;

    // mount angle: d_tilde = Rz(psi)^T d_hat
    // (you currently use psi=0, but we keep it for completeness)
    const Eigen::Vector3d d_tilde = Rz(params_.psi[i]).transpose() * out.d_hat[i];

    // Motor axis is +Y -> use (x,z) components in motor-aligned frame
    const double dx = d_tilde.x();
    const double dz = d_tilde.z();

    computeAlphaCandidates_Yaxis(dx, dz, k, params_.eps,
                                 alpha_cands[i], valid_cands[i],
                                 rho[i], arg[i]);
    out.rho[i] = rho[i];
    out.arg[i] = arg[i];

    // feasibility check based on original arg (inside compute we used strict test)
    if (!(valid_cands[i][0] && valid_cands[i][1])) {
      all_feasible = false;
    }
  }

  // If infeasible, return with debug info (you can later add projection)
  if (!all_feasible) {
    out.feasible = false;
    return out;
  }

  // Branch selection:
  // We have 2 candidates per leg => 4 combinations.
  // Choose the combination that is closest to prev_alpha (continuity).
  double best_cost = std::numeric_limits<double>::infinity();
  std::array<double,2> best_alpha{0.0, 0.0};
  std::array<int,2> best_branch{0, 0};

  for (int b0 = 0; b0 < 2; ++b0) {
    for (int b1 = 0; b1 < 2; ++b1) {
      const std::array<double,2> a_try = {alpha_cands[0][b0], alpha_cands[1][b1]};

      double cost = 0.0;
      if (prev_alpha.has_value()) {
        // minimize wrapped angle differences
        const double d0 = wrapToPi(a_try[0] - (*prev_alpha)[0]);
        const double d1 = wrapToPi(a_try[1] - (*prev_alpha)[1]);
        cost = d0*d0 + d1*d1;
      } else {
        // no prev => prefer smaller magnitude (arbitrary but stable)
        cost = a_try[0]*a_try[0] + a_try[1]*a_try[1];
      }

      if (cost < best_cost) {
        best_cost = cost;
        best_alpha = a_try;
        best_branch = {b0, b1};
      }
    }
  }

  out.alpha = best_alpha;
  out.branch = best_branch;
  out.feasible = true;

  // Residual diagnostics: dx*cos(alpha) + dz*sin(alpha) - k (should be near 0)
  for (int i = 0; i < 2; ++i) {
    const Eigen::Vector3d d_tilde = Rz(params_.psi[i]).transpose() * out.d_hat[i];
    const double dx = d_tilde.x();
    const double dz = d_tilde.z();
    const double a = out.alpha[i];
    out.residual[i] = dx*std::cos(a) + dz*std::sin(a) - out.k[i];
  }

  return out;
}

} // namespace rsu

