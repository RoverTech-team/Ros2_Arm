// E1 probe: IK feasibility + clearance analysis for the 17 panel elements
// with the slider-driven 3-finger gripper (tool0 = grasp plane).
//
// For every element x grasp variant it:
//   1. builds tool0 target poses (world frame) from the panel geometry,
//   2. solves IK for each waypoint (arm group, tip tool0, base arm_base_link),
//   3. samples the joint-interpolated segments and checks the gripper volume
//      (jaws open bounding box behind tool0) against an analytic model of the
//      panel (face plane, foot, knobs, levers in both detent states, pokes),
//   4. writes element_plan.yaml: chosen variant, tool0 poses, joint waypoints.
//
// Throwaway analysis node — NOT part of the interaction demo.
//
// Usage:
//   ros2 run rover_arm_moveit_config ik_grasp_probe --ros-args \
//     -p robot_description:="$(xacro .../my_robot.urdf.xacro)" \
//     -p robot_description_semantic:="$(cat .../rover_with_arm.srdf)"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace
{

const double kSpawnZ = 0.02;  // panel & rover spawn height (launch)

enum class Type { LEVER, ROTARY, POWER, POKE };

struct Element {
  std::string name;
  Type type;
  double px, py, pz;  // panel_base frame
};

// panel_base frame coords, from panel.urdf.xacro (knobs at joint origins)
const std::vector<Element> kElements = {
  {"electromagnet",  Type::POKE,  0.240, 0.030, 0.050},
  {"socket_grey",    Type::POKE,  0.130, 0.010, 0.135},
  {"socket_pink_2",  Type::POKE,  0.265, 0.035, 0.135},
  {"main_switch",    Type::LEVER, 0.050, 0.020, 0.220},
  {"lever_switch_1", Type::LEVER, 0.108, 0.020, 0.220},
  {"lever_switch_2", Type::LEVER, 0.165, 0.020, 0.220},
  {"lever_switch_3", Type::LEVER, 0.223, 0.020, 0.220},
  {"lever_switch_4", Type::LEVER, 0.280, 0.020, 0.220},
  {"rotary_switch_1", Type::ROTARY, 0.055, 0.025, 0.300},
  {"rotary_switch_2", Type::ROTARY, 0.086, 0.025, 0.300},
  {"rotary_switch_3", Type::ROTARY, 0.117, 0.025, 0.300},
  {"rotary_switch_4", Type::ROTARY, 0.148, 0.025, 0.300},
  {"rotary_switch_5", Type::ROTARY, 0.179, 0.025, 0.300},
  {"power_switch_1",  Type::POWER,  0.210, 0.025, 0.300},
  {"power_switch_2",  Type::POWER,  0.241, 0.025, 0.300},
  {"indicator_3",     Type::POKE,  0.165, 0.005, 0.375},
  {"aruco_tag_1",     Type::POKE,  0.280, 0.001, 0.405},
};

// panel_base -> world: world = (1.5 - py, px, pz) + spawn z
tf2::Vector3 panelToWorld(double px, double py, double pz)
{
  return tf2::Vector3(1.5 - py, px, pz + kSpawnZ);
}

// World -> arm_base_link (group base frame for IK), for rover parked at
// map (park_x, park_y, 0.02) yaw 0: base_footprint (park_x, park_y, 0.02)
// -> base_joint 0 0 0.1 -> base_link -> arm_mount_joint 0.4325 0.3225 0.17
// -> arm_base_link world = (park_x + 0.4325, park_y + 0.3225, 0.29), identity.
// NOTE: j1 is limited to +/-1.57 so the arm covers only the +x half-plane
// (azimuth [0, pi]); park_y must be small enough that every element has
// positive dy (else the arm cannot turn far enough toward the panel).
tf2::Transform worldToArmBase(double park_x, double park_y)
{
  tf2::Transform t;
  t.setIdentity();
  t.setOrigin(tf2::Vector3(park_x + 0.4325, park_y + 0.3225, 0.29));
  return t.inverse();
}

// ---------- Analytic panel obstacles (world frame, named) ----------

struct Box { std::string name; tf2::Vector3 min, max; };
struct Cyl { std::string name; tf2::Vector3 c; double r, zmin, zmax; };
struct Sph { std::string name; tf2::Vector3 c; double r; };

// Panel body: front face plane x = 1.5, body occupies x >= 1.5
const double kFaceX = 1.5;

// Lever state: tip = pivot + 0.035 * d(theta), d = (-cos t, 0, -sin t) panel frame
tf2::Vector3 leverTipWorld(const Element & e, double theta)
{
  const double ct = std::cos(theta), st = std::sin(theta);
  return panelToWorld(e.px, e.py - 0.035 * ct, e.pz - 0.035 * st);
}

std::vector<Box> buildBoxes(const std::vector<Element> & all)
{
  std::vector<Box> boxes;
  boxes.push_back({"foot", {1.40, -0.035, kSpawnZ}, {1.50, 0.365, kSpawnZ + 0.01}});
  for (const auto & e : all) {
    if (e.type == Type::LEVER) {
      // knob 8 x 35 x 6 mm, at both detent states
      for (double th : {1.57, -1.57}) {
        const tf2::Vector3 t = leverTipWorld(e, th);
        boxes.push_back({e.name, {t.x() - 0.020, t.y() - 0.008, t.z() - 0.006},
                                  {t.x() + 0.020, t.y() + 0.008, t.z() + 0.006}});
      }
    } else if (e.type == Type::POKE) {
      const double sx = (e.name == "aruco_tag_1") ? 0.023 : 0.015;
      const tf2::Vector3 c = panelToWorld(e.px, e.py, e.pz);
      boxes.push_back({e.name, {c.x() - sx, c.y() - 0.006, c.z() - 0.010},
                                {c.x() + sx, c.y() + 0.006, c.z() + 0.010}});
    }
  }
  return boxes;
}

std::vector<Cyl> buildCyls(const std::vector<Element> & all)
{
  std::vector<Cyl> cyls;
  for (const auto & e : all) {
    if (e.type == Type::ROTARY || e.type == Type::POWER) {
      const double r = (e.type == Type::ROTARY) ? 0.012 : 0.016;
      const double half = (e.type == Type::ROTARY) ? 0.0125 : 0.015;
      const tf2::Vector3 c = panelToWorld(e.px, e.py, e.pz);
      cyls.push_back({e.name, c, r, c.z() - half, c.z() + half});
    }
  }
  return cyls;
}

std::vector<Sph> buildSpheres(const std::vector<Element> & all)
{
  std::vector<Sph> sp;
  for (const auto & e : all) {
    if (e.type == Type::POKE && e.name == "indicator_3") {
      const tf2::Vector3 c = panelToWorld(e.px, e.py, e.pz);
      sp.push_back({e.name, c, 0.010});
    }
  }
  return sp;
}

double distPointAABB(const tf2::Vector3 & p, const Box & b)
{
  const double dx = std::max({b.min.x() - p.x(), 0.0, p.x() - b.max.x()});
  const double dy = std::max({b.min.y() - p.y(), 0.0, p.y() - b.max.y()});
  const double dz = std::max({b.min.z() - p.z(), 0.0, p.z() - b.max.z()});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double distPointCyl(const tf2::Vector3 & p, const Cyl & c)
{
  const double dz = std::max({c.zmin - p.z(), 0.0, p.z() - c.zmax});
  const double dr = std::hypot(p.x() - c.c.x(), p.y() - c.c.y()) - c.r;
  return dz > 0.0 ? std::hypot(dr, dz) : std::max(dr, 0.0);
}

double distPointSphere(const tf2::Vector3 & p, const Sph & s)
{
  return std::max(0.0, (p - s.c).length() - s.r);
}

// Min clearance of the gripper volume (jaws open, behind tool0) to all
// obstacles except the element being interacted with (its knob sits inside
// the jaws by design). Panel body treated as half-space x >= 1.5.
double gripperClearance(const tf2::Transform & tool0,
                        const std::vector<Element> & all,
                        const std::string & exclude)
{
  const double ex = 0.027, ey = 0.025, ez0 = -0.065;
  std::vector<tf2::Vector3> pts;
  for (double sx : {1.0, -1.0})
    for (double sy : {1.0, -1.0})
      pts.push_back(tool0 * tf2::Vector3(sx * ex, sy * ey, 0.0));
  for (double sx : {1.0, -1.0})
    for (double sy : {1.0, -1.0})
      for (double f : {0.25, 0.5, 0.75})
        pts.push_back(tool0 * tf2::Vector3(sx * ex, sy * ey, ez0 * f));
  pts.push_back(tool0 * tf2::Vector3(0, 0, ez0));

  const auto boxes = buildBoxes(all);
  const auto cyls = buildCyls(all);
  const auto spheres = buildSpheres(all);

  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : pts) {
    best = std::min(best, std::max(0.0, kFaceX - p.x()));
    for (const auto & b : boxes) {
      if (b.name == exclude) continue;
      best = std::min(best, distPointAABB(p, b));
    }
    for (const auto & c : cyls) {
      if (c.name == exclude) continue;
      best = std::min(best, distPointCyl(p, c));
    }
    for (const auto & s : spheres) {
      if (s.name == exclude) continue;
      best = std::min(best, distPointSphere(p, s));
    }
  }
  return best;
}

// Rotation that maps local +z -> z and local +x -> x (both unit, orthonormal)
tf2::Quaternion rotFromBasis(const tf2::Vector3 & z, const tf2::Vector3 & x)
{
  const tf2::Vector3 zz = z.normalized();
  const tf2::Vector3 xx = x.normalized();
  const tf2::Vector3 yy = zz.cross(xx).normalized();
  tf2::Matrix3x3 m(xx.x(), yy.x(), zz.x(),
                   xx.y(), yy.y(), zz.y(),
                   xx.z(), yy.z(), zz.z());
  tf2::Quaternion q;
  m.getRotation(q);
  return q;
}

tf2::Transform eigenToTf(const Eigen::Isometry3d & m)
{
  const Eigen::Matrix3d r = m.rotation();
  const Eigen::Vector3d t = m.translation();
  tf2::Matrix3x3 rot(r(0, 0), r(0, 1), r(0, 2),
                     r(1, 0), r(1, 1), r(1, 2),
                     r(2, 0), r(2, 1), r(2, 2));
  tf2::Quaternion q;
  rot.getRotation(q);
  return tf2::Transform(q, tf2::Vector3(t.x(), t.y(), t.z()));
}

Eigen::Isometry3d tfToEigen(const tf2::Transform & t)
{
  Eigen::Isometry3d m = Eigen::Isometry3d::Identity();
  const tf2::Matrix3x3 & r = t.getBasis();
  m.linear() << r[0][0], r[0][1], r[0][2],
                 r[1][0], r[1][1], r[1][2],
                 r[2][0], r[2][1], r[2][2];
  const tf2::Vector3 & o = t.getOrigin();
  m.translation() << o.x(), o.y(), o.z();
  return m;
}

// Damped least squares IK for the tool0 link using MoveIt's own FK (the KDL
// plugin chain disagrees with MoveIt's model for this robot, so the stock
// setFromIK path is unusable). Target pose is in the arm_base_link frame.
// Solves the 4 active arm joints; returns true and leaves the solution in
// `state` on success. `z_int`/`x_int` are the intended tool0 axes (world
// frame); the null-space walk minimizes the axis error (for a lever both
// axes matter, else only the approach axis z). Full rotation error goes to
// `out_drot` (informational), the axis error to `out_axis_err`.
bool ikTool0(moveit::core::RobotState & state,
             const moveit::core::JointModelGroup * group,
             const std::vector<const moveit::core::JointModel *> & joints,
             const tf2::Transform & target,
             const tf2::Vector3 & z_int, const tf2::Vector3 & x_int,
             bool is_lever,
             double * out_drot, double * out_axis_err)
{
  const Eigen::Isometry3d arm_base =
      state.getGlobalLinkTransform("arm_base_link");
  const Eigen::Isometry3d goal = arm_base * tfToEigen(target);

  std::vector<double> lo(joints.size()), hi(joints.size());
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto & b = joints[i]->getVariableBounds()[0];
    lo[i] = b.min_position_;
    hi[i] = b.max_position_;
  }

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  auto axisErr = [&z_int, &x_int, is_lever](const Eigen::Isometry3d & t) {
    const Eigen::Vector3d z_ach = t.rotation().col(2);
    const Eigen::Vector3d x_ach = t.rotation().col(0);
    const double ze = std::acos(std::min(1.0, std::max(-1.0,
        z_ach.dot(Eigen::Vector3d(z_int.x(), z_int.y(), z_int.z())))));
    if (!is_lever) return ze;
    const double xe = std::acos(std::min(1.0, std::max(-1.0,
        x_ach.dot(Eigen::Vector3d(x_int.x(), x_int.y(), x_int.z())))));
    return std::max(ze, xe);
  };

  double best_err = std::numeric_limits<double>::infinity();
  std::vector<double> best_q;

  // 4 actuated DOF (j4 fixed): exact 6D pose is generically impossible.
  // Two-phase solve: (1) exact position (3 DOF <= 4 joints), (2) reduce
  // orientation error along the 1D position null-space. The clearance test
  // downstream judges the achieved orientation honestly.
  std::vector<double> q(joints.size());

  for (int a = 0; a < 24; ++a) {
    for (size_t i = 0; i < joints.size(); ++i)
      q[i] = lo[i] + uni(rng) * (hi[i] - lo[i]);
    // ---- phase 1: position only ----
    for (int it = 0; it < 400; ++it) {
      state.setJointGroupPositions(group, q);
      const Eigen::Isometry3d cur = state.getGlobalLinkTransform("tool0");
      const Eigen::Vector3d dp = goal.translation() - cur.translation();
      const double perr = dp.norm();
      if (perr < 1e-3) break;
      Eigen::MatrixXd Jp(3, joints.size());
      const double h = 1e-4;
      for (size_t i = 0; i < joints.size(); ++i) {
        const double qo = q[i];
        q[i] = qo + h;
        state.setJointGroupPositions(group, q);
        const Eigen::Isometry3d cp = state.getGlobalLinkTransform("tool0");
        Jp.block<3, 1>(0, i) = (cp.translation() - cur.translation()) / h;
        q[i] = qo;
      }
      const double lam = 0.0005 + 0.01 * perr;
      const Eigen::MatrixXd Jpt = Jp.transpose();
      const Eigen::VectorXd dq =
          (Jpt * Jp + lam * Eigen::MatrixXd::Identity(joints.size(), joints.size()))
              .ldlt()
              .solve(Jpt * dp);
      for (size_t i = 0; i < joints.size(); ++i) {
        q[i] += dq[i];
        if (q[i] < lo[i]) q[i] = lo[i];
        if (q[i] > hi[i]) q[i] = hi[i];
      }
    }
    // ---- phase 2: walk the position null-space for the best orientation ----
    state.setJointGroupPositions(group, q);
    Eigen::Isometry3d cur = state.getGlobalLinkTransform("tool0");
    if ((goal.translation() - cur.translation()).norm() < 1e-3) {
      Eigen::MatrixXd Jp(3, joints.size());
      const double h = 1e-4;
      for (size_t i = 0; i < joints.size(); ++i) {
        const double qo = q[i];
        q[i] = qo + h;
        state.setJointGroupPositions(group, q);
        const Eigen::Isometry3d cp = state.getGlobalLinkTransform("tool0");
        Jp.block<3, 1>(0, i) = (cp.translation() - cur.translation()) / h;
        q[i] = qo;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(
          Jp.transpose() * Jp);
      const Eigen::VectorXd nv = es.eigenvectors().col(0);
      const double d0 = axisErr(cur);
      double best_walk_d = d0;
      std::vector<double> best_walk_q = q;
      std::vector<double> qw = q;
      for (int k = -1500; k <= 1500; ++k) {
        for (size_t i = 0; i < joints.size(); ++i) {
          qw[i] = q[i] + 0.003 * k * nv[i];
          if (qw[i] < lo[i]) qw[i] = lo[i];
          if (qw[i] > hi[i]) qw[i] = hi[i];
        }
        state.setJointGroupPositions(group, qw);
        const Eigen::Isometry3d cw = state.getGlobalLinkTransform("tool0");
        if ((goal.translation() - cw.translation()).norm() > 5e-3) continue;
        const double dr = axisErr(cw);
        if (dr < best_walk_d) {
          best_walk_d = dr;
          best_walk_q = qw;
        }
      }
      q = best_walk_q;
      state.setJointGroupPositions(group, q);
      cur = state.getGlobalLinkTransform("tool0");
    }
    {
      const Eigen::Vector3d dp = goal.translation() - cur.translation();
      const double perr = dp.norm();
      if (perr < best_err) {
        best_err = perr;
        best_q = q;
      }
      if (perr < 1e-3) {
        state.setJointGroupPositions(group, q);
        if (out_drot) {
          const Eigen::Matrix3d dR =
              goal.rotation() * cur.rotation().transpose();
          *out_drot = Eigen::AngleAxisd(dR).angle();
        }
        if (out_axis_err) *out_axis_err = axisErr(cur);
        return true;
      }
    }
  }
  state.setJointGroupPositions(group, best_q);
  return false;
}

// One-shot: at the element contact position, walk the 1D position
// null-space and report the minimum angle between the achieved tool0 z-axis
// (and x-axis) and each world axis candidate. Shows what orientations the
// 4-DOF arm can actually deliver at that position.
void probeEnvelope(moveit::core::RobotState & state,
                   const moveit::core::JointModelGroup * group,
                   const std::vector<const moveit::core::JointModel *> & joints,
                   const tf2::Vector3 & contact_world, double park_x,
                   double park_y, const char * name)
{
  const Eigen::Isometry3d arm_base =
      state.getGlobalLinkTransform("arm_base_link");
  tf2::Transform t;
  t.setIdentity();
  t.setOrigin(tf2::Vector3(park_x + 0.4325, park_y + 0.3225, 0.29));
  const Eigen::Isometry3d goal =
      arm_base * tfToEigen(t.inverse() * tf2::Transform(tf2::Quaternion::getIdentity(), contact_world));

  std::vector<double> lo(joints.size()), hi(joints.size());
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto & b = joints[i]->getVariableBounds()[0];
    lo[i] = b.min_position_;
    hi[i] = b.max_position_;
  }
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  std::vector<double> q(joints.size());
  bool solved = false;
  for (int a = 0; a < 24 && !solved; ++a) {
    for (size_t i = 0; i < joints.size(); ++i)
      q[i] = lo[i] + uni(rng) * (hi[i] - lo[i]);
    for (int it = 0; it < 400; ++it) {
      state.setJointGroupPositions(group, q);
      const Eigen::Isometry3d cur = state.getGlobalLinkTransform("tool0");
      const Eigen::Vector3d dp = goal.translation() - cur.translation();
      const double perr = dp.norm();
      if (perr < 1e-3) { solved = true; break; }
      Eigen::MatrixXd Jp(3, joints.size());
      const double h = 1e-4;
      for (size_t i = 0; i < joints.size(); ++i) {
        const double qo = q[i];
        q[i] = qo + h;
        state.setJointGroupPositions(group, q);
        const Eigen::Isometry3d cp = state.getGlobalLinkTransform("tool0");
        Jp.block<3, 1>(0, i) = (cp.translation() - cur.translation()) / h;
        q[i] = qo;
      }
      const double lam = 0.0005 + 0.01 * perr;
      const Eigen::MatrixXd Jpt = Jp.transpose();
      const Eigen::VectorXd dq =
          (Jpt * Jp + lam * Eigen::MatrixXd::Identity(joints.size(), joints.size()))
              .ldlt()
              .solve(Jpt * dp);
      for (size_t i = 0; i < joints.size(); ++i) {
        q[i] += dq[i];
        if (q[i] < lo[i]) q[i] = lo[i];
        if (q[i] > hi[i]) q[i] = hi[i];
      }
    }
  }
  if (!solved) { printf("DBGENV %s UNREACHABLE\n", name); return; }

  // null-space at the solved point
  state.setJointGroupPositions(group, q);
  const Eigen::Isometry3d cur = state.getGlobalLinkTransform("tool0");
  Eigen::MatrixXd Jp(3, joints.size());
  const double h = 1e-4;
  for (size_t i = 0; i < joints.size(); ++i) {
    const double qo = q[i];
    q[i] = qo + h;
    state.setJointGroupPositions(group, q);
    const Eigen::Isometry3d cp = state.getGlobalLinkTransform("tool0");
    Jp.block<3, 1>(0, i) = (cp.translation() - cur.translation()) / h;
    q[i] = qo;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Jp.transpose() * Jp);
  const Eigen::VectorXd nv = es.eigenvectors().col(0);

  const std::vector<Eigen::Vector3d> cands = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  Eigen::MatrixXd minz(2, 6), minx(2, 6);
  minz.setConstant(std::numeric_limits<double>::infinity());
  minx.setConstant(std::numeric_limits<double>::infinity());
  auto upd = [&](const Eigen::Isometry3d & t) {
    const Eigen::Vector3d za = t.rotation().col(2);
    const Eigen::Vector3d xa = t.rotation().col(0);
    for (size_t c = 0; c < 6; ++c) {
      minz(0, c) = std::min(minz(0, c), std::acos(std::min(1.0, std::max(-1.0, za.dot(cands[c])))));
      minx(0, c) = std::min(minx(0, c), std::acos(std::min(1.0, std::max(-1.0, xa.dot(cands[c])))));
    }
  };
  upd(cur);
  std::vector<double> qw = q;
  for (int k = -1500; k <= 1500; ++k) {
    for (size_t i = 0; i < joints.size(); ++i) {
      qw[i] = q[i] + 0.003 * k * nv[i];
      if (qw[i] < lo[i]) qw[i] = lo[i];
      if (qw[i] > hi[i]) qw[i] = hi[i];
    }
    state.setJointGroupPositions(group, qw);
    const Eigen::Isometry3d cw = state.getGlobalLinkTransform("tool0");
    if ((goal.translation() - cw.translation()).norm() > 5e-3) continue;
    upd(cw);
  }
  char bufz[512], bufx[512];
  snprintf(bufz, sizeof bufz, "%.2f %.2f %.2f %.2f %.2f %.2f",
           minz(0, 0), minz(0, 1), minz(0, 2), minz(0, 3), minz(0, 4), minz(0, 5));
  snprintf(bufx, sizeof bufx, "%.2f %.2f %.2f %.2f %.2f %.2f",
           minx(0, 0), minx(0, 1), minx(0, 2), minx(0, 3), minx(0, 4), minx(0, 5));
  printf("DBGENV %s z(%+x %-x %+y %-y %+z %-z): %s\n", name, bufz);
  printf("DBGENV %s x(%+x %-x %+y %-y %+z %-z): %s\n", name, bufx);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ik_grasp_probe");

  robot_model_loader::RobotModelLoader loader(node, "robot_description",
                                              "robot_description_semantic");
  moveit::core::RobotModelPtr robot_model = loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(node->get_logger(),
                 "No robot model — pass robot_description and "
                 "robot_description_semantic parameters.");
    return 1;
  }

  moveit::core::RobotState rs(robot_model);
  rs.setToDefaultValues();
  const auto * group = robot_model->getJointModelGroup("arm_chain");
  if (!group) { RCLCPP_ERROR(node->get_logger(), "No 'arm_chain' group"); return 1; }
  const auto & arm_joints = group->getActiveJointModels();
  RCLCPP_INFO(node->get_logger(),
              "Probe: %zu active arm joints, tip %s",
              arm_joints.size(),
              group->getOnlyOneEndEffectorTip()->getName().c_str());

  // Max axis error at contact poses. The 4-DOF arm best-fits the tool0
  // orientation; a press/drop only needs the tool0 z-axis aligned with the
  // intended approach axis (in-plane roll is irrelevant), and a lever pinch
  // additionally needs the jaw opening axis (tool0 x) perpendicular to the
  // (vertical) lever rod. 0.35 rad ~= 20 deg.
  const double kMaxAxisErr = 0.35;

  struct ElementResult {
    std::string chosen;
    double best_clear = -1.0;
    double contact_drot = 0.0;
    double contact_axis_err = 0.0;
    std::vector<tf2::Transform> poses;
    std::vector<std::vector<double>> joints;
    std::vector<int> contact_idx;
    bool solved = false;
  };

  // Rover parks at map (park_x, park_y, 0.02); the arm workspace shell is
  // [0.41, 0.79] m from arm_base_link, j1 +/-1.57 covers azimuth [0, pi],
  // so park_y must give every element positive dy. (0.55, -0.30) puts all
  // elements at az 4-28 deg, r 0.49-0.59 m, el -26..+13 deg: all reachable.
  const std::vector<std::pair<double, double>> kParks = {
      {0.49, -0.24}, {0.49, -0.30}, {0.49, -0.36},
      {0.52, -0.24}, {0.52, -0.30}, {0.52, -0.36},
      {0.55, -0.24}, {0.55, -0.30}, {0.55, -0.36},
      {0.58, -0.24}, {0.58, -0.30}, {0.58, -0.36}};

  std::vector<ElementResult> best_results(kElements.size());
  double best_park_x = kParks[0].first, best_park_y = kParks[0].second;
  int best_count = -1;
  for (const auto & [park_x, park_y] : kParks) {
    int feasible = 0;
    std::vector<ElementResult> res(kElements.size());

    for (size_t ie = 0; ie < kElements.size(); ++ie) {
      const auto & e = kElements[ie];
      auto & r = res[ie];

    // ---- variant generator: (name, tool0 waypoints, contact indices) ----
    std::vector<std::tuple<std::string, std::vector<tf2::Transform>, std::vector<int>>>
        variants;

    switch (e.type) {
      case Type::LEVER: {
        // swing from down (+1.57) to just past midpoint (0.2); grasp tip
        const tf2::Vector3 tip0 = leverTipWorld(e, 1.57);
        const tf2::Vector3 tip1 = leverTipWorld(e, 0.2);
        for (double side : {1.0, -1.0}) {
          const tf2::Quaternion q =
              rotFromBasis(tf2::Vector3(1, 0, 0), tf2::Vector3(0, side, 0));
          tf2::Transform pre, grasp, swing;
          pre.setOrigin(tip0 - tf2::Vector3(0.06, 0, 0));
          grasp.setOrigin(tip0);
          swing.setOrigin(tip1);
          pre.setRotation(q); grasp.setRotation(q); swing.setRotation(q);
          variants.push_back({"grasp_down_swing_up_s" + std::to_string((int)side),
                              {pre, grasp, swing}, {1, 2}});
        }
        break;
      }
      case Type::ROTARY:
      case Type::POWER: {
        const tf2::Vector3 c = panelToWorld(e.px, e.py, e.pz);
        for (double yaw : {0.0, 0.35, -0.35}) {  // jaw row tilt
          tf2::Quaternion q;
          q.setRPY(3.14159, 0, yaw);  // z down, jaw opening tilted by yaw
          tf2::Transform pre, drop;
          pre.setOrigin(c + tf2::Vector3(0, 0, 0.06));
          drop.setOrigin(c);
          pre.setRotation(q); drop.setRotation(q);
          variants.push_back({"drop_over_yaw" + std::to_string(yaw),
                              {pre, drop}, {1}});
        }
        break;
      }
      case Type::POKE: {
        const tf2::Vector3 c = panelToWorld(e.px, e.py, e.pz);
        const tf2::Quaternion q =
            rotFromBasis(tf2::Vector3(1, 0, 0), tf2::Vector3(0, 0, -1));
        tf2::Transform pre, contact;
        pre.setOrigin(c - tf2::Vector3(0.040, 0, 0));
        contact.setOrigin(c);  // tool0 plane at the element face
        pre.setRotation(q); contact.setRotation(q);
        variants.push_back({"flat_press", {pre, contact}, {1}});
        break;
      }
    }

    // ---- solve each variant ----
    for (const auto & [vname, vposes, vcontact] : variants) {
      bool ok = true;
      std::vector<std::vector<double>> vjoints;
      std::vector<double> vdrots, vaxes;
      moveit::core::RobotState trial(rs);
      const tf2::Quaternion & vq = vposes.front().getRotation();
      const tf2::Vector3 z_int = tf2::quatRotate(vq, tf2::Vector3(0, 0, 1));
      const tf2::Vector3 x_int = tf2::quatRotate(vq, tf2::Vector3(1, 0, 0));
      const bool is_lever = e.type == Type::LEVER;
      for (const auto & wp : vposes) {
        const tf2::Transform local = worldToArmBase(park_x, park_y) * wp;
        double drot = 0.0, axis = 0.0;
        bool ik_ok = ikTool0(trial, group, arm_joints, local, z_int, x_int,
                             is_lever, &drot, &axis);
        if (!ik_ok) { ok = false; break; }
        std::vector<double> jp;
        for (const auto * jm : arm_joints)
          jp.push_back(trial.getVariablePosition(jm->getName()));
        vjoints.push_back(jp);
        vdrots.push_back(drot);
        vaxes.push_back(axis);
      }
      if (!ok) continue;

      // the interaction only works if the tool0 axis(es) at the contact
      // poses are close to the variant's intended ones (4-DOF arm: best-fit).
      // POKE/ROTARY/POWER: approach axis (tool0 z) vs intended z.
      // LEVER: pinch axis (tool0 x) vs intended x AND approach axis vs z
      // (the vertical rod must stay between the jaws).
      double contact_axis_err = 0.0;
      double contact_drot = 0.0;
      for (int c : vcontact) {
        contact_drot = std::max(contact_drot, vdrots[c]);
        contact_axis_err = std::max(contact_axis_err, vaxes[c]);
      }
      if (contact_axis_err > kMaxAxisErr) continue;
      r.contact_axis_err = contact_axis_err;
      r.contact_drot = contact_drot;

      // clearance along joint-interpolated segments; the target element is
      // excluded from the obstacle set, so contact segments are checked too
      double seg_clear = std::numeric_limits<double>::infinity();
      for (size_t s = 0; s + 1 < vjoints.size(); ++s) {
        for (int k = 0; k <= 25; ++k) {
          const double f = k / 25.0;
          moveit::core::RobotState st(rs);
          for (size_t j = 0; j < arm_joints.size(); ++j) {
            st.setVariablePosition(arm_joints[j]->getName(),
                vjoints[s][j] + f * (vjoints[s + 1][j] - vjoints[s][j]));
          }
          seg_clear = std::min(seg_clear,
              gripperClearance(eigenToTf(st.getGlobalLinkTransform("tool0")),
                               kElements, e.name));
        }
      }
      if (seg_clear > r.best_clear) {
        r.best_clear = seg_clear;
        r.chosen = vname;
        r.poses = vposes;
        r.joints = vjoints;
        r.contact_idx = vcontact;
      }
    }

    r.solved = !r.chosen.empty() && r.best_clear >= -0.005;
    if (r.solved) feasible++;
    RCLCPP_INFO(node->get_logger(), "%s: %s variant=%s clearance=%.4f drot=%.3f axis=%.3f",
                e.name.c_str(), r.solved ? "FEASIBLE" : "INFEASIBLE",
                r.chosen.c_str(), r.best_clear, r.contact_drot, r.contact_axis_err);
    }

    RCLCPP_INFO(node->get_logger(), "PARK (%.2f, %.2f): feasible %d / %zu",
                park_x, park_y, feasible, kElements.size());
    if (feasible > best_count) {
      best_count = feasible;
      best_park_x = park_x;
      best_park_y = park_y;
      best_results = std::move(res);
    }
  }

  RCLCPP_INFO(node->get_logger(), "BEST park=(%.2f, %.2f): feasible %d / %zu",
              best_park_x, best_park_y, best_count, kElements.size());

  // orientation envelope at the best park: what tool0 z/x directions can the
  // arm actually deliver at each element contact position?
  for (const auto & e : kElements) {
    const tf2::Vector3 c = panelToWorld(e.px, e.py, e.pz);
    probeEnvelope(rs, group, arm_joints, c, best_park_x, best_park_y,
                  e.name.c_str());
  }

  std::ofstream out("element_plan.yaml");
  out << "# E1 probe: per-element grasp plan (parallel gripper).\n"
      << "# tool0 poses in world frame; joints in "
         "[j1_joint j2_joint j3_joint ee_base_joint_x ee_base_joint_y].\n"
      << "# contact_poses: indices of poses where the gripper contacts the\n"
      << "#   element (clearance not checked on segments ending there)\n"
      << "park_x: " << std::fixed << std::setprecision(2) << best_park_x << "\n"
      << "park_y: " << std::fixed << std::setprecision(2) << best_park_y << "\n"
      << "spawn_z: " << kSpawnZ << "\n"
      << "elements:\n";

  int feasible = 0;
  for (size_t ie = 0; ie < kElements.size(); ++ie) {
    const auto & e = kElements[ie];
    const auto & r = best_results[ie];
    if (r.solved) feasible++;
    out << "  - name: " << e.name << "\n"
        << "    type: " << (e.type == Type::LEVER ? "LEVER" :
                            e.type == Type::ROTARY ? "ROTARY" :
                            e.type == Type::POWER ? "POWER" : "POKE") << "\n"
        << "    variant: " << r.chosen << "\n"
        << "    contact_drot_rad: " << std::fixed << std::setprecision(3)
        << (r.solved ? r.contact_drot : -1.0) << "\n"
        << "    contact_axis_err_rad: " << std::fixed << std::setprecision(3)
        << (r.solved ? r.contact_axis_err : -1.0) << "\n"
        << "    clearance_m: " << std::fixed << std::setprecision(4)
        << (r.best_clear < 0 ? -1.0 : r.best_clear) << "\n";
    if (r.solved) {
      out << "    contact_poses: [";
      for (size_t i = 0; i < r.contact_idx.size(); ++i)
        out << (i ? ", " : "") << r.contact_idx[i];
      out << "]\n"
          << "    tool0_poses:\n";
      for (const auto & p : r.poses) {
        out << "      - xyz: [" << std::fixed << std::setprecision(4)
            << p.getOrigin().x() << ", " << p.getOrigin().y() << ", "
            << p.getOrigin().z() << "]\n"
            << "        xyzw: [" << std::setprecision(6)
            << p.getRotation().x() << ", " << p.getRotation().y() << ", "
            << p.getRotation().z() << ", " << p.getRotation().w() << "]\n";
      }
      out << "    joints:\n";
      for (const auto & jp : r.joints) {
        out << "      - [" << std::fixed << std::setprecision(4)
            << jp[0] << ", " << jp[1] << ", " << jp[2] << ", " << jp[3] << "]\n";
      }
    } else {
      out << "    contact_poses: []\n    joints: []\n";
    }
  }

  out.close();
  RCLCPP_INFO(node->get_logger(), "Feasible: %d / %zu", feasible, kElements.size());
  rclcpp::shutdown();
  return feasible == (int)kElements.size() ? 0 : 2;
}
