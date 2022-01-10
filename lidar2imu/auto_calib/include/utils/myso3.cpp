#include "myso3.hpp"
#include <iostream>

// Extracted from "https://github.com/strasdat/Sophus"

SO3::SO3() { unit_quaternion_.setIdentity(); }

SO3::SO3(const SO3 &other) : unit_quaternion_(other.unit_quaternion_) {}

SO3::SO3(const Matrix3d &R) : unit_quaternion_(R) {}

SO3::SO3(const Quaterniond &quat) : unit_quaternion_(quat) {
  assert(unit_quaternion_.squaredNorm() > SMALL_EPS);
  unit_quaternion_.normalize();
}

SO3::SO3(double rot_x, double rot_y, double rot_z) {
  unit_quaternion_ = (SO3::exp(Vector3d(rot_x, 0.f, 0.f)) *
                      SO3::exp(Vector3d(0.f, rot_y, 0.f)) *
                      SO3::exp(Vector3d(0.f, 0.f, rot_z)))
                         .unit_quaternion_;
}

void SO3::operator=(const SO3 &other) {
  this->unit_quaternion_ = other.unit_quaternion_;
}

SO3 SO3::operator*(const SO3 &other) const {
  SO3 result(*this);
  result.unit_quaternion_ *= other.unit_quaternion_;
  result.unit_quaternion_.normalize();
  return result;
}

void SO3::operator*=(const SO3 &other) {
  unit_quaternion_ *= other.unit_quaternion_;
  unit_quaternion_.normalize();
}

Vector3d SO3::operator*(const Vector3d &xyz) const {
  return unit_quaternion_._transformVector(xyz);
}

SO3 SO3::inverse() const { return SO3(unit_quaternion_.conjugate()); }

Matrix3d SO3::matrix() const { return unit_quaternion_.toRotationMatrix(); }

Matrix3d SO3::Adj() const { return matrix(); }

Matrix3d SO3::generator(int i) {
  assert(i >= 0 && i < 3);
  Vector3d e;
  e.setZero();
  e[i] = 1.f;
  return hat(e);
}

Vector3d SO3::log() const { return SO3::log(*this); }

Vector3d SO3::log(const SO3 &other) {
  double theta;
  return logAndTheta(other, &theta);
}

Vector3d SO3::logAndTheta(const SO3 &other, double *theta) {
  double n = other.unit_quaternion_.vec().norm();
  double w = other.unit_quaternion_.w();
  double squared_w = w * w;

  double two_atan_nbyw_by_n;
  // Atan-based log thanks to
  //
  // C. Hertzberg et al.:
  // "Integrating Generic Sensor Fusion Algorithms with Sound State
  // Representation through Encapsulation of Manifolds"
  // Information Fusion, 2011

  if (n < SMALL_EPS) {
    // If quaternion is normalized and n=1, then w should be 1;
    // w=0 should never happen here!
    assert(fabs(w) > SMALL_EPS);

    two_atan_nbyw_by_n = 2. / w - 2. * (n * n) / (w * squared_w);
  } else {
    if (fabs(w) < SMALL_EPS) {
      if (w > 0) {
        two_atan_nbyw_by_n = M_PI / n;
      } else {
        two_atan_nbyw_by_n = -M_PI / n;
      }
    }
    two_atan_nbyw_by_n = 2 * atan(n / w) / n;
  }

  *theta = two_atan_nbyw_by_n * n;
  return two_atan_nbyw_by_n * other.unit_quaternion_.vec();
}

SO3 SO3::exp(const Vector3d &omega) {
  double theta;
  return expAndTheta(omega, &theta);
}

SO3 SO3::expAndTheta(const Vector3d &omega, double *theta) {
  *theta = omega.norm();
  double half_theta = 0.5 * (*theta);

  double imag_factor;
  double real_factor = cos(half_theta);
  if ((*theta) < SMALL_EPS) {
    double theta_sq = (*theta) * (*theta);
    double theta_po4 = theta_sq * theta_sq;
    imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / (*theta);
  }

  return SO3(Quaterniond(real_factor, imag_factor * omega.x(),
                         imag_factor * omega.y(), imag_factor * omega.z()));
}

Matrix3d SO3::hat(const Vector3d &v) {
  Matrix3d Omega;
  Omega << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return Omega;
}

Vector3d SO3::vee(const Matrix3d &Omega) {
  assert(fabs(Omega(2, 1) + Omega(1, 2)) < SMALL_EPS);
  assert(fabs(Omega(0, 2) + Omega(2, 0)) < SMALL_EPS);
  assert(fabs(Omega(1, 0) + Omega(0, 1)) < SMALL_EPS);
  return Vector3d(Omega(2, 1), Omega(0, 2), Omega(1, 0));
}

Vector3d SO3::lieBracket(const Vector3d &omega1, const Vector3d &omega2) {
  return omega1.cross(omega2);
}

Matrix3d SO3::d_lieBracketab_by_d_a(const Vector3d &b) { return -hat(b); }

void SO3::setQuaternion(const Quaterniond &quaternion) {
  assert(quaternion.norm() != 0);
  unit_quaternion_ = quaternion;
  unit_quaternion_.normalize();
}
