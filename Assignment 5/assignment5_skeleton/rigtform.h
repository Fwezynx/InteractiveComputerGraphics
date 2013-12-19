#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    t_ = t;
    r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {
    t_ = t;
    r_ = Quat();
  }

  explicit RigTForm(const Quat& r) {
    t_ = Cvec3();
    r_ = r;
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    Cvec4 cvec = r_ * a;
    cvec += Cvec4(t_);
    return cvec;
  }

  RigTForm operator * (const RigTForm& a) const {
    const Cvec3& t = (Cvec3(*this * Cvec4(a.getTranslation())));
    const Quat& r = r_ * a.getRotation();
    return RigTForm(t, r);
  }
};

inline RigTForm inv(const RigTForm& tform) {
    const Quat& r = inv(tform.getRotation());
    const Cvec3& t = Cvec3(-(r * Cvec4(tform.getTranslation())));
    return RigTForm(t, r);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}


inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
    Matrix4 m = Matrix4::makeTranslation(tform.getTranslation()) * quatToMatrix(tform.getRotation());
    return m;
}

inline RigTForm lerp(const RigTForm& tform0, const RigTForm& tform1, const double alpha) {
    return RigTForm(lerp(tform0.getTranslation(), tform1.getTranslation(), alpha), slerp(tform0.getRotation(), tform1.getRotation(), alpha));
}

#endif

