#pragma once

#include "application/gui/viz/core/Math.hpp"

namespace viz {
class Transform {
public:
  Vec3 &Position() { return position_; }
  const Vec3 &Position() const { return position_; }
  void SetPosition(Vec3 position) { position_ = position; }

  Vec3 &RotationDeg() { return rotationDeg_; }
  const Vec3 &RotationDeg() const { return rotationDeg_; }
  void SetRotationDeg(Vec3 rotationDeg) { rotationDeg_ = rotationDeg; }

  Vec3 &Scale() { return scale_; }
  const Vec3 &Scale() const { return scale_; }
  void SetScale(Vec3 scale) { scale_ = scale; }

  Vec3 TransformPoint(Vec3 localPoint) const {
    Vec3 worldPoint = localPoint * scale_;
    worldPoint = RotateX(worldPoint, rotationDeg_.x * DegToRad);
    worldPoint = RotateY(worldPoint, rotationDeg_.y * DegToRad);
    worldPoint = RotateZ(worldPoint, rotationDeg_.z * DegToRad);
    return worldPoint + position_;
  }

private:
  Vec3 position_{};
  Vec3 rotationDeg_{};
  Vec3 scale_{1.0F, 1.0F, 1.0F};
};
} // namespace viz
