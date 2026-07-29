#pragma once

#include "application/gui/viz/core/VizContext.hpp"

namespace viz {
class GameObject;
class Transform;

class Component {
public:
  virtual ~Component();

  GameObject &GetGameObject();
  const GameObject &GetGameObject() const;
  Transform &GetTransform();
  const Transform &GetTransform() const;

  virtual void Update(const UpdateContext &) {}
  virtual void Render(RenderContext &) const {}

private:
  friend class GameObject;

  void SetGameObject(GameObject *gameObject);

  GameObject *gameObject_ = nullptr;
};
} // namespace viz
