#pragma once

#include "application/gui/viz/core/VizContext.hpp"

namespace viz {
class GameObject;
class Transform;

class Component {
public:
  virtual ~Component();

  // Owner-provided state
  GameObject &GetGameObject();
  const GameObject &GetGameObject() const;
  Transform &GetTransform();
  const Transform &GetTransform() const;

  // Frame lifecycle
  virtual void OnTick(const TickContext &) {}
  virtual void Render(RenderContext &) const {}

private:
  friend class GameObject;

  // Owner assignment
  void SetGameObject(GameObject *gameObject);

  // Scene attachment
  GameObject *gameObject_ = nullptr;
};
} // namespace viz
