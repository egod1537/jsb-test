#pragma once

#include "application/gui/viz/core/Component.hpp"
#include "application/gui/viz/core/Transform.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace viz {
class GameObject {
public:
  explicit GameObject(std::string name);
  ~GameObject();

  GameObject(const GameObject &) = delete;
  GameObject &operator=(const GameObject &) = delete;

  const std::string &GetName() const { return name_; }

  Transform &GetTransform() { return transform_; }
  const Transform &GetTransform() const { return transform_; }

  template <typename T, typename... Args> T &AddComponent(Args &&...args) {
    static_assert(std::is_base_of_v<Component, T>,
        "T must inherit from viz::Component");

    auto component = std::make_unique<T>(std::forward<Args>(args)...);
    component->SetGameObject(this);
    T &componentRef = *component;
    components_.push_back(std::move(component));
    return componentRef;
  }

  void Update(const UpdateContext &context);
  void Render(RenderContext &context) const;

private:
  std::string name_;
  Transform transform_;
  std::vector<std::unique_ptr<Component>> components_;
};
} // namespace viz
