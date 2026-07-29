#include "application/gui/viz/core/GameObject.hpp"

#include <utility>

namespace viz {
GameObject::GameObject(std::string name) : name_(std::move(name)) {}

GameObject::~GameObject() = default;

void GameObject::Update(const UpdateContext &context) {
  for (const auto &component : components_) {
    component->Update(context);
  }
}

void GameObject::Render(RenderContext &context) const {
  for (const auto &component : components_) {
    component->Render(context);
  }
}
} // namespace viz
