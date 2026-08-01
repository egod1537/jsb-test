namespace gnc {
template <typename T, typename... Args>
T *Autopilot::AddController(Args &&...args) {
  static_assert(std::is_base_of_v<Controller, T>,
      "T must inherit from gnc::Controller");

  auto controller = std::make_unique<T>(std::forward<Args>(args)...);
  T *result = controller.get();
  controllers_.push_back(std::move(controller));
  return result;
}

template <typename T> T *Autopilot::GetController() {
  static_assert(std::is_base_of_v<Controller, T>,
      "T must inherit from gnc::Controller");

  for (const auto &controller : controllers_) {
    if (auto *result = dynamic_cast<T *>(controller.get())) {
      return result;
    }
  }

  return nullptr;
}

template <typename T> const T *Autopilot::GetController() const {
  static_assert(std::is_base_of_v<Controller, T>,
      "T must inherit from gnc::Controller");

  for (const auto &controller : controllers_) {
    if (const auto *result = dynamic_cast<const T *>(controller.get())) {
      return result;
    }
  }

  return nullptr;
}

template <typename T> bool Autopilot::RemoveController() {
  static_assert(std::is_base_of_v<Controller, T>,
      "T must inherit from gnc::Controller");

  for (auto iterator = controllers_.begin(); iterator != controllers_.end();
      ++iterator) {
    if (dynamic_cast<T *>(iterator->get()) != nullptr) {
      controllers_.erase(iterator);
      return true;
    }
  }

  return false;
}
} // namespace gnc
