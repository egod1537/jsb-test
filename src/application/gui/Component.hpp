#pragma once

namespace gui {
class GUI;

class Component {
public:
  // Lifetime
  virtual ~Component() = default;

  Component(const Component &) = delete;
  Component &operator=(const Component &) = delete;

  // Enabled state
  bool IsEnabled() const { return enabled_; }
  void SetEnabled(bool enabled) { enabled_ = enabled; }

  // GUI lifecycle entry points
  void StartIfNeeded(GUI &gui) {
    if (!enabled_ || started_) {
      return;
    }

    OnStart(gui);
    started_ = true;
  }

  void Tick(GUI &gui) {
    if (!enabled_) {
      return;
    }

    StartIfNeeded(gui);
    OnTick(gui);
  }

protected:
  Component() = default;

  // Extension hooks
  virtual void OnStart(GUI &) {}
  virtual void OnTick(GUI &) = 0;

private:
  // Lifecycle state
  bool enabled_ = true;
  bool started_ = false;
};
} // namespace gui
