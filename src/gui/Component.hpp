#pragma once

namespace gui {
class GUI;

class Component {
public:
  virtual ~Component() = default;

  Component(const Component &) = delete;
  Component &operator=(const Component &) = delete;

  bool IsEnabled() const { return enabled_; }
  void SetEnabled(bool enabled) { enabled_ = enabled; }

  void StartIfNeeded(GUI &gui) {
    if (!enabled_ || started_) {
      return;
    }

    Start(gui);
    started_ = true;
  }

  void Tick(GUI &gui) {
    if (!enabled_) {
      return;
    }

    StartIfNeeded(gui);
    Update(gui);
  }

protected:
  Component() = default;

  virtual void Start(GUI &) {}
  virtual void Update(GUI &) = 0;

private:
  bool enabled_ = true;
  bool started_ = false;
};
} // namespace gui
