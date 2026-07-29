#pragma once

namespace sim {
class Context;
struct Tick;

class System {
public:
  virtual ~System() = default;

  virtual bool Initialize(Context &) { return true; }
  virtual bool Reset(Context &) { return true; }

  virtual bool PreStep(Context &, const Tick &) { return true; }
  virtual bool PostStep(Context &, const Tick &) { return true; }

  virtual void Shutdown(Context &) {}
};
} // namespace sim
