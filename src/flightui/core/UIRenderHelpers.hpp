#pragma once

#include <string>

namespace FlightUI::Internal {
class IdScope {
public:
  explicit IdScope(const std::string &id);
  IdScope(const IdScope &) = delete;
  IdScope &operator=(const IdScope &) = delete;
  ~IdScope();

private:
  bool m_Active = false;
};

class DisabledScope {
public:
  explicit DisabledScope(bool disabled);
  DisabledScope(const DisabledScope &) = delete;
  DisabledScope &operator=(const DisabledScope &) = delete;
  ~DisabledScope();

private:
  bool m_Active = false;
};

class ItemWidthScope {
public:
  explicit ItemWidthScope(float width);
  ItemWidthScope(const ItemWidthScope &) = delete;
  ItemWidthScope &operator=(const ItemWidthScope &) = delete;
  ~ItemWidthScope();

private:
  bool m_Active = false;
};

void ShowTooltipIfHovered(const std::string &tooltip);
} // namespace FlightUI::Internal
