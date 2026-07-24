#include "flightui/plot/DataView.hpp"

namespace FlightUI {
DataView::DataView() : m_Data(nullptr), m_Count(0), m_Type(DataType::None) {}

DataView::DataView(const double *data, std::size_t count)
    : m_Data(data), m_Count(count), m_Type(DataType::Double) {}

DataView::DataView(const float *data, std::size_t count)
    : m_Data(data), m_Count(count), m_Type(DataType::Float) {}

DataView DataView::From(const std::vector<double> &values) {
  return DataView(values.data(), values.size());
}

DataView DataView::From(const std::vector<float> &values) {
  return DataView(values.data(), values.size());
}

DataView DataView::From(std::span<const double> values) {
  return DataView(values.data(), values.size());
}

DataView DataView::From(std::span<const float> values) {
  return DataView(values.data(), values.size());
}

const void *DataView::GetData() const { return m_Data; }

std::size_t DataView::GetCount() const { return m_Count; }

DataType DataView::GetType() const { return m_Type; }
} // namespace FlightUI
