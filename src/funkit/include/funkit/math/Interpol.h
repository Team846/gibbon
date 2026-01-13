#pragma once

#include <networktables/NetworkTableInstance.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "funkit/base/Loggable.h"

namespace frc846::math {

template <typename X, typename Y>
class PiecewiseLinearInterpolator : public funkit::base::Loggable {
public:
  PiecewiseLinearInterpolator(funkit::base::Loggable& parent,
      std::string_view table_name, std::vector<X> x_vals, std::vector<Y> y_vals)
      : funkit::base::Loggable{parent, std::string(table_name)},
        pref_table_{nt::NetworkTableInstance::GetDefault().GetTable(
            "Preferences/" + name())} {
    if (x_vals.size() != y_vals.size() || x_vals.size() < 2) {
      throw std::invalid_argument("needs at least 2 points");
    }
    X x_sample{};
    Y y_sample{};
    x_key_ = fmt::format("x_vals ({})", x_sample.dims());
    y_key_ = fmt::format("y_vals ({})", y_sample.dims());

    std::vector<double> x_raw, y_raw;
    x_raw.reserve(x_vals.size());
    y_raw.reserve(y_vals.size());
    for (const auto& v : x_vals)
      x_raw.push_back(v.value());
    for (const auto& v : y_vals)
      y_raw.push_back(v.value());

    if (!pref_table_->ContainsKey(x_key_)) {
      pref_table_->PutNumberArray(x_key_, x_raw);
    }
    if (!pref_table_->ContainsKey(y_key_)) {
      pref_table_->PutNumberArray(y_key_, y_raw);
    }
    pref_table_->GetEntry(x_key_).SetPersistent();
    pref_table_->GetEntry(y_key_).SetPersistent();
  }

  void Setup() {
    auto x_raw = pref_table_->GetNumberArray(x_key_, {});
    auto y_raw = pref_table_->GetNumberArray(y_key_, {});

    std::vector<std::pair<X, Y>> points;
    points.reserve(x_raw.size());
    for (size_t i = 0; i < x_raw.size(); ++i) {
      points.emplace_back(X{x_raw[i]}, Y{y_raw[i]});
    }
    std::sort(points.begin(), points.end());
    sorted_points_ = std::move(points);

    inv_dx_.clear();
    inv_dx_.reserve(sorted_points_.size() - 1);
    for (size_t i = 0; i < sorted_points_.size() - 1; ++i) {
      double dx =
          (sorted_points_[i + 1].first - sorted_points_[i].first).value();
      inv_dx_.push_back(1.0 / dx);
    }
  }

  Y Interpolate(X x) const {
    if (sorted_points_.empty()) { return Y{}; }
    if (x <= sorted_points_.front().first) {
      return sorted_points_.front().second;
    }
    if (x >= sorted_points_.back().first) {
      return sorted_points_.back().second;
    }
    auto it = std::lower_bound(sorted_points_.begin(), sorted_points_.end(), x,
        [](const std::pair<X, Y>& p, X val) { return p.first < val; });
    size_t idx = it - sorted_points_.begin();
    if (idx == 0) return sorted_points_[0].second;
    const auto& p1 = sorted_points_[idx - 1];
    const auto& p2 = sorted_points_[idx];
    double t = (x - p1.first).value() * inv_dx_[idx - 1];
    return p1.second + (p2.second - p1.second) * t;
  }

private:
  std::vector<std::pair<X, Y>> sorted_points_;
  std::vector<double> inv_dx_;
  std::shared_ptr<nt::NetworkTable> pref_table_;
  std::string x_key_;
  std::string y_key_;
};

template <typename X, typename Y>
class PiecewiseQuadraticInterpolator : public funkit::base::Loggable {
public:
  PiecewiseQuadraticInterpolator(funkit::base::Loggable& parent,
      std::string_view table_name, std::vector<X> x_vals, std::vector<Y> y_vals)
      : funkit::base::Loggable{parent, std::string(table_name)},
        pref_table_{nt::NetworkTableInstance::GetDefault().GetTable(
            "Preferences/" + name())} {
    if (x_vals.size() != y_vals.size() || x_vals.size() < 3) {
      throw std::invalid_argument("requires at least 3 points");
    }
    X x_sample{};
    Y y_sample{};
    x_key_ = fmt::format("x_vals ({})", x_sample.dims());
    y_key_ = fmt::format("y_vals ({})", y_sample.dims());

    std::vector<double> x_raw, y_raw;
    x_raw.reserve(x_vals.size());
    y_raw.reserve(y_vals.size());
    for (const auto& v : x_vals)
      x_raw.push_back(v.value());
    for (const auto& v : y_vals)
      y_raw.push_back(v.value());

    if (!pref_table_->ContainsKey(x_key_)) {
      pref_table_->PutNumberArray(x_key_, x_raw);
    }
    if (!pref_table_->ContainsKey(y_key_)) {
      pref_table_->PutNumberArray(y_key_, y_raw);
    }
    pref_table_->GetEntry(x_key_).SetPersistent();
    pref_table_->GetEntry(y_key_).SetPersistent();
  }

  void Setup() {
    auto x_raw = pref_table_->GetNumberArray(x_key_, {});
    auto y_raw = pref_table_->GetNumberArray(y_key_, {});

    std::vector<std::pair<X, Y>> points;
    points.reserve(x_raw.size());
    for (size_t i = 0; i < x_raw.size(); ++i) {
      points.emplace_back(X{x_raw[i]}, Y{y_raw[i]});
    }
    std::sort(points.begin(), points.end());

    size_t n = points.size();
    x_vals_.clear();
    y_vals_.clear();
    x_vals_.reserve(n);
    y_vals_.reserve(n);
    for (const auto& p : points) {
      x_vals_.push_back(p.first.value());
      y_vals_.push_back(p.second.value());
    }
    x_min_ = points.front().first;
    x_max_ = points.back().first;

    size_t num_segs = n - 2;
    seg_coeffs_.clear();
    seg_coeffs_.reserve(num_segs);
    for (size_t i = 0; i < num_segs; ++i) {
      double x0 = x_vals_[i], x1 = x_vals_[i + 1], x2 = x_vals_[i + 2];
      double y0 = y_vals_[i], y1 = y_vals_[i + 1], y2 = y_vals_[i + 2];
      double inv_d0 = 1.0 / ((x0 - x1) * (x0 - x2));
      double inv_d1 = 1.0 / ((x1 - x0) * (x1 - x2));
      double inv_d2 = 1.0 / ((x2 - x0) * (x2 - x1));
      seg_coeffs_.push_back(
          {y0 * inv_d0, y1 * inv_d1, y2 * inv_d2, x0, x1, x2});
    }
  }

  [[nodiscard]] Y Interpolate(X x) const {
    size_t n = x_vals_.size();
    if (n == 0) return Y{};

    if (x <= x_min_) { return EvalSegment(0, x); }
    if (x >= x_max_) { return EvalSegment(n - 3, x); }

    double xd = x.value();
    auto it = std::lower_bound(x_vals_.begin(), x_vals_.end(), xd);
    size_t idx = it - x_vals_.begin();

    size_t base;
    if (idx <= 1) {
      base = 0;
    } else if (idx >= n - 1) {
      base = n - 3;
    } else {
      base = idx - 1;
    }

    return EvalSegment(base, x);
  }

private:
  struct SegCoeffs {
    double c0, c1, c2;
    double x0, x1, x2;
  };

  std::vector<double> x_vals_;
  std::vector<double> y_vals_;
  std::vector<SegCoeffs> seg_coeffs_;
  X x_min_{};
  X x_max_{};
  std::shared_ptr<nt::NetworkTable> pref_table_;
  std::string x_key_;
  std::string y_key_;

  [[nodiscard]] Y EvalSegment(size_t seg, X x) const {
    const auto& c = seg_coeffs_[seg];
    double xd = x.value();
    double v0 = c.c0 * (xd - c.x1) * (xd - c.x2);
    double v1 = c.c1 * (xd - c.x0) * (xd - c.x2);
    double v2 = c.c2 * (xd - c.x0) * (xd - c.x1);
    return Y{v0 + v1 + v2};
  }
};

}  // namespace frc846::math
