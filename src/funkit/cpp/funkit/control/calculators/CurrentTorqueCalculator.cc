#include "funkit/control/calculators/CurrentTorqueCalculator.h"

#include <algorithm>
#include <cmath>

namespace funkit::control::calculators {
namespace {

pdcsu::units::ohm_t calculate_winding_resistance(
    pdcsu::units::amp_t stall_current) {
  return pdcsu::units::volt_t{12.0} / stall_current;
}

pdcsu::units::ohm_t calculate_total_resistance(
    pdcsu::units::amp_t stall_current, pdcsu::units::ohm_t circuit_resistance) {
  return calculate_winding_resistance(stall_current) + circuit_resistance;
}

pdcsu::units::volt_t calculate_back_emf(pdcsu::units::rpm_t rpm,
    pdcsu::units::rpm_t free_speed, pdcsu::units::volt_t v_supply) {
  const scalar_t pct_speed = rpm / free_speed;
  return pct_speed * v_supply;
}

}  // namespace

pdcsu::units::amp_t CurrentTorqueCalculator::predict_current_draw(
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current) {
  return predict_current_draw_signed(
      duty_cycle, rpm, v_supply, circuit_resistance, free_speed, stall_current);
}
pdcsu::units::amp_t CurrentTorqueCalculator::predict_current_supply(
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current) {
  return predict_current_supply_signed(
      duty_cycle, rpm, v_supply, circuit_resistance, free_speed, stall_current);
}

pdcsu::units::amp_t CurrentTorqueCalculator::predict_current_draw_signed(
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current) {
  const pdcsu::units::ohm_t total_resistance =
      calculate_total_resistance(stall_current, circuit_resistance);
  const pdcsu::units::volt_t back_emf =
      calculate_back_emf(rpm, free_speed, v_supply);
  const pdcsu::units::volt_t voltage_difference =
      duty_cycle * v_supply - back_emf;
  return voltage_difference / total_resistance;
}

pdcsu::units::amp_t CurrentTorqueCalculator::predict_current_supply_signed(
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current) {
  const pdcsu::units::amp_t motor_current = predict_current_draw_signed(
      duty_cycle, rpm, v_supply, circuit_resistance, free_speed, stall_current);
  return duty_cycle * motor_current;
}

pdcsu::units::nm_t CurrentTorqueCalculator::predict_torque(double duty_cycle,
    pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current, pdcsu::units::nm_t stall_torque) {
  const pdcsu::units::amp_t current_draw = predict_current_draw(
      duty_cycle, rpm, v_supply, circuit_resistance, free_speed, stall_current);

  return current_to_torque(current_draw, stall_current, stall_torque);
}

double CurrentTorqueCalculator::scale_current_supply(double scale_factor,
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::rpm_t free_speed) {
  const double pct_speed = (rpm / free_speed).value();

  const volt_t back_emf = pct_speed * v_supply;

  const volt_t denom = 2.0 * v_supply * duty_cycle - back_emf;
  if (u_abs(denom) < volt_t{1e-6}) return duty_cycle;

  const double D_new =
      duty_cycle +
      (scale_factor - 1.0) *
          ((duty_cycle * (duty_cycle * v_supply - back_emf)) / denom).value();
  return std::clamp(D_new, -1.0, 1.0);
}

double CurrentTorqueCalculator::limit_current_draw(double duty_cycle,
    pdcsu::units::amp_t current_limit, pdcsu::units::rpm_t rpm,
    pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
    pdcsu::units::rpm_t free_speed, pdcsu::units::amp_t stall_current) {
  const pdcsu::units::amp_t current_draw = predict_current_draw(
      duty_cycle, rpm, v_supply, circuit_resistance, free_speed, stall_current);
  if (std::abs(current_draw.value()) > current_limit.value()) {
    const pdcsu::units::amp_t limited_current = pdcsu::units::amp_t{
        std::copysign(current_limit.value(), current_draw.value())};
    return current_control(limited_current, rpm, v_supply, circuit_resistance,
        free_speed, stall_current);
  }
  return duty_cycle;
}

pdcsu::units::amp_t CurrentTorqueCalculator::torque_to_current(
    pdcsu::units::nm_t torque, pdcsu::units::nm_t stall_torque,
    pdcsu::units::amp_t stall_current) {
  return (torque / stall_torque) * stall_current;
}
pdcsu::units::nm_t CurrentTorqueCalculator::current_to_torque(
    pdcsu::units::amp_t current, pdcsu::units::amp_t stall_current,
    pdcsu::units::nm_t stall_torque) {
  return (current / stall_current) * stall_torque;
}

double CurrentTorqueCalculator::current_control(
    pdcsu::units::amp_t target_current, pdcsu::units::rpm_t rpm,
    pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
    pdcsu::units::rpm_t free_speed, pdcsu::units::amp_t stall_current) {
  const scalar_t pct_speed = rpm / free_speed;
  const pdcsu::units::ohm_t winding_resistance =
      pdcsu::units::volt_t{12.0} / stall_current;
  const pdcsu::units::ohm_t total_resistance =
      winding_resistance + circuit_resistance;
  const scalar_t duty_cycle_added = (target_current / stall_current) *
                                    (total_resistance / winding_resistance);
  return std::clamp((pct_speed + duty_cycle_added).value(), -1.0, 1.0);
}

double CurrentTorqueCalculator::supply_current_control(
    double original_duty_cycle, pdcsu::units::amp_t target_current,
    pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current) {
  const pdcsu::units::ohm_t total_resistance =
      calculate_total_resistance(stall_current, circuit_resistance);
  const pdcsu::units::volt_t back_emf =
      calculate_back_emf(rpm, free_speed, v_supply);

  const double a = (v_supply / total_resistance).value();
  const double b = (-back_emf / total_resistance).value();
  const double c = -target_current.value();

  if (std::abs(a) < 1e-9) {
    return std::abs(b) < 1e-9 ? 0.0 : std::clamp(-c / b, -1.0, 1.0);
  }

  const double discrim = b * b - 4.0 * a * c;
  const double sqrt_discrim = discrim >= 0.0 ? std::sqrt(discrim) : 0.0;
  const double d1 = std::clamp((-b + sqrt_discrim) / (2.0 * a), -1.0, 1.0);
  const double d2 = std::clamp((-b - sqrt_discrim) / (2.0 * a), -1.0, 1.0);
  const double e1 = std::abs(a * d1 * d1 + b * d1 + c);
  const double e2 = std::abs(a * d2 * d2 + b * d2 + c);

  static constexpr double kSupplyErrorTolerance = 1e-6;
  if (e1 + kSupplyErrorTolerance < e2) return d1;
  if (e2 + kSupplyErrorTolerance < e1) return d2;

  double scaled_duty_cycle = d2;
  if (std::abs(d1 - original_duty_cycle) <=
      std::abs(d2 - original_duty_cycle)) {
    scaled_duty_cycle = d1;
  }
  return scaled_duty_cycle;
}

double CurrentTorqueCalculator::torque_control(pdcsu::units::nm_t target_torque,
    pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
    pdcsu::units::amp_t stall_current, pdcsu::units::nm_t stall_torque) {
  return current_control(
      torque_to_current(target_torque, stall_torque, stall_current), rpm,
      v_supply, circuit_resistance, free_speed, stall_current);
}

}  // namespace funkit::control::calculators