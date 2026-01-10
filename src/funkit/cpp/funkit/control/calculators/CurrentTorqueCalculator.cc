#include "funkit/control/calculators/CurrentTorqueCalculator.h"

#include <algorithm>

namespace funkit::control::calculators {

pdcsu::units::amp_t CurrentTorqueCalculator::predict_current_draw(
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs) {
  double pct_speed = rpm.value() / specs.free_speed.value();

  pdcsu::units::amp_t stall_current = specs.stall_current;
  pdcsu::units::amp_t free_current = specs.free_current;
  pdcsu::units::ohm_t winding_resistance =
      pdcsu::units::volt_t{12.0} / (stall_current - free_current);
  pdcsu::units::ohm_t total_resistance =
      winding_resistance + circuit_resistance;

  pdcsu::units::volt_t back_emf =
      pdcsu::units::volt_t{pct_speed * v_supply.value()};
  pdcsu::units::volt_t voltage_difference =
      pdcsu::units::volt_t{duty_cycle * v_supply.value()} - back_emf;

  pdcsu::units::amp_t current_draw = voltage_difference / total_resistance;

  return current_draw;
}

pdcsu::units::nm_t CurrentTorqueCalculator::predict_torque(double duty_cycle,
    pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs) {
  pdcsu::units::amp_t current_draw = predict_current_draw(
      duty_cycle, rpm, v_supply, circuit_resistance, specs);

  return current_to_torque(current_draw, specs);
}

double CurrentTorqueCalculator::scale_current_draw(double scale_factor,
    double duty_cycle, pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    MotorMonkeyType mmtype) {
  MotorSpecs specs = MotorSpecificationPresets::get(mmtype);
  double pct_speed = rpm.value() / specs.free_speed.value();
  pdcsu::units::volt_t back_emf =
      pdcsu::units::volt_t{pct_speed * v_supply.value()};
  pdcsu::units::volt_t voltage_difference =
      pdcsu::units::volt_t{duty_cycle * v_supply.value()} - back_emf;
  pdcsu::units::volt_t output =
      pdcsu::units::volt_t{voltage_difference.value() * scale_factor} +
      back_emf;
  return output.value() / v_supply.value();
}

double CurrentTorqueCalculator::limit_current_draw(double duty_cycle,
    pdcsu::units::amp_t current_limit, pdcsu::units::rpm_t rpm,
    pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
    MotorSpecs specs) {
  pdcsu::units::amp_t current_draw = predict_current_draw(
      duty_cycle, rpm, v_supply, circuit_resistance, specs);
  if (std::abs(current_draw.value()) > current_limit.value()) {
    pdcsu::units::amp_t limited_current = pdcsu::units::amp_t{
        std::copysign(current_limit.value(), current_draw.value())};
    return current_control(
        limited_current, rpm, v_supply, circuit_resistance, specs);
  }
  return duty_cycle;
}

pdcsu::units::amp_t CurrentTorqueCalculator::torque_to_current(
    pdcsu::units::nm_t torque, MotorSpecs specs) {
  return pdcsu::units::amp_t{(torque.value() / specs.stall_torque.value()) *
                             specs.stall_current.value()};
}
pdcsu::units::nm_t CurrentTorqueCalculator::current_to_torque(
    pdcsu::units::amp_t current, MotorSpecs specs) {
  return pdcsu::units::nm_t{(current.value() / specs.stall_current.value()) *
                            specs.stall_torque.value()};
}

double CurrentTorqueCalculator::current_control(
    pdcsu::units::amp_t target_current, pdcsu::units::rpm_t rpm,
    pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
    MotorSpecs specs) {
  double pct_speed = rpm.value() / specs.free_speed.value();

  pdcsu::units::amp_t stall_current = specs.stall_current;
  pdcsu::units::amp_t free_current = specs.free_current;
  pdcsu::units::ohm_t winding_resistance =
      pdcsu::units::volt_t{12.0} / (stall_current - free_current);
  pdcsu::units::ohm_t total_resistance =
      winding_resistance + circuit_resistance;

  double duty_cycle_added =
      (target_current.value() / stall_current.value()) *
      (total_resistance.value() / winding_resistance.value());

  double DC_target = std::clamp(pct_speed + duty_cycle_added, -1.0, 1.0);

  return DC_target;
}

double CurrentTorqueCalculator::torque_control(pdcsu::units::nm_t target_torque,
    pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
    pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs) {
  return current_control(torque_to_current(target_torque, specs), rpm, v_supply,
      circuit_resistance, specs);
}

}  // namespace funkit::control::calculators