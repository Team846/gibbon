#pragma once

#include "funkit/control/base/motor_control_base.h"
#include "funkit/control/base/motor_specs.h"
#include "pdcsu_units.h"

namespace funkit::control::calculators {

using namespace funkit::control::base;

/*
CurrentTorqueCalculator

This class includes static methods to:
    - predict current draw
    - predict torque
    - convert between current and torque
    - control current
    - control torque
*/
class CurrentTorqueCalculator {
public:
  /*
  predict_current_draw()

  @param duty_cycle: The target duty cycle of the motor.
  @param rpm: The current speed of the motor.
  @param v_supply: The motor supply voltage (battery voltage).
  @param circuit_resistance: The resistance of the circuit leading up to the
  motor.
  @param specs: The MotorSpecs of the motor.

  @note rpm is the current speed of the motor, NOT the controlled mechanism.
  */
  static pdcsu::units::amp_t predict_current_draw(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs);
  static pdcsu::units::amp_t predict_current_draw(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorMonkeyType mmtype) {
    return predict_current_draw(duty_cycle, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }

  static double scale_current_draw(double scale_factor, double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      MotorMonkeyType mmtype);

  static double limit_current_draw(double duty_cycle,
      pdcsu::units::amp_t current_limit, pdcsu::units::rpm_t rpm,
      pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
      MotorSpecs specs);
  static double limit_current_draw(double duty_cycle,
      pdcsu::units::amp_t current_limit, pdcsu::units::rpm_t rpm,
      pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
      MotorMonkeyType mmtype) {
    return limit_current_draw(duty_cycle, current_limit, rpm, v_supply,
        circuit_resistance, MotorSpecificationPresets::get(mmtype));
  }
  /*
  predict_torque()

  @param duty_cycle: The target duty cycle of the motor.
  @param rpm: The current speed of the motor.
  @param v_supply: The motor supply voltage (battery voltage).
  @param circuit_resistance: The resistance of the circuit leading up to the
  motor.
  @param specs: The MotorSpecs of the motor.

  @note rpm is the current speed of the motor, NOT the controlled mechanism.
  */
  static pdcsu::units::nm_t predict_torque(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs);
  static pdcsu::units::nm_t predict_torque(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorMonkeyType mmtype) {
    return predict_torque(duty_cycle, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }

  /*
  torque_to_current()

  Converts a torque value to required current draw, given MotorSpecs.
  */
  static pdcsu::units::amp_t torque_to_current(
      pdcsu::units::nm_t torque, MotorSpecs specs);
  static pdcsu::units::amp_t torque_to_current(
      pdcsu::units::nm_t torque, MotorMonkeyType mmtype) {
    return torque_to_current(torque, MotorSpecificationPresets::get(mmtype));
  }

  /*
  current_to_torque()

  Converts a current draw to torque output, given MotorSpecs.
  */
  static pdcsu::units::nm_t current_to_torque(
      pdcsu::units::amp_t current, MotorSpecs specs);
  static pdcsu::units::nm_t current_to_torque(
      pdcsu::units::amp_t current, MotorMonkeyType mmtype) {
    return current_to_torque(current, MotorSpecificationPresets::get(mmtype));
  }

  /*
  current_control()

  Returns a duty cycle (-1 to 1) such that the motor draws the target current.
  If the target current is greater than possible, it is forced to the maximum
  possible current draw.
  */
  static double current_control(pdcsu::units::amp_t target_current,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs);
  static double current_control(pdcsu::units::amp_t target_current,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorMonkeyType mmtype) {
    return current_control(target_current, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }

  /*
  torque_control()

  Returns a duty cycle (-1 to 1) such that the motor outputs the target
  torque. If the target torque is greater than possible, it is forced to the
  maximum possible torque.
  */
  static double torque_control(pdcsu::units::nm_t target_torque,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorSpecs specs);
  static double torque_control(pdcsu::units::nm_t target_torque,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, MotorMonkeyType mmtype) {
    return torque_control(target_torque, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }
};

}  // namespace funkit::control::calculators