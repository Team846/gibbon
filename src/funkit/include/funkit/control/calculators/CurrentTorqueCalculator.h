#pragma once

#include "pdcsu_units.h"

namespace funkit::control::calculators {

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
  @param free_speed: The free speed of the motor.
  @param stall_current: The stall current of the motor.

  @note rpm is the current speed of the motor, NOT the controlled mechanism.
  */
  static pdcsu::units::amp_t predict_current_draw(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
      pdcsu::units::amp_t stall_current);
  static pdcsu::units::amp_t predict_current_supply(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
      pdcsu::units::amp_t stall_current);

  static double scale_current_supply(double scale_factor, double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::rpm_t free_speed);

  static double limit_current_draw(double duty_cycle,
      pdcsu::units::amp_t current_limit, pdcsu::units::rpm_t rpm,
      pdcsu::units::volt_t v_supply, pdcsu::units::ohm_t circuit_resistance,
      pdcsu::units::rpm_t free_speed, pdcsu::units::amp_t stall_current);
  /*
  predict_torque()

  @param duty_cycle: The target duty cycle of the motor.
  @param rpm: The current speed of the motor.
  @param v_supply: The motor supply voltage (battery voltage).
  @param circuit_resistance: The resistance of the circuit leading up to the
  motor.
  @param free_speed: The free speed of the motor.
  @param stall_current: The stall current of the motor.
  @param stall_torque: The stall torque of the motor.

  @note rpm is the current speed of the motor, NOT the controlled mechanism.
  */
  static pdcsu::units::nm_t predict_torque(double duty_cycle,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
      pdcsu::units::amp_t stall_current, pdcsu::units::nm_t stall_torque);

  /*
  torque_to_current()

  Converts a torque value to required current draw.
  */
  static pdcsu::units::amp_t torque_to_current(pdcsu::units::nm_t torque,
      pdcsu::units::nm_t stall_torque, pdcsu::units::amp_t stall_current);

  /*
  current_to_torque()

  Converts a current draw to torque output.
  */
  static pdcsu::units::nm_t current_to_torque(pdcsu::units::amp_t current,
      pdcsu::units::amp_t stall_current, pdcsu::units::nm_t stall_torque);

  /*
  current_control()

  Returns a duty cycle (-1 to 1) such that the motor draws the target current.
  If the target current is greater than possible, it is forced to the maximum
  possible current draw.
  */
  static double current_control(pdcsu::units::amp_t target_current,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
      pdcsu::units::amp_t stall_current);

  /*
  torque_control()

  Returns a duty cycle (-1 to 1) such that the motor outputs the target
  torque. If the target torque is greater than possible, it is forced to the
  maximum possible torque.
  */
  static double torque_control(pdcsu::units::nm_t target_torque,
      pdcsu::units::rpm_t rpm, pdcsu::units::volt_t v_supply,
      pdcsu::units::ohm_t circuit_resistance, pdcsu::units::rpm_t free_speed,
      pdcsu::units::amp_t stall_current, pdcsu::units::nm_t stall_torque);
};

}  // namespace funkit::control::calculators