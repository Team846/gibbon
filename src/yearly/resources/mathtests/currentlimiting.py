import math

CURRENT_LIMIT_A = 40.0
COMMANDED_DC = -0.75
LAST_DC = 0

FREE_SPEED_RPM = 6000.0
STALL_CURRENT_A = 366.0
V_SUPPLY = 12.0
R_CIRCUIT = 0.021


def winding_resistance():
    return 12.0 / STALL_CURRENT_A

def total_resistance():
    return winding_resistance() + R_CIRCUIT

def back_emf(rpm):
    return (rpm / FREE_SPEED_RPM) * V_SUPPLY

def predict_current_draw(duty_cycle, rpm):
    return (duty_cycle * V_SUPPLY - back_emf(rpm)) / total_resistance()

def predict_current_supply(duty_cycle, rpm):
    return duty_cycle * predict_current_draw(duty_cycle, rpm)

def supply_current_control(original_duty_cycle, target_current_a, rpm):
    R = total_resistance()
    emf = back_emf(rpm)
    a = V_SUPPLY / R
    b = -emf / R
    c = -target_current_a
    if abs(a) < 1e-9:
        return 0.0 if abs(b) < 1e-9 else max(-1.0, min(1.0, -c / b))
    disc = b * b - 4.0 * a * c
    sd = math.sqrt(disc) if disc >= 0.0 else 0.0
    d1 = max(-1.0, min(1.0, (-b + sd) / (2.0 * a)))
    d2 = max(-1.0, min(1.0, (-b - sd) / (2.0 * a)))
    e1 = abs(a * d1 * d1 + b * d1 + c)
    e2 = abs(a * d2 * d2 + b * d2 + c)
    err_tol = 1e-6
    if e1 + err_tol < e2:
        return d1
    if e2 + err_tol < e1:
        return d2
    return d1 if abs(d1 - original_duty_cycle) <= abs(d2 - original_duty_cycle) else d2


def main():
    rpm = LAST_DC * FREE_SPEED_RPM
    i_motor_cmd = predict_current_draw(COMMANDED_DC, rpm)
    i_supply_cmd = predict_current_supply(COMMANDED_DC, rpm)

    print(f"Current limit (supply) : {CURRENT_LIMIT_A:+.2f} A")
    print(f"Commanded DC           : {COMMANDED_DC:+.4f}")
    print(f"Last DC (-> rpm)       : {LAST_DC:+.4f}  ({rpm:.0f} RPM)")
    print()

    if abs(i_supply_cmd) <= CURRENT_LIMIT_A:
        print("Result: NOT limited")
        print(f"  Output DC : {COMMANDED_DC:+.4f}")
        print(f"  I_motor   : {i_motor_cmd:+8.2f} A")
        print(f"  I_supply  : {i_supply_cmd:+8.2f} A")
        return

    target = math.copysign(CURRENT_LIMIT_A, i_supply_cmd)
    d_out = supply_current_control(COMMANDED_DC, target, rpm)
    i_motor_out = predict_current_draw(d_out, rpm)
    i_supply_out = predict_current_supply(d_out, rpm)
    print("Result: LIMITED")
    print(f"  At commanded DC : I_motor = {i_motor_cmd:+8.2f} A, "
          f"I_supply = {i_supply_cmd:+8.2f} A")
    print(f"  Output DC       : {d_out:+.4f}")
    print(f"  I_motor         : {i_motor_out:+8.2f} A")
    print(f"  I_supply        : {i_supply_out:+8.2f} A")


if __name__ == "__main__":
    main()
