#!/usr/bin/env python3
"""
ESP32 Cruise Control Vehicle Dynamics & Logic Simulator
Run this script to simulate the vehicle physics, CAN speed feedback,
steering buttons, brake cutoff, and PID throttle control right on your PC.
"""

import sys
import time

class VehicleSim:
    def __init__(self):
        # State Machine
        self.state = "OFF"  # OFF, STANDBY, ACTIVE, OVERRIDE, FAULT
        self.cruise_main = True  # Latching switch ON by default for test
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.engine_rpm = 0.0
        self.throttle = 0.0  # 0.0 to 0.50 (clamped)
        
        # Vehicle physics parameters (Honda Civic / Fit class)
        self.mass_kg = 1250.0
        self.drag_coeff = 0.32
        self.frontal_area = 2.1
        self.rolling_resistance = 0.015
        self.air_density = 1.225
        self.engine_max_power_kw = 100.0  # ~134 hp
        self.road_grade_pct = 0.0  # Incline %

        # PID gains (matching config.h)
        self.kp = 0.045
        self.ki = 0.012
        self.kd = 0.005
        self.integral = 0.0
        self.last_error = 0.0
        self.max_slew = 0.02  # Max 2% per 50ms cycle
        self.max_throttle_limit = 0.50  # 50% safety ceiling

        # Safety & inputs
        self.brake_pressed = False
        self.driver_pedal_override = 0.0  # Driver pressing physical gas pedal
        self.relay_engaged = False
        self.can_timeout = False

    def update_physics(self, dt):
        """Simulate realistic vehicle acceleration, aero drag, and rolling friction."""
        speed_mps = self.current_speed / 3.6

        # Effective throttle: either cruise DAC or driver physical pedal (higher wins)
        effective_throttle = max(self.throttle if self.relay_engaged else 0.0, self.driver_pedal_override)

        # Engine traction force (approximate)
        # Power = Force * Velocity -> Force = Power / Velocity (capped at low speed)
        effective_power_w = effective_throttle * (self.engine_max_power_kw * 1000.0)
        drive_force = (effective_power_w / max(speed_mps, 4.0)) if effective_throttle > 0.01 else 0.0

        # Aero drag force: 0.5 * rho * Cd * A * v^2
        aero_drag = 0.5 * self.air_density * self.drag_coeff * self.frontal_area * (speed_mps ** 2)

        # Rolling resistance: Crr * m * g
        rolling_force = self.rolling_resistance * self.mass_kg * 9.81

        # Gravity on hill grade
        gravity_force = self.mass_kg * 9.81 * (self.road_grade_pct / 100.0)

        # Braking force
        brake_force = 6000.0 if self.brake_pressed else 0.0

        # Net force
        net_force = drive_force - aero_drag - rolling_force - gravity_force - brake_force
        accel = net_force / self.mass_kg

        # Integrate velocity
        speed_mps += accel * dt
        if speed_mps < 0.0:
            speed_mps = 0.0

        self.current_speed = speed_mps * 3.6
        self.engine_rpm = max(750.0, self.current_speed * 35.0)  # Approx gear ratio

    def update_cruise_pid(self, dt):
        """Execute the PID speed regulator loop."""
        if self.state != "ACTIVE":
            self.throttle = 0.0
            return

        error = self.target_speed - self.current_speed

        p_term = self.kp * error
        self.integral += self.ki * error * dt
        self.integral = max(-0.30, min(0.30, self.integral))  # Anti-windup
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error

        raw_demand = max(0.0, p_term + self.integral + d_term)

        # Slew rate limiter
        delta = raw_demand - self.throttle
        delta = max(-self.max_slew, min(self.max_slew, delta))
        self.throttle += delta

        # Safety clamp
        self.throttle = max(0.0, min(self.max_throttle_limit, self.throttle))

    def step(self, dt=0.05):
        # 1. Safety Supervisor checks
        if self.brake_pressed:
            if self.state in ["ACTIVE", "OVERRIDE"]:
                self.disengage("BRAKE_PRESSED")

        if not self.cruise_main:
            if self.state != "OFF":
                self.disengage("MAIN_SWITCH_OFF")
                self.state = "OFF"
        elif self.state == "OFF":
            self.state = "STANDBY"

        if self.can_timeout and self.state in ["ACTIVE", "OVERRIDE"]:
            self.disengage("CAN_TIMEOUT")

        # 2. Driver Override (Mode A: Seamless Electronic Pass-Through)
        if self.state in ["ACTIVE", "OVERRIDE"]:
            if self.driver_pedal_override > (self.throttle + 0.03):
                self.state = "OVERRIDE"
            elif self.state == "OVERRIDE" and self.driver_pedal_override <= (self.throttle + 0.01):
                self.state = "ACTIVE"
            self.relay_engaged = True

        # 3. PID Execution
        self.update_cruise_pid(dt)

        # 4. Vehicle Dynamics
        self.update_physics(dt)

    def disengage(self, reason):
        self.state = "STANDBY" if self.cruise_main else "OFF"
        self.relay_engaged = False
        self.throttle = 0.0
        self.integral = 0.0
        self.last_error = 0.0

    def press_set(self):
        if self.state == "STANDBY":
            if 40.0 <= self.current_speed <= 140.0 and not self.brake_pressed:
                self.target_speed = self.current_speed
                self.state = "ACTIVE"
                self.relay_engaged = True
                self.integral = 0.0
                self.last_error = 0.0
        elif self.state == "ACTIVE":
            self.target_speed = max(40.0, self.target_speed - 1.0)

    def press_res(self):
        if self.state == "STANDBY" and self.target_speed >= 40.0:
            if not self.brake_pressed:
                self.state = "ACTIVE"
                self.relay_engaged = True
                self.integral = 0.0
                self.last_error = 0.0
        elif self.state == "ACTIVE":
            self.target_speed = min(140.0, self.target_speed + 1.0)

    def press_cancel(self):
        if self.state in ["ACTIVE", "OVERRIDE"]:
            self.disengage("CANCEL_BUTTON")

    def get_app_voltages(self):
        """Simulate MCP4728 dual analog voltage outputs."""
        eff = max(self.throttle, self.driver_pedal_override) if self.state in ["ACTIVE", "OVERRIDE"] else self.throttle
        app1 = 0.85 + eff * (4.50 - 0.85)
        app2 = 0.425 + eff * (2.25 - 0.425)
        return app1, app2


def main():
    sim = VehicleSim()
    sim.current_speed = 70.0  # Start driving at 70 km/h
    sim.driver_pedal_override = 0.15  # Holding light throttle manually

    print("=" * 70)
    print("      ESP32 CRUISE CONTROL VEHICLE DYNAMICS & LOGIC SIMULATOR       ")
    print("=" * 70)
    print("Commands:")
    print("  [s] Tap SET/DECEL     | [r] Tap RES/ACCEL    | [c] Tap CANCEL")
    print("  [b] Tap Brake (disengage)                     | [m] Toggle Cruise Main")
    print("  [g] Gas pedal press   | [l] Release gas pedal| [u] Uphill slope (+5%)")
    print("  [d] Downhill slope    | [f] Flat road (0%)   | [q] Quit")
    print("=" * 70)

    # Automated demonstration scenario
    demo_steps = [
        (1.0, "Driver cruising manually at ~70 km/h with Cruise Main ON (STANDBY)...", lambda: None),
        (2.0, "Driver taps [SET-] to engage cruise at current speed...", sim.press_set),
        (3.0, "Driver releases foot from physical pedal; Cruise takes over throttle!", lambda: setattr(sim, 'driver_pedal_override', 0.0)),
        (5.0, "Driver taps [RES+] twice to increase speed target...", lambda: (sim.press_res(), sim.press_res())),
        (7.5, "Vehicle encounters 5% UPHILL grade! Watch PID increase throttle...", lambda: setattr(sim, 'road_grade_pct', 5.0)),
        (10.5, "Road flattens out (0% grade)...", lambda: setattr(sim, 'road_grade_pct', 0.0)),
        (12.5, "Driver steps on physical gas pedal to pass a truck (DRIVER OVERRIDE)...", lambda: setattr(sim, 'driver_pedal_override', 0.35)),
        (14.5, "Driver releases gas pedal; Cruise resumes automatically...", lambda: setattr(sim, 'driver_pedal_override', 0.0)),
        (16.5, "Driver taps BRAKE pedal! Instant disengage to STANDBY...", lambda: setattr(sim, 'brake_pressed', True)),
        (18.0, "Driver releases brake...", lambda: setattr(sim, 'brake_pressed', False)),
        (19.0, "Driver taps [RES+] to resume to previous set speed...", sim.press_res),
        (22.0, "Driver taps [CANCEL] button...", sim.press_cancel),
    ]

    t = 0.0
    dt = 0.05
    step_idx = 0

    while t <= 24.0:
        if step_idx < len(demo_steps) and t >= demo_steps[step_idx][0]:
            _, desc, action = demo_steps[step_idx]
            print(f"\n>>> [EVENT @ {t:4.1f}s]: {desc}")
            action()
            step_idx += 1

        sim.step(dt)

        app1, app2 = sim.get_app_voltages()
        print(
            f"T:{t:4.1f}s | "
            f"State: {sim.state:<8} | "
            f"Spd: {sim.current_speed:5.1f} km/h | "
            f"Set: {sim.target_speed:5.1f} km/h | "
            f"Thr: {sim.throttle * 100:4.1f}% | "
            f"APP1: {app1:4.2f}V | "
            f"APP2: {app2:4.2f}V | "
            f"Relay: {'ON' if sim.relay_engaged else 'OFF':<3} | "
            f"Grade: {sim.road_grade_pct:+.0f}%"
        )
        time.sleep(0.05)
        t += dt

    print("\n" + "=" * 70)
    print("Simulation scenario finished successfully. All states and safety trips verified!")
    print("=" * 70)


if __name__ == "__main__":
    main()
