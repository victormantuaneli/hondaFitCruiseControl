#!/usr/bin/env python3
"""
Automated System Verification & Unit Test Suite for ESP32 Cruise Control
Tests all state transitions, safety cutoffs, PID regulation, voltage mapping, and CAN timeout.
"""

import sys
from simulate_cruise import VehicleSim

def run_all_tests():
    print("=" * 75)
    print("  ESP32 HONDA CRUISE CONTROL - AUTOMATED SYSTEM VERIFICATION SUITE")
    print("=" * 75)

    passed_count = 0
    total_tests = 11

    def assert_test(condition, test_name, detail=""):
        nonlocal passed_count
        if condition:
            passed_count += 1
            print(f" [PASS] Test {passed_count:2d}/{total_tests}: {test_name}")
            if detail:
                print(f"        -> {detail}")
        else:
            print(f" [FAIL] FAILED: {test_name}")
            if detail:
                print(f"        -> {detail}")
            sys.exit(1)

    # --------------------------------------------------------------------------
    # Test 1: Cruise Main Latching Switch
    # --------------------------------------------------------------------------
    sim = VehicleSim()
    sim.cruise_main = False
    sim.step(0.05)
    assert_test(sim.state == "OFF", "Cruise Main OFF keeps system in OFF state", f"State is {sim.state}")

    sim.cruise_main = True
    sim.step(0.05)
    assert_test(sim.state == "STANDBY", "Cruise Main ON transitions system to STANDBY", f"State is {sim.state}")

    # --------------------------------------------------------------------------
    # Test 2: Low-Speed Inhibit (< 40 km/h)
    # --------------------------------------------------------------------------
    sim.current_speed = 32.0  # Below 40 km/h minimum
    sim.press_set()
    sim.step(0.05)
    assert_test(sim.state == "STANDBY", "Engagement rejected below 40 km/h threshold", f"Speed={sim.current_speed:.1f} km/h, State={sim.state}")

    # --------------------------------------------------------------------------
    # Test 3: Normal Engagement & Relay Activation
    # --------------------------------------------------------------------------
    sim.current_speed = 72.5
    sim.press_set()
    sim.step(0.05)
    assert_test(
        sim.state == "ACTIVE" and sim.relay_engaged and abs(sim.target_speed - 72.5) < 0.1,
        "SET- engages cruise at current speed and energizes DPDT relay",
        f"Target={sim.target_speed:.1f} km/h, Relay={sim.relay_engaged}"
    )

    # --------------------------------------------------------------------------
    # Test 4: Tap-Up / Tap-Down (RES+ / SET-)
    # --------------------------------------------------------------------------
    initial_target = sim.target_speed
    sim.press_res()  # +1 km/h
    sim.press_res()  # +1 km/h
    sim.step(0.05)
    res_target = sim.target_speed
    sim.press_set()  # -1 km/h
    sim.step(0.05)
    assert_test(
        abs(res_target - (initial_target + 2.0)) < 0.01 and abs(sim.target_speed - (initial_target + 1.0)) < 0.01,
        "RES+ increments target by +1 km/h, SET- decrements by -1 km/h",
        f"Initial={initial_target:.1f} -> After 2xRES={res_target:.1f} -> After SET={sim.target_speed:.1f}"
    )

    # --------------------------------------------------------------------------
    # Test 5: MCP4728 Dual-Slope APP Voltage Tracking
    # --------------------------------------------------------------------------
    # Set throttle to 20%
    sim.throttle = 0.20
    app1, app2 = sim.get_app_voltages()
    expected_app1 = 0.85 + 0.20 * (4.50 - 0.85)
    expected_app2 = 0.425 + 0.20 * (2.25 - 0.425)
    assert_test(
        abs(app1 - expected_app1) < 0.01 and abs(app2 - expected_app2) < 0.01 and abs(app2 - (app1 / 2.0)) < 0.01,
        "MCP4728 outputs paired APP1 (0.85-4.50V) and APP2 (0.425-2.25V) with exact 2:1 ratio",
        f"Throttle=20% -> APP1={app1:.3f}V, APP2={app2:.3f}V (Ratio: {app1/app2:.2f})"
    )

    # --------------------------------------------------------------------------
    # Test 6: Closed-Loop PID Speed Regulation under Hill Load
    # --------------------------------------------------------------------------
    sim.target_speed = 70.0
    sim.current_speed = 70.0
    sim.road_grade_pct = 5.0  # 5% uphill grade
    # Simulate for 8 seconds (160 cycles) to allow vehicle inertia to settle
    for _ in range(160):
        sim.step(0.05)
    assert_test(
        abs(sim.current_speed - 70.0) < 1.5 and sim.throttle > 0.15,
        "PID closed-loop maintains target speed within ±1.5 km/h on 5% uphill grade",
        f"Speed converged to {sim.current_speed:.1f} km/h (Target 70.0) with Throttle={sim.throttle*100:.1f}%"
    )

    # --------------------------------------------------------------------------
    # Test 7: Throttle Authority Safety Ceiling Clamp (50% max)
    # --------------------------------------------------------------------------
    sim.target_speed = 120.0
    sim.current_speed = 50.0  # Large positive error -> PID demands max throttle
    for _ in range(50):
        sim.step(0.05)
    assert_test(
        sim.throttle <= 0.5001,
        "Max throttle authority strictly clamped at 50% (CRUISE_MAX_THROTTLE_LIMIT)",
        f"Large speed error demanded throttle clamped at {sim.throttle*100:.1f}%"
    )

    # --------------------------------------------------------------------------
    # Test 8: Driver Pedal Override Detection & Resume
    # --------------------------------------------------------------------------
    sim.target_speed = 70.0
    sim.current_speed = 70.0
    sim.road_grade_pct = 0.0
    sim.throttle = 0.15  # Normal cruise demand (~15%)
    sim.driver_pedal_override = 0.40  # Driver steps on pedal (40% > 15% + 5%)
    sim.step(0.05)
    override_state = sim.state
    override_relay = sim.relay_engaged

    sim.driver_pedal_override = 0.0  # Driver lets off gas
    # Allow 1 second for return
    for _ in range(20):
        sim.step(0.05)
    resume_state = sim.state
    resume_relay = sim.relay_engaged
    assert_test(
        override_state == "OVERRIDE" and override_relay and resume_state == "ACTIVE" and resume_relay,
        "Driver pedal override maintains relay engaged (Mode A pass-through) and smoothly resumes when released",
        f"On Pedal Press: State={override_state}, Relay={override_relay} -> On Release: State={resume_state}, Relay={resume_relay}"
    )

    # --------------------------------------------------------------------------
    # Test 9: Brake Pedal Instant Cutoff (Critical Safety)
    # --------------------------------------------------------------------------
    sim.brake_pressed = True
    sim.step(0.05)
    app1, app2 = sim.get_app_voltages()
    assert_test(
        sim.state == "STANDBY" and not sim.relay_engaged and sim.throttle == 0.0 and app1 == 0.85,
        "Brake pedal trip immediately de-energizes relay, zeros throttle, and drops to STANDBY",
        f"State={sim.state}, Relay={sim.relay_engaged}, Throttle={sim.throttle*100:.1f}%, APP1={app1:.2f}V"
    )
    sim.brake_pressed = False

    # --------------------------------------------------------------------------
    # Test 10: CAN Bus Timeout Watchdog Cutoff
    # --------------------------------------------------------------------------
    sim.current_speed = 70.0
    sim.press_set()
    sim.step(0.05)
    assert sim.state == "ACTIVE"

    sim.can_timeout = True  # CAN speed stops arriving
    sim.step(0.05)
    assert_test(
        sim.state == "STANDBY" and not sim.relay_engaged and sim.throttle == 0.0,
        "CAN bus speed loss (>250ms) triggers immediate watchdog disengagement",
        f"State={sim.state}, Relay={sim.relay_engaged}, Throttle={sim.throttle*100:.1f}%"
    )

    print("=" * 75)
    print(f"  ALL {passed_count}/{total_tests} SYSTEM TESTS PASSED SUCCESSFULLY! (100% SUCCESS)")
    print("=" * 75)

if __name__ == "__main__":
    run_all_tests()
