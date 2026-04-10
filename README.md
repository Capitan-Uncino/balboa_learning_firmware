This is an independent project and is not affiliated with Pololu. 

# Balboa Diagnostic Guide

Use this guide to interpret the onboard LEDs and Serial Monitor when the Raspberry Pi is connected.

## LED Reference Table

| LED State | Meaning | Required Action |
| :--- | :--- | :--- |
| **Solid Red** | **I2C Bus Contention.** The Pi is polling so aggressively that the Arduino cannot talk to its sensors. | Increase the polling interval in your Rust code (try 15ms or 20ms). |
| **Blinking Red** | **Heartbeat.** The Arduino CPU is alive and the loop is running correctly. | System is healthy. |
| **Yellow OFF** | **Angle Gate Blocked.** The robot thinks its tilt is > 45°. It will never start the motors in this state. | Hold the robot perfectly vertical. Check Serial Monitor for drift. |
| **Yellow ON** | **Ready to Start.** The robot is upright and waiting for the internal trigger (balance.cpp logic) to start balancing. | Check if Button A needs to be pressed or if the "start angle" is reached. |
| **Solid Green** | **Active Mode.** The motors should be spinning. | If Green is ON but motors are silent, check battery power and motor gains (K). |

## Serial Monitor Diagnostics (115200 Baud)

Open the Arduino Serial Monitor while the Pi is running:

1. **`BUSY_ERRS` Increasing:** If this number is going up fast, your Rust code is "stealing" the bus. You must slow down the Rust loop.
2. **`TILT` Frozen at 0.00:** The IMU has crashed. Check the I2C wires.
3. **`IS_BAL` stays 0:** The Arduino code hasn't seen the "Start" condition. Check your `Balance.cpp` `balanceUpdate()` logic.

## Common Failures

### 1. The "Zero Gain" Death
* **Symptom:** Green LED is ON, but motors are silent. 
* **Cause:** The Pi sent a K-matrix of `[0,0,0,0]`. 
* **Fix:** Ensure your Rust code initializes the gains to the stable defaults before starting.

### 2. The Pi Boot-Crash
* **Symptom:** Red LED stops blinking exactly when the Pi finishes booting.
* **Cause:** Raspberry Pi is using Pin 15 as a Serial Console.
* **Fix:** Run `sudo raspi-config` -> Interface -> Serial -> Disable Login Shell.
