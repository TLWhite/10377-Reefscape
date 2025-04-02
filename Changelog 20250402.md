
# Changelog - April 2, 2025

NOTE:  I didn't have the updated code from night, so first thing, update your substems constants file. Verify CAN ID's as well as Limits and Setpoints!

## 📝 Commented Code
- **Elevator and Arm Code**: Added comments to all Elevator and Arm code in `RobotContainer.java` for clarity and maintainability.

---

## 🆕 New Subsystem: Corral
### ✅ Added Corral Controls:
Created and integrated new files with comments:
- `Corral.java`
- `CorralConstants.java`
- `CorralIO.java`
- `CorralIOReal.java`
- `CorralIOSim.java`

**Functionality**: Spins a Kraken motor for corral operations.

### 🔁 Updated `RobotContainer.java`:
- Integrated corral controls.

---

## ⚙️ Git Configuration
- **Updated `.gitignore`**: Excluded AdvantageKit auto-generated files (e.g., `*AutoLogged.java`) from Git uploads.
  - These files must now be built locally on each machine.

---

## 🎮 Controller Mappings

### 🧭 Driver Controller
- **Left Trigger**:
  - Action: Runs corral motor forward (loading direction).
  - Behavior: Moves Arm and Elevator to `LOAD_POS` if not already there.
  - **On Release**:
    - Rotates Arm to `L0_POS`.
    - Waits 0.3 seconds, then lowers Elevator to `L1_POS`.

- **XYAB Buttons**: Move Arm and Elevator to scoring positions:
  - A: `L2_POS`
  - B: `L3_POS`
  - Y: `L4_POS`
  - X: `L1_POS` (confirmed earlier)

- **Right Trigger**:
  - Action: Runs corral motor in reverse (scoring direction).
  - Behavior: Stops when released.

### 🕹️ Secondary Controller
- **Left Stick Y**: Manual Elevator control.
- **Right Stick Y**: Manual Arm control.

---

## 🛠️ Subsystem Enhancements

### 🔄 Corral Subsystem
- **Integration**: Fully implemented with support for:
  - Forward and reverse motor control via driver triggers.
  - Stop functionality on trigger release.

- **Driver Controls**:
  - Left trigger: Activates corral forward, coordinates with Arm and Elevator.
  - Right trigger: Activates corral reverse for scoring.

---

## 🔁 Logic and Behavior

### 🎯 Control Logic

- **Left Trigger Behavior**:
  - Activates corral motor forward.
  - Rotates Arm to `L0_POS`.
  - After 0.3-second delay, lowers Elevator from `LOAD_POS` to `L1_POS`.

- **Right Trigger Behavior**:
  - Runs corral motor in reverse when Arm and Elevator reach target scoring positions.
  - Stops motor on release.

- **Added Logic**: Ensures motor stops when triggers are released.

### ✏️ Adjusted Logic
- Refined conditions for:
  - `canFold`
  - `canLift`
  - `wristLimiter`

---

## 🧪 Tools and UI

### 💻 Simulation & Shuffleboard
- **Preserved Features**:
  - Maintained simulation support, auto chooser, and Elastic Dashboard integration.

- **Tuning**:
  - Kept dashboard commands for tuning Elevator and Arm PID.

- **Corral Controls**:
  - Added toggle controls for corral forward, reverse, and stop within the "Arm" tab on Shuffleboard.

---

## 🐛 Bug Fixes

### ✅ Resolved Issues
- **Method References**: Fixed issues with `setPosition` and `setVoltage` by ensuring proper implementations in subsystems.
- **Imports**: Correctly imported `WaitCommand` and `WaitUntilCommand` from `edu.wpi.first.wpilibj2.command`.
- **Syntax Cleanup**: Resolved syntax errors and mismatched braces in `RobotContainer.java`.

---

## 📌 Miscellaneous

### 🧾 Other Updates
- **Logging**: Retained all AdvantageKit logging features.
- **Autonomous**: Verified autonomous command setup remains functional.
- **Controller Mappings**: Preserved mappings for both driver and secondary controllers.

---

## 🚀 Next Steps

### ✅ Review Constants:
- Update `ArmConstants.java`, `ElevatorConstants.java`, and `CorralConstants.java` with correct CAN IDs and setpoint values.
- Check for other needed adjustments (e.g., tolerances, feedforward coefficients).

### 🔧 Test Deployment:
- Deploy and test controller mappings, especially left/right trigger behaviors and setpoint transitions.

### 🔍 Resolve Sync Issue:
- Investigate why last night’s "Saving to USB Drive" updates didn’t sync and ensure proper logging to USB.
