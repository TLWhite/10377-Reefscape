Change log to 10377 Code

Made new Branch Called Dasboard Updates. Intention is to create dashboard inputs as well as verify advantagekit istallation
2025 03 31 
1.  Modified Build.gradle to allow for Real Drivers Dashboard to be used with Simulation.

2.  Deleted non working path and autos from PathPlanner that were not working.
3. Added workable Autos.  Nothing fancy.  If used, they need triggers added to make things like elevator to move to score. Current Triggers to not match the 10377 Robot.

4. Added Absolute Encoder Offset to the ArmConstants.java file (Line 11).

5. Added Limit switch code to zero arm, and button to dashboard to simulate limit switch in the case we don't install one for arm starting postion.

6. Added dashboard inputs for arm PID tuning.

7. Updated SIM to allow use of actual Drivers Station in Simulation (dont forget to wait for box to check after SIM Launch)

2025 04 01

1. Changes to Arm Code: 
	✅ Proper use of limit switch and encoder

	✅ Manual and PID control support

	✅ Feedforward voltage compensation

	✅ PID values are editable via Shuffleboard and saved with Preferences

	✅ Voltage limiting logic fixed

	✅ Setpoint and position indicators on dashboard

	✅ All constants aligned with expected encoder scale

	✅ Initialization logic improved

2. Robot Container updates: 
	✅Keep canFold with Override: Retain the safety condition for manual arm control with a "Manual Override" toggle.

	✅Full Arm PID Tuning: Add controls to tune kP, kI, kD and apply them via a button.

	✅Arm Setpoint Tuning: Add editable setpoints for L0–L4 with test buttons.

	✅Arm Position Indicators: Show green when the arm is at each setpoint (within tolerance) and yellow otherwise.
	
 Changelog – April 1, 2025
🧠 Strategic Planning & Logic
	Finalized the scoring logic flowchart:

	Inputs from Driver Controller (A, B, Y = L2, L3, L4; X = Load)

	Incorporated canFold and canRaise logic for interlocking arm and elevator movement

	Sequenced: rotate arm to safe → raise/lower elevator → rotate arm to setpoint

🎮 Controller Mappings (Driver - Controller 0)
	Button A → Score at Level 2 (L2)

	Button B → Score at Level 3 (L3)

	Button Y → Score at Level 4 (L4)

	Button X → Move to Loading Position

	Arm moves to LOAD_POS (~ -0.25)

	Elevator remains in L0 position

	Elevator will not raise again until arm rotates outward past canRaise threshold

🧰 Subsystem Enhancements
	Arm
	Added LOAD_POS setpoint and safe position logic for folding/raising

	Implemented canFold() and canRaise() checks

	Added dashboard PID tuning entries and visual layout improvements

	Refactored Arm.java for improved clarity and shorter code without losing functionality

	Preserved your working files and restored when necessary

	Elevator
	Added column-style layout for setpoints and "move to" buttons on the dashboard

	Graph is now moved down to prevent widget overlap

	PID controls grouped together

	Added canFold logic status to Shuffleboard for diagnostics

🖥️ Shuffleboard Layout
	Rebuilt standard layout with clean tab separation for Arm and Elevator

	All widgets aligned in clean columns

	No widgets on top of graphs

	Graphs and positions moved one block down

	Elevator and Arm Setpoints now align vertically, starting 1 square from top

🔧 RobotContainer.java
	Implemented configureButtonBindings():

	All controller logic added for driver scoring buttons

	Sequence of actions follows safe movement logic

	Fixed X Button: previously mapped to “brake,” now correctly moves arm to LOAD_POS

🚫 Bug Fixes & Recovery
	Fixed broken Arm.java, ArmIOReal.java, and RobotContainer.java after misgeneration

	Re-integrated your known-good files

	Resolved build issues related to missing constants in ArmConstants.java

	GEAR_RATIO, ARM_LENGTH, ARM_MASS added












Notes:
Download Pathplanner and updated AdvantageScope on Drivers station computer, as well as computers that are going to be used for programming. Provided below are also other links for future reference:  The advantage kit and Maple sim, are primarly for future reference,  I would not spend too much time on those prior to this weeks competion.

  Link to PathPlanner Documents:  https://pathplanner.dev/pathplanner-gui.html

  Link to PathPlanner install on the Microsoft Store:  https://apps.microsoft.com/detail/9nqbkb5dw909?hl=en-US&gl=US

  Link to AdvantageScope: https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v4.1.5

  Link to advantagekit documentation:  https://docs.advantagekit.org/

  Link to Maple-Sim Documentaion:  https://shenzhen-robotics-alliance.github.io/maple-sim/
