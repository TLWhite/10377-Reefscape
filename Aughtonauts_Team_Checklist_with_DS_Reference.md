# Aughtonauts Robotics – Team Task Plan (Digital Version)

NOTE:  This is not an official TODO list. Just some things that I was thinking about.  Take these to your lead Mentor to discuss.

## 🟧 Wiring & Hardware (Build Team)
- [ ] 🔥 Clean up all wiring and run end effector cables (including Absolute Encoder) through the cable chain
- [ ] 🔥 Install protective cover over electronics area (prevent game piece damage)
- [ ] 🔥 Install new Robot Radio and connect AUX port for Orange Pi
- [ ] 🔥 Run Ethernet cable from Orange Pi to Robot Radio
- [ ] ✅ Mount Orange Pi on robot and secure power
- [ ] ⚠️ Evaluate corral game piece entanglement risk and redesign guard/protection
- [ ] 🧠 Finalize camera locations based on whether PhotonVision will be used or driver assist is primary

## 🧩 Programming & Software (Code Team)
- [ ] 🔥 Install PathPlanner and latest AdvantageScope on all programming laptops
- [ ] 🔥 Verify all laptops are using the latest WPILib 2025 version
- [ ] 🔥 Fix arm absolute encoder offset in code (apply angle correction)
- [ ] 🔥 Fix arm min/max limits — currently not respected in code
- [ ] 🔥 Add Dashboard PID tuning inputs for both Arm and Elevator
- [ ] 🔥 Add code logic for safe movement: block elevator up if arm is down; rotate arm out when elevator lowers
- [ ] ✅ Add missing setpoint values for all scoring levels (L1–L4, Load, Start)
- [ ] ✅ Tune elevator PID once controls are fixed
- [ ] ✅ Build logic to calculate Arm + Elevator setpoints for scoring levels using encoder feedback
- [ ] 🧠 Add logic to switch between PhotonVision pose estimation vs. dashboard camera-only driver assist

## 🧪 Simulation & Operation
- [ ] ✅ Use Field2d + Shuffleboard for visual debugging of robot position
- [ ] ⚙️ Train students on how to use PathPlanner GUI and simulate/test auto paths
- [ ] ⚙️ Teach process of autonomous startup: robot positioning, path start, DS prep

## 🎮 Strategy & Drive Team
- [ ] 🔥 Discuss and assign roles: Driver, Operator, Human Player, Coach
- [ ] ✅ Practice game strategy — including scoring routes, reef alignment, corral piece management
- [ ] 🧠 Decide final vision strategy (PhotonVision vs driver cameras) and camera mounting locations
## 🏁 Competition Schedule Overview
- [ ] **Wednesday**: Load-in, robot setup
- [ ] **Thursday**: Inspection, test & tune, and practice matches
- [ ] **Friday**: Qualification rounds all day
- [ ] **Saturday Morning**: Final qualification matches
- [ ] **Saturday Afternoon**: Alliance Selection, Elimination Matches, Finals, and Awards Ceremony
## 🎯 Strategy & Drive Team Prep
- [ ] Review scoring strategies for autonomous, midgame, and endgame
- [ ] Prepare counter-strategies against common defense bots
- [ ] Create a printed scoring flowchart for field-side reference
- [ ] Hold mock match scenarios to test timing and coordination
- [ ] Review alliance communication roles and signals

## 🧰 Pit Crew Checklist
- [ ] Bring fully stocked pit toolbox (wrenches, zip ties, electrical tape, cutters)
- [ ] Spare motor controllers and motors (Kraken, SparkMax)
- [ ] Laptop with latest code and backup flash drives
- [ ] Label all tools with team number
- [ ] Bring printed wiring diagram and robot map
- [ ] Battery beak and multimeter
- [ ] At least 3 charged batteries at all times
- [ ] Pit banner and safety glasses for every team member

## 📦 Robot Packing List
- [ ] Robot + bumpers (inspected for weight and fit)
- [ ] Batteries (minimum 5) and labeled charger
- [ ] Battery charger cart and extension cables
- [ ] Driver station laptop + joysticks/controllers
- [ ] Programming laptop(s)
- [ ] Radios and Ethernet cables
- [ ] Orange Pi and Vision cameras
- [ ] Game-specific tools (alignment gauges, custom guides)
- [ ] Inspection checklist printout
- [ ] Driver and pit binders
## 🕹️ Driver Station Quick Reference

- [ ] Left Xbox Controller (Driver)
  - Left Stick: Drive
  - Right Stick: Turn
  - Button A: Elevator to Level 2
  - Button B: Elevator to Level 3
  - Button Y: Elevator to Level 4
  - Left Trigger: Intake
  - Right Trigger: Shoot
  - Start: Reset Gyro

- [ ] Right Xbox Controller (Operator)
  - Left Joystick: Manual Elevator Control
  - Right Joystick: Manual Arm Control
  - Additional buttons: Arm preset levels (L0, L1, L2, L3, L4)
