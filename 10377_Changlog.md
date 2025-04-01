Change log to 10377 Code

Made new Branch Called Dasboard Updates. Intention is to create dashboard inputs as well as verify advantagekit istallation

1.  Modified Build.gradle to allow for Real Drivers Dashboard to be used with Simulation.

2.  Deleted non working path and autos from PathPlanner that were not working.
3. Added workable Autos.  Nothing fancy.  If used, they need triggers added to make things like elevator to move to score. Current Triggers to not match the 10377 Robot.

4. Added Absolute Encoder Offset to the ArmConstants.java file (Line 11).

5. Added Limit switch code to zero arm, and button to dashboard to simulate limit switch in the case we don't install one for arm starting postion.

6. Added dashboard inputs for arm PID tuning.

7. Updated SIM to allow use of actual Drivers Station in Simulation (dont forget to wait for box to check after SIM Launch)

8.






Notes:
Download Pathplanner and updated AdvantageScope on Drivers station computer, as well as computers that are going to be used for programming. Provided below are also other links for future reference:  The advantage kit and Maple sim, are primarly for future reference,  I would not spend too much time on those prior to this weeks competion.

  Link to PathPlanner Documents:  https://pathplanner.dev/pathplanner-gui.html

  Link to PathPlanner install on the Microsoft Store:  https://apps.microsoft.com/detail/9nqbkb5dw909?hl=en-US&gl=US

  Link to AdvantageScope: https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v4.1.5

  Link to advantagekit documentation:  https://docs.advantagekit.org/

  Link to Maple-Sim Documentaion:  https://shenzhen-robotics-alliance.github.io/maple-sim/
