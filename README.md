# Base Neo Swerve 2024

*Untested* Base Neo Swerve Code

#### Includes 
* Pathplanner auto command templates/example and pathplanner on the fly path generation with obstacle avoidance
* Teleop Swerve control with adjustable speeds for more precision
* Sendable Chooser for changing auto plays on the fly
* Tunable PID for auto and for teleop Swerve
* April Tag support and the ability to adjust odometry based on April Tags

#### TODO
* Before you can use you have to tune the teleop swerve PID that can be found in the RevUtils.java file.
    * Make the smartDashboardTuning boolean is true when calling setDriveMotorConfig() or     setTurnMotorConfig() to be able to tuneteleop swerve PID through smartDashboard
* Tune pathplanner PID through SwerveDrive.java
    * There are two seperate sets of PID one for translation and one for rotation
    * There are comments to discern them from eachother
