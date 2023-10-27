# Base Neo Swerve

*Untested* Base Neo Swerve Code

#### Includes 
* Pathplanner auto command templates/example with built in pre and post commands
* Teleop Swerve control with adjustable speeds for more precision
* Sendable Chooser for changing auto plays on the fly
* Tunable PID for auto and for teleop Swerve

#### TODO
* Before you can use you have to tune the teleop swerve PID that can be found in the RevUtils.java file.
    * Make the smartDashboardTuning boolean true when calling setDriveMotorConfig() or     setTurnMotorConfig() to tune teleop swerve PID through smartDashboard for easier tuning
* Tune pathplanner PID through CreatePath.java
    * There are two seperate sets of PID one for translation and one for rotation. There are comments to discern them from eachother
