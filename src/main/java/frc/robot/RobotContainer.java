// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.SwerveDrivebase.ZeroGyroscope;
// import frc.robot.commands.SwerveDrivebase.CreateAuto;
import frc.robot.commands.SwerveDrivebase.TeleopSwerve;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // SWERVE CONTROLS \\
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;   
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // CONTROLLER PORTS \\
  public static final int kDriverControllerPort = 0;

  // CONTROLLERS \\
  CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);

  // SUBSYSTEMS \\
  public static final SwerveDrive m_drivebase = new SwerveDrive(FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE);

  // DELCARE AUTO PLAYS \\
  private final Command play1;
  // play2;

  // SENDABLE CHOOSER \\
  public static SendableChooser<Command> m_auto_chooser;

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\
    

    play1 = new WaitCommand(1);

    // play2 = new PathPlannerAuto("TestAuto");

    // play2 = new CreateAuto("TestAuto");

    m_auto_chooser = new SendableChooser<>();

    // SET AUTO CHOOSER PLAYS \\
    m_auto_chooser.setDefaultOption("Do Nothing", play1);
    // m_auto_chooser.addOption("Test", play2);

    SmartDashboard.putData(m_auto_chooser);

    // CONFIGURE DEFAULT COMMANDS \\
    m_drivebase.setDefaultCommand(new TeleopSwerve(
      m_drivebase, 
      m_driverController, 
      translationAxis, 
      strafeAxis, 
      rotationAxis, 
      true, 
      REGULAR_SPEED));



    configureButtonBindings();
  }


  private void configureButtonBindings() {
    // XBOX 0
    m_driverController.leftBumper().onTrue(new TeleopSwerve(
      m_drivebase, 
      m_driverController,
      translationAxis, 
      strafeAxis, 
      rotationAxis, 
      true,  
      SLOW_SPEED));

    m_driverController.leftBumper().onFalse(new TeleopSwerve(
      m_drivebase, 
      m_driverController, 
      translationAxis, 
      strafeAxis, 
      rotationAxis, 
      true, 
      REGULAR_SPEED));

    m_driverController.back().onTrue(new ZeroGyroscope(m_drivebase));

    // m_driverController.a().onTrue(AutoBuilder.pathfindToPose(
    //   new Pose2d(5, 10, Rotation2d.fromDegrees(180)),
    //   new PathConstraints(3, 4, Units.degreesToRadians(540), Units.degreesToRadians(720)),
    //   0.0, // Goal end velocity in meters/sec
    //   0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    // ));
  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_auto_chooser.getSelected();
    return new PathPlannerAuto("TestAuto");

  }
}