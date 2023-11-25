// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrivebase.TeleopSwerve;
import frc.robot.commands.SwerveDrivebase.TurnToAngle;
import frc.robot.subsystems.APTag;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.LoggedDashboardChooser;

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
  public static final int DriverControllerPort = 0;

  // CONTROLLERS \\
  CommandXboxController driverController = new CommandXboxController(0);
  XboxController testcontroller = new XboxController(0);

  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  public static ShuffleboardTab infoTab = INFO ? Shuffleboard.getTab("Info") : null;
  public static ShuffleboardTab tuningTab = TUNING ? Shuffleboard.getTab("Tuning") : null;

  // SUBSYSTEMS \\
  public static final SwerveDrive m_driveBase =
      new SwerveDrive(FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE);
  public static final APTag m_apTag = new APTag();

  // ROBOT STATE MACHINE \\
  // public static final RobotStateMachine m_machine =
  //     new RobotStateMachine();

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>(
          "Auto Picker", AutoBuilder.buildAutoChooser(), mainTab, 0, 0, 2, 1);

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\
    // you would never do this while following a path, its just to show how to implement
    NamedCommands.registerCommand(
        "Zero Yaw", new InstantCommand(() -> m_driveBase.zeroGyroscope()));

    // CONFIGURE DEFAULT COMMANDS \\
    m_driveBase.setDefaultCommand(
        new TeleopSwerve(
            m_driveBase,
            driverController,
            translationAxis,
            strafeAxis,
            rotationAxis,
            true,
            REGULAR_SPEED));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // XBOX 0
    driverController
        .leftBumper()
        .onTrue(
            new TeleopSwerve(
                m_driveBase,
                driverController,
                translationAxis,
                strafeAxis,
                rotationAxis,
                true,
                SLOW_SPEED));

    driverController
        .leftBumper()
        .onFalse(
            new TeleopSwerve(
                m_driveBase,
                driverController,
                translationAxis,
                strafeAxis,
                rotationAxis,
                true,
                REGULAR_SPEED));

    driverController.back().onTrue(new InstantCommand(m_driveBase::zeroGyroscope));

    // Example of an automatic path generated to score in the B2 zone
    driverController
        .a()
        .onTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(1.8252, 2.779, Rotation2d.fromDegrees(180)),
                new PathConstraints(3, 4, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                0.0,
                0.0));

    // Example of an automatic path generated to pick up from the human player
    driverController
        .b()
        .onTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(16.06056, 6.270, Rotation2d.fromDegrees(0)),
                new PathConstraints(3, 4, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                0.0,
                0.0));

    driverController.x().onTrue(new TurnToAngle(m_driveBase, 30));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void tuningInit() {
    m_driveBase.tuningInit();
  }

  public void tuningPeriodic() {
    m_driveBase.tuningPeriodic();
  }

  public void infoInit() {
    m_driveBase.infoInit();
  }

  public void infoPeriodic() {
    m_driveBase.infoPeriodic();
  }
}
