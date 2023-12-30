// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ManualSubsystem;
import frc.robot.commands.SwerveDrivebase.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.manager.SuperstructureStateManager;
import frc.robot.subsystems.manager.SuperstructureStateManager.SuperstructureState;
import frc.robot.utilities.Alert;
import frc.robot.utilities.Alert.AlertType;
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

  // CONTROLLERS \\
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final XboxController m_operatorController =
      new XboxController(OperatorConstants.kOperatorControllerPort);

  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  public static ShuffleboardTab infoTab = kInfoMode ? Shuffleboard.getTab("Info") : null;
  public static ShuffleboardTab driveTuningTab =
      kTuningMode ? Shuffleboard.getTab("Drive Tuning") : null;
  public static ShuffleboardTab mechTuningTab =
      kTuningMode ? Shuffleboard.getTab("Mech Tuning") : null;

  // SUBSYSTEMS \\
  public static final SwerveDrive m_drivebase =
      new SwerveDrive(
          DriveConstants.kFrontLeft,
          DriveConstants.kFrontRight,
          DriveConstants.kBackLeft,
          DriveConstants.kBackRight);
  public static final Limelight m_limelight = new Limelight();
  public static final Arm m_arm = new Arm(ArmConstants.kArmConstants);
  public static final Elevator m_elevator = new Elevator(ElevatorConstants.kElevatorConstants);
  public static final Wrist m_wrist = new Wrist(WristConstants.kWristConstants);
  public static final SuperstructureStateManager m_manager =
      new SuperstructureStateManager(SuperstructureState.HOME);

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>(
          "Auto Picker", AutoBuilder.buildAutoChooser(), mainTab, 0, 0, 2, 1);

  // ALERTS \\
  private Alert infoAlert =
      new Alert("Info Mode Activated, expect decreased network performance.", AlertType.INFO);
  private Alert tuningAlert =
      new Alert("Tuning Mode Activated, expect decreased network performance.", AlertType.INFO);

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\
    // you would never do this while following a path, its just to show how to implement
    NamedCommands.registerCommand(
        "Zero Yaw", new InstantCommand(() -> m_drivebase.zeroGyroscope()));

    // CONFIGURE DEFAULT COMMANDS \\
    m_drivebase.setDefaultCommand(
        new TeleopSwerve(
            m_drivebase,
            m_driverController,
            translationAxis,
            strafeAxis,
            rotationAxis,
            true,
            DriveConstants.kRegularSpeed,
            true));
    m_elevator.setDefaultCommand(new ManualSubsystem(m_elevator));
    m_arm.setDefaultCommand(new ManualSubsystem(m_arm));
    m_wrist.setDefaultCommand(new ManualSubsystem(m_wrist));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // XBOX 0
    // m_driverController
    //     .leftBumper()
    //     .onTrue(
    //         new TeleopSwerve(
    //             m_drivebase,
    //             m_driverController,
    //             translationAxis,
    //             strafeAxis,
    //             rotationAxis,
    //             true,
    //             DriveConstants.kSlowSpeed,
    //             true));

    // m_driverController
    //     .leftBumper()
    //     .onFalse(
    //         new TeleopSwerve(
    //             m_drivebase,
    //             m_driverController,
    //             translationAxis,
    //             strafeAxis,
    //             rotationAxis,
    //             true,
    //             DriveConstants.kRegularSpeed,
    //             true));

    // m_driverController.back().onTrue(new InstantCommand(m_drivebase::zeroGyroscope));

    // Example of an automatic path generated to score in the B2 zone
    // m_driverController
    //     .a()
    //     .onTrue(
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(1.8252, 2.779, Rotation2d.fromDegrees(180)),
    //             new PathConstraints(3, 4, Units.degreesToRadians(360),
    // Units.degreesToRadians(540)),
    //             0.0,
    //             0.0));

    // Example of an automatic path generated to pick up from the human player
    // m_driverController
    //     .b()
    //     .onTrue(
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(16.06056, 6.270, Rotation2d.fromDegrees(0)),
    //             new PathConstraints(3, 4, Units.degreesToRadians(360),
    // Units.degreesToRadians(540)),
    //             0.0,
    //             0.0));

    // m_driverController.b().onTrue(new TurnToAngle(m_drivebase, 30));

    // m_driverController
    //     .a()
    //     .onTrue(new InstantCommand(() -> m_drivebase.setAngleToSnap(AngleToSnap.BACKWARD)));
    // m_driverController
    //     .y()
    //     .onTrue(new InstantCommand(() -> m_drivebase.setAngleToSnap(AngleToSnap.FORWARD)));
    // m_driverController.x().onTrue(new InstantCommand(m_drivebase::toggleXWheels));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void tuningInit() {
    m_drivebase.tuningInit();
    tuningAlert.set(true);
  }

  public void tuningPeriodic() {
    m_drivebase.tuningPeriodic();
  }

  public void infoInit() {
    m_drivebase.infoInit();
    infoAlert.set(true);
  }

  public void infoPeriodic() {
    m_drivebase.infoPeriodic();
  }

  public void realPeriodic() {
    m_drivebase.realPeriodic();
    m_limelight.realPeriodic();
  }

  public void periodic() {
    if (m_operatorController.getAButtonPressed()) {
      m_manager.goToState(SuperstructureState.HOME, m_arm, m_elevator, m_wrist).schedule();
    }
    if (m_operatorController.getXButtonPressed()) {
      m_manager
          .goToState(SuperstructureState.SUBSTATION_PICKUP, m_arm, m_elevator, m_wrist)
          .schedule();
    }
    if (m_operatorController.getBButtonPressed()) {
      m_manager.goToState(SuperstructureState.GROUND_PICKUP, m_arm, m_elevator, m_wrist).schedule();
    }
    if (m_operatorController.getPOV() == 0) {
      m_manager.goToState(SuperstructureState.SCORE_HIGH, m_arm, m_elevator, m_wrist).schedule();
    }
    if (m_operatorController.getPOV() == 90) {
      m_manager.goToState(SuperstructureState.SCORE_MID, m_arm, m_elevator, m_wrist).schedule();
    }
    if (m_operatorController.getPOV() == 180) {
      m_manager.goToState(SuperstructureState.SCORE_LOW, m_arm, m_elevator, m_wrist).schedule();
    }
  }
}
