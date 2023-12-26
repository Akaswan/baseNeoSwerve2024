// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.GeometryUtils;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private ProfiledPIDController angleController;

  private SwerveDrive m_drivebase;
  private double m_targetAngle;
  private double deadBand = .5;

  /**
   *
   *
   * <h3>TurnToAngle</h3>
   *
   * Turns to a specified angle using a pid controller
   *
   * @param m_drivebase The swerve drive that moves the robot
   * @param targetAngle Target angle to turn to
   */
  public TurnToAngle(SwerveDrive drivebase, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    m_targetAngle = targetAngle;

    angleController = new ProfiledPIDController(.1, 0, 0, new TrapizoidProfile.constrains(10, 5));

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speeds =
        angleController.calculate(
            GeometryUtils.getAdjustedYawDegrees(m_drivebase.getYawDegrees(), m_targetAngle), 180);

    m_drivebase.drive(0, 0, speeds, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_drivebase.getYawDegrees() - m_targetAngle) < deadBand) {
      return true;
    } else {
      return false;
    }
  }
}
