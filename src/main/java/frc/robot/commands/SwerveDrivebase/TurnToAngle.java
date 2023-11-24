// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private PIDController angleController;

  private SwerveDrive m_driveBase;
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
    m_driveBase = drivebase;
    m_targetAngle = targetAngle;

    angleController = new PIDController(.1, 0, 0);

    addRequirements(m_driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speeds =
        angleController.calculate(m_driveBase.getAdjustedYawDegrees(m_targetAngle), 180);

    m_driveBase.drive(0, 0, speeds, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_driveBase.getAdjustedYawDegrees() - m_targetAngle) < deadBand) {
      return true;
    } else {
      return false;
    }
  }
}
