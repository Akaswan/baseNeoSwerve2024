// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.lib.utilities.GeometryUtils;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private PIDController angleController;

  private SwerveDrive m_drivebase;
  private double m_targetAngle = 0;
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

    angleController = new PIDController(.1, 0, 0);

    addRequirements(m_drivebase);
  }

  /**
   *
   *
   * <h3>TurnToAngle</h3>
   *
   * Turns to the speaker using a pid controller
   *
   * @param m_drivebase The swerve drive that moves the robot
   */
  public TurnToAngle(SwerveDrive drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;

    angleController = new PIDController(.1, 0, 0);

    addRequirements(m_drivebase);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_targetAngle == 0) {
      m_targetAngle = m_drivebase.calculateAngleToSpeaker() < 0 ? m_drivebase.calculateAngleToSpeaker() + 180 : m_drivebase.calculateAngleToSpeaker() - 180;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speeds =
        angleController.calculate(
            GeometryUtils.getAdjustedYawDegrees(m_drivebase.getYawDegrees(), m_targetAngle), 180);

    m_drivebase.drive(0, 0, speeds, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetAngle = 0;
  }

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
