// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.manager.ServoMotorSubsystem;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemType;

public class ManualSubsystem extends Command {

  private ServoMotorSubsystem m_subsystem;

  public ManualSubsystem(ServoMotorSubsystem subsystem) {
    m_subsystem = subsystem;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = 0;
    if (m_subsystem.getSubsystemType() == SubsystemType.ARM) {
      throttle =
          RobotContainer.m_driverController.getRightTriggerAxis()
              - RobotContainer.m_driverController.getLeftTriggerAxis();
    } else if (m_subsystem.getSubsystemType() == SubsystemType.ELEVATOR) {
      throttle = -RobotContainer.m_driverController.getRightY();
    } else if (m_subsystem.getSubsystemType() == SubsystemType.WRIST) {
      throttle =
          RobotContainer.m_operatorController.getHID().getLeftBumper()
              ? 1
              : 0 + (RobotContainer.m_operatorController.getHID().getRightBumper() ? -1 : 0);
    }

    m_subsystem.manualControl(throttle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
