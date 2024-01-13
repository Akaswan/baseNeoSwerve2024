// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.templates.SuperstructureSubsystem;
import frc.robot.subsystems.templates.VelocitySubsystem;
import frc.robot.subsystems.templates.SuperstructureSubsystem.SuperstructureState;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemState;

public class SetVelocitySubsystemState extends Command {
  /** Creates a new SetMechState. */
  private VelocitySubsystem m_subsystem;
  private VelocitySubsystemState m_subsystemState;

  private SuperstructureSubsystem m_superstructure;
  private SuperstructureState m_superstructureState;

  public SetVelocitySubsystemState(VelocitySubsystem subsystem, VelocitySubsystemState subsystemState, SuperstructureSubsystem superstructure, SuperstructureState superstructureState) {
    m_subsystem = subsystem;
    m_subsystemState = subsystemState;
    m_superstructure = superstructure;
    m_superstructureState = superstructureState;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setDesiredState(m_subsystemState);
    m_superstructure.setDesiredState(m_superstructureState);
    m_superstructure.setCurrentState(m_superstructure.getTransitionState());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetpoint();
  }
}
