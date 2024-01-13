// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherSuperstructure;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class SetSuperstructureState extends Command {
  /** Creates a new SetSuperstructureState. */
  private LauncherSuperstructureState m_desiredState;
  private SuperstructureSubsystem m_superstructure = LauncherSuperstructure.getInstance();

  public SetSuperstructureState(LauncherSuperstructureState desiredState) {
    m_desiredState = desiredState;
    addRequirements(m_superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_superstructure.setSuperstructureState(m_desiredState).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_superstructure.getCurrentState() == m_desiredState;
  }
}
