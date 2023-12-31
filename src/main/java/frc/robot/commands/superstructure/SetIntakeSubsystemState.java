// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.templates.IntakeSubsystem;
import frc.robot.subsystems.templates.IntakeSubsystem.IntakeSubsystemState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeSubsystemState extends InstantCommand {

  private IntakeSubsystem m_subsystem;
  private IntakeSubsystemState m_state;

  public SetIntakeSubsystemState(IntakeSubsystem subsystem, IntakeSubsystemState state) {
    m_subsystem = subsystem;
    m_state = state;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setState(m_state);
  }
}
