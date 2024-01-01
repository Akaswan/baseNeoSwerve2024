// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.templates.ServoSubsystem;
import frc.robot.subsystems.templates.ServoSubsystem.ServoSubsystemState;

public class SetServoSubsystemState extends Command {
  /** Creates a new SetMechState. */
  private ServoSubsystem m_subsystem;

  private Superstructure m_superstructure = Superstructure.getInstance();

  private ServoSubsystemState m_state;
  private SuperstructureState m_superStructureState;

  public SetServoSubsystemState(ServoSubsystem subsystem, SuperstructureState superStructureState) {
    m_subsystem = subsystem;
    m_superStructureState = superStructureState;
    m_state = null;

    addRequirements(m_subsystem);
    setName(m_subsystem.getSubsystemType().name());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_state == null) {
      switch (m_subsystem.getSubsystemType()) {
        case ARM:
          m_state = m_superStructureState.armState;
          break;
        case ELEVATOR:
          m_state = m_superStructureState.elevatorState;
          break;
        case WRIST:
          m_state = m_superStructureState.wristState;
          break;
      }
    }

    m_subsystem.setState(m_state);

    // m_superstructure.setScheduledCommand(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_superstructure.getDesiredState() != m_superStructureState)
      m_superstructure.setDesiredState(m_superStructureState);
    if (m_superstructure.getCurrentState() != SuperstructureState.TRANSITION)
      m_superstructure.setCurrentState(SuperstructureState.TRANSITION);
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
