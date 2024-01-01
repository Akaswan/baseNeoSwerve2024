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

  private Superstructure m_manager = Superstructure.getInstance();

  private ServoSubsystemState m_state;
  private SuperstructureState m_superStructureState;

  private ServoSubsystem[] m_order;

  public SetServoSubsystemState(
      ServoSubsystem subsystem, SuperstructureState superStructureState, ServoSubsystem[] order) {
    m_subsystem = subsystem;
    m_superStructureState = superStructureState;
    m_state = null;
    m_order = order;

    addRequirements(m_subsystem);
    setName(m_subsystem.getSubsystemType().name());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_state == null) {
      switch (m_subsystem.getSubsystemType()) {
        case ARM:
          m_state = m_superStructureState.getArmState();
          break;
        case ELEVATOR:
          m_state = m_superStructureState.getElevatorState();
          break;
        case WRIST:
          m_state = m_superStructureState.getWristState();
          break;
      }
    }

    m_subsystem.setState(m_state);

    // m_manager.setScheduledCommand(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_manager.getDesiredState() != m_superStructureState)
      m_manager.setDesiredState(m_superStructureState);
    if (m_manager.getCurrentState() != SuperstructureState.TRANSITION)
      m_manager.setCurrentState(SuperstructureState.TRANSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_order[m_order.length - 1] == m_subsystem) {
      m_manager.setCurrentState(m_superStructureState);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetpoint();
  }
}
