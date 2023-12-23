// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.manager.ServoMotorSubsystem;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemState;
import frc.robot.subsystems.manager.SuperstructureStateManager;
import frc.robot.subsystems.manager.SuperstructureStateManager.SuperstructureState;

public class SetSubsystemState extends Command {
  /** Creates a new SetMechState. */
  private ServoMotorSubsystem m_subsystem;

  private SuperstructureStateManager m_manager = RobotContainer.m_manager;

  private SubsystemState m_state;
  private SuperstructureState m_superStructureState;

  public SetSubsystemState(ServoMotorSubsystem subsystem, SubsystemState state) {
    m_subsystem = subsystem;
    m_state = state;
    m_superStructureState = null;

    addRequirements(m_subsystem);
  }

  public SetSubsystemState(ServoMotorSubsystem subsystem, SuperstructureState superStructureState) {
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

    m_manager.setScheduledCommand(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SuperstructureState.TRANSITION.getArmState() != RobotContainer.m_arm.getCurrentState()) {
      SuperstructureState.TRANSITION.setArmState(RobotContainer.m_arm.getCurrentState());
    }
    if (SuperstructureState.TRANSITION.getWristState()
        != RobotContainer.m_wrist.getCurrentState()) {
      SuperstructureState.TRANSITION.setWristState(RobotContainer.m_wrist.getCurrentState());
    }
    if (SuperstructureState.TRANSITION.getElevatorState()
        != RobotContainer.m_elevator.getCurrentState()) {
      SuperstructureState.TRANSITION.setElevatorState(RobotContainer.m_elevator.getCurrentState());
    }
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
