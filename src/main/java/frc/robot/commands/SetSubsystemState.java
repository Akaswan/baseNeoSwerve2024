// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manager.ServoMotorSubsystem;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemState;
import frc.robot.subsystems.manager.SuperstructureStateManager;
import frc.robot.subsystems.manager.SuperstructureStateManager.SuperstructureState;

public class SetSubsystemState extends Command {
  /** Creates a new SetMechState. */
  private ServoMotorSubsystem m_subsystem;

  private SuperstructureStateManager m_manager = SuperstructureStateManager.getInstance();

  private SubsystemState m_state;
  private SuperstructureState m_superStructureState;

  private ServoMotorSubsystem[] m_order;

  public SetSubsystemState(ServoMotorSubsystem subsystem, SubsystemState state) {
    m_subsystem = subsystem;
    m_state = state;
    m_superStructureState = null;

    addRequirements(m_subsystem);
  }

  public SetSubsystemState(
      ServoMotorSubsystem subsystem,
      SuperstructureState superStructureState,
      ServoMotorSubsystem[] order) {
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

    m_manager.setScheduledCommand(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_manager.getDesiredState() != m_superStructureState)
      m_manager.setDesiredState(m_superStructureState);
    if (m_manager.getCurrentState() != SuperstructureState.TRANSITION)
      m_manager.setCurrentState(SuperstructureState.TRANSITION);

    // if (SuperstructureState.TRANSITION.getArmState() != RobotContainer.m_arm.getCurrentState()) {
    //   SuperstructureState.TRANSITION.setArmState(RobotContainer.m_arm.getCurrentState());
    // }
    // if (SuperstructureState.TRANSITION.getWristState()
    //     != RobotContainer.m_wrist.getCurrentState()) {
    //   SuperstructureState.TRANSITION.setWristState(RobotContainer.m_wrist.getCurrentState());
    // }
    // if (SuperstructureState.TRANSITION.getElevatorState()
    //     != RobotContainer.m_elevator.getCurrentState()) {
    //
    // SuperstructureState.TRANSITION.setElevatorState(RobotContainer.m_elevator.getCurrentState());
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_order[2] == m_subsystem) {
      m_manager.setCurrentState(m_superStructureState);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetpoint();
  }
}
