// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.MechStateMachine;
import frc.robot.StateMachine.MechStates.MechState;

public class SetMechState extends Command {
  /** Creates a new SetMechState. */
  private MechStateMachine m_stateMachine;

  private MechState m_state;

  public SetMechState(MechStateMachine stateMachine, MechState state) {
    m_stateMachine = stateMachine;
    m_state = state;

    // addRequirements(RobotContainer.m_shoulder, RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stateMachine.setState(m_state, true);
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
    return m_stateMachine.atSetPoint();
  }
}
