// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.templates.ServoMotorSubsystem;

public class SetSuperstructureState extends Command {
  /** Creates a new SetSuperstructureState. */
  private SuperstructureState m_desiredState;

  private ServoMotorSubsystem[] order;
  private Arm m_arm = Arm.getInstance();
  private Elevator m_elevator = Elevator.getInstance();
  private Wrist m_wrist = Wrist.getInstance();
  private Superstructure m_manager = Superstructure.getInstance();

  public SetSuperstructureState(SuperstructureState desiredState) {
    m_desiredState = desiredState;
    addRequirements(m_manager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SuperstructureState m_currentState = m_manager.getCurrentState();

    if (m_currentState == SuperstructureState.SCORE_HIGH
        || m_currentState == SuperstructureState.SCORE_MID
        || m_currentState == SuperstructureState.SUBSTATION_PICKUP) {
      order = new ServoMotorSubsystem[] {m_wrist, m_elevator, m_arm};
    } else if (m_currentState == SuperstructureState.HOME
        || m_currentState == SuperstructureState.GROUND_PICKUP
        || m_currentState == SuperstructureState.SCORE_LOW) {
      order = new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist};
    } else {
      order = new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist};
    }

    m_manager.setSuperstructureState(order, m_desiredState).schedule();
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
    return m_manager.getCurrentState() == m_desiredState;
  }
}
