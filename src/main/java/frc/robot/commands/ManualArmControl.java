// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.StateMachine.MechStates.ShoulderState;
import frc.robot.StateMachine.RobotState;
import frc.robot.subsystems.Arm;

public class ManualArmControl extends Command {

  private Arm m_arm;
  private final double STICK_DEAD_BAND = 0.1;
  public double m_axis;

  /** Creates a new ManualMechControl. */
  public ManualArmControl(Arm arm, double axis) {
    m_arm = arm;
    m_axis = axis;

    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setState(ShoulderState.MANUAL, false);
    RobotContainer.m_machine.setRobotState(RobotState.MANUAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_axis) > STICK_DEAD_BAND) {
      m_arm.addIntendedPosition(m_axis);
    }
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
