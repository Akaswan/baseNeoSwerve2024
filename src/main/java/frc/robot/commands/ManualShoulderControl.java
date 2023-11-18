// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.StateMachine.MechStateMachine;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.StateMachineTypes;
import frc.robot.StateMachine.MechStates.ArmState;
import frc.robot.StateMachine.MechStates.MechState;
import frc.robot.StateMachine.MechStates.ShoulderState;
import frc.robot.subsystems.Shoulder;

public class ManualShoulderControl extends Command {

  private Shoulder m_mech;
  private final double STICK_DEAD_BAND = 0.1;
  private CommandXboxController m_controller;
  public double m_percentSpeed;

  /** Creates a new ManualMechControl. */
  public ManualShoulderControl(Shoulder mech, CommandXboxController controller, double percentSpeed) {
    m_mech = mech;
    m_controller = controller;
    m_percentSpeed = percentSpeed;

    addRequirements(m_mech);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_mech.setState(ShoulderState.MANUAL, false);
    RobotContainer.m_machine.setRobotState(RobotState.MANUAL);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axis = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();

    if (Math.abs(axis) > STICK_DEAD_BAND) {
      m_mech.addIntendedPosition(axis * m_percentSpeed);
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
