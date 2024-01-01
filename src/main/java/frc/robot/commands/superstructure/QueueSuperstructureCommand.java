// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QueueSuperstructureCommand extends InstantCommand {

  private SuperstructureState m_superStructureState;
  private boolean m_eject;

  public QueueSuperstructureCommand(SuperstructureState superstructureState, boolean eject) {
    m_superStructureState = superstructureState;
    m_eject = eject;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Superstructure.getInstance()
        .queueCommand(new SetSuperstructureState(m_superStructureState, m_eject));
  }
}
