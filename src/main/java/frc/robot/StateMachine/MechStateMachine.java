package frc.robot.StateMachine;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.StateMachine.MechStates.MechState;
import java.util.Map;

public interface MechStateMachine extends Subsystem {
  void initializeStateMap();

  void handleStateAction();

  void setState(MechState state, boolean action);

  MechState getState();

  Map<MechState, Runnable> getStateMap();

  double getPosition();

  boolean atSetPoint();

  StateMachineTypes getType();

  void setIntendedPosition(double intendedPosition);

  class StateMachineInputs {
    public MechState state;
    public MechState previousState;

    public int order;
  }
}
