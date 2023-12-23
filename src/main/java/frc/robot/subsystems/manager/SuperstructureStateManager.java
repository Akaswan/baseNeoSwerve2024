package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetSubsystemState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemState;
import java.util.ArrayList;
import java.util.Arrays;

public class SuperstructureStateManager {

  private SuperstructureState m_lastHeldState;
  private SuperstructureState m_currentState;
  private SuperstructureState m_desiredState;

  private Command m_scheduledCommand;
  private SequentialCommandGroup m_scheduledSequentialCommandGroup = new SequentialCommandGroup();

  public SuperstructureStateManager(SuperstructureState initialState) {
    m_currentState = initialState;
    m_desiredState = initialState;
    m_lastHeldState = initialState;
  }

  public void setDesiredState(SuperstructureState desiredState) {
    m_lastHeldState = m_desiredState;
    m_desiredState = desiredState;
  }

  public void setCurrentState(SuperstructureState currentState) {
    m_currentState = currentState;
  }

  public SuperstructureState getDesiredState() {
    return m_desiredState;
  }

  public SuperstructureState getCurrentState() {
    return m_currentState;
  }

  public SuperstructureState getLastHeldState() {
    return m_lastHeldState;
  }

  public void setScheduledCommand(Command command) {
    m_scheduledCommand = command;
  }

  public Command getScheduledCommand() {
    return m_scheduledCommand != null ? m_scheduledCommand : new Command() {};
  }

  public ServoMotorSubsystem[] swapOrder(ServoMotorSubsystem[] originalOrder) {
    ArrayList<ServoMotorSubsystem> newOrder = new ArrayList<>(Arrays.asList(originalOrder));
    int interruptedIndex = 0;
    for (int i = 0; i < originalOrder.length; i++) {
      if (originalOrder[i].getName().toUpperCase().equals(getScheduledCommand().getName())) {
        interruptedIndex = i;
        break;
      }
    }
    ServoMotorSubsystem removedItem = newOrder.remove(interruptedIndex);
    newOrder.add(0, removedItem);
    return newOrder.toArray(originalOrder);
  }

  public SequentialCommandGroup setSuperstructureState(
      ServoMotorSubsystem[] order, SuperstructureState desiredState) {
    SequentialCommandGroup outputCommand = new SequentialCommandGroup();
    setDesiredState(desiredState);
    setCurrentState(SuperstructureState.TRANSITION);

    if (CommandScheduler.getInstance().isScheduled(m_scheduledSequentialCommandGroup)) {
      order = swapOrder(order);
    }

    for (int i = 0; i < order.length; i++) {
      outputCommand.addCommands(new SetSubsystemState(order[i], desiredState));
    }

    outputCommand.addCommands(
        new InstantCommand(
            () -> {
              m_currentState = desiredState;
              m_lastHeldState = desiredState;
            }));

    m_scheduledSequentialCommandGroup = outputCommand;
    return outputCommand;
  }

  public enum SuperstructureState {
    TRANSITION(ArmState.TRANSITION, WristState.TRANSITION, ElevatorState.TRANSITION, "Transition"),
    HOME(ArmState.HOME, WristState.HOME, ElevatorState.HOME, "Home"),
    PICKUP(ArmState.IN, WristState.IN, ElevatorState.IN, "Pickup"),
    PLACE(ArmState.OUT, WristState.OUT, ElevatorState.OUT, "Place");

    SubsystemState armState;
    SubsystemState elevatorState;
    SubsystemState wristState;
    String name;

    private SuperstructureState(
        SubsystemState armState,
        SubsystemState wristState,
        SubsystemState elevatorState,
        String name) {
      this.armState = armState;
      this.wristState = wristState;
      this.elevatorState = elevatorState;
      this.name = name;
    }

    public SubsystemState getArmState() {
      return armState;
    }

    public SubsystemState getElevatorState() {
      return elevatorState;
    }

    public SubsystemState getWristState() {
      return wristState;
    }

    public void setArmState(SubsystemState armState) {
      this.armState = armState;
    }

    public void setElevatorState(SubsystemState elevatorState) {
      this.elevatorState = elevatorState;
    }

    public void setWristState(SubsystemState wristState) {
      this.wristState = wristState;
    }

    public String getName() {
      return name;
    }
  }

  // Logic for if going from a specific state from a specific state would break the robot
  // Do an if statement for those specific cases, else return a default setSuperstructureState
  // public SequentialCommandGroup goToState(SuperstructureState desiredState) {
  //     switch (m_currentState) {
  //         case HOME:
  //             switch (desiredState) {
  //                 case HOME:
  //                     break;
  //                 case PICKUP:
  //                     break;
  //                 case PLACE:
  //                     break;
  //                 default:
  //                     break;
  //             }
  //             break;
  //         case PICKUP:
  //             break;
  //         case PLACE:
  //             break;
  //         default:
  //             break;

  //     }
  // }

}
