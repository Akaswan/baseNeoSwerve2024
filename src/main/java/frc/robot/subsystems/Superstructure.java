package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.superstructure.SetSubsystemState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.templates.ServoMotorSubsystem;
import frc.robot.subsystems.templates.StatedSubsystem.SubsystemState;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private SuperstructureState m_lastHeldState;
  private SuperstructureState m_currentState;
  private SuperstructureState m_desiredState;

  private Command m_scheduledCommand;
  private SequentialCommandGroup m_scheduledSequentialCommandGroup = new SequentialCommandGroup();

  private static Superstructure m_instance = null;

  public static synchronized Superstructure getInstance() {
    if (m_instance == null) {
      m_instance = new Superstructure(SuperstructureState.HOME);
    }

    return m_instance;
  }

  public Superstructure(SuperstructureState initialState) {
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
    // setDesiredState(desiredState);
    // setCurrentState(SuperstructureState.TRANSITION);

    if (CommandScheduler.getInstance().isScheduled(m_scheduledSequentialCommandGroup)) {
      order = swapOrder(order);
    }

    for (int i = 0; i < order.length; i++) {
      outputCommand.addCommands(new SetSubsystemState(order[i], desiredState, order));
    }

    m_scheduledSequentialCommandGroup = outputCommand;
    return outputCommand;
  }

  public enum SuperstructureState {
    TRANSITION(ArmState.TRANSITION, WristState.TRANSITION, ElevatorState.TRANSITION, "Transition"),
    HOME(ArmState.HOME, WristState.HOME, ElevatorState.HOME, "Home"),
    GROUND_PICKUP(ArmState.HOME, WristState.GROUND_PICKUP, ElevatorState.HOME, "Ground Pickup"),
    SUBSTATION_PICKUP(
        ArmState.SUBSTATION_PICKUP,
        WristState.SUBSTATION_PICKUP,
        ElevatorState.SUBSTATION_PICKUP,
        "Substation Pickup"),
    SCORE_HIGH(ArmState.SCORE_HIGH, WristState.SCORE_HIGH, ElevatorState.SCORE_HIGH, "Score High"),
    SCORE_MID(ArmState.SCORE_MID, WristState.SCORE_MID, ElevatorState.SCORE_MID, "Score Mid"),
    SCORE_LOW(ArmState.SCORE_LOW, WristState.SCORE_LOW, ElevatorState.SCORE_LOW, "Score Low");

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
  //   ServoMotorSubsystem[] order;
  //   Arm arm = Arm.getInstance();
  //   Elevator elevator = Elevator.getInstance();
  //   Wrist wrist = Wrist.getInstance();
  //   // switch (desiredState) {
  //   //   case GROUND_PICKUP:
  //   //     if (m_currentState == SuperstructureState.SCORE_HIGH ||m_currentState ==
  //   // SuperstructureState.SCORE_MID || m_currentState == SuperstructureState.SUBSTATION_PICKUP)
  // {
  //   //       order = new ServoMotorSubsystem[]{elevator, arm, wrist};
  //   //     } else if (m_currentState == SuperstructureState.HOME || m_currentState ==
  //   // SuperstructureState.SCORE_LOW) {
  //   //       order = new ServoMotorSubsystem[]{arm, elevator, wrist};
  //   //     }
  //   //     break;
  //   //   case HOME:
  //   //     if (m_currentState == SuperstructureState.SCORE_HIGH ||m_currentState ==
  //   // SuperstructureState.SCORE_MID || m_currentState == SuperstructureState.SUBSTATION_PICKUP)
  // {
  //   //       order = new ServoMotorSubsystem[]{elevator, arm, wrist};
  //   //     } else if (m_currentState == SuperstructureState.GROUND_PICKUP || m_currentState ==
  //   // SuperstructureState.SCORE_LOW) {
  //   //       order = new ServoMotorSubsystem[]{arm, elevator, wrist};
  //   //     }
  //   //     break;
  //   //   case SCORE_HIGH:
  //   //     if (m_currentState == SuperstructureState.SCORE_HIGH ||m_currentState ==
  //   // SuperstructureState.SCORE_MID || m_currentState == SuperstructureState.SUBSTATION_PICKUP)
  // {
  //   //       order = new ServoMotorSubsystem[]{elevator, arm, wrist};
  //   //     } else if (m_currentState == SuperstructureState.GROUND_PICKUP || m_currentState ==
  //   // SuperstructureState.SCORE_LOW) {
  //   //       order = new ServoMotorSubsystem[]{arm, elevator, wrist};
  //   //     }
  //   //     break;
  //   //   case SCORE_LOW:
  //   //     break;
  //   //   case SCORE_MID:
  //   //     break;
  //   //   case SUBSTATION_PICKUP:
  //   //     break;
  //   //   case TRANSITION:
  //   //     break;
  //   //   default:
  //   //     order = new ServoMotorSubsystem[] {arm, elevator, wrist};
  //   //     break;
  //   // }

  //   if (m_currentState == SuperstructureState.SCORE_HIGH
  //       || m_currentState == SuperstructureState.SCORE_MID
  //       || m_currentState == SuperstructureState.SUBSTATION_PICKUP) {
  //     order = new ServoMotorSubsystem[] {wrist, elevator, arm};
  //   } else if (m_currentState == SuperstructureState.HOME
  //       || m_currentState == SuperstructureState.GROUND_PICKUP
  //       || m_currentState == SuperstructureState.SCORE_LOW) {
  //     order = new ServoMotorSubsystem[] {arm, elevator, wrist};
  //   } else {
  //     order = new ServoMotorSubsystem[] {arm, elevator, wrist};
  //   }

  //   return setSuperstructureState(order, desiredState);
  // }

  @Override
  public void periodic() {
    Logger.recordOutput("Current State", m_currentState);
    Logger.recordOutput("Desired State", m_desiredState);
  }
}
