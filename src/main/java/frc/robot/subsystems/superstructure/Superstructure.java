package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.superstructure.SetIntakeSubsystemState;
import frc.robot.commands.superstructure.SetServoSubsystemState;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.Wrist.WristState;
import frc.robot.subsystems.superstructure.wrist.WristIntake;
import frc.robot.subsystems.superstructure.wrist.WristIntake.WristIntakeState;
import frc.robot.subsystems.templates.IntakeSubsystem.IntakeSubsystemState;
import frc.robot.subsystems.templates.ServoSubsystem;
import frc.robot.subsystems.templates.ServoSubsystem.ServoSubsystemState;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private SuperstructureState m_currentState;
  private SuperstructureState m_desiredState;

  // private SequentialCommandGroup m_scheduledSequentialCommandGroup = new
  // SequentialCommandGroup();

  private Command m_scheduledCommand;
  /*
   * First item = command scheduled right now
   * Subsequent items = commands in queue
   * No items = no command in queue
   */
  private ArrayList<Command> m_commandQueue = new ArrayList<>();

  private static Superstructure m_instance = null;

  public Superstructure(SuperstructureState initialState) {
    m_currentState = initialState;
    m_desiredState = initialState;
  }

  public static synchronized Superstructure getInstance() {
    if (m_instance == null) {
      m_instance = new Superstructure(SuperstructureState.HOME);
    }

    return m_instance;
  }

  public void setDesiredState(SuperstructureState desiredState) {
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

  public void setScheduledCommand(Command command) {
    m_scheduledCommand = command;
  }

  public void queueCommand(Command command) {
    m_commandQueue.add(command);
  }

  public void commandFinished() {
    if (!m_commandQueue.isEmpty()) {
      m_commandQueue.remove(0);
    }
  }

  public Command getScheduledCommand() {
    return m_scheduledCommand != null ? m_scheduledCommand : new Command() {};
  }

  // public ServoSubsystem[] swapOrder(ServoSubsystem[] originalOrder) {
  //   ArrayList<ServoSubsystem> newOrder = new ArrayList<>(Arrays.asList(originalOrder));
  //   int interruptedIndex = 0;
  //   for (int i = 0; i < originalOrder.length; i++) {
  //     if (originalOrder[i].getName().toUpperCase().equals(getScheduledCommand().getName())) {
  //       interruptedIndex = i;
  //       break;
  //     }
  //   }
  //   ServoSubsystem removedItem = newOrder.remove(interruptedIndex);
  //   newOrder.add(0, removedItem);
  //   return newOrder.toArray(originalOrder);
  // }

  public SequentialCommandGroup setSuperstructureState(
      ServoSubsystem[] order, SuperstructureState desiredState, boolean eject) {
    SequentialCommandGroup outputCommand = new SequentialCommandGroup();
    // setDesiredState(desiredState);
    // setCurrentState(SuperstructureState.TRANSITION);

    // if (CommandScheduler.getInstance().isScheduled(m_scheduledSequentialCommandGroup)) {
    //   order = swapOrder(order);
    // }

    for (int i = 0; i < order.length; i++) {
      if (order[i] == Wrist.getInstance())
        outputCommand.addCommands(
            new SetIntakeSubsystemState(
                WristIntake.getInstance(), desiredState.getWristIntakeState()));
      outputCommand.addCommands(new SetServoSubsystemState(order[i], desiredState, order));
    }

    // m_scheduledSequentialCommandGroup = outputCommand;
    return outputCommand;
  }

  public enum SuperstructureState {
    TRANSITION(
        ArmState.TRANSITION,
        WristState.TRANSITION,
        ElevatorState.TRANSITION,
        WristIntakeState.STANDBY,
        "Transition"),
    HOME(ArmState.HOME, WristState.HOME, ElevatorState.HOME, WristIntakeState.STANDBY, "Home"),
    GROUND_PICKUP(
        ArmState.HOME,
        WristState.GROUND_PICKUP,
        ElevatorState.HOME,
        WristIntakeState.INTAKE,
        "Ground Pickup"),
    SUBSTATION_PICKUP(
        ArmState.SUBSTATION_PICKUP,
        WristState.SUBSTATION_PICKUP,
        ElevatorState.SUBSTATION_PICKUP,
        WristIntakeState.INTAKE,
        "Substation Pickup"),
    SCORE_HIGH(
        ArmState.SCORE_HIGH,
        WristState.SCORE_HIGH,
        ElevatorState.SCORE_HIGH,
        WristIntakeState.OUTTAKE,
        "Score High"),
    SCORE_MID(
        ArmState.SCORE_MID,
        WristState.SCORE_MID,
        ElevatorState.SCORE_MID,
        WristIntakeState.OUTTAKE,
        "Score Mid"),
    SCORE_LOW(
        ArmState.SCORE_LOW,
        WristState.SCORE_LOW,
        ElevatorState.SCORE_LOW,
        WristIntakeState.OUTTAKE,
        "Score Low");

    ServoSubsystemState armState;
    ServoSubsystemState elevatorState;
    ServoSubsystemState wristState;
    IntakeSubsystemState wristIntakeState;
    String name;

    private SuperstructureState(
        ServoSubsystemState armState,
        ServoSubsystemState wristState,
        ServoSubsystemState elevatorState,
        IntakeSubsystemState wristIntakeState,
        String name) {
      this.armState = armState;
      this.wristState = wristState;
      this.elevatorState = elevatorState;
      this.wristIntakeState = wristIntakeState;
      this.name = name;
    }

    public ServoSubsystemState getArmState() {
      return armState;
    }

    public ServoSubsystemState getElevatorState() {
      return elevatorState;
    }

    public ServoSubsystemState getWristState() {
      return wristState;
    }

    public IntakeSubsystemState getWristIntakeState() {
      return wristIntakeState;
    }

    public String getName() {
      return name;
    }
  }

  @Override
  public void periodic() {
    if (!m_commandQueue.isEmpty()) {
      if (!CommandScheduler.getInstance().isScheduled(m_commandQueue.get(0)))
        m_commandQueue.get(0).schedule();
    }

    Logger.recordOutput("Current State", m_currentState);
    Logger.recordOutput("Desired State", m_desiredState);
  }
}
