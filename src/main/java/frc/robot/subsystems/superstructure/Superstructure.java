package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetIntakeSubsystemState;
import frc.robot.commands.superstructure.SetServoSubsystemState;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.LED.LEDState;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.Wrist.WristState;
import frc.robot.subsystems.superstructure.wrist.WristIntake;
import frc.robot.subsystems.superstructure.wrist.WristIntake.WristIntakeState;
import frc.robot.subsystems.templates.IntakeSubsystem.IntakeSubsystemState;
import frc.robot.subsystems.templates.ServoSubsystemSparkMax;
import frc.robot.subsystems.templates.ServoSubsystemSparkMax.ServoSubsystemState;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private SuperstructureState m_currentState;
  private SuperstructureState m_desiredState;

  private Arm m_arm = Arm.getInstance();
  private Elevator m_elevator = Elevator.getInstance();
  private Wrist m_wrist = Wrist.getInstance();
  private WristIntake m_wristIntake = WristIntake.getInstance();

  // private SequentialCommandGroup m_scheduledSequentialCommandGroup = new
  // SequentialCommandGroup();

  // private Command m_scheduledCommand;
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

  public static Superstructure getInstance() {
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

  // public void setScheduledCommand(Command command) {
  //   m_scheduledCommand = command;
  // }

  public void queueCommand(Command command) {
    m_commandQueue.add(command);
  }

  public void commandFinished() {
    if (!m_commandQueue.isEmpty()) {
      m_commandQueue.remove(0);
    }
  }

  // public Command getScheduledCommand() {
  //   return m_scheduledCommand != null ? m_scheduledCommand : new Command() {};
  // }

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
      ServoSubsystemSparkMax[] order, SuperstructureState desiredState, boolean eject) {
    SequentialCommandGroup outputCommand = new SequentialCommandGroup();
    // setDesiredState(desiredState);
    // setCurrentState(SuperstructureState.TRANSITION);

    // if (CommandScheduler.getInstance().isScheduled(m_scheduledSequentialCommandGroup)) {
    //   order = swapOrder(order);
    // }

    ArrayList<ServoSubsystemSparkMax> m_order = new ArrayList<>(Arrays.asList(order));

    outputCommand.addCommands(
        new InstantCommand(
            () ->
                RobotContainer.m_superStructureLED.setState(desiredState.superstructureLEDState)));

    if (eject) {
      outputCommand.addCommands(
          new SetIntakeSubsystemState(m_wristIntake, WristIntakeState.OUTTAKE));
      outputCommand.addCommands(new WaitCommand(.25));
    }

    for (int i = 0; i < m_order.size(); i++) {
      if (m_order.get(i) == m_wrist || m_order.get(i) == m_elevator) {
        if (!eject) {
          outputCommand.addCommands(
              new SetIntakeSubsystemState(m_wristIntake, desiredState.wristIntakeState));
        }
        outputCommand.addCommands(
            new SetServoSubsystemState(m_elevator, desiredState)
                .alongWith(new SetServoSubsystemState(m_wrist, desiredState)));
        m_order.remove(m_order.get(i) == m_wrist ? m_elevator : m_wrist);
      } else {
        outputCommand.addCommands(new SetServoSubsystemState(m_order.get(i), desiredState));
      }
    }

    outputCommand.addCommands(new InstantCommand(() -> setCurrentState(desiredState)));

    // m_scheduledSequentialCommandGroup = outputCommand;
    return outputCommand;
  }

  public enum SuperstructureState {
    TRANSITION(
        ArmState.TRANSITION,
        WristState.TRANSITION,
        ElevatorState.TRANSITION,
        WristIntakeState.STANDBY,
        LEDState.BLUE,
        "Transition"),
    HOME(
        ArmState.HOME,
        WristState.HOME,
        ElevatorState.HOME,
        WristIntakeState.STANDBY,
        LEDState.GREEN,
        "Home"),
    GROUND_PICKUP(
        ArmState.HOME,
        WristState.GROUND_PICKUP,
        ElevatorState.HOME,
        WristIntakeState.INTAKE,
        LEDState.RED,
        "Ground Pickup"),
    SUBSTATION_PICKUP(
        ArmState.SUBSTATION_PICKUP,
        WristState.SUBSTATION_PICKUP,
        ElevatorState.SUBSTATION_PICKUP,
        WristIntakeState.INTAKE,
        LEDState.RED,
        "Substation Pickup"),
    SCORE_HIGH(
        ArmState.SCORE_HIGH,
        WristState.SCORE_HIGH,
        ElevatorState.SCORE_HIGH,
        WristIntakeState.OUTTAKE,
        LEDState.BLUE,
        "Score High"),
    SCORE_MID(
        ArmState.SCORE_MID,
        WristState.SCORE_MID,
        ElevatorState.SCORE_MID,
        WristIntakeState.OUTTAKE,
        LEDState.BLUE,
        "Score Mid"),
    SCORE_LOW(
        ArmState.SCORE_LOW,
        WristState.SCORE_LOW,
        ElevatorState.SCORE_LOW,
        WristIntakeState.OUTTAKE,
        LEDState.BLUE,
        "Score Low");

    public ServoSubsystemState armState;
    public ServoSubsystemState elevatorState;
    public ServoSubsystemState wristState;
    public IntakeSubsystemState wristIntakeState;
    public LEDState superstructureLEDState;
    public String name;

    private SuperstructureState(
        ServoSubsystemState armState,
        ServoSubsystemState wristState,
        ServoSubsystemState elevatorState,
        IntakeSubsystemState wristIntakeState,
        LEDState superstructureLEDState,
        String name) {
      this.armState = armState;
      this.wristState = wristState;
      this.elevatorState = elevatorState;
      this.wristIntakeState = wristIntakeState;
      this.superstructureLEDState = superstructureLEDState;
      this.name = name;
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

    Logger.recordOutput(
        "Superstructure/Mech3d",
        new Pose3d[] {
          new Pose3d(
              -.2347,
              0,
              .254,
              new Rotation3d(Math.toRadians(-m_arm.getPosition() + 90), 0, Math.toRadians(90))),
          new Pose3d(
              -.2347
                  + MathUtil.clamp(
                          m_elevator.getPosition(), 0, m_elevator.m_constants.kMaxPosition / 2)
                      * Math.cos(Math.toRadians(m_arm.getPosition())),
              0,
              .254
                  + .005
                  + MathUtil.clamp(
                          m_elevator.getPosition(), 0, m_elevator.m_constants.kMaxPosition / 2)
                      * Math.sin(Math.toRadians(m_arm.getPosition())),
              new Rotation3d(Math.toRadians(-m_arm.getPosition() + 90), 0, Math.toRadians(90))),
          new Pose3d(
              -.2347 + m_elevator.getPosition() * Math.cos(Math.toRadians(m_arm.getPosition())),
              0,
              .254
                  + .005
                  + m_elevator.getPosition() * Math.sin(Math.toRadians(m_arm.getPosition())),
              new Rotation3d(Math.toRadians(-m_arm.getPosition() + 90), 0, Math.toRadians(90))),
          new Pose3d(
              -0.2574
                  + (m_elevator.getPosition() + .54)
                      * Math.cos(Math.toRadians(m_arm.getPosition())),
              0,
              0.2715
                  + (m_elevator.getPosition() + .54)
                      * Math.sin(Math.toRadians(m_arm.getPosition())),
              new Rotation3d(
                  Math.toRadians(-m_arm.getPosition() - m_wrist.getPosition() + 90 + 155),
                  0,
                  Math.toRadians(90)))
        });

    Logger.recordOutput("Superstructure/Current State", m_currentState);
    Logger.recordOutput("Superstructure/Desired State", m_desiredState);
  }
}
