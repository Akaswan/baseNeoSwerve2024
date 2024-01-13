package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class LauncherSuperstructure extends SuperstructureSubsystem {

  private LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  private LauncherWrist m_launcherWrist = LauncherWrist.getInstance();

  private static LauncherSuperstructure m_instance = null;

  public LauncherSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);
  }

  public static LauncherSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new LauncherSuperstructure(LauncherSuperstructureState.OFF, "Launcher");
    }

    return m_instance;
  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    LauncherSuperstructureState launcherDesiredState = (LauncherSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(new SetVelocitySubsystemState(m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState).alongWith(new SetPositionSubsystemState(m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState)));
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = launcherDesiredState));

    return outputCommand;
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return LauncherSuperstructureState.TRANSITION;
  }

  public enum LauncherSuperstructureState implements SuperstructureState {
    OFF(
        LauncherFlywheelState.OFF,
        LauncherWristState.DOWN,
        "Off"),
    IDLE(
        LauncherFlywheelState.IDLE,
        LauncherWristState.DOWN,
        "Idle"),
    LAUNCH(
        LauncherFlywheelState.RUNNING,
        LauncherWristState.UP,
        "Launch"),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      "Transition"
    );


    public LauncherFlywheelState launcherFlywheelState;
    public LauncherWristState launcherWristState;
    public String name;

    private LauncherSuperstructureState(
        LauncherFlywheelState launcherFlywheelState,
        LauncherWristState launcherWristState,
        String name) {
      this.launcherFlywheelState = launcherFlywheelState;
      this.launcherWristState = launcherWristState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }
}
