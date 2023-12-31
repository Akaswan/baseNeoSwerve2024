package frc.robot.subsystems.superstructure.wrist;

import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.templates.IntakeSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.IntakeSubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class WristIntake extends IntakeSubsystem {

  private static WristIntake m_instance = null;

  public WristIntake(IntakeSubsystemConstants constants) {
    super(constants);
  }

  public static synchronized WristIntake getInstance() {
    if (m_instance == null) {
      m_instance = new WristIntake(WristConstants.kWristIntakeConstants);
    }

    return m_instance;
  }

  @Override
  public void subsystemPeriodic() {}

  @Override
  public void outputTelemetry() {
    Logger.recordOutput(m_constants.kName + "/Voltage", getVolts());
  }

  public enum WristIntakeState implements IntakeSubsystemState {
    IDLE(0, "Idle"),
    STANDBY(3, "Standby"),
    INTAKE(12, "Intake"),
    HOLD(3, "Hold"),
    OUTTAKE(6, "Outtake");

    private double voltage;
    private String name;

    private WristIntakeState(double voltage, String name) {
      this.voltage = voltage;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }

    @Override
    public double getVoltage() {
      return voltage;
    }
  }
}
