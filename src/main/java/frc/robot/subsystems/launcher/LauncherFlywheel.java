package frc.robot.subsystems.launcher;


import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.templates.VelocitySubsystem;

public class LauncherFlywheel extends VelocitySubsystem {


    private static LauncherFlywheel m_instance = null;

    public LauncherFlywheel(VelocitySubsystemConstants constants) {
        super(constants);
    }

    public static LauncherFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherFlywheel(LauncherConstants.kLauncherFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
    }

    @Override
    public void outputTelemetry() {
    }

    public enum LauncherFlywheelState implements VelocitySubsystemState {
        OFF(0, "Off"),
        IDLE(1000, "Idle"),
        TRANSITION(0, "Transition"),
        RUNNING(5000, "Running");
    
        private double velocity;
        private String name;
    
        private LauncherFlywheelState(double velocity, String name) {
          this.velocity = velocity;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }
    }
    
}
