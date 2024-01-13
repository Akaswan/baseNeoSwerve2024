package frc.robot.subsystems.launcher;


import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.templates.PositionSubsystem;

public class LauncherWrist extends PositionSubsystem {


    private static LauncherWrist m_instance = null;

    public LauncherWrist(PositionSubsystemConstants constants) {
        super(constants);
    }

    public static LauncherWrist getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherWrist(LauncherConstants.kLauncherWristConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
    }

    @Override
    public void outputTelemetry() {
    }

    public enum LauncherWristState implements PositionSubsystemState {
        DOWN(0, 0, "Down"),
        UP(45, 0, "Up"),
        TRANSITION(0, 0, "Transition"),
        MANUAL(0, 0, "Transition");
    
        private double position;
        private double velocity;
        private String name;
    
        private LauncherWristState(double position, double velocity, String name) {
          this.position = position;
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

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void setPosition(double position) {
            this.position = position;
        }
    }
    
}
