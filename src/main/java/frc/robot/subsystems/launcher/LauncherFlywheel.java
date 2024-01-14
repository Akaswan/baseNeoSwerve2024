package frc.robot.subsystems.launcher;


import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
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
        LauncherFlywheelState.FIELD_BASED_VELOCITY.setVelocity(calculateRPM());
        SmartDashboard.putNumber("Calculated shooter rpm", getVelocity());
    }

    @Override
    public void outputTelemetry() {}

    public double calculateRPM() {
        double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kSpeakerPosition);

        if (distance > 0 && distance < LauncherConstants.kDistanceRPMMap.lastKey()) {
            double lowerRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.floorKey(distance));
            double upperRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.ceilingKey(distance));
            return lowerRPM + (distance - Math.floor(distance)) * (upperRPM - lowerRPM);
        } else {
            return 0;
        }
    }

    public enum LauncherFlywheelState implements VelocitySubsystemState {
        OFF(0, "Off"),
        IDLE(1000, "Idle"),
        TRANSITION(0, "Transition"),
        FIELD_BASED_VELOCITY(0, "Field Based Velocity"),
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
