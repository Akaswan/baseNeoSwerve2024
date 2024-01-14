// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.utilities.CtreUtils;
import frc.robot.utilities.LoggedShuffleboardTunableNumber;
import frc.robot.utilities.RevUtils;
import frc.robot.utilities.SwerveModuleConstants;
import java.util.Map;

public class SwerveModule extends SubsystemBase {
  private TalonFX m_driveMotor;
  private TalonFX m_turningMotor;
  private CANcoder m_angleEncoder;

  // public final RelativeEncoder m_driveEncoder;
  // private final RelativeEncoder m_turnEncoder;

  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;

  private LoggedShuffleboardTunableNumber m_angleOffset;
  private double m_lastAngleOffset;

  // private final SparkPIDController m_driveController;
  // private SparkPIDController m_turnController;

  double m_currentAngle;
  double m_lastAngle;

  private SimpleWidget m_moduleAngleWidget;
  private SimpleWidget m_moduleSpeedWidget;

  private int m_moduleNumber;

  /**
   * Constructs a SwerveModule.
   *
   * @param moduleNumber The module number
   * @param swerveModuleConstants Swerve modules constants to setup swerve module
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants swerveModuleConstants) {
    m_moduleNumber = moduleNumber;

    m_driveMotor =
        new TalonFX(
            swerveModuleConstants.driveMotorChannel);
    m_turningMotor =
        new TalonFX(
            swerveModuleConstants.turningMotorChannel);

    m_angleEncoder = new CANcoder(swerveModuleConstants.cancoderID, "rio");
    m_angleOffset =
        new LoggedShuffleboardTunableNumber(
            "Module " + m_moduleNumber + " Offset",
            swerveModuleConstants.angleOffset,
            RobotContainer.driveTuningTab,
            BuiltInWidgets.kTextView,
            Map.of("min", 0),
            m_moduleNumber,
            2);
    m_lastAngleOffset = m_angleOffset.get();

    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_driveMotor.setInverted(true); // MK4i drive motor is inverted
    m_driveMotor.getConfigurator().apply(CtreUtils.generateDriveMotorConfig());
    m_driveMotor.setPosition(0);

    m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_turningMotor.setInverted(true);
    m_turningMotor.getConfigurator().apply(CtreUtils.generateTurnMotorConfig());

    m_angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    m_angleEncoder.getConfigurator().apply(CtreUtils.generateCanCoderConfig());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition().getValueAsDouble() - m_angleOffset.get();
    m_turningMotor.setPosition(angle);
  }

  public double getHeadingDegrees() {
    if (RobotBase.isReal()) return m_turningMotor.getPosition().getValue();
    else return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    if (RobotBase.isReal()) return m_driveMotor.getPosition().getValue();
    else return m_simDriveEncoderPosition;
  }

  public double getDriveMetersPerSecond() {
    if (RobotBase.isReal()) return m_driveMotor.getVelocity().getValue();
    else return m_simDriveEncoderVelocity;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxMetersPerSecond;
      m_driveMotor.set(percentOutput);
    } else {
      m_driveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (DriveConstants.kMaxMetersPerSecond
                    * 0.01)) // Prevent rotating module if speed is less than 1%. Prevents
            // Jittering.
            ? m_lastAngle
            : desiredState.angle.getDegrees();
    angle = desiredState.angle.getDegrees();
    m_turningMotor.setControl(new PositionVoltage(angle));
    m_lastAngle = angle;

    if (RobotBase.isSimulation()) {
      simUpdateDrivePosition(desiredState);
      // simTurnPosition(angle);
      m_currentAngle = angle;
    }
  }

  @Override
  public void periodic() {}

  private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;
  }

  public void tuningInit() {}

  public void tuningPeriodic(boolean applyConfigs) {

    if (applyConfigs) {
      System.out.println("Bruh");
      m_driveMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withSlot0(
          new Slot0Configs()
            .withKP(SwerveDrive.drivekp.get())
            .withKI(SwerveDrive.driveki.get())
            .withKD(SwerveDrive.drivekd.get()))
        .withOpenLoopRamps(
          new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(SwerveDrive.driveRampRate.get())
        )
      );
      m_driveMotor.getConfigurator().apply(
          new Slot0Configs()
            .withKP(SwerveDrive.turnkp.get())
            .withKI(SwerveDrive.turnki.get())
            .withKD(SwerveDrive.turnkd.get())
      );
    }

    if (m_lastAngleOffset != m_angleOffset.get()) {
      resetAngleToAbsolute();
      m_lastAngleOffset = m_angleOffset.get();
    }
  }

  public void infoInit() {
    m_moduleAngleWidget =
        RobotContainer.infoTab
            .add("Module Angle " + m_moduleNumber, getHeadingDegrees())
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(5 + m_moduleNumber * 2, 2);

    m_moduleSpeedWidget =
        RobotContainer.infoTab
            .add("Module Speed " + m_moduleNumber, Units.metersToFeet(getDriveMetersPerSecond()))
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -16, "max", 16))
            .withPosition(5 + m_moduleNumber * 2, 4)
            .withSize(2, 2);
  }

  public void infoPeriodic() {
    m_moduleAngleWidget.getEntry().setDouble(getHeadingDegrees());
    m_moduleSpeedWidget.getEntry().setDouble(Units.metersToFeet(getDriveMetersPerSecond()));
  }
}
