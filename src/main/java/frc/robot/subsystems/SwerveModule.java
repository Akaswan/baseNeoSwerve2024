// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.Constants.*;

import java.util.Map;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.RevUtils;
import frc.robot.utilities.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 0;

  public static final double kDriveRevToMeters = ((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO);
  public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60.0;
  public static final double kTurnRotationsToDegrees = 360.0 / TURN_GEAR_RATIO;

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  private CANCoder m_angleEncoder;

  public final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;

  private GenericEntry angleOffsetEntry;
  private double m_angleOffset;

  private final SparkMaxPIDController m_driveController;
  private SparkMaxPIDController m_turnController;

  double m_currentAngle;
  double m_lastAngle;

  private SimpleWidget m_moduleAngleWidget;
  private SimpleWidget m_moduleSpeedWidget;

  private int m_moduleNumber;

  /**
   * Constructs a SwerveModule.
   *
   * @param moduleNumber The module number
   * @param swerveModuleConstants     Swerve modules constants to setup swerve module
   * @param tuning     Decide whether to tune the angle offset and PID of the module
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants swerveModuleConstants) {
    m_moduleNumber = moduleNumber;

    m_driveMotor = new CANSparkMax(swerveModuleConstants.driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(swerveModuleConstants.turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_angleEncoder = new CANCoder(swerveModuleConstants.cancoderID, "rio");
    m_angleOffset = swerveModuleConstants.angleOffset;

    m_driveMotor.restoreFactoryDefaults();
    RevUtils.setDriveMotorConfig(m_driveMotor);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveMotor.setInverted(true);// MK4i drive motor is inverted

    m_turningMotor.restoreFactoryDefaults();
    RevUtils.setTurnMotorConfig(m_turningMotor);
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.setSmartCurrentLimit(25);
    m_turningMotor.enableVoltageCompensation(12.6);
    m_turningMotor.setInverted(true); // MK4i Steer Motor is inverted

    m_angleEncoder.configFactoryDefault();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kDriveRevToMeters);
    m_driveEncoder.setVelocityConversionFactor(kDriveRpmToMetersPerSecond);
    m_driveEncoder.setPosition(0);

    m_turnEncoder = m_turningMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees);
    m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60);

    m_driveController = m_driveMotor.getPIDController();
    m_turnController = m_turningMotor.getPIDController();

    if (TUNING) {
      angleOffsetEntry = RobotContainer.tuningTab.add("Angle Offset " + m_moduleNumber, m_angleOffset)
      .getEntry();
    }

    if (INFO) {
      m_moduleAngleWidget = RobotContainer.infoTab.add("Module Angle " + m_moduleNumber, getHeadingDegrees())
        .withWidget(BuiltInWidgets.kGyro)
        .withPosition(5 + m_moduleNumber * 2, 2);

      m_moduleSpeedWidget = RobotContainer.infoTab.add("Module Speed " + m_moduleNumber, Units.metersToFeet(getDriveMetersPerSecond()))
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -16, "max", 16))
        .withPosition(5 + m_moduleNumber * 2, 4)
        .withSize(2, 2);
    }
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
    double angle = m_angleEncoder.getAbsolutePosition() - m_angleOffset;
    m_turnEncoder.setPosition(angle);
  }

  public double getHeadingDegrees() {
    if (RobotBase.isReal())
      return m_turnEncoder.getPosition();
    else
      return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    if (RobotBase.isReal())
      return m_driveEncoder.getPosition();
    else
      return m_simDriveEncoderPosition;
  }

  public double getDriveMetersPerSecond() {
    if (RobotBase.isReal())
      return m_driveEncoder.getVelocity();
    else
      return m_simDriveEncoderVelocity;
  }

  public SwerveDriveKinematics getSwerveKinematics() {
    return SwerveDrive.kDriveKinematics;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / MAX_METERS_PER_SECOND;
      m_driveMotor.set(percentOutput);
    } else {
      int DRIVE_PID_SLOT = VEL_SLOT;
      m_driveController.setReference(
          desiredState.speedMetersPerSecond,
          CANSparkMax.ControlType.kVelocity,
          DRIVE_PID_SLOT);
    }
    
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_METERS_PER_SECOND * 0.01)) // Prevent rotating module if speed is less than 1%. Prevents Jittering.
        ? m_lastAngle
        : desiredState.angle.getDegrees(); 
    angle = desiredState.angle.getDegrees();
    m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, POS_SLOT);
    m_lastAngle = angle;

    if (RobotBase.isSimulation()) {
      simUpdateDrivePosition(desiredState);
      // simTurnPosition(angle); 
      m_currentAngle = angle;

    }
  }

  @Override
  public void periodic() {
    if (TUNING) {
      m_angleOffset = angleOffsetEntry.getDouble(m_angleOffset);

      m_driveController.setP(SwerveDrive.dummyDriveController.getP());
      m_driveController.setI(SwerveDrive.dummyDriveController.getI());
      m_driveController.setD(SwerveDrive.dummyDriveController.getD());
      m_driveController.setFF(SwerveDrive.dummyDriveController.getSetpoint());
      m_driveMotor.setOpenLoopRampRate(SwerveDrive.driveRampRateTuning);

      m_turnController.setP(SwerveDrive.dummyTurnController.getP());
      m_turnController.setI(SwerveDrive.dummyTurnController.getI());
      m_turnController.setD(SwerveDrive.dummyTurnController.getD());
      m_turnController.setFF(SwerveDrive.dummyTurnController.getSetpoint());
    }

    if (INFO) {
      m_moduleAngleWidget.getEntry().setDouble(getHeadingDegrees());
      m_moduleSpeedWidget.getEntry().setDouble(Units.metersToFeet(getDriveMetersPerSecond()));
    }

  }

  private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;

    SmartDashboard.putNumber("Module encoder pos" + m_moduleNumber, m_simDriveEncoderPosition);
  }
}
