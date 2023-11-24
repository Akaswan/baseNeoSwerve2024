// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.GeometryUtils;
import frc.robot.utilities.LocalADStarAK;
import frc.robot.utilities.SwerveModuleConstants;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private boolean isFieldRelative = true;
  public static final Translation2d[] kModuleTranslations = {
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
  };

  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(kModuleTranslations);

  // https://github.com/Thunderstamps/navx2workaround.git
  // link to import navX vendordeps
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 50);

  public final Field2d m_field = new Field2d();

  private SimpleWidget m_gyroWidget;

  private GenericEntry driveRampRateEntry;
  public static double driveRampRateTuning;

  public static PIDController dummyDriveController = new PIDController(0, 0, 0);
  public static PIDController dummyTurnController = new PIDController(0, 0, 0);

  private static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0; // Last year 11.5?
  private static final boolean invertGyro = false;

  private double m_simYaw;

  private SwerveModuleState[] moduleStates;

  private SwerveModule[] m_SwerveMods;

  private ChassisSpeeds robotRelativeChassisSpeeds;

  private SwerveDrivePoseEstimator poseEstimator;

  /**
   *
   *
   * <h3>SwerveDrive</h3>
   *
   * Builds the swerve drivebase from the swerve module class
   *
   * @param frontLeftModuleConstants
   * @param frontRightModuleConstants
   * @param backLeftModuleConstants
   * @param backRightModuleConstants
   * @param m_tuning Decide whether to tune the angle offset and PID of the modules
   */
  public SwerveDrive(
      SwerveModuleConstants frontLeftModuleConstants,
      SwerveModuleConstants frontRightModuleConstants,
      SwerveModuleConstants backLeftModuleConstants,
      SwerveModuleConstants backRightModuleConstants) {

    m_SwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, frontLeftModuleConstants),
          new SwerveModule(1, frontRightModuleConstants),
          new SwerveModule(2, backLeftModuleConstants),
          new SwerveModule(3, backRightModuleConstants)
        };

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetAngleToAbsolute();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kDriveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9));

    m_navx.zeroYaw();
    m_simYaw = 0;

    if (INFO) {
      RobotContainer.infoTab.add("Field", m_field).withPosition(0, 0).withSize(5, 3);
      m_gyroWidget =
          RobotContainer.infoTab
              .add(
                  "Robot Rotation",
                  RobotBase.isReal() ? getYaw() : Units.radiansToDegrees(m_simYaw))
              .withWidget(BuiltInWidgets.kGyro)
              .withPosition(0, 4);
    }

    robotRelativeChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    if (TUNING) {
      RobotContainer.tuningTab.add("Drive Motor PID", dummyDriveController);
      RobotContainer.tuningTab.add("Turn Motor PID", dummyTurnController);
      driveRampRateEntry =
          RobotContainer.tuningTab
              .add("Drive Ramp Rate", 0)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0))
              .getEntry();
      driveRampRateTuning = driveRampRateEntry.getDouble(0);
    }

    AutoBuilder.configureHolonomic(
        this::getPoseMeters, // Robot pose supplier
        this::updateEstimatorWithPose, // Method to reset odometry (will be called if your auto has
        // a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        this // Reference to this subsystem to set requirements
        );

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  public void drive(double throttle, double strafe, double rotation, boolean isOpenLoop) {

    throttle = throttle * MAX_METERS_PER_SECOND;
    strafe = strafe * MAX_METERS_PER_SECOND;
    rotation = rotation * kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getYaw())
            : new ChassisSpeeds(throttle, strafe, rotation);

    moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(moduleStates, isOpenLoop);
    chassisSpeeds = correctForDynamics(chassisSpeeds);

    Logger.recordOutput("Drive/SwerveStateSetpoints", AKitStates(moduleStates));

    robotRelativeChassisSpeeds = correctForDynamics(new ChassisSpeeds(throttle, strafe, rotation));
  }

  public void autoDrive(ChassisSpeeds speeds) {
    moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setSwerveModuleStates(moduleStates, false);
    speeds = correctForDynamics(speeds);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_METERS_PER_SECOND);

    for (int i = 0; i < m_SwerveMods.length; i++) {
      SwerveModule module = m_SwerveMods[i];
      states[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
      module.setDesiredState(states[i], isOpenLoop);
    }
  }

  public Pose2d getPoseMeters() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModule getSwerveModule(int moduleNumber) {
    return m_SwerveMods[moduleNumber];
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_SwerveMods[0].getState(),
      m_SwerveMods[1].getState(),
      m_SwerveMods[2].getState(),
      m_SwerveMods[3].getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_SwerveMods[0].getPosition(),
      m_SwerveMods[1].getPosition(),
      m_SwerveMods[2].getPosition(),
      m_SwerveMods[3].getPosition()
    };
  }

  public void updateEstimatorWithPose(Pose2d updatedPose) {
    poseEstimator.resetPosition(getYaw(), getModulePositions(), updatedPose);
  }

  public static SwerveDriveKinematics getSwerveKinematics() {
    return kDriveKinematics;
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  public Rotation2d getYaw() {
    if (RobotBase.isReal()) {
      return Rotation2d.fromDegrees(-(m_navx.getYaw() + 180));
    } else {
      return (invertGyro)
          ? Rotation2d.fromDegrees(360 - Units.radiansToDegrees(m_simYaw))
          : Rotation2d.fromDegrees(Units.radiansToDegrees(m_simYaw));
    }
  }

  public double getYawDegrees() {
    return getYaw().getDegrees();
  }

  public double getAdjustedYawDegrees() {
    return getYawDegrees() % 360 < 0 ? (getYawDegrees() % 360) + 360.0 : (getYawDegrees() % 360);
  }

  public double getAdjustedYawDegrees(double addedValue) {
    double numTo180 = 180 - addedValue;

    return (getYawDegrees() + numTo180) % 360 < 0
        ? ((getYawDegrees() + numTo180) % 360) + 360.0
        : ((getYawDegrees() + numTo180) % 360);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  // Only used for pathplanner, for some reason they need a chassis speeds supplier
  public ChassisSpeeds getChassisSpeeds() {
    return robotRelativeChassisSpeeds;
  }

  /** Resets each SwerveModule to the absolute position. */
  public void resetAngleToAbsolute() {
    for (SwerveModule mod : m_SwerveMods) {
      mod.resetAngleToAbsolute();
    }
  }

  public double[] AKitStates(SwerveModuleState[] states) {
    double[] output = new double[8];
    for (int i = 0; i < states.length; i++) {
      output[i * 2] = states[i].angle.getDegrees();
      output[i * 2 + 1] = states[i].speedMetersPerSecond;
    }
    return output;
  }

  public double[] AKitOdometry(Pose2d pose) {
    double[] output = new double[3];
    output[0] = pose.getX();
    output[1] = pose.getY();
    output[2] = pose.getRotation().getDegrees();
    return output;
  }

  @Override
  public void periodic() {
    poseEstimator.update(getYaw(), getModulePositions());

    if (RobotBase.isReal()
        && poseEstimator
                .getEstimatedPosition()
                .getTranslation()
                .getDistance(RobotContainer.m_apTag.getPose2d().getTranslation())
            <= 1.0) {
      poseEstimator.addVisionMeasurement(
          RobotContainer.m_apTag.getPose2d(),
          Timer.getFPGATimestamp() - (RobotContainer.m_apTag.getBotPose()[6] / 1000.0));
    }

    Logger.recordOutput("Drive/Gyro", getYawDegrees());
    Logger.recordOutput("Drive/SwerveStates", AKitStates(getModuleStates()));
    Logger.recordOutput("Drive/Pose", AKitOdometry(getPoseMeters()));

    if (TUNING) {
      driveRampRateTuning = driveRampRateEntry.getDouble(0);
    }

    if (INFO) {
      m_gyroWidget.getEntry().setDouble(getYaw().getDegrees());
    }

    m_field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("New Angle", getAdjustedYawDegrees(90));
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kDriveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
  }
}
