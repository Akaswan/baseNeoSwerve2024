// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.GeometryUtils;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

  public static Limelight camera;

  private NetworkTableInstance m_visionTable;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tl;
  private NetworkTableEntry m_cl;
  private NetworkTableEntry m_tcornxy;
  private NetworkTableEntry m_botpose;
  private NetworkTableEntry m_targetpose_robotspace;

  NetworkTable currentData;

  public double area;
  public int fidId;
  public String camName;

  public Limelight() {
    Limelight.camera = this;
    m_visionTable = NetworkTableInstance.getDefault();
    m_tv = m_visionTable.getEntry("tv");
    m_ta = m_visionTable.getEntry("ta");
    m_tl = m_visionTable.getEntry("tl");
    m_cl = m_visionTable.getEntry("cl");
    m_tcornxy = m_visionTable.getEntry("m_tcornxy");
    m_botpose = m_visionTable.getEntry("botpose");
    m_targetpose_robotspace = m_visionTable.getEntry("targetpose_robotspace");

    camName = "limelight";
  }

  public double getA() {
    return m_ta.getDouble(0.0);
  }

  public double getV() {
    return m_tv.getDouble(0.0);
  }

  public double[] getBotPose() {
    return m_botpose.getDoubleArray(new double[6]);
  }

  public Pose2d getLimelightPose() {
    double[] botpose = getBotPose();

    return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
  }

  public Pose2d getTargetPose() {
    double[] targetpose = m_targetpose_robotspace.getDoubleArray(new double[6]);

    return new Pose2d(targetpose[0], targetpose[1], Rotation2d.fromDegrees(targetpose[5]));
  }

  @Override
  public void periodic() {
    currentData = m_visionTable.getTable(camName);
    m_tv = currentData.getEntry("tv");
    m_ta = currentData.getEntry("ta");
    m_tl = currentData.getEntry("tl");
    m_cl = currentData.getEntry("cl");
    m_tcornxy = currentData.getEntry("m_tcornxy");
    m_botpose = currentData.getEntry("botpose");
    m_targetpose_robotspace = currentData.getEntry("targetpose_robotspace");
  }

  public void realPeriodic() {
    Logger.recordOutput("Limelight/Limelight Pose", GeometryUtils.AKitOdometry(getLimelightPose()));
    Logger.recordOutput("Limelight/Valid Targets", m_tv.getInteger(0));
    Logger.recordOutput("Limelight/Target Area", m_ta.getDouble(0));
    Logger.recordOutput("Limelight/Pipeline Latency", m_tl.getDouble(0));
    Logger.recordOutput("Limelight/Capture Latency", m_cl.getDouble(0));
    Logger.recordOutput(
        "Limelight/Corners/X",
        GeometryUtils.AKitCorner(m_tcornxy.getDoubleArray(new double[2]))[0]);
    Logger.recordOutput(
        "Limelight/Corners/Y",
        GeometryUtils.AKitCorner(m_tcornxy.getDoubleArray(new double[2]))[1]);
    Logger.recordOutput(
        "Limelight/AprilTag Pose",
        GeometryUtils.AKitOdometry(
            GeometryUtils.PoseSpaceToFieldSpace(
                getTargetPose(), RobotContainer.m_drivebase.getPose())));
  }
}
