// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class APTag extends SubsystemBase {

  public static APTag camera;
  
  private NetworkTableInstance m_visionTable;
  private NetworkTableEntry m_visV;
  private NetworkTableEntry m_visX;
  private NetworkTableEntry m_visY;
  private NetworkTableEntry m_visA;
  private NetworkTableEntry m_poseArr;

  NetworkTable currentData;

  public double area;
  public int fidId;
  public String camName;

  public APTag() {
    APTag.camera = this;
    m_visionTable = NetworkTableInstance.getDefault();
    m_visV = m_visionTable.getEntry("tv");
    m_visX = m_visionTable.getEntry("tx");
    m_visY = m_visionTable.getEntry("ty");
    m_visA = m_visionTable.getEntry("ta");
    
    m_poseArr = m_visionTable.getEntry("botpose");
    camName = "limelight";
  }

  
  public double getX() {
    return m_visX.getDouble(0.0);
  }

  public double getY() {
    return m_visY.getDouble(0.0);
  }

  public double getA() {
    return m_visA.getDouble(0.0);
  }

  public double getV() {
    return m_visV.getDouble(0.0);
  }

  public double[] getBotPose(){
    return m_poseArr.getDoubleArray(new double[6]);
  }

  public double getYaw(){
    return getBotPose()[5];
  }

  public Pose2d getPose2d(){
    double[] poseArr = getBotPose();

    return new Pose2d(poseArr[0], poseArr[1], Rotation2d.fromDegrees(poseArr[5]));
  }

  @Override
  public void periodic() {
    currentData = m_visionTable.getTable(camName);
    m_visV = currentData.getEntry("tv");
    m_visX = currentData.getEntry("tx");
    m_visY = currentData.getEntry("ty");
    m_visA = currentData.getEntry("ta");
    m_poseArr = currentData.getEntry("botpose");

    SmartDashboard.putNumberArray("Limelight Pose", getBotPose());

    // Uncomment this if you want the robots odometry to be changed by the limelight
    // RobotContainer.m_swerveDrive.resetOdometry(getPose2d());
  }
}