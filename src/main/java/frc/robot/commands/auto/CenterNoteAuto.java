// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class CenterNoteAuto extends Command {
  /** Creates a new CenterNoteAuto. */
  
  private SwerveDrive m_drivebase;
  private boolean[] m_availableNotes = new boolean[] {false, false, false, true, true};
  private int indexToUse = -1;

  public CenterNoteAuto(SwerveDrive drivebase) {
    m_drivebase = drivebase;


    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < m_availableNotes.length; i++) {
      if (m_availableNotes[i]) {
        indexToUse = i;
        break;
      }
    }
    

    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    //     new Pose2d(AutoConstants.kNotePlacements[indexToUse].getX() - 1, AutoConstants.kNotePlacements[indexToUse].getY(), Rotation2d.fromDegrees(0)),
    //     new Pose2d(AutoConstants.kNotePlacements[indexToUse], Rotation2d.fromDegrees(0))
    // );

    // PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints,
    //     new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    //     new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    // );

    PathPlannerPath path = PathPlannerPath.fromPathFile("Note 5");

    // AutoBuilder.pathfindToPose(path.getPreviewStartingHolonomicPose(), new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI)).andThen(
    //   AutoBuilder.followPath(path)
    // ).schedule();

    AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI)).schedule();

    // AutoBuilder.followPath(path).schedule();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
