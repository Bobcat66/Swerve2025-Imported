package org.pihisamurai.frc2025.robot.commands.drive;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;
import org.pihisamurai.frc2025.robot.utils.Localization;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.ReefFace;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.fasterxml.jackson.core.TreeNode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;


/** An example command that uses an example subsystem. */
public class DirectDriveToNearestBranch extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final Boolean isLeftBranch;
  private Pose2d targetPose;
  private Command pathfindCommand;
  private final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.fromDegrees(0));

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DirectDriveToNearestBranch(DriveSubsystem subsystem, Boolean isLeftBranch) {
    
    m_subsystem = subsystem;
    this.isLeftBranch = isLeftBranch;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = m_subsystem.getPose();
    ReefFace closestFace = Localization.getClosestReefFace(currentPose);
    targetPose = isLeftBranch ? closestFace.leftBranch : closestFace.rightBranch;
    targetPose.rotateBy(Rotation2d.fromDegrees(180));
    targetPose.transformBy(robotOffset);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        currentPose,
        targetPose
    );
    PathPlannerPath path = new PathPlannerPath(waypoints, AutoConstants.pathConstraints, null, new GoalEndState(0, targetPose.getRotation()));
    path.preventFlipping = true;
    pathfindCommand = AutoBuilder.followPath(path);
    pathfindCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindCommand.isFinished();
  }
}
