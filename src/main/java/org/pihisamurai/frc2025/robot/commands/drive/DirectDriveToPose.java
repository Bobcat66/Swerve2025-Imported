package org.pihisamurai.frc2025.robot.commands.drive;

import java.util.List;

import org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DirectDriveToPose extends Command {

    private final Command pathfindCommand;
    
    public DirectDriveToPose(DriveSubsystem drive,Pose2d targetPose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            drive.getPose(),
            targetPose
        );
        PathPlannerPath path = new PathPlannerPath(waypoints, AutoConstants.pathConstraints, null, new GoalEndState(0, targetPose.getRotation()));
        path.preventFlipping = true;
        pathfindCommand = AutoBuilder.followPath(path);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        pathfindCommand.initialize();
    }

    @Override
    public void execute(){
        pathfindCommand.execute();
    }
    
    @Override
    public boolean isFinished() {
        return pathfindCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        pathfindCommand.end(interrupted);
    }

}
