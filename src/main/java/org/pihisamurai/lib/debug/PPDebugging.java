package org.pihisamurai.lib.debug;

import java.util.List;
import java.util.Optional;

import org.apache.commons.lang3.NotImplementedException;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Contains utility methods for debugging PathPlanner
 */
public final class PPDebugging {
    public static PathPlannerTrajectory getTrajectoryFromFPC(FollowPathCommand fpc) {
        return ReflectionDebugger.getInstance().getMonitor(FollowPathCommand.class).<PathPlannerTrajectory>getMonitor("trajectory",fpc).get();
    }

    public static Command getAutoCommand(PathPlannerAuto command) {
        return ReflectionDebugger.getInstance().getMonitor(PathPlannerAuto.class).<Command>getMonitor("autoCommand",command).get();
    }

    public static Timer getTimerFromFPC(FollowPathCommand fpc){
        return ReflectionDebugger.getInstance().getMonitor(FollowPathCommand.class).<Timer>getMonitor("timer",fpc).get();
    }

    /** Returns the Nth command from a sequential command group. WARNING: The sequential command group should NOT be scheduled after calling this method on it */
    public static Optional<Command> getNthCommandFromSCG(SequentialCommandGroup commandGroup, int n){
        List<Command> commandList = ReflectionDebugger.getInstance().getMonitor(SequentialCommandGroup.class).<List<Command>>getMonitor("m_commands",commandGroup).get();
        try {
            CommandScheduler.getInstance().clearComposedCommands();
            return Optional.of(commandList.get(n));
        } catch (IndexOutOfBoundsException e) {
            return Optional.empty();
        }
    }

    private PPDebugging(){
        throw new NotImplementedException("This is a utility class!");
    }
}
