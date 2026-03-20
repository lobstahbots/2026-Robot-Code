package frc.robot;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.math.ShotData;
import frc.robot.util.sysId.CharacterizableSubsystem;
import frc.robot.util.trajectory.AlliancePoseMirror;

public class AutoFactory {
    private final Supplier<List<Object>> responses;
    private final DriveBase driveBase;
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            DriveConstants.TRANSLATION_PID_CONSTANTS, DriveConstants.ROTATION_PID_CONSTANTS);
    private final Consumer<Pose2d> poseReset;
    private final Intake intake;
    private final Shooter shooter;
    private final Indexer indexer;

    /**
     * Create a new auto factory.
     * 
     * @param driveBase         {@link DriveBase} to drive.
     * @param responsesSupplier Responses to auto chooser questions.
     * @see frc.robot.util.auto.AutonSelector
     */
    public AutoFactory(DriveBase driveBase, Intake intake, Shooter shooter, Indexer indexer,
            Supplier<List<Object>> responsesSupplier, Consumer<Pose2d> poseReset) {
        this.responses = responsesSupplier;
        this.driveBase = driveBase;
        this.poseReset = poseReset;
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;

        AutoBuilder.configure(driveBase::getPose, driveBase::resetPose, driveBase::getRobotRelativeSpeeds,
                (chassisSpeeds, driveFeedforwards) -> driveBase.driveRobotRelative(chassisSpeeds), driveController,
                DriveConstants.ROBOT_CONFIG, () -> {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }, driveBase);
    }

    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or
     * using PathPlanner. Default should be Choreo.
     */
    public enum PathType {
        CHOREO, PATHPLANNER
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Pose2d targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose,
                new PathConstraints(4, 1.5, PathConstants.CONSTRAINTS.maxAngularVelocityRadPerSec(),
                        PathConstants.CONSTRAINTS.maxAngularAccelerationRadPerSecSq()),
                0.0 // Goal end velocity in meters/sec
        ).andThen(driveBase.stop());

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose Supplier for the desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Supplier<Pose2d> targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose.get(), PathConstants.CONSTRAINTS, 0.0 // Goal end velocity in meters/sec
        ).andThen(driveBase.stop());

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     *                 out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted
     *                 trajectory. Files ending in .path should be imported as
     *                 PATHPLANNER, while files ending in .traj should be imported
     *                 as CHOREO.
     * @param segment  The segment of the Choreo path to choose, zero-indexed
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType, int segment) {
        PathPlannerPath path;
        try {
            switch (pathType) {
                case CHOREO:
                    path = PathPlannerPath.fromChoreoTrajectory(pathname, segment);
                    break;
                case PATHPLANNER:
                    path = PathPlannerPath.fromPathFile(pathname);
                    break;
                default:
                    path = PathPlannerPath.fromChoreoTrajectory(pathname, segment);
            }
            return AutoBuilder.followPath(path);
        } catch (Exception exception) {
            DriverStation.reportError("Could not load path " + pathname + ". Error: " + exception.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed. If it is a Choreo
     * path, get the first split segment.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     *                 out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted
     *                 trajectory. Files ending in .path should be imported as
     *                 PATHPLANNER, while files ending in .traj should be imported
     *                 as CHOREO.
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType) {
        PathPlannerPath path;
        try {
            switch (pathType) {
                case CHOREO:
                    path = PathPlannerPath.fromChoreoTrajectory(pathname);
                    break;
                case PATHPLANNER:
                    path = PathPlannerPath.fromPathFile(pathname);
                    break;
                default:
                    path = PathPlannerPath.fromChoreoTrajectory(pathname);
            }
            return AutoBuilder.followPath(path);
        } catch (Exception exception) {
            DriverStation.reportError("Could not load path " + pathname + ". Error: " + exception.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Get a choreo trajectory.
     * 
     * @param pathname path name
     * @return path
     */
    public PathPlannerPath getChoreoPath(String pathname) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(pathname);
        } catch (Exception exception) {
            DriverStation.reportError("Could not load path " + pathname + ". Error: " + exception.getMessage(), false);
            return new PathPlannerPath(null, null, null, null);
        }
    }

    /**
     * Get a choreo trajectory.
     * 
     * @param pathname   path name
     * @param splitIndex split index
     * @return path
     */
    public PathPlannerPath getChoreoPath(String pathname, int splitIndex) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(pathname, splitIndex);
        } catch (Exception exception) {
            DriverStation.reportError("Could not load path " + pathname + ". Error: " + exception.getMessage(), false);
            return new PathPlannerPath(null, null, null, null);
        }
    }

    /**
     * Constructs a path following command through a provided set of waypoints. Ends
     * with desired holonomic rotation.
     * 
     * @param goalEndRotationHolonomic Desired holonomic end rotation
     * @param poses                    List of bezier poses. Each {@link Pose2d}
     *                                 represents one waypoint. The rotation
     *                                 component of the pose should be the direction
     *                                 of travel. Do not use holonomic rotation.
     * @return The constructed path following command through provided poses, with
     *         set end rotation.
     */
    public Supplier<Command> getPathFromWaypoints(Rotation2d goalEndRotationHolonomic, Pose2d... poses) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(waypoints, PathConstants.CONSTRAINTS, null,
                new GoalEndState(0.0, goalEndRotationHolonomic) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        Supplier<Command> pathCommand = () -> AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS);
        return pathCommand;
    }

    public Command getLeaveAuto() {
        return driveBase.relativeDrive(0.2, 0, 0).withTimeout(3);
    }

    public Supplier<Command> call(Function<String, Command> func) {
        return () -> func.apply((String) responses.get().get(0));
    }

    public Command getLeftTrench() {
        PathPlannerPath go = getChoreoPath("leftTrench");
        return shooter.zero()
                .andThen(AutoBuilder.resetOdom(go.getStartingDifferentialPose())
                        .andThen(AutoBuilder.followPath(go).deadlineFor(intake.spin())).andThen(indexer.spindex())
                        .alongWith(shooter.operate(() -> Rotation2d.fromRotations(0.09), () -> RPM.of(1000))));
    }

    public Command shoot() {
        return shooter.operate(() -> ShotData.getShotData(driveBase.getPose()))
                .alongWith(driveBase.joystickDrive(() -> 0, () -> 0, () -> 0, true))
                .until(() -> indexer.getIndexerSpeed() >= 6000);
    }

    public Command disrupt(String side) {
        if (!"left".equals(side) && !"right".equals(side)) {
            side = AlliancePoseMirror.mirrorPose2d(driveBase.getPose()).getY() > FieldConstants.fieldWidth / 2 ? "left"
                    : "right";
        }
        PathPlannerPath path = getChoreoPath("disrupt_" + side);
        return AutoBuilder.followPath(path).alongWith(shooter.zero())
                .deadlineFor(intake.deploy().andThen(intake.spin())).andThen(driveBase.stopOnce()).andThen(shoot());
    }

    public Command swipe(String side) {
        if (!"left".equals(side) && !"right".equals(side)) {
            side = AlliancePoseMirror.mirrorPose2d(driveBase.getPose()).getY() > FieldConstants.fieldWidth / 2 ? "left"
                    : "right";
        }
        PathPlannerPath swipe1 = getChoreoPath("swipe_" + side, 0);
        PathPlannerPath swipe2 = getChoreoPath("swipe_" + side, 1);
        return AutoBuilder.followPath(swipe1).alongWith(shooter.zero())
                .deadlineFor(intake.deploy().andThen(intake.spin())).andThen(driveBase.stopOnce()).andThen(shoot())
                .andThen(AutoBuilder.followPath(swipe2).alongWith(shooter.zero())
                        .deadlineFor(intake.deploy().andThen(intake.spin())).andThen(driveBase.stopOnce())
                        .andThen(shoot()));
    }

    public static enum CharacterizationRoutine {
        QUASISTATIC_FORWARD, QUASISTATIC_BACKWARD, DYNAMIC_FORWARD, DYNAMIC_BACKWARD,
    }

    public Command getCharacterizationRoutine() {
        CharacterizableSubsystem subsystem = (CharacterizableSubsystem) responses.get().get(0);
        CharacterizationRoutine routine = (CharacterizationRoutine) responses.get().get(1);

        var sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((Voltage voltage) -> subsystem.runVolts(voltage.in(Volts)), null, // No log consumer, since data is recorded by AdvantageKit
                        subsystem));
        switch (routine) {
            case QUASISTATIC_FORWARD:
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
            case QUASISTATIC_BACKWARD:
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
            case DYNAMIC_FORWARD:
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
            case DYNAMIC_BACKWARD:
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
            default:
                return new WaitCommand(1);
        }
    }
}
