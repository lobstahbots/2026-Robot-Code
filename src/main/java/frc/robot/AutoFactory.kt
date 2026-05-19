package frc.robot

import com.lobstahbots.units.rpm
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.Constants.PathConstants
import frc.robot.subsystems.drive.DriveBase
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.util.math.ShotData
import frc.robot.util.sysId.CharacterizableSubsystem
import frc.robot.util.trajectory.mirrorPose2d
import org.littletonrobotics.junction.Logger

class AutoFactory(
    private val driveBase: DriveBase,
    private val intake: Intake,
    private val shooter: Shooter,
    private val indexer: Indexer,
    private val responses: () -> List<Any>,
    private val poseReset: (Pose2d) -> Unit
) {
    private val driveController = PPHolonomicDriveController(
        Constants.DriveConstants.TRANSLATION_PID_CONSTANTS, Constants.DriveConstants.ROTATION_PID_CONSTANTS
    )

    init {
        AutoBuilder.configure(
            driveBase::pose,
            { pose -> driveBase.pose = pose },
            driveBase::robotRelativeSpeeds,
            { chassisSpeeds, _ ->
                driveBase.robotRelativeSpeeds = chassisSpeeds
            },
            driveController,
            Constants.DriveConstants.ROBOT_CONFIG,
            { DriverStation.getAlliance().get() == DriverStation.Alliance.Red },
            driveBase
        )
    }

    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or
     * using PathPlanner. Default should be Choreo.
     */
    enum class PathType {
        CHOREO, PATHPLANNER
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    fun getPathFindToPoseCommand(targetPose: Pose2d): Command = AutoBuilder.pathfindToPoseFlipped(
        targetPose, PathConstraints(
            4.0,
            1.5,
            PathConstants.CONSTRAINTS.maxAngularVelocityRadPerSec(),
            PathConstants.CONSTRAINTS.maxAngularAccelerationRadPerSecSq()
        ), 0.0
    ).andThen(driveBase.stop())

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose Supplier for the desired end pose of the generated path.
     * @return The constructed path following command
     */
    fun getPathFindToPoseCommand(targetPose: () -> Pose2d): Command = AutoBuilder.pathfindToPoseFlipped(
        targetPose(), PathConstants.CONSTRAINTS, 0.0
    ).andThen(driveBase.stop())

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     * out the .traj or .path ending).
     * @param pathType A [PathType] determining the format of the inputted
     * trajectory. Files ending in .path should be imported as
     * PATHPLANNER, while files ending in .traj should be imported
     * as CHOREO.
     * @param segment  The segment of the Choreo path to choose, zero-indexed
     * @return The constructed path following command
     */
    fun getPathFindToPathCommand(pathname: String, pathType: PathType, segment: Int): Command {
        val path: PathPlannerPath?
        try {
            path = when (pathType) {
                PathType.CHOREO -> PathPlannerPath.fromChoreoTrajectory(pathname, segment)
                PathType.PATHPLANNER -> PathPlannerPath.fromPathFile(pathname)
            }
            return AutoBuilder.followPath(path)
        } catch (exception: Exception) {
            DriverStation.reportError("Could not load path " + pathname + ". Error: " + exception.message, false)
            return Commands.none()
        }
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed. If it is a Choreo
     * path, get the first split segment.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     * out the .traj or .path ending).
     * @param pathType A [PathType] determining the format of the inputted
     * trajectory. Files ending in .path should be imported as
     * PATHPLANNER, while files ending in .traj should be imported
     * as CHOREO.
     * @return The constructed path following command
     */
    fun getPathFindToPathCommand(pathname: String, pathType: PathType): Command {
        val path: PathPlannerPath?
        try {
            path = when (pathType) {
                PathType.CHOREO -> PathPlannerPath.fromChoreoTrajectory(pathname)
                PathType.PATHPLANNER -> PathPlannerPath.fromPathFile(pathname)
            }
            return AutoBuilder.followPath(path)
        } catch (exception: Exception) {
            DriverStation.reportError("Could not load path $pathname. Error: ${exception.message}", false)
            return Commands.none()
        }
    }

    /**
     * Get a choreo trajectory.
     * 
     * @param pathname path name
     * @return path
     */
    fun getChoreoPath(pathname: String): PathPlannerPath {
        try {
            return PathPlannerPath.fromChoreoTrajectory(pathname)
        } catch (exception: Exception) {
            DriverStation.reportError("Could not load path $pathname. Error: ${exception.message}", false)
            return PathPlannerPath(null, null, null, null)
        }
    }

    /**
     * Get a choreo trajectory.
     * 
     * @param pathname   path name
     * @param splitIndex split index
     * @return path
     */
    fun getChoreoPath(pathname: String, splitIndex: Int): PathPlannerPath {
        try {
            return PathPlannerPath.fromChoreoTrajectory(pathname, splitIndex)
        } catch (exception: Exception) {
            DriverStation.reportError("Could not load path $pathname. Error: ${exception.message}", false)
            return PathPlannerPath(null, null, null, null)
        }
    }

    /**
     * Constructs a path following command through a provided set of waypoints. Ends
     * with desired holonomic rotation.
     * 
     * @param goalEndRotationHolonomic Desired holonomic end rotation
     * @param poses                    List of bezier poses. Each [Pose2d]
     * represents one waypoint. The rotation
     * component of the pose should be the direction
     * of travel. Do not use holonomic rotation.
     * @return The constructed path following command through provided poses, with
     * set end rotation.
     */
    fun getPathFromWaypoints(goalEndRotationHolonomic: Rotation2d, vararg poses: Pose2d): () -> Command = {
        AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(*poses), PathConstants.CONSTRAINTS, null, GoalEndState(
                    0.0, goalEndRotationHolonomic
                )
            ), PathConstants.CONSTRAINTS
        )
    }

    val leaveAuto: Command
        get() = driveBase.relativeDrive(0.2, 0.0, 0.0).withTimeout(3.0)

    fun call(func: (s: String) -> Command): () -> Command = { func(responses()[0] as String) }

    val leftTrench: Command
        get() {
            val go = getChoreoPath("leftTrench")
            return shooter.zero().andThen(
                AutoBuilder.resetOdom(go.getStartingDifferentialPose())
                    .andThen(AutoBuilder.followPath(go).deadlineFor(intake.spin())).andThen(indexer.spindex())
                    .alongWith(
                        shooter.operate({ Rotation2d.fromRotations(0.09) }, { 1000.rpm })
                    )
            )
        }

    fun shoot(): Command = shooter.operate { ShotData.getShotData(driveBase.pose) }
        .alongWith(driveBase.joystickDrive({ 0.0 }, { 0.0 }, { 0.0 }, true)).until { indexer.indexerSpeed >= 6000 }

    fun disrupt(side: String): Command {
        var side = side
        if ("left" != side && "right" != side) {
            side = if (mirrorPose2d(driveBase.pose).y > FieldConstants.fieldWidth / 2) "left"
            else "right"
        }
        val path = getChoreoPath("disrupt_$side")
        return AutoBuilder.followPath(path).alongWith(shooter.zero())
            .deadlineFor(intake.deploy().andThen(intake.spin())).andThen(driveBase.stopOnce()).andThen(shoot())
    }

    fun swipe(side: String): Command {
        var side = side
        if ("left" != side && "right" != side) {
            side = if (mirrorPose2d(driveBase.pose).y > FieldConstants.fieldWidth / 2) "left"
            else "right"
        }
        val swipe1 = getChoreoPath("swipe_$side", 0)
        val swipe2 = getChoreoPath("swipe_$side", 1)
        return AutoBuilder.followPath(swipe1).alongWith(shooter.zero())
            .deadlineFor(intake.deploy().andThen(intake.spin())).andThen(driveBase.stopOnce()).andThen(shoot()).andThen(
                AutoBuilder.followPath(swipe2).alongWith(shooter.zero())
                    .deadlineFor(intake.deploy().andThen(intake.spin())).andThen(driveBase.stopOnce()).andThen(shoot())
            )
    }

    enum class CharacterizationRoutine {
        QUASISTATIC_FORWARD, QUASISTATIC_BACKWARD, DYNAMIC_FORWARD, DYNAMIC_BACKWARD,
    }

    val characterizationRoutine: Command
        get() {
            val subsystem = responses()[0] as CharacterizableSubsystem
            val routine = responses()[1] as CharacterizationRoutine

            val sysIdRoutine = SysIdRoutine(
                SysIdRoutine.Config(
                    null, null, null
                )  // Use default config
                { state ->
                    Logger.recordOutput(
                        "SysIdTestState", state.toString()
                    )
                }, Mechanism(
                    { voltage -> subsystem.runVolts(voltage.`in`(Units.Volts)) },
                    null,  // No log consumer, since data is recorded by AdvantageKit
                    subsystem
                )
            )
            return when (routine) {
                CharacterizationRoutine.QUASISTATIC_FORWARD -> sysIdRoutine.quasistatic(
                    SysIdRoutine.Direction.kForward
                )

                CharacterizationRoutine.QUASISTATIC_BACKWARD -> sysIdRoutine.quasistatic(
                    SysIdRoutine.Direction.kReverse
                )

                CharacterizationRoutine.DYNAMIC_FORWARD -> sysIdRoutine.dynamic(
                    SysIdRoutine.Direction.kForward
                )

                CharacterizationRoutine.DYNAMIC_BACKWARD -> sysIdRoutine.dynamic(
                    SysIdRoutine.Direction.kReverse
                )
            }
        }
}
