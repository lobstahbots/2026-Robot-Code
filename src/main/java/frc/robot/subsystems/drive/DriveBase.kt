package frc.robot.subsystems.drive

import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.*
import frc.robot.Constants.IOConstants.ControllerIOConstants
import frc.robot.FieldConstants
import frc.robot.subsystems.vision.Camera
import frc.robot.util.led.LEDs
import frc.robot.util.math.LobstahMath
import frc.robot.util.sysId.CharacterizableSubsystem
import frc.robot.util.trajectory.AlliancePoseMirror
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.withSign

class DriveBase(
    private val gyro: GyroIO,
    private val cameras: List<Camera>,
    frontLeft: SwerveModuleIO,
    frontRight: SwerveModuleIO,
    backLeft: SwerveModuleIO,
    backRight: SwerveModuleIO,
    private var isOpenLoop: Boolean
) : CharacterizableSubsystem() {
    private val modules = listOf(
        SwerveModule(frontLeft, DriveConstants.FrontLeftModuleConstants.moduleID),
        SwerveModule(frontRight, DriveConstants.FrontRightModuleConstants.moduleID),
        SwerveModule(backLeft, DriveConstants.BackLeftModuleConstants.moduleID),
        SwerveModule(backRight, DriveConstants.BackRightModuleConstants.moduleID)
    )

    // private val setpointGenerator = SwerveSetpointGenerator(DriveConstants.KINEMATICS, DriveConstants.MODULE_LOCATIONS)
    // private var swerveSetpoint = SwerveSetpoint(ChassisSpeeds(), Array(4) { SwerveModuleState())
    private val gyroInputs = GyroIOInputsAutoLogged()
    private var hasSeenTag = false
    private var needGyroReset = true
    private val swerveOdometry = SwerveDrivePoseEstimator(
        DriveConstants.KINEMATICS, gyroInputs.yawPosition, positions, Pose2d.kZero
    )
    private val visionlessOdometry = SwerveDrivePoseEstimator(
        DriveConstants.KINEMATICS, gyroInputs.yawPosition, positions, Pose2d.kZero
    )

    private val xController = PIDController(
        DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
        DriveConstants.AUTO_ALIGN_TRANSLATION_kI,
        DriveConstants.AUTO_ALIGN_TRANSLATION_kD
    )
    private val yController = PIDController(
        DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
        DriveConstants.AUTO_ALIGN_TRANSLATION_kI,
        DriveConstants.AUTO_ALIGN_TRANSLATION_kD
    )
    private val thetaController = PIDController(
        DriveConstants.ROTATION_PID_CONSTANTS.kP,
        DriveConstants.ROTATION_PID_CONSTANTS.kI,
        DriveConstants.ROTATION_PID_CONSTANTS.kD
    )
    private val field = Field2d()

    init {
        SmartDashboard.putData("Field", field)
        xController.setTolerance(0.02)
        yController.setTolerance(0.02)
        thetaController.enableContinuousInput(-Math.PI, Math.PI)
        resetPose(pose)
    }

    /**
     * Resets pose of odometry to a given pose.
     *
     * @param pose The desired pose to reset the odometry to.
     */
    fun resetPose(pose: Pose2d) {
        val newPose = Pose2d(pose.translation, gyroInputs.yawPosition)
        swerveOdometry.resetPosition(gyroInputs.yawPosition, positions, newPose)
        visionlessOdometry.resetPosition(gyroInputs.yawPosition, positions, newPose)
    }

    /**
     * The current estimated odometry pose. Setting this property resets odometry using [resetPose].
     *
     * @see SwerveDrivePoseEstimator
     */
    var pose: Pose2d
        get() = swerveOdometry.estimatedPosition
        set(value) = resetPose(value)

    /**
     * The states of the four swerve modules. Setting this sets the desired swerve module states.
     *
     * @see SwerveModuleState
     */
    var states: Array<SwerveModuleState>
        get() = modules.sortedBy { it.moduleID }.map { it.state }.toTypedArray()
        set(desiredStates) {
            val optimizedStates = arrayOfNulls<SwerveModuleState>(4)


            // SwerveDriveKinematics.desaturateWheelSpeeds(
            // desiredStates, DriveConstants.MAX_DRIVE_SPEED);
            for (module in modules) {
                optimizedStates[module.moduleID] = module.setDesiredState(
                    desiredStates[module.moduleID], isOpenLoop
                )
            }
            // swerveSetpoint.moduleStates = optimizedStates
            Logger.recordOutput("SwerveStates/Desired", *desiredStates)
            Logger.recordOutput("SwerveStates/Optimized", *optimizedStates)
        }

    /**
     * The positions of the four swerve modules.
     *
     * @see SwerveModulePosition
     */
    val positions: Array<SwerveModulePosition>
        get() = modules.sortedBy { it.moduleID }.map { it.position }.toTypedArray()

    /**
     * The robot-relative chassis speeds. Setting this drives the robot robot-relative according to the provided speeds.
     */
    var robotRelativeSpeeds: ChassisSpeeds
        get() = DriveConstants.KINEMATICS.toChassisSpeeds(*states)
        set(chassisSpeeds) {
            val desiredStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
            Logger.recordOutput("Unoptimized:", *desiredStates)
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_DRIVE_SPEED)
            states = desiredStates
        }

    /**
     * The field relative chassis speeds of the robot.
     */
    val fieldRelativeSpeeds: ChassisSpeeds
        get() = pose.rotation.let { angle ->
            ChassisSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond * angle.cos - robotRelativeSpeeds.vyMetersPerSecond * angle.sin,
                robotRelativeSpeeds.vyMetersPerSecond * angle.cos + robotRelativeSpeeds.vxMetersPerSecond * angle.sin,
                robotRelativeSpeeds.omegaRadiansPerSecond
            )
        }

    var idleMode: SparkBaseConfig.IdleMode = SparkBaseConfig.IdleMode.kBrake
        set(value) {
            field = value
            modules.forEach { it.setIdleMode(field) }
        }

    /**
     * Stop all the modules' motors.
     */
    private fun stopMotors() = modules.forEach { it.stop() }

    val gyroAngle: Rotation2d
        get() = gyroInputs.yawPosition

    /**
     * Runs motors during characterization voltage ramp routines.
     *
     * @param volts Voltage to run motors at.
     * @see CharacterizableSubsystem
     */
    override fun runVolts(volts: Double) = modules.forEach { it.runVolts(volts) }

    override fun periodic() {
        if (needGyroReset and !gyroInputs.isCalibrating) {
            gyro.zeroGyro()
            needGyroReset = false
        }

        swerveOdometry.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, positions)
        visionlessOdometry.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, positions)
        Logger.recordOutput("Has seen tag", hasSeenTag)
        cameras.forEach { camera ->
            camera.periodic()
            val estimatedPose = camera.getEstimatedPose(pose)
            if (estimatedPose.pose.isPresent and (!hasSeenTag or (LobstahMath.getDistBetweenPoses(
                    estimatedPose.pose.get(), pose
                ) <= 8)) and (abs(
                    estimatedPose.pose.get().z
                ) < 0.3)
            ) {
                if (!hasSeenTag) {
                    pose = estimatedPose.pose.get().toPose2d()
                    hasSeenTag = true
                }
                swerveOdometry.addVisionMeasurement(
                    estimatedPose.pose.get().toPose2d(), estimatedPose.timestamp.get(), estimatedPose.stdev().get()
                )
                Logger.recordOutput("Vision/" + camera.name + "Used", true)
            } else Logger.recordOutput("Vision/" + camera.name + "Used", false)
        }
        pose = Pose2d(
            MathUtil.clamp(pose.x, 0.0, FieldConstants.fieldLength),
            MathUtil.clamp(pose.y, 0.0, FieldConstants.fieldWidth),
            pose.rotation
        )
        field.robotPose = pose
        Logger.recordOutput("Odometry", pose)
        Logger.recordOutput("Visionless", visionlessOdometry.estimatedPosition)
        gyro.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)

        val robotPose3d = Pose3d(pose).exp(
            Twist3d(
                0.0,
                0.0,
                abs(gyroInputs.pitchPosition.radians) * RobotConstants.TRACK_WIDTH / 2.0,
                0.0,
                gyroInputs.pitchPosition.radians,
                0.0
            )
        ).exp(
                Twist3d(
                    0.0,
                    0.0,
                    abs(gyroInputs.rollPosition.radians) * RobotConstants.TRACK_WIDTH / 2.0,
                    gyroInputs.rollPosition.radians,
                    0.0,
                    0.0
                )
            )
        Logger.recordOutput("Odometry/Robot3d", robotPose3d)
        cameras.forEach { camera ->
            Logger.recordOutput("Vision/" + camera.name + "/Pose3d", robotPose3d.plus(camera.robotToCamera))
        }
        modules.forEach { it.periodic() }
        if (DriverStation.isDisabled()) stopMotors()

        Logger.recordOutput("SwerveStates/Measured", *states)
        SmartDashboard.putData("Drivebase subsystem", this)
    }

    /**
     * Constructs a command which holds this in place.
     *
     * @return the constructed command
     */
    fun stop(): Command = run(this::stopMotors)

    /**
     * Constructs a command which stops this and immediately returns.
     *
     * @return the constructed command
     */
    fun stopOnce(): Command = runOnce(this::stopMotors)

    /**
     * Constructs a command to auto-align to a particular pose. It will also light
     * the aligning LEDs at the beginning and turn them off at the end.
     *
     * @param targetPoseSupplier a supplier which supplies the pose; the target pose
     * will be obtained from this supplier every time the
     * command is scheduled.
     * @return the constructed command
     */
    fun alignToPose(targetPoseSupplier: Supplier<Pose2d>): Command = startRun({
        xController.reset()
        yController.reset()
        thetaController.reset()
        val targetPose = targetPoseSupplier.get()
        Logger.recordOutput("AutoAlignTargetPose", targetPose)
        xController.setSetpoint(targetPose.x)
        yController.setSetpoint(targetPose.y)
        thetaController.setSetpoint(targetPose.rotation.radians)
    }, {
        robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(pose.x),
            yController.calculate(pose.y),
            thetaController.calculate(pose.rotation.radians),
            pose.rotation
        )

    }).until { xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint() }.andThen(stop())
        .beforeStarting({ LEDs.getInstance().setAligning(true) })
        .finallyDo { _ -> LEDs.getInstance().setAligning(false) }


    /**
     * Create a new command to drive field-relative.
     *
     * @param strafeXSupplier  Supplier for strafe in X direction, e.g. from a
     * joystick.
     * @param strafeYSupplier  Supplier for strafe in Y direction, e.g. from a
     * joystick.
     * @param rotationSupplier Supplier for rotation, e.g. from a joystick.
     * @param assist           Whether to apply drive assist.
     * @return constructed command
     */
    @JvmOverloads
    fun joystickDrive(
        strafeXSupplier: DoubleSupplier,
        strafeYSupplier: DoubleSupplier,
        rotationSupplier: DoubleSupplier,
        assist: Boolean = false
    ): Command = run {
        var linearMagnitude = MathUtil.applyDeadband(
            hypot(strafeXSupplier.asDouble, strafeYSupplier.asDouble), IOConstants.JOYSTICK_DEADBAND
        )
        var linearDirection = if (linearMagnitude > 0) Rotation2d(strafeXSupplier.asDouble, strafeYSupplier.asDouble)
        else Rotation2d.kZero
        if (AlliancePoseMirror.isRedAlliance()) linearDirection = linearDirection.plus(Rotation2d.k180deg)
        var omega = MathUtil.applyDeadband(rotationSupplier.asDouble, IOConstants.JOYSTICK_DEADBAND)

        // Square values
        if (ControllerIOConstants.SQUARE_INPUTS) {
            linearMagnitude *= linearMagnitude
            omega = (omega * omega).withSign(omega)
        }

        // Calculate new linear velocity
        val linearVelocity = Pose2d(Translation2d(), linearDirection).transformBy(
                Transform2d(
                    linearMagnitude,
                    0.0,
                    Rotation2d()
                )
            ).translation

        if (assist && AlliancePoseMirror.mirrorPose2d(pose).x < FieldConstants.LinesVertical.neutralZoneNear) {
            val targetPose =
                AlliancePoseMirror.mirrorTranslation2d(FieldConstants.Hub.innerCenterPoint.toTranslation2d())
            val targetAngle = targetPose.minus(pose.translation).angle
            omega = thetaController.calculate(
                pose.rotation.plus(Rotation2d.kCCW_Pi_2).radians, targetAngle.radians
            )
        }

        val chassisSpeeds = ChassisSpeeds(
            linearVelocity.x * DriveConstants.MAX_DRIVE_SPEED,
            linearVelocity.y * DriveConstants.MAX_DRIVE_SPEED,
            omega * DriveConstants.MAX_ANGULAR_SPEED
        )
        robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, pose.rotation)
    }.finallyDo(this::stopMotors)

    /**
     * Drive the robot robot-relative.
     *
     * @param strafeX  constant strafe in X (m/s)
     * @param strafeY  constant strafe in Y (m/s)
     * @param rotation constant rotation (rad/s)
     * @return constructed command
     */
    fun relativeDrive(strafeX: Double, strafeY: Double, rotation: Double): Command {
        val speeds = ChassisSpeeds(strafeX, strafeX, rotation)
        return run { robotRelativeSpeeds = speeds }.finallyDo(this::stopMotors)
    }
}