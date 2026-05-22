// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.lobstahbots.units.*
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.indexer.IndexerIO
import frc.robot.subsystems.indexer.IndexerIOSparkMax
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIO
import frc.robot.subsystems.intake.IntakeIOSimBasic
import frc.robot.subsystems.intake.IntakeIOSparkMax
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterIO
import frc.robot.subsystems.shooter.ShooterIOSparkMax
import frc.robot.subsystems.vision.Camera
import frc.robot.subsystems.vision.CameraIOPhoton
import frc.robot.subsystems.vision.CameraIOSim
import frc.robot.subsystems.vision.CameraIOSim.Companion.addSimPose
import frc.robot.util.auto.AutonSelector
import frc.robot.util.auto.AutonSelector.AutoQuestion
import frc.robot.util.led.LEDs
import frc.robot.util.math.ShotData
import frc.robot.util.trajectory.mirrorPose2d
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.robot
import frc.robot.Constants.mode
import frc.robot.Constants.VisionConstants
import frc.robot.Constants.RobotType
import frc.robot.Constants.RobotMode
import frc.robot.Constants.IntakeConstants
import frc.robot.Constants.IndexerConstants
import frc.robot.Constants.ShooterConstants
import frc.robot.Constants.IOConstants.ControllerIOConstants
import frc.robot.Constants.SimConstants
import frc.robot.Constants.RobotConstants

class RobotContainer {
    private val leds: LEDs = LEDs()

    private val driveBase: DriveBase
    private val intake: Intake
    private val indexer: Indexer
    private val shooter: Shooter

    private val autoChooser = AutonSelector<Any>(
        "Auto Chooser", "Do Nothing", listOf(), Commands::none
    )
    private val autoFactory: AutoFactory

    private var driveSimulation: SwerveDriveSimulation? = null

    //private int scoreLevel = 1;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        if (robot == RobotType.WAFFLE && mode == RobotMode.REAL) {
            val frontLeft = SwerveModuleIOSparkMax(
                DriveConstants.FrontLeftModuleConstants.moduleID,
                "Front left ",
                DriveConstants.FrontLeftModuleConstants.angleID,
                DriveConstants.FrontLeftModuleConstants.driveID,
                DriveConstants.FrontLeftModuleConstants.angleOffset,
                DriveConstants.FrontLeftModuleConstants.inverted
            )
            val frontRight = SwerveModuleIOSparkMax(
                DriveConstants.FrontRightModuleConstants.moduleID,
                "Front right",
                DriveConstants.FrontRightModuleConstants.angleID,
                DriveConstants.FrontRightModuleConstants.driveID,
                DriveConstants.FrontRightModuleConstants.angleOffset,
                DriveConstants.FrontRightModuleConstants.inverted
            )
            val backLeft = SwerveModuleIOSparkMax(
                DriveConstants.BackLeftModuleConstants.moduleID,
                "Back left",
                DriveConstants.BackLeftModuleConstants.angleID,
                DriveConstants.BackLeftModuleConstants.driveID,
                DriveConstants.BackLeftModuleConstants.angleOffset,
                DriveConstants.BackLeftModuleConstants.inverted
            )
            val backRight = SwerveModuleIOSparkMax(
                DriveConstants.BackRightModuleConstants.moduleID,
                "Back right",
                DriveConstants.BackRightModuleConstants.angleID,
                DriveConstants.BackRightModuleConstants.driveID,
                DriveConstants.BackRightModuleConstants.angleOffset,
                DriveConstants.BackRightModuleConstants.inverted
            )

            val cameras =
                VisionConstants.WAFFLE_CAMERA_TRANSFORMS.keys.map { name: String? -> Camera(CameraIOPhoton(name!!)) }
            driveBase = DriveBase(GyroIONavX(), cameras, frontLeft, frontRight, backLeft, backRight, false)
            intake = Intake(object : IntakeIO {})
            indexer = Indexer(object : IndexerIO {})
            shooter = Shooter(object : ShooterIO {})
        } else if (robot == RobotType.COMP && mode == RobotMode.REAL) {
            val frontLeft = SwerveModuleIOTalonFX(
                DriveConstants.FrontLeftModuleConstants.moduleID,
                "Front left ",
                DriveConstants.FrontLeftModuleConstants.angleID,
                DriveConstants.FrontLeftModuleConstants.driveID,
                DriveConstants.FrontLeftModuleConstants.angleOffset,
                DriveConstants.FrontLeftModuleConstants.inverted
            )
            val frontRight = SwerveModuleIOTalonFX(
                DriveConstants.FrontRightModuleConstants.moduleID,
                "Front right",
                DriveConstants.FrontRightModuleConstants.angleID,
                DriveConstants.FrontRightModuleConstants.driveID,
                DriveConstants.FrontRightModuleConstants.angleOffset,
                DriveConstants.FrontRightModuleConstants.inverted
            )
            val backLeft = SwerveModuleIOTalonFX(
                DriveConstants.BackLeftModuleConstants.moduleID,
                "Back left",
                DriveConstants.BackLeftModuleConstants.angleID,
                DriveConstants.BackLeftModuleConstants.driveID,
                DriveConstants.BackLeftModuleConstants.angleOffset,
                DriveConstants.BackLeftModuleConstants.inverted
            )
            val backRight = SwerveModuleIOTalonFX(
                DriveConstants.BackRightModuleConstants.moduleID,
                "Back right",
                DriveConstants.BackRightModuleConstants.angleID,
                DriveConstants.BackRightModuleConstants.driveID,
                DriveConstants.BackRightModuleConstants.angleOffset,
                DriveConstants.BackRightModuleConstants.inverted
            )

            val cameras =
                VisionConstants.COMP_CAMERA_TRANSFORMS.keys.map { name: String? -> Camera(CameraIOPhoton(name!!)) }
            driveBase = DriveBase(
                GyroIOCanandgyro(RobotConstants.GYRO_ID),
                cameras,
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                false
            )
            intake = Intake(IntakeIOSparkMax(IntakeConstants.ARM_ID, IntakeConstants.ROLLER_ID))
            indexer = Indexer(
                IndexerIOSparkMax(IndexerConstants.INDEXER_MOTOR_ID, IndexerConstants.FEEDER_MOTOR_ID)
            )
            shooter = Shooter(
                ShooterIOSparkMax(
                    ShooterConstants.FLYWHEEL1_ID,
                    ShooterConstants.FLYWHEEL2_ID,
                    ShooterConstants.FLYWHEEL3_ID,
                    ShooterConstants.HOOD_ID
                )
            )
            println("real")
        } else if (mode != RobotMode.REPLAY) {
            driveSimulation = SwerveDriveSimulation(
                DriveConstants.MAPLE_SIM_CONFIG, Pose2d(
                    FieldConstants.RightTrench.openingTopLeft.plus(FieldConstants.RightTrench.openingTopRight).div(2.0)
                        .toTranslation2d(),
                    if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance()
                            .get() == Alliance.Red
                    ) Rotation2d.kZero
                    else Rotation2d.k180deg
                )
            )
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

            val modules = driveSimulation!!.modules
            val frontLeft = SwerveModuleIOSim(DriveConstants.FrontLeftModuleConstants.angleOffset, modules[0], 0)
            val frontRight = SwerveModuleIOSim(DriveConstants.FrontRightModuleConstants.angleOffset, modules[1], 1)
            val backLeft = SwerveModuleIOSim(DriveConstants.BackLeftModuleConstants.angleOffset, modules[2], 2)
            val backRight = SwerveModuleIOSim(DriveConstants.BackRightModuleConstants.angleOffset, modules[3], 3)
            val cameras = if (SimConstants.VISION_SIM) {
                VisionConstants.COMP_CAMERA_TRANSFORMS.keys.map { Camera(CameraIOSim(it)) }
            } else {
                ArrayList()
            }
            driveBase = DriveBase(
                GyroIOSim(driveSimulation!!.getGyroSimulation()),
                cameras,
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                false
            )

            intake = if (robot == RobotType.SIM_BASIC) {
                Intake(IntakeIOSimBasic())
            } else {
                Intake(IntakeIOSimBasic())
            }
            indexer = Indexer(object : IndexerIO {})
            shooter = Shooter(object : ShooterIO {})
        } else {
            driveBase = DriveBase(
                object : GyroIO {},
                mutableListOf(),
                object : SwerveModuleIO {},
                object : SwerveModuleIO {},
                object : SwerveModuleIO {},
                object : SwerveModuleIO {},
                false
            )
            intake = Intake(object : IntakeIO {})
            indexer = Indexer(object : IndexerIO {})
            shooter = Shooter(object : ShooterIO {})
        }

        this.autoFactory = AutoFactory(
            driveBase, intake, shooter, indexer, { autoChooser.responses }
        ) { newPose ->
            if (Robot.isSimulation) driveSimulation!!.setSimulationWorldPose(newPose)
            driveBase.pose = newPose
        }

        setDefaultCommands()
        smartDashSetup()
        configureButtonBindings()
    }

    private fun setDefaultCommands() {
        driveBase.defaultCommand = driveBase.joystickDrive(
            { -Controllers.driver.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL) },
            { -Controllers.driver.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL) },
            { -Controllers.driver.getRawAxis(ControllerIOConstants.RIGHT_STICK_HORIZONTAL) })
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         * 
         * @return the command to run in autonomous
         */
        get() = if (Robot.isSimulation) Commands.runOnce({
            SimulatedArena.getInstance().resetFieldForAuto()
        }).andThen(autoChooser.command) else autoChooser.command

    fun configureButtonBindings() {
        Controllers.driver.LBButton.whileTrue(intake.spin())

        //Shoot
        Controllers.driver.RTButton.onTrue(shooter.operate {
            if (mirrorPose2d(driveBase.pose).x < FieldConstants.LinesVertical.neutralZoneNear) ShotData.getShotData(
                driveBase.pose
            )
            else ShotData.getPassData(driveBase.pose)
        }).whileTrue(
            driveBase.joystickDrive(
                { -Controllers.driver.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL) },
                { -Controllers.driver.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL) },
                { -Controllers.driver.getRawAxis(ControllerIOConstants.RIGHT_STICK_HORIZONTAL) },
                true
            )
        )
        Controllers.driver.LTButton.onTrue(shooter.operate {
            ShotData(
                4000.rpm, Rotation2d.fromDegrees(48.0), 1.second
            )
        })
        Controllers.driver.RTButton.or { DriverStation.isAutonomousEnabled() }.or(Controllers.driver.RBButton)
            .or(Controllers.driver.LTButton).and(shooter.atSpeed).whileTrue(indexer.spindex())
        Controllers.driver.RTButton.onFalse(indexer.feed().withTimeout(1.0).andThen(shooter.idle()))
        Controllers.driver.RBButton.onFalse(indexer.feed().withTimeout(1.0).andThen(shooter.idle()))
        Controllers.driver.LTButton.onFalse(indexer.feed().withTimeout(1.0).andThen(shooter.idle()))

        //Operator Test
        Controllers.operator.YButton.onTrue(intake.deploy())
        Controllers.operator.AButton.onTrue(intake.stow())
        Controllers.operator.leftPaddle.whileTrue(intake.spin())
        Controllers.operator.RBButton.whileTrue(indexer.spindex())
    }

    val operatorConnected: Boolean
        get() = Controllers.operator.isConnected

    val driverConnected: Boolean
        get() = Controllers.driver.isConnected

    fun smartDashSetup() {
        autoChooser.addRoutine("Leave", listOf()) { autoFactory.leaveAuto }

        autoChooser.addRoutine(
            "Characterize", listOf(
                AutoQuestion("Which Subsystem?", mapOf("DriveBase" to driveBase)), AutoQuestion(
                    "Which Routine", mapOf(
                        "Quasistatic Forward" to
                                AutoFactory.CharacterizationRoutine.QUASISTATIC_FORWARD,
                        "Quasistatic Backward" to
                                AutoFactory.CharacterizationRoutine.QUASISTATIC_BACKWARD,
                        "Dynamic Forward" to
                                AutoFactory.CharacterizationRoutine.DYNAMIC_FORWARD,
                        "Dynamic Backward" to
                                AutoFactory.CharacterizationRoutine.DYNAMIC_BACKWARD
                    )
                )
            )
        ) { autoFactory.characterizationRoutine }

        autoChooser.addRoutine(
            "Disrupt", listOf(
                AutoQuestion.makeQuestion(
                    "Which Side", listOf("left", "right")
                )
            ), autoFactory.call { side -> autoFactory.disrupt(side) })
        autoChooser.addRoutine(
            "Double Swipe",
            listOf(AutoQuestion.makeQuestion("Which Side?", listOf("left"))),
            autoFactory.call { side -> autoFactory.swipe(side) })
        autoChooser.addRoutine(
            "Zero shooter", listOf()) { shooter.zero() }
        autoChooser.addRoutine(
            "Shoot", listOf()
        ) { shooter.zero().andThen(shooter.operate { ShotData.getShotData(1.0) }) }
    }

    fun displaySimField() {
        if (mode != RobotMode.SIM) return

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose)
        Logger.recordOutput(
            "FieldSimulation/Fuel", *SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")
        )
        addSimPose(Pose3d(driveSimulation!!.simulatedDriveTrainPose))
        if (robot == RobotType.SIM_BASIC) driveBase.resetPose(driveSimulation!!.simulatedDriveTrainPose)
    }

    fun setIdleMode(isBrakeMode: Boolean) {
        driveBase.idleMode = if (isBrakeMode) IdleMode.kBrake else IdleMode.kCoast
    }

    fun periodic() {}
}
