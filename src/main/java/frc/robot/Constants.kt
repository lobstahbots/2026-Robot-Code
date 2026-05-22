// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("PropertyName", "ConstPropertyName")

package frc.robot

import com.lobstahbots.units.*
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RuntimeType
import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.drive.SwerveKinematicLimits
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import kotlin.math.ln
import kotlin.math.sqrt

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * 
 * 
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
object Constants {
    private val robotType = RobotType.COMP

    // DO NOT EDIT
    val robot: RobotType = if (RuntimeType.getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation) robotType
    else if (robotType == RobotType.SIM_BASIC || robotType == RobotType.SIM_ADVANCED) RobotType.COMP
    else robotType

    val mode: RobotMode = if (robot == RobotType.SIM_BASIC || robot == RobotType.SIM_ADVANCED) RobotMode.SIM
    else if (RuntimeType.getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation) RobotMode.REPLAY
    else RobotMode.REAL

    enum class RobotType {
        WAFFLE, COMP, SIM_BASIC, SIM_ADVANCED
    }

    enum class RobotMode {
        REAL, REPLAY, SIM
    }

    object PathConstants {
        val CONSTRAINTS: PathConstraints = PathConstraints(
            4.metersPerSecond, 4.metersPerSecondPerSecond, 540.radiansPerSecond, 720.radiansPerSecondPerSecond
        )
    }

    object IOConstants {
        val JOYSTICK_DEADBAND = if (Robot.isReal) 0.05 else 0.15

        object ControllerIOConstants {
            const val DRIVER_CONTROLLER_PORT: Int = 0
            const val OPERATOR_CONTROLLER_PORT: Int = 1

            const val LEFT_STICK_HORIZONTAL: Int = 0
            const val LEFT_STICK_VERTICAL: Int = 1
            const val RIGHT_STICK_HORIZONTAL: Int = 4
            const val RIGHT_STICK_VERTICAL: Int = 5

            const val A_BUTTON: Int = 1
            const val B_BUTTON: Int = 2
            const val X_BUTTON: Int = 3
            const val Y_BUTTON: Int = 4

            const val LT_BUTTON: Int = 2
            const val RT_BUTTON: Int = 3

            const val LB_BUTTON: Int = 5
            const val RB_BUTTON: Int = 6

            const val RIGHT_PADDLE: Int = 8
            const val LEFT_PADDLE: Int = 7

            const val D_PAD_UP: Int = 0
            const val D_PAD_DOWN: Int = 180
            const val D_PAD_LEFT: Int = 270
            const val D_PAD_RIGHT: Int = 90

            const val SQUARE_INPUTS: Boolean = true
        }
    }

    object RobotConstants {
        val WHEELBASE = 24.inches
        val TRACK_WIDTH = 30.inches
        val EDGE_TO_MODULE_CENTER = 1.75.inches

        // Distance from robot center to module center
        val RADIUS =
            sqrt(((WHEELBASE / 2.0 - EDGE_TO_MODULE_CENTER).let { it * it } + (TRACK_WIDTH / 2.0 - EDGE_TO_MODULE_CENTER).let { it * it }).baseUnitMagnitude()).meters
        val WHEEL_DIAMETER = 3.inches
        const val DRIVE_GEAR_RATIO = 5.08
        const val ANGLE_GEAR_RATIO = 9424.0 / 203.0
        val MAX_DRIVE_SPEED = 5.23.metersPerSecond // from https://www.reca.lc/drive
        val WEIGHT = 150.pounds
        val MOI = 6.kilogramSquareMeters

        const val GYRO_ID: Int = 0
    }

    object DriveConstants {
        const val MAX_ACCELERATION = 45.0
        const val MAX_DRIVE_SPEED = 100.0
        const val MAX_ANGULAR_SPEED = 50.0
        const val SLOWDOWN_PERCENT = 0.5
        val DRIVE_MOTOR_CURRENT_LIMIT = 40.amps
        val ANGLE_MOTOR_CURRENT_LIMIT = 20.amps
        val MODULE_LOCATIONS = arrayOf(
            Translation2d(
                RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
                RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER
            ),
            Translation2d(
                RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
                -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER
            ),
            Translation2d(
                -RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
                RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER
            ),
            Translation2d(
                -RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
                -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER
            ),
        )
        val KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(*MODULE_LOCATIONS)
        val MODULE_LIMITS: SwerveKinematicLimits = SwerveKinematicLimits(
            MAX_DRIVE_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED
        )

        var FIELD_CENTRIC: Boolean = true
        const val IS_OPEN_LOOP: Boolean = false

        const val PATH_MAX_ACCEL = 3.0
        const val PATH_MAX_VELOCITY = 3.0

        val TURN_DEADBAND = Units.degreesToRadians(5.0)

        const val WHEEL_COF = 1.5

        val ROBOT_CONFIG = RobotConfig(
            RobotConstants.WEIGHT,  // Robot mass
            RobotConstants.MOI,  // Robot moment of inertia
            ModuleConfig(
                RobotConstants.WHEEL_DIAMETER / 2.0,  // wheel diameter
                RobotConstants.MAX_DRIVE_SPEED,  // max drive velocity (m/s)
                WHEEL_COF,  // cof between wheels and ground
                DCMotor.getNEO(1)
                    .withReduction(RobotConstants.DRIVE_GEAR_RATIO),  // DCMotor representing motor, including reduction
                DRIVE_MOTOR_CURRENT_LIMIT,  // current limit for drive motors
                1 // number of drive motors per module
            ), *MODULE_LOCATIONS
        )
        val ROTATION_PID_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)
        val TRANSLATION_PID_CONSTANTS = PIDConstants(7.0, 0.01, 0.15)

        const val AUTO_ALIGN_TRANSLATION_kP = 5.0
        const val AUTO_ALIGN_TRANSLATION_kI = 0.2
        const val AUTO_ALIGN_TRANSLATION_kD = 0.25

        val MAPLE_SIM_CONFIG: DriveTrainSimulationConfig =
            DriveTrainSimulationConfig.Default().withCustomModuleTranslations(MODULE_LOCATIONS)
                .withGyro(COTS.ofGenericGyro()).withRobotMass(RobotConstants.WEIGHT)
                .withSwerveModule(COTS.ofMAXSwerve(DCMotor.getFalcon500(1), DCMotor.getNeo550(1), WHEEL_COF, 2))

        object FrontLeftModuleConstants {
            const val moduleID = 0
            const val driveID = 14
            const val angleID = 15
            const val angleOffset = -90.0
            const val inverted = false
        }

        object BackRightModuleConstants {
            const val moduleID = 3
            const val driveID = 11
            const val angleID = 10
            const val angleOffset = 90.0
            const val inverted = false
        }

        object FrontRightModuleConstants {
            const val moduleID = 1
            const val driveID = 17
            const val angleID = 16
            const val angleOffset = 0.0
            const val inverted = false
        }

        object BackLeftModuleConstants {
            const val moduleID = 2
            const val driveID = 13
            const val angleID = 12
            const val angleOffset = 180.0
            const val inverted = false
        }
    }

    object SwerveConstants {
        const val invertGyro = true

        const val KS = 0.1
        const val KA = 0.1
        const val KV = 0.1

        const val DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = 1 / RobotConstants.DRIVE_GEAR_RATIO
        const val DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = (DRIVING_ENCODER_POSITION_CONVERSION_FACTOR / 60.0)
        const val TURNING_ENCODER_POSITION_CONVERSION_FACTOR = (2 * Math.PI)
        const val TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = (TURNING_ENCODER_POSITION_CONVERSION_FACTOR / 60.0)

        const val TURN_PID_MIN_INPUT = -Math.PI
        const val TURN_PID_MAX_INPUT = Math.PI

        const val DRIVE_PID_MIN_OUTPUT = -1.0
        const val DRIVE_PID_MAX_OUTPUT = 1.0
        const val DRIVE_PID_P = 0.045
        const val DRIVE_PID_I = 0.00
        const val DRIVE_PID_D = 0.00
        const val DRIVE_PID_FF = 0.0

        const val TURN_PID_MIN_OUTPUT = -2 * Math.PI
        const val TURN_PID_MAX_OUTPUT = 2 * Math.PI
        const val TURN_PID_P = 5.0
        const val TURN_PID_I = 0.0
        const val TURN_PID_D = 0.15
        const val TURN_PID_FF = 0.0
    }

    object SimConstants {
        const val LOOP_TIME = 0.02
        const val REPLAY_LOG_PATH: String = "akit_25-05-19_16-17-20.wpilog"

        val SWERVE_CHANNELS: IntArray = intArrayOf(1, 2, 3, 4, 5, 6, 7, 8)
        val ELEVATOR_CHANNELS: IntArray = intArrayOf(9, 10)
        const val PIVOT_CHANNEL: Int = 11

        const val VISION_SIM: Boolean = true
    }

    object VisionConstants {
        val POSE_STRATEGY: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR

        val WAFFLE_CAMERA_TRANSFORMS: Map<String, Transform3d> = mapOf(
            "frontleft" to Transform3d(
                11.2435.inches, 13.42.inches, 7.308.inches, Rotation3d(
                    0.degrees, (-20).degrees, (-35).degrees
                )
            ), "frontright" to Transform3d(
                11.2435.inches, (-13.42).inches, 7.164.inches, Rotation3d(
                    0.degrees, (-20).degrees, 35.degrees
                )
            )
        )

        val COMP_CAMERA_TRANSFORMS: Map<String, Transform3d> = mapOf(
            "left" to Transform3d(
                (-9.528).inches, 11.430.inches, 20.584.inches, Rotation3d(
                    0.degrees, 20.degrees, 0.degrees
                ).rotateBy(
                    Rotation3d(
                        0.degrees, 0.degrees, 185.degrees
                    )
                )
            ), "leftcenter" to Transform3d(
                (-7.793).inches, 14.500.inches, 20.580.inches, Rotation3d(
                    0.degrees, 20.degrees, 0.degrees
                ).rotateBy(
                    Rotation3d(
                        0.degrees, 0.degrees, 120.degrees
                    )
                )
            ), "rightcenter" to Transform3d(
                (-1.788).inches, 14.722.inches, 20.583.inches, Rotation3d(
                    0.degrees, 20.degrees, 0.degrees
                ).rotateBy(
                    Rotation3d(
                        0.degrees, 0.degrees, 60.degrees
                    )
                )
            ), "right" to Transform3d(
                (-0.134).inches, 11.563.inches, 20.578.inches, Rotation3d(
                    0.degrees, 20.degrees, 0.degrees
                ).rotateBy(
                    Rotation3d(
                        0.degrees, 0.degrees, (-5).degrees
                    )
                )
            )
        )

        val CAMERA_TRANSFORMS: Map<String, Transform3d> =
            if (robot == RobotType.WAFFLE) WAFFLE_CAMERA_TRANSFORMS
            else COMP_CAMERA_TRANSFORMS
        const val VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD = 5.0
        const val CAMERA_RES_WIDTH = 1280
        const val CAMERA_RES_HEIGHT = 960
        const val CAMERA_FOV_DEG = 70
        const val CAMERA_AVG_LATENCY_MS = 35.0
        const val AVG_ERROR_PX = 0.25
        const val ERROR_STDEV_PX = 0.08
        const val FPS = 20.0
        const val CAMERA_LATENCY_STDEV_MS = 5.0

        const val APRIL_TAG_NUMBER_CONFIDENCE_SCALE = 3.0 // Higher makes confidence lower at each number of

        // AprilTags
        val APRIL_TAG_NUMBER_EXPONENT =
            (-1 / (APRIL_TAG_NUMBER_CONFIDENCE_SCALE * ln(APRIL_TAG_NUMBER_CONFIDENCE_SCALE)))
        const val APRIL_TAG_AREA_CONFIDENCE_SCALE = 1.7 // Higher makes confidence lower at each area of

        // AprilTags
        // See https://www.desmos.com/calculator/i5z7ddbjy4
        const val REPROJ_TO_STDEV_EXP = 0.01
        val BASE_STDEV: Vector<N3> = VecBuilder.fill(0.1, 0.1, 100000.0) // x, y, angle
        const val AMBIGUITY_ACCEPTANCE_THRESHOLD = 0.2
        const val SIM_BUFFER_LENGTH = 1.5
    }

    object IntakeConstants {
        val CURRENT_LIMIT = 100.amps
        const val ARM_DEPLOY_SPEED = 1.0
        const val ARM_DEPLOY_CURRENT_THRESHOLD = 20.0
        const val GEAR_RATIO = (60 / 16.0) * (60 / 26.0) * (40 / 20.0)

        const val kP = 50.0
        const val kI = 0.0
        const val kD = 0.0
        const val kS = 0.0
        const val kV = 0.0
        const val kA = 0.0
        const val kG = 0.0
        const val CRUISE_VELOCITY = 10.0
        const val MAX_ACCELERATION = 30.0
        const val ALLOWED_PROFILE_ERROR = 0.4 // rotations

        val DEPLOYED: Rotation2d = Rotation2d.fromRadians(0.855)
        val STOWED: Rotation2d = Rotation2d.kZero
        val MAX_ERROR: Rotation2d = Rotation2d.fromDegrees(3.0)

        const val ARM_ID = 20
        const val ROLLER_ID = 21
    }

    object IndexerConstants {
        val SPINDEXER_CURRENT_LIMIT = 40.amps
        val FEEDER_MOTOR_CURRENT_LIMIT = 20.amps
        const val INDEXER_MOTOR_ID = 30
        const val FEEDER_MOTOR_ID = 31
    }

    object ShooterConstants {
        const val HOOD_ID = 40
        const val FLYWHEEL1_ID = 41
        const val FLYWHEEL2_ID = 42
        const val FLYWHEEL3_ID = 43

        const val FLYWHEEL_GEAR_RATIO = (16 / 10.0) * (15 / 30.0)
        const val MAXPLANETARY_RATIO = 3.0
        const val HERRINGBONE_RATIO = 158 / 10.0
        const val HOOD_GEAR_RATIO = MAXPLANETARY_RATIO * HERRINGBONE_RATIO

        const val HOOD_kP = 150.0
        const val HOOD_kI = 0.0
        const val HOOD_kD = 0.0
        const val HOOD_kS = 0.0
        const val HOOD_kV = 0.0
        const val HOOD_kA = 0.0
        const val HOOD_kG = 0.0
        const val HOOD_CRUISE_VELOCITY = 200.0
        const val HOOD_MAX_ACCELERATION = 240.0
        const val HOOD_ALLOWED_PROFILE_ERROR = 0.4 // rotations
        val HOOD_CURRENT_LIMIT = 30.amps

        val MIN_ANGLE: Rotation2d = Rotation2d.kZero
        val MAX_ANGLE: Rotation2d = Rotation2d.fromRotations(0.13)

        const val FLYWHEEL_kP = 0.0008
        const val FLYWHEEL_kI = 0.0 / 60
        const val FLYWHEEL_kD = 0.0
        const val FLYWHEEL_kS = 0.272137
        const val FLYWHEEL_kV = 0.00167205
        const val FLYWHEEL_kA = 0.0 / 60
        const val FLYWHEEL_MAX_ACCELERATION = 8000.0
        val FLYWHEEL_CURRENT_LIMIT = 60.amps

        val TOLERANCE = 400.rpm
    }

    object DriverAssistConstants {
        val TRENCH_ASSIST_RADIUS = 8.feet
        const val TRENCH_ASSIST_STRENGTH = 2.0
    }

    object TempConstants {
        const val OVERHEAT_TEMP = 80
        const val SAFE_TEMP = 80
    }

    object AlertConstants {
        const val LOW_BATTERY_VOLTAGE = 11.5
        const val ENDGAME_ALERT_1_TIME = 45
        const val ENDGAME_ALERT_2_TIME = 30
    }

    object LoggingConstants {
        const val LOG_ALERT_INTERVAL = 5.0 // Interval (in s) between logs of an alert if its text doesn't change
    }

    object LEDConstants {
        const val LED_PORT = 9

        const val INTAKE_VELOCITY_THRESHOLD = 0.2
        const val ALIGNED_DISTANCE = 0.03 // Meters, I think
        const val ALIGNED_ANGLE = 2.0 // Degrees

        object LengthConstants {
            // LEFT MID RIGHT
            const val LEFT: Int = 23
            const val MID: Int = 21
            const val RIGHT: Int = 24

            const val TOTAL = LEFT + MID + RIGHT
        }

        object ColorConstants {
            val LOADING: Color = Color.kWhite
            val SUCCESS: Color = Color(77, 255, 79)
            val RED: Color = Color(255, 25, 25)
            val PINK: Color = Color(255, 69, 70)
            val BLUE: Color = Color(25, 25, 255)
            val TEAL: Color = Color(160, 170, 255)
            val AUTON_1: Color = Color(255, 69, 118)
            val AUTON_2: Color = Color(255, 30, 180)
            val AUTON_3: Color = Color(100, 25, 25)
            val USER_SIGNAL: Color = Color.kWhite

            val PRIDE_RED: Color = Color.kRed
            val PRIDE_ORANGE: Color = Color.kOrangeRed
            val PRIDE_YELLOW: Color = Color.kYellow
            val PRIDE_GREEN: Color = Color.kGreen
            val PRIDE_BLUE: Color = Color.kBlue
            val PRIDE_PURPLE: Color = Color.kPurple
            val TRANS_PINK: Color = Color.kDeepPink
            val TRANS_TEAL: Color = Color(0.15, 0.3, 1.0)
        }
    }
}
