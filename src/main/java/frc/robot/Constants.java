// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.drive.SwerveKinematicLimits;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static enum RobotType {
        WAFFLE, COMP, SIM_BASIC, SIM_ADVANCED
    }

    private static final RobotType robotType = RobotType.SIM_BASIC;

    // DO NOT EDIT
    private static final RobotType trueRobotType = RuntimeType
            .getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation ? robotType
                    : robotType == RobotType.SIM_BASIC || robotType == RobotType.SIM_ADVANCED ? RobotType.COMP : robotType;

    public static final RobotType getRobot() {
        return trueRobotType;
    }

    public static enum RobotMode {
        REAL, REPLAY, SIM
    }

    private static final RobotMode robotMode = getRobot() == RobotType.SIM_BASIC || getRobot() == RobotType.SIM_ADVANCED ? RobotMode.SIM
            : RuntimeType.getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation ? RobotMode.REPLAY
                    : RobotMode.REAL;

    public static final RobotMode getMode() {
        return robotMode;
    }

    public static class PathConstants {
        public static final PathConstraints CONSTRAINTS = new PathConstraints(4, 4, Units.degreesToRadians(540),
                Units.degreesToRadians(720));
    }

    public static class IOConstants {
        public static final double JOYSTICK_DEADBAND = Robot.isReal() ? 0.05 : 0.15;

        public static class ControllerIOConstants {
            public static final int DRIVER_CONTROLLER_PORT = 0;
            public static final int OPERATOR_CONTROLLER_PORT = 1;

            public static final int LEFT_STICK_HORIZONTAL = 0;
            public static final int LEFT_STICK_VERTICAL = 1;
            public static final int RIGHT_STICK_HORIZONTAL = 4;
            public static final int RIGHT_STICK_VERTICAL = 5;

            public static final int A_BUTTON = 1;
            public static final int B_BUTTON = 2;
            public static final int X_BUTTON = 3;
            public static final int Y_BUTTON = 4;

            public static final int LT_BUTTON = 2;
            public static final int RT_BUTTON = 3;

            public static final int LB_BUTTON = 5;
            public static final int RB_BUTTON = 6;

            public static final int RIGHT_PADDLE = 7;
            public static final int LEFT_PADDLE = 8;

            public static final int D_PAD_UP = 0;
            public static final int D_PAD_DOWN = 180;
            public static final int D_PAD_LEFT = 270;
            public static final int D_PAD_RIGHT = 90;

            public static final boolean SQUARE_INPUTS = true;
        }
    }

    public static class Waffle {
        public static class RobotConstants {
            public static final double WHEELBASE = Units.inchesToMeters(29);
            public static final double TRACK_WIDTH = WHEELBASE;
            public static final double EDGE_TO_MODULE_CENTER = Units.inchesToMeters(1.75);
            public static final double RADIUS = Math.sqrt(Math.pow(WHEELBASE / 2 - EDGE_TO_MODULE_CENTER, 2)
                    + Math.pow(TRACK_WIDTH / 2 - EDGE_TO_MODULE_CENTER, 2));
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
            public static final double DRIVE_GEAR_RATIO = 5.5;
            public static final double ANGLE_GEAR_RATIO = 9424 / 203;
            public static final double MAX_DRIVE_SPEED = 5.23; // from https://www.reca.lc/drive
            public static final Mass WEIGHT = Pounds.of(150);
            public static final MomentOfInertia MOI = KilogramSquareMeters.of(6);
        }

        public static class DriveConstants {
            public static final double MAX_ACCELERATION = 45;
            public static final double MAX_DRIVE_SPEED = 100;
            public static final double MAX_ANGULAR_SPEED = 50;
            public static final double SLOWDOWN_PERCENT = 0.5;
            public static final int DRIVE_MOTOR_CURRENT_LIMIT = 30;
            public static final int ANGLE_MOTOR_CURRENT_LIMIT = 20;
            public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
                    new Translation2d(RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
                            RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER),
                    new Translation2d(RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
                            -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER),
                    new Translation2d(-RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
                            RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER),
                    new Translation2d(-RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
                            -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER), };
            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);
            public static final SwerveKinematicLimits MODULE_LIMITS = new SwerveKinematicLimits(MAX_DRIVE_SPEED,
                    MAX_ACCELERATION, MAX_ANGULAR_SPEED);

            public static boolean FIELD_CENTRIC = true;
            public static final boolean IS_OPEN_LOOP = false;

            public static final double PATH_MAX_ACCEL = 3;
            public static final double PATH_MAX_VELOCITY = 3;

            public static final double TURN_DEADBAND = Units.degreesToRadians(5);

            public static final double WHEEL_COF = 1.5;

            public static final RobotConfig ROBOT_CONFIG = new RobotConfig(RobotConstants.WEIGHT, // Robot mass
                    RobotConstants.MOI, // Robot moment of inertia
                    new ModuleConfig(RobotConstants.WHEEL_DIAMETER / 2, // wheel diameter
                            RobotConstants.MAX_DRIVE_SPEED, // max drive velocity (m/s)
                            WHEEL_COF, // cof between wheels and ground
                            DCMotor.getNEO(1).withReduction(RobotConstants.DRIVE_GEAR_RATIO), // DCMotor representing motor, including reduction
                            DRIVE_MOTOR_CURRENT_LIMIT, // current limit for drive motors
                            1 // number of drive motors per module
                    ), MODULE_LOCATIONS);
            public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0);
            public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(7, 0.01, 0.15);

            public static final double AUTO_ALIGN_TRANSLATION_kP = 5;
            public static final double AUTO_ALIGN_TRANSLATION_kI = 0.2;
            public static final double AUTO_ALIGN_TRANSLATION_kD = 0.25;

            public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
                    .withCustomModuleTranslations(MODULE_LOCATIONS).withGyro(COTS.ofNav2X())
                    .withRobotMass(Pounds.of(125))
                    .withSwerveModule(COTS.ofMAXSwerve(DCMotor.getNEO(1), DCMotor.getNeo550(1), WHEEL_COF, 1));

            public static class FrontLeftModuleConstants {
                public static final int moduleID = 0;
                public static final int driveID = 14;
                public static final int angleID = 15;
                public static final double angleOffset = -90;
                public static final boolean inverted = false;
            }

            public static class BackRightModuleConstants {
                public static final int moduleID = 3;
                public static final int driveID = 11;
                public static final int angleID = 10;
                public static final double angleOffset = 90;
                public static final boolean inverted = false;
            }

            public static class FrontRightModuleConstants {
                public static final int moduleID = 1;
                public static final int driveID = 17;
                public static final int angleID = 16;
                public static final double angleOffset = 0;
                public static final boolean inverted = false;
            }

            public static class BackLeftModuleConstants {
                public static final int moduleID = 2;
                public static final int driveID = 13;
                public static final int angleID = 12;
                public static final double angleOffset = 180;
                public static final boolean inverted = false;
            }
        }

        public static class SwerveConstants {
            public static final boolean invertGyro = true;

            public static final double KS = 0.1;
            public static final double KA = 0.1;
            public static final double KV = 0.1;

            public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = 1 / RobotConstants.DRIVE_GEAR_RATIO;
            public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = DRIVING_ENCODER_POSITION_CONVERSION_FACTOR
                    / 60.0;
            public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = (2 * Math.PI);
            public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR
                    / 60.0;

            public static final double TURN_PID_MIN_INPUT = -Math.PI;
            public static final double TURN_PID_MAX_INPUT = Math.PI;

            public static final double DRIVE_PID_MIN_OUTPUT = -1;
            public static final double DRIVE_PID_MAX_OUTPUT = 1;
            public static final double DRIVE_PID_P = 0.045;
            public static final double DRIVE_PID_I = 0.00;
            public static final double DRIVE_PID_D = 0.00;
            public static final double DRIVE_PID_FF = 0;

            public static final double TURN_PID_MIN_OUTPUT = -2 * Math.PI;
            public static final double TURN_PID_MAX_OUTPUT = 2 * Math.PI;
            public static final double TURN_PID_P = 5;
            public static final double TURN_PID_I = 0;
            public static final double TURN_PID_D = 0.15;
            public static final double TURN_PID_FF = 0;
        }
    }

    public static class Comp {
        public static class RobotConstants {
            public static final double WHEELBASE = Units.inchesToMeters(24);
            public static final double TRACK_WIDTH = Units.inchesToMeters(30);
            public static final double EDGE_TO_MODULE_CENTER = Units.inchesToMeters(1.75);
            // Distance from robot center to module center
            public static final double RADIUS = Math.sqrt(Math.pow(WHEELBASE / 2 - EDGE_TO_MODULE_CENTER, 2)
                    + Math.pow(TRACK_WIDTH / 2 - EDGE_TO_MODULE_CENTER, 2));
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
            public static final double DRIVE_GEAR_RATIO = 5.08;
            public static final double ANGLE_GEAR_RATIO = 9424 / 203;
            public static final double MAX_DRIVE_SPEED = 5.23; // from https://www.reca.lc/drive
            public static final Mass WEIGHT = Pounds.of(150);
            public static final MomentOfInertia MOI = KilogramSquareMeters.of(6);

            public static final int GYRO_ID = 3;
        }

        public static class DriveConstants {
            public static final double MAX_ACCELERATION = 45;
            public static final double MAX_DRIVE_SPEED = 100;
            public static final double MAX_ANGULAR_SPEED = 50;
            public static final double SLOWDOWN_PERCENT = 0.5;
            public static final int DRIVE_MOTOR_CURRENT_LIMIT = 30;
            public static final int ANGLE_MOTOR_CURRENT_LIMIT = 20;
            public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
                    new Translation2d(RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
                            RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER),
                    new Translation2d(RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
                            -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER),
                    new Translation2d(-RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
                            RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER),
                    new Translation2d(-RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
                            -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER), };
            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);
            public static final SwerveKinematicLimits MODULE_LIMITS = new SwerveKinematicLimits(MAX_DRIVE_SPEED,
                    MAX_ACCELERATION, MAX_ANGULAR_SPEED);

            public static boolean FIELD_CENTRIC = true;
            public static final boolean IS_OPEN_LOOP = false;

            public static final double PATH_MAX_ACCEL = 3;
            public static final double PATH_MAX_VELOCITY = 3;

            public static final double TURN_DEADBAND = Units.degreesToRadians(5);

            public static final double WHEEL_COF = 1.5;

            public static final RobotConfig ROBOT_CONFIG = new RobotConfig(RobotConstants.WEIGHT, // Robot mass
                    RobotConstants.MOI, // Robot moment of inertia
                    new ModuleConfig(RobotConstants.WHEEL_DIAMETER / 2, // wheel diameter
                            RobotConstants.MAX_DRIVE_SPEED, // max drive velocity (m/s)
                            WHEEL_COF, // cof between wheels and ground
                            DCMotor.getNEO(1).withReduction(RobotConstants.DRIVE_GEAR_RATIO), // DCMotor representing motor, including reduction
                            DRIVE_MOTOR_CURRENT_LIMIT, // current limit for drive motors
                            1 // number of drive motors per module
                    ), MODULE_LOCATIONS);
            public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0);
            public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(7, 0.01, 0.15);

            public static final double AUTO_ALIGN_TRANSLATION_kP = 5;
            public static final double AUTO_ALIGN_TRANSLATION_kI = 0.2;
            public static final double AUTO_ALIGN_TRANSLATION_kD = 0.25;

            public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
                    .withCustomModuleTranslations(MODULE_LOCATIONS).withGyro(COTS.ofGenericGyro())
                    .withRobotMass(RobotConstants.WEIGHT)
                    .withSwerveModule(COTS.ofMAXSwerve(DCMotor.getFalcon500(1), DCMotor.getNeo550(1), WHEEL_COF, 2));

            public static class FrontLeftModuleConstants {
                public static final int moduleID = 0;
                public static final int driveID = 14;
                public static final int angleID = 15;
                public static final double angleOffset = -90;
                public static final boolean inverted = false;
            }

            public static class BackRightModuleConstants {
                public static final int moduleID = 3;
                public static final int driveID = 11;
                public static final int angleID = 10;
                public static final double angleOffset = 90;
                public static final boolean inverted = false;
            }

            public static class FrontRightModuleConstants {
                public static final int moduleID = 1;
                public static final int driveID = 17;
                public static final int angleID = 16;
                public static final double angleOffset = 0;
                public static final boolean inverted = false;
            }

            public static class BackLeftModuleConstants {
                public static final int moduleID = 2;
                public static final int driveID = 13;
                public static final int angleID = 12;
                public static final double angleOffset = 180;
                public static final boolean inverted = false;
            }
        }

        public static class SwerveConstants {
            public static final boolean invertGyro = true;

            public static final double KS = 0.1;
            public static final double KA = 0.1;
            public static final double KV = 0.1;

            public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = 1 / RobotConstants.DRIVE_GEAR_RATIO;
            public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = DRIVING_ENCODER_POSITION_CONVERSION_FACTOR
                    / 60.0;
            public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = (2 * Math.PI);
            public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR
                    / 60.0;

            public static final double TURN_PID_MIN_INPUT = -Math.PI;
            public static final double TURN_PID_MAX_INPUT = Math.PI;

            public static final double DRIVE_PID_MIN_OUTPUT = -1;
            public static final double DRIVE_PID_MAX_OUTPUT = 1;
            public static final double DRIVE_PID_P = 0.045;
            public static final double DRIVE_PID_I = 0.00;
            public static final double DRIVE_PID_D = 0.00;
            public static final double DRIVE_PID_FF = 0;

            public static final double TURN_PID_MIN_OUTPUT = -2 * Math.PI;
            public static final double TURN_PID_MAX_OUTPUT = 2 * Math.PI;
            public static final double TURN_PID_P = 5;
            public static final double TURN_PID_I = 0;
            public static final double TURN_PID_D = 0.15;
            public static final double TURN_PID_FF = 0;
        }
    }

    public static class SimConstants {
        public static final double LOOP_TIME = 0.02;
        public static final String REPLAY_LOG_PATH = "akit_25-05-19_16-17-20.wpilog";

        public static final int[] SWERVE_CHANNELS = { 1, 2, 3, 4, 5, 6, 7, 8 };
        public static final int[] ELEVATOR_CHANNELS = { 9, 10 };
        public static final int PIVOT_CHANNEL = 11;

        public static final boolean VISION_SIM = true;
    }

    public static class VisionConstants {
        public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final Map<String, Transform3d> WAFFLE_CAMERA_TRANSFORMS = new HashMap<>();
        static {
            WAFFLE_CAMERA_TRANSFORMS.put("frontleft", new Transform3d(Inches.of(11.2435), Inches.of(13.42), Inches.of(7.308),
                    new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-35))));
            WAFFLE_CAMERA_TRANSFORMS.put("frontright", new Transform3d(Inches.of(11.2435), Inches.of(-13.42), Inches.of(7.164),
                    new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(35))));
        }
        public static final Map<String, Transform3d> COMP_CAMERA_TRANSFORMS = new HashMap<>();
        public static final Map<String, Transform3d> CAMERA_TRANSFORMS = getRobot() == RobotType.WAFFLE ? WAFFLE_CAMERA_TRANSFORMS : COMP_CAMERA_TRANSFORMS;
        public static final double VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD = 5;
        public static final int CAMERA_RES_WIDTH = 1280;
        public static final int CAMERA_RES_HEIGHT = 960;
        public static final int CAMERA_FOV_DEG = 70;
        public static final double CAMERA_AVG_LATENCY_MS = 35;
        public static final double AVG_ERROR_PX = 0.25;
        public static final double ERROR_STDEV_PX = 0.08;
        public static final double FPS = 20;
        public static final double CAMERA_LATENCY_STDEV_MS = 5;

        public static final double APRIL_TAG_NUMBER_CONFIDENCE_SCALE = 3; // Higher makes confidence lower at each number of
                                                                          // AprilTags
        public static final double APRIL_TAG_NUMBER_EXPONENT = -1
                / (APRIL_TAG_NUMBER_CONFIDENCE_SCALE * Math.log(APRIL_TAG_NUMBER_CONFIDENCE_SCALE));
        public static final double APRIL_TAG_AREA_CONFIDENCE_SCALE = 1.7; // Higher makes confidence lower at each area of
                                                                          // AprilTags
                                                                          // See https://www.desmos.com/calculator/i5z7ddbjy4

        public static final double REPROJ_TO_STDEV_EXP = 0.01;
        public static final Vector<N3> BASE_STDEV = VecBuilder.fill(0.1, 0.1, 1000.0); // x, y, angle
        public static final double AMBIGUITY_ACCEPTANCE_THRESHOLD = 0.2;
        public static final double SIM_BUFFER_LENGTH = 1.5;
    }

    public static class IntakeConstants {
        public static final int CURRENT_LIMIT = 40;
        public static final double ARM_DEPLOY_SPEED = 1.0;
        public static final double ARM_DEPLOY_CURRENT_THRESHOLD = 20;
        public static final double GEAR_RATIO = (60 / 16.0) * (60 / 26.0) * (40 / 20.0);

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
        public static final double CRUISE_VELOCITY = 10.0;
        public static final double MAX_ACCELERATION = 30.0;
        public static final double ALLOWED_PROFILE_ERROR = 0.4; // rotations

        public static final Rotation2d DEPLOYED = Rotation2d.kZero;
        public static final Rotation2d STOWED = Rotation2d.kCCW_90deg;
        public static final Rotation2d MAX_ERROR = Rotation2d.fromDegrees(2);

        public static final int ARM_ID = 20;
        public static final int ROLLER_ID = 21;
    }

    public static class TempConstants {
        public static final int OVERHEAT_TEMP = 80;
        public static final int SAFE_TEMP = 80;
    }

    public static class AlertConstants {
        public static final double LOW_BATTERY_VOLTAGE = 11.5;
        public static final int ENDGAME_ALERT_1_TIME = 45;
        public static final int ENDGAME_ALERT_2_TIME = 30;
    }

    public static class LoggingConstants {
        public static final double LOG_ALERT_INTERVAL = 5; // Interval (in s) between logs of an alert if its text doesn't change
    }

    public static class LEDConstants {
        public static final int LED_PORT = 9;

        public static class LengthConstants {
            // LEFT MID RIGHT
            public static final int LEFT = 23;
            public static final int MID = 21;
            public static final int RIGHT = 24;

            public static final int TOTAL = LEFT + MID + RIGHT;
        }

        public static class ColorConstants {
            public static final Color LOADING = Color.kWhite;
            public static final Color SUCCESS = new Color(77, 255, 79);
            public static final Color RED = new Color(255, 25, 25);
            public static final Color PINK = new Color(255, 69, 70);
            public static final Color BLUE = new Color(25, 25, 255);
            public static final Color TEAL = new Color(160, 170, 255);
            public static final Color AUTON_1 = new Color(255, 69, 118);
            public static final Color AUTON_2 = new Color(255, 30, 180);
            public static final Color AUTON_3 = new Color(100, 25, 25);
            public static final Color USER_SIGNAL = Color.kWhite;

            public static final Color PRIDE_RED = Color.kRed;
            public static final Color PRIDE_ORANGE = Color.kOrangeRed;
            public static final Color PRIDE_YELLOW = Color.kYellow;
            public static final Color PRIDE_GREEN = Color.kGreen;
            public static final Color PRIDE_BLUE = Color.kBlue;
            public static final Color PRIDE_PURPLE = Color.kPurple;
            public static final Color TRANS_PINK = Color.kDeepPink;
            public static final Color TRANS_TEAL = new Color(0.15, 0.3, 1.0);
        }

        public static final double INTAKE_VELOCITY_THRESHOLD = 0.2;
        public static final double ALIGNED_DISTANCE = 0.03; // Meters, I think
        public static final double ALIGNED_ANGLE = 2; // Degrees
    }

        public static class RobotConstants {
        public static final double WHEELBASE;
        public static final double TRACK_WIDTH;
        public static final double EDGE_TO_MODULE_CENTER;
        public static final double RADIUS; // Distance from robot center to module center
        public static final double WHEEL_DIAMETER;
        public static final double DRIVE_GEAR_RATIO;
        public static final double ANGLE_GEAR_RATIO;
        public static final double MAX_DRIVE_SPEED;
        public static final Mass WEIGHT;
        public static final MomentOfInertia MOI;

        static {
            if (getRobot() == RobotType.WAFFLE) {
                WHEELBASE = Waffle.RobotConstants.WHEELBASE;
                TRACK_WIDTH = Waffle.RobotConstants.TRACK_WIDTH;
                EDGE_TO_MODULE_CENTER = Waffle.RobotConstants.EDGE_TO_MODULE_CENTER;
                RADIUS = Waffle.RobotConstants.RADIUS;
                WHEEL_DIAMETER = Waffle.RobotConstants.WHEEL_DIAMETER;
                DRIVE_GEAR_RATIO = Waffle.RobotConstants.DRIVE_GEAR_RATIO;
                ANGLE_GEAR_RATIO = Waffle.RobotConstants.ANGLE_GEAR_RATIO;
                MAX_DRIVE_SPEED = Waffle.RobotConstants.MAX_DRIVE_SPEED;
                WEIGHT = Waffle.RobotConstants.WEIGHT;
                MOI = Waffle.RobotConstants.MOI;
            } else {
                WHEELBASE = Comp.RobotConstants.WHEELBASE;
                TRACK_WIDTH = Comp.RobotConstants.TRACK_WIDTH;
                EDGE_TO_MODULE_CENTER = Comp.RobotConstants.EDGE_TO_MODULE_CENTER;
                RADIUS = Comp.RobotConstants.RADIUS;
                WHEEL_DIAMETER = Comp.RobotConstants.WHEEL_DIAMETER;
                DRIVE_GEAR_RATIO = Comp.RobotConstants.DRIVE_GEAR_RATIO;
                ANGLE_GEAR_RATIO = Comp.RobotConstants.ANGLE_GEAR_RATIO;
                MAX_DRIVE_SPEED = Comp.RobotConstants.MAX_DRIVE_SPEED;
                WEIGHT = Comp.RobotConstants.WEIGHT;
                MOI = Comp.RobotConstants.MOI;
            }
        }
    }

    public static class DriveConstants {
        public static final double MAX_ACCELERATION;
        public static final double MAX_DRIVE_SPEED;
        public static final double MAX_ANGULAR_SPEED;
        public static final double SLOWDOWN_PERCENT;
        public static final int DRIVE_MOTOR_CURRENT_LIMIT;
        public static final int ANGLE_MOTOR_CURRENT_LIMIT;
        public static final Translation2d[] MODULE_LOCATIONS;
        public static final SwerveDriveKinematics KINEMATICS;
        public static final SwerveKinematicLimits MODULE_LIMITS;
        public static boolean FIELD_CENTRIC;
        public static final boolean IS_OPEN_LOOP;
        public static final double PATH_MAX_ACCEL;
        public static final double PATH_MAX_VELOCITY;
        public static final double TURN_DEADBAND;
        public static final double WHEEL_COF;
        public static final RobotConfig ROBOT_CONFIG;
        public static final PIDConstants ROTATION_PID_CONSTANTS;
        public static final PIDConstants TRANSLATION_PID_CONSTANTS;
        public static final double AUTO_ALIGN_TRANSLATION_kP;
        public static final double AUTO_ALIGN_TRANSLATION_kI;
        public static final double AUTO_ALIGN_TRANSLATION_kD;
        public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG;

        static {
            if (getRobot() == RobotType.WAFFLE) {
                MAX_ACCELERATION = Waffle.DriveConstants.MAX_ACCELERATION;
                MAX_DRIVE_SPEED = Waffle.DriveConstants.MAX_DRIVE_SPEED;
                MAX_ANGULAR_SPEED = Waffle.DriveConstants.MAX_ANGULAR_SPEED;
                SLOWDOWN_PERCENT = Waffle.DriveConstants.SLOWDOWN_PERCENT;
                DRIVE_MOTOR_CURRENT_LIMIT = Waffle.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
                ANGLE_MOTOR_CURRENT_LIMIT = Waffle.DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT;
                MODULE_LOCATIONS = Waffle.DriveConstants.MODULE_LOCATIONS;
                KINEMATICS = Waffle.DriveConstants.KINEMATICS;
                MODULE_LIMITS = Waffle.DriveConstants.MODULE_LIMITS;
                FIELD_CENTRIC = Waffle.DriveConstants.FIELD_CENTRIC;
                IS_OPEN_LOOP = Waffle.DriveConstants.IS_OPEN_LOOP;
                PATH_MAX_ACCEL = Waffle.DriveConstants.PATH_MAX_ACCEL;
                PATH_MAX_VELOCITY = Waffle.DriveConstants.PATH_MAX_VELOCITY;
                TURN_DEADBAND = Waffle.DriveConstants.TURN_DEADBAND;
                WHEEL_COF = Waffle.DriveConstants.WHEEL_COF;
                ROBOT_CONFIG = Waffle.DriveConstants.ROBOT_CONFIG;
                ROTATION_PID_CONSTANTS = Waffle.DriveConstants.ROTATION_PID_CONSTANTS;
                TRANSLATION_PID_CONSTANTS = Waffle.DriveConstants.TRANSLATION_PID_CONSTANTS;
                AUTO_ALIGN_TRANSLATION_kP = Waffle.DriveConstants.AUTO_ALIGN_TRANSLATION_kP;
                AUTO_ALIGN_TRANSLATION_kI = Waffle.DriveConstants.AUTO_ALIGN_TRANSLATION_kI;
                AUTO_ALIGN_TRANSLATION_kD = Waffle.DriveConstants.AUTO_ALIGN_TRANSLATION_kD;
                MAPLE_SIM_CONFIG = Waffle.DriveConstants.MAPLE_SIM_CONFIG;
            } else {
                MAX_ACCELERATION = Comp.DriveConstants.MAX_ACCELERATION;
                MAX_DRIVE_SPEED = Comp.DriveConstants.MAX_DRIVE_SPEED;
                MAX_ANGULAR_SPEED = Comp.DriveConstants.MAX_ANGULAR_SPEED;
                SLOWDOWN_PERCENT = Comp.DriveConstants.SLOWDOWN_PERCENT;
                DRIVE_MOTOR_CURRENT_LIMIT = Comp.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
                ANGLE_MOTOR_CURRENT_LIMIT = Comp.DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT;
                MODULE_LOCATIONS = Comp.DriveConstants.MODULE_LOCATIONS;
                KINEMATICS = Comp.DriveConstants.KINEMATICS;
                MODULE_LIMITS = Comp.DriveConstants.MODULE_LIMITS;
                FIELD_CENTRIC = Comp.DriveConstants.FIELD_CENTRIC;
                IS_OPEN_LOOP = Comp.DriveConstants.IS_OPEN_LOOP;
                PATH_MAX_ACCEL = Comp.DriveConstants.PATH_MAX_ACCEL;
                PATH_MAX_VELOCITY = Comp.DriveConstants.PATH_MAX_VELOCITY;
                TURN_DEADBAND = Comp.DriveConstants.TURN_DEADBAND;
                WHEEL_COF = Comp.DriveConstants.WHEEL_COF;
                ROBOT_CONFIG = Comp.DriveConstants.ROBOT_CONFIG;
                ROTATION_PID_CONSTANTS = Comp.DriveConstants.ROTATION_PID_CONSTANTS;
                TRANSLATION_PID_CONSTANTS = Comp.DriveConstants.TRANSLATION_PID_CONSTANTS;
                AUTO_ALIGN_TRANSLATION_kP = Comp.DriveConstants.AUTO_ALIGN_TRANSLATION_kP;
                AUTO_ALIGN_TRANSLATION_kI = Comp.DriveConstants.AUTO_ALIGN_TRANSLATION_kI;
                AUTO_ALIGN_TRANSLATION_kD = Comp.DriveConstants.AUTO_ALIGN_TRANSLATION_kD;
                MAPLE_SIM_CONFIG = Comp.DriveConstants.MAPLE_SIM_CONFIG;
            }
        }

        public static class FrontLeftModuleConstants {
            public static final int moduleID;
            public static final int driveID;
            public static final int angleID;
            public static final double angleOffset;
            public static final boolean inverted;

            static {
                if (getRobot() == RobotType.WAFFLE) {
                    moduleID = Waffle.DriveConstants.FrontLeftModuleConstants.moduleID;
                    driveID = Waffle.DriveConstants.FrontLeftModuleConstants.driveID;
                    angleID = Waffle.DriveConstants.FrontLeftModuleConstants.angleID;
                    angleOffset = Waffle.DriveConstants.FrontLeftModuleConstants.angleOffset;
                    inverted = Waffle.DriveConstants.FrontLeftModuleConstants.inverted;
                } else {
                    moduleID = Comp.DriveConstants.FrontLeftModuleConstants.moduleID;
                    driveID = Comp.DriveConstants.FrontLeftModuleConstants.driveID;
                    angleID = Comp.DriveConstants.FrontLeftModuleConstants.angleID;
                    angleOffset = Comp.DriveConstants.FrontLeftModuleConstants.angleOffset;
                    inverted = Comp.DriveConstants.FrontLeftModuleConstants.inverted;
                }
            }
        }

        public static class BackRightModuleConstants {
            public static final int moduleID;
            public static final int driveID;
            public static final int angleID;
            public static final double angleOffset;
            public static final boolean inverted;

            static {
                if (getRobot() == RobotType.WAFFLE) {
                    moduleID = Waffle.DriveConstants.BackRightModuleConstants.moduleID;
                    driveID = Waffle.DriveConstants.BackRightModuleConstants.driveID;
                    angleID = Waffle.DriveConstants.BackRightModuleConstants.angleID;
                    angleOffset = Waffle.DriveConstants.BackRightModuleConstants.angleOffset;
                    inverted = Waffle.DriveConstants.BackRightModuleConstants.inverted;
                } else {
                    moduleID = Comp.DriveConstants.BackRightModuleConstants.moduleID;
                    driveID = Comp.DriveConstants.BackRightModuleConstants.driveID;
                    angleID = Comp.DriveConstants.BackRightModuleConstants.angleID;
                    angleOffset = Comp.DriveConstants.BackRightModuleConstants.angleOffset;
                    inverted = Comp.DriveConstants.BackRightModuleConstants.inverted;
                }
            }
        }

        public static class FrontRightModuleConstants {
            public static final int moduleID;
            public static final int driveID;
            public static final int angleID;
            public static final double angleOffset;
            public static final boolean inverted;

            static {
                if (getRobot() == RobotType.WAFFLE) {
                    moduleID = Waffle.DriveConstants.FrontRightModuleConstants.moduleID;
                    driveID = Waffle.DriveConstants.FrontRightModuleConstants.driveID;
                    angleID = Waffle.DriveConstants.FrontRightModuleConstants.angleID;
                    angleOffset = Waffle.DriveConstants.FrontRightModuleConstants.angleOffset;
                    inverted = Waffle.DriveConstants.FrontRightModuleConstants.inverted;
                } else {
                    moduleID = Comp.DriveConstants.FrontRightModuleConstants.moduleID;
                    driveID = Comp.DriveConstants.FrontRightModuleConstants.driveID;
                    angleID = Comp.DriveConstants.FrontRightModuleConstants.angleID;
                    angleOffset = Comp.DriveConstants.FrontRightModuleConstants.angleOffset;
                    inverted = Comp.DriveConstants.FrontRightModuleConstants.inverted;
                }
            }
        }

        public static class BackLeftModuleConstants {
            public static final int moduleID;
            public static final int driveID;
            public static final int angleID;
            public static final double angleOffset;
            public static final boolean inverted;

            static {
                if (getRobot() == RobotType.WAFFLE) {
                    moduleID = Waffle.DriveConstants.BackLeftModuleConstants.moduleID;
                    driveID = Waffle.DriveConstants.BackLeftModuleConstants.driveID;
                    angleID = Waffle.DriveConstants.BackLeftModuleConstants.angleID;
                    angleOffset = Waffle.DriveConstants.BackLeftModuleConstants.angleOffset;
                    inverted = Waffle.DriveConstants.BackLeftModuleConstants.inverted;
                } else {
                    moduleID = Comp.DriveConstants.BackLeftModuleConstants.moduleID;
                    driveID = Comp.DriveConstants.BackLeftModuleConstants.driveID;
                    angleID = Comp.DriveConstants.BackLeftModuleConstants.angleID;
                    angleOffset = Comp.DriveConstants.BackLeftModuleConstants.angleOffset;
                    inverted = Comp.DriveConstants.BackLeftModuleConstants.inverted;
                }
            }
        }
    }

    public static class SwerveConstants {
        public static final boolean invertGyro;
        public static final double KS;
        public static final double KA;
        public static final double KV;
        public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR;
        public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR;
        public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR;
        public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR;
        public static final double TURN_PID_MIN_INPUT;
        public static final double TURN_PID_MAX_INPUT;
        public static final double DRIVE_PID_MIN_OUTPUT;
        public static final double DRIVE_PID_MAX_OUTPUT;
        public static final double DRIVE_PID_P;
        public static final double DRIVE_PID_I;
        public static final double DRIVE_PID_D;
        public static final double DRIVE_PID_FF;
        public static final double TURN_PID_MIN_OUTPUT;
        public static final double TURN_PID_MAX_OUTPUT;
        public static final double TURN_PID_P;
        public static final double TURN_PID_I;
        public static final double TURN_PID_D;
        public static final double TURN_PID_FF;

        static {
            if (getRobot() == RobotType.WAFFLE) {
                invertGyro = Waffle.SwerveConstants.invertGyro;
                KS = Waffle.SwerveConstants.KS;
                KA = Waffle.SwerveConstants.KA;
                KV = Waffle.SwerveConstants.KV;
                DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = Waffle.SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR;
                DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = Waffle.SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR;
                TURNING_ENCODER_POSITION_CONVERSION_FACTOR = Waffle.SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR;
                TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = Waffle.SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR;
                TURN_PID_MIN_INPUT = Waffle.SwerveConstants.TURN_PID_MIN_INPUT;
                TURN_PID_MAX_INPUT = Waffle.SwerveConstants.TURN_PID_MAX_INPUT;
                DRIVE_PID_MIN_OUTPUT = Waffle.SwerveConstants.DRIVE_PID_MIN_OUTPUT;
                DRIVE_PID_MAX_OUTPUT = Waffle.SwerveConstants.DRIVE_PID_MAX_OUTPUT;
                DRIVE_PID_P = Waffle.SwerveConstants.DRIVE_PID_P;
                DRIVE_PID_I = Waffle.SwerveConstants.DRIVE_PID_I;
                DRIVE_PID_D = Waffle.SwerveConstants.DRIVE_PID_D;
                DRIVE_PID_FF = Waffle.SwerveConstants.DRIVE_PID_FF;
                TURN_PID_MIN_OUTPUT = Waffle.SwerveConstants.TURN_PID_MIN_OUTPUT;
                TURN_PID_MAX_OUTPUT = Waffle.SwerveConstants.TURN_PID_MAX_OUTPUT;
                TURN_PID_P = Waffle.SwerveConstants.TURN_PID_P;
                TURN_PID_I = Waffle.SwerveConstants.TURN_PID_I;
                TURN_PID_D = Waffle.SwerveConstants.TURN_PID_D;
                TURN_PID_FF = Waffle.SwerveConstants.TURN_PID_FF;
            } else {
                invertGyro = Comp.SwerveConstants.invertGyro;
                KS = Comp.SwerveConstants.KS;
                KA = Comp.SwerveConstants.KA;
                KV = Comp.SwerveConstants.KV;
                DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = Comp.SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR;
                DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = Comp.SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR;
                TURNING_ENCODER_POSITION_CONVERSION_FACTOR = Comp.SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR;
                TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = Comp.SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR;
                TURN_PID_MIN_INPUT = Comp.SwerveConstants.TURN_PID_MIN_INPUT;
                TURN_PID_MAX_INPUT = Comp.SwerveConstants.TURN_PID_MAX_INPUT;
                DRIVE_PID_MIN_OUTPUT = Comp.SwerveConstants.DRIVE_PID_MIN_OUTPUT;
                DRIVE_PID_MAX_OUTPUT = Comp.SwerveConstants.DRIVE_PID_MAX_OUTPUT;
                DRIVE_PID_P = Comp.SwerveConstants.DRIVE_PID_P;
                DRIVE_PID_I = Comp.SwerveConstants.DRIVE_PID_I;
                DRIVE_PID_D = Comp.SwerveConstants.DRIVE_PID_D;
                DRIVE_PID_FF = Comp.SwerveConstants.DRIVE_PID_FF;
                TURN_PID_MIN_OUTPUT = Comp.SwerveConstants.TURN_PID_MIN_OUTPUT;
                TURN_PID_MAX_OUTPUT = Comp.SwerveConstants.TURN_PID_MAX_OUTPUT;
                TURN_PID_P = Comp.SwerveConstants.TURN_PID_P;
                TURN_PID_I = Comp.SwerveConstants.TURN_PID_I;
                TURN_PID_D = Comp.SwerveConstants.TURN_PID_D;
                TURN_PID_FF = Comp.SwerveConstants.TURN_PID_FF;
            }
        }
    }
}
