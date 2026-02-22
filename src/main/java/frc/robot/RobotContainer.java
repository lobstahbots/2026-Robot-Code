// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.Constants.Comp;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMode;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.IOConstants.ControllerIOConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOCanandgyro;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.SwerveModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSimBasic;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.util.auto.AutonSelector;
import frc.robot.util.auto.AutonSelector.AutoQuestion;
import frc.robot.util.led.LEDs;

public class RobotContainer {
    private final LEDs leds;

    private final DriveBase driveBase;
    private final Intake intake;

    private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
            () -> Commands.none());
    private final AutoFactory autoFactory;

    private SwerveDriveSimulation driveSimulation = null;

    //private int scoreLevel = 1;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        leds = new LEDs();
        if (Constants.getRobot() == Constants.RobotType.WAFFLE && Constants.getMode() == Constants.RobotMode.REAL) {
            SwerveModuleIOSparkMax frontLeft = new SwerveModuleIOSparkMax(FrontLeftModuleConstants.moduleID,
                    "Front left ", FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID,
                    FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax frontRight = new SwerveModuleIOSparkMax(FrontRightModuleConstants.moduleID,
                    "Front right", FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID,
                    FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
            SwerveModuleIOSparkMax backLeft = new SwerveModuleIOSparkMax(BackLeftModuleConstants.moduleID, "Back left",
                    BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID,
                    BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax backRight = new SwerveModuleIOSparkMax(BackRightModuleConstants.moduleID,
                    "Back right", BackRightModuleConstants.angleID, BackRightModuleConstants.driveID,
                    BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);

            List<Camera> cameras = VisionConstants.WAFFLE_CAMERA_TRANSFORMS.keySet().stream()
                    .map(name -> new Camera(new CameraIOPhoton(name))).toList();
            driveBase = new DriveBase(new GyroIONavX(), cameras, frontLeft, frontRight, backLeft, backRight, false);
            intake = new Intake(new IntakeIO() {});
        } else if (Constants.getRobot() == Constants.RobotType.COMP
                && Constants.getMode() == Constants.RobotMode.REAL) {
            SwerveModuleIOTalonFX frontLeft = new SwerveModuleIOTalonFX(FrontLeftModuleConstants.moduleID,
                    "Front left ", FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID,
                    FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
            SwerveModuleIOTalonFX frontRight = new SwerveModuleIOTalonFX(FrontRightModuleConstants.moduleID,
                    "Front right", FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID,
                    FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
            SwerveModuleIOTalonFX backLeft = new SwerveModuleIOTalonFX(BackLeftModuleConstants.moduleID, "Back left",
                    BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID,
                    BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
            SwerveModuleIOTalonFX backRight = new SwerveModuleIOTalonFX(BackRightModuleConstants.moduleID, "Back right",
                    BackRightModuleConstants.angleID, BackRightModuleConstants.driveID,
                    BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);

            List<Camera> cameras = VisionConstants.COMP_CAMERA_TRANSFORMS.keySet().stream()
                    .map(name -> new Camera(new CameraIOPhoton(name))).toList();
            driveBase = new DriveBase(new GyroIOCanandgyro(Comp.RobotConstants.GYRO_ID), cameras, frontLeft, frontRight,
                    backLeft, backRight, false);
            intake = new Intake(new IntakeIOSparkMax(IntakeConstants.ARM_ID, IntakeConstants.ROLLER_ID));
        } else if (!(Constants.getMode() == RobotMode.REPLAY)) {
            driveSimulation = new SwerveDriveSimulation(DriveConstants.MAPLE_SIM_CONFIG,
                    new Pose2d(3, 3, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

            var modules = driveSimulation.getModules();
            SwerveModuleIOSim frontLeft = new SwerveModuleIOSim(FrontLeftModuleConstants.angleOffset, modules[0], 0);
            SwerveModuleIOSim frontRight = new SwerveModuleIOSim(FrontRightModuleConstants.angleOffset, modules[1], 1);
            SwerveModuleIOSim backLeft = new SwerveModuleIOSim(BackLeftModuleConstants.angleOffset, modules[2], 2);
            SwerveModuleIOSim backRight = new SwerveModuleIOSim(BackRightModuleConstants.angleOffset, modules[3], 3);

            List<Camera> cameras;
            if (SimConstants.VISION_SIM) {
                cameras = VisionConstants.COMP_CAMERA_TRANSFORMS.keySet().stream()
                        .map(name -> new Camera(new CameraIOSim(name))).toList();
            } else {
                cameras = new ArrayList<>();
            }
            driveBase = new DriveBase(new GyroIOSim(driveSimulation.getGyroSimulation()), cameras, frontLeft,
                    frontRight, backLeft, backRight, false);

            if (Constants.getRobot() == RobotType.SIM_BASIC) {
                intake = new Intake(new IntakeIOSimBasic());
            } else {
                intake = new Intake(new IntakeIO() {});
            }
        } else {
            driveBase = new DriveBase(new GyroIO() {}, List.of(), new SwerveModuleIO() {}, new SwerveModuleIO() {},
                    new SwerveModuleIO() {}, new SwerveModuleIO() {}, false);
            intake = new Intake(new IntakeIO() {});
        }

        this.autoFactory = new AutoFactory(driveBase, autoChooser::getResponses, (Pose2d newPose) -> {
            if (Robot.isSimulation()) driveSimulation.setSimulationWorldPose(newPose);
            driveBase.resetPose(newPose);
        });

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();

    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(
                driveBase.joystickDrive(() -> -Controllers.driver.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL),
                        () -> -Controllers.driver.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL),
                        () -> -Controllers.driver.getRawAxis(ControllerIOConstants.RIGHT_STICK_HORIZONTAL)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command command = autoChooser.getCommand();
        if (Robot.isSimulation())
            command = Commands.runOnce(SimulatedArena.getInstance()::resetFieldForAuto).andThen(command);
        return command;
    }

    public void configureButtonBindings() {}

    public boolean getOperatorConnected() {
        return Controllers.operator.isConnected();
    }

    public boolean getDriverConnected() {
        return Controllers.driver.isConnected();
    }

    public void smartDashSetup() {
        autoChooser.addRoutine("Leave", List.of(), autoFactory::getLeaveAuto);

        autoChooser.addRoutine("Characterize",
                List.of(new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase)),
                        new AutoQuestion<>("Which Routine",
                                Map.of("Quasistatic Forward", CharacterizationRoutine.QUASISTATIC_FORWARD,
                                        "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD,
                                        "Dynamic Forward", CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward",
                                        CharacterizationRoutine.DYNAMIC_BACKWARD))),
                autoFactory::getCharacterizationRoutine);
    }

    public void displaySimField() {
        if (Constants.getMode() != Constants.RobotMode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        CameraIOSim.addSimPose(new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    }

    public void setIdleMode(boolean isBrakeMode) {
        driveBase.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void periodic() {}
}
