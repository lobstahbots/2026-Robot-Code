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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.IOConstants.ControllerIOConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.util.auto.AutonSelector;
import frc.robot.util.auto.AutonSelector.AutoQuestion;
import frc.robot.util.led.LEDs;

public class RobotContainer {
    private final LEDs leds;

    private final DriveBase driveBase;

    //sticks
    private final Joystick driverJoystick = new Joystick(ControllerIOConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick operatorJoystick = new Joystick(ControllerIOConstants.OPERATOR_CONTROLLER_PORT);

    //Driver
    private final Trigger driverLTButton = new Trigger(
            () -> driverJoystick.getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.2); //Outake Coral
    private final Trigger driverRTButton = new Trigger(
            () -> driverJoystick.getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.2); //Outake Algae

    private final JoystickButton driverLBButton = new JoystickButton(driverJoystick, ControllerIOConstants.LB_BUTTON); //Barge automation
    private final JoystickButton driverRBButton = new JoystickButton(driverJoystick, ControllerIOConstants.RB_BUTTON); //Intake Sequence

    private final JoystickButton driverLeftPaddle = new JoystickButton(driverJoystick,
            ControllerIOConstants.LEFT_PADDLE); //Autoalign left reef
    private final JoystickButton driverRightPaddle = new JoystickButton(driverJoystick,
            ControllerIOConstants.RIGHT_PADDLE); //Autoalign right reef

    //Operator
    private final Trigger operatorLTButton = new Trigger(
            () -> operatorJoystick.getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.5); //Outake Coral
    private final Trigger operatorRTButton = new Trigger(
            () -> operatorJoystick.getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.5); //Intake Coral

    private final JoystickButton operatorLBButton = new JoystickButton(operatorJoystick,
            ControllerIOConstants.LB_BUTTON); //Outake Algae
    private final JoystickButton operatorRBButton = new JoystickButton(operatorJoystick,
            ControllerIOConstants.RB_BUTTON); //L1 Setpoint (potentially to change)

    private final JoystickButton operatorXButton = new JoystickButton(operatorJoystick, ControllerIOConstants.X_BUTTON); //L2
    private final JoystickButton operatorYButton = new JoystickButton(operatorJoystick, ControllerIOConstants.Y_BUTTON); //L3
    private final JoystickButton operatorBButton = new JoystickButton(operatorJoystick, ControllerIOConstants.B_BUTTON); //L4
    private final JoystickButton operatorAButton = new JoystickButton(operatorJoystick, ControllerIOConstants.A_BUTTON); //Theoretically L1 or intake

    private final JoystickButton operatorLeftPaddle = new JoystickButton(operatorJoystick,
            ControllerIOConstants.LEFT_PADDLE);
    private final JoystickButton operatorRightPaddle = new JoystickButton(operatorJoystick,
            ControllerIOConstants.RIGHT_PADDLE);

    private final POVButton operatorDpadUp = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_UP); // L3 Algae Removal
    private final POVButton operatorDpadDown = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_DOWN); // L2 Algae Removal
    private final POVButton operatorDpadLeft = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_LEFT); // Barge Setpoint
    private final POVButton operatorDpadRight = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_RIGHT); // Processor Setpoint


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
        if (Robot.isReal()) {
            SwerveModuleIOSparkMax frontLeft = new SwerveModuleIOSparkMax(FrontLeftModuleConstants.moduleID,
                    "Front left ", FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID,
                    FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax frontRight = new SwerveModuleIOSparkMax(FrontRightModuleConstants.moduleID,
                    " Front right", FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID,
                    FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
            SwerveModuleIOSparkMax backLeft = new SwerveModuleIOSparkMax(BackLeftModuleConstants.moduleID, " Back left",
                    BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID,
                    BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax backRight = new SwerveModuleIOSparkMax(BackRightModuleConstants.moduleID,
                    "Back right", BackRightModuleConstants.angleID, BackRightModuleConstants.driveID,
                    BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);

            List<Camera> cameras = VisionConstants.CAMERA_TRANSFORMS.keySet().stream()
                    .map(name -> new Camera(new CameraIOPhoton(name))).toList();
            driveBase = new DriveBase(new GyroIONavX(), cameras, frontLeft, frontRight, backLeft, backRight, false);
        } else if (!SimConstants.REPLAY) {
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
                cameras = VisionConstants.CAMERA_TRANSFORMS.keySet().stream()
                        .map(name -> new Camera(new CameraIOSim(name))).toList();
            } else {
                cameras = new ArrayList<>();
            }
            driveBase = new DriveBase(new GyroIOSim(driveSimulation.getGyroSimulation()), cameras, frontLeft,
                    frontRight, backLeft, backRight, false);
        } else {
            driveBase = new DriveBase(new GyroIO() {}, List.of(), new SwerveModuleIO() {}, new SwerveModuleIO() {},
                    new SwerveModuleIO() {}, new SwerveModuleIO() {}, false);
        }

        this.autoFactory = new AutoFactory(driveBase, autoChooser::getResponses,
                (Pose2d newPose) -> {
                    if (Robot.isSimulation()) driveSimulation.setSimulationWorldPose(newPose);
                    driveBase.resetPose(newPose);
                });

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();

    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(
                driveBase.joystickDrive(() -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL),
                        () -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL),
                        () -> -driverJoystick.getRawAxis(ControllerIOConstants.RIGHT_STICK_HORIZONTAL)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command command = autoChooser.getCommand();
        if (Robot.isSimulation()) command = Commands.runOnce(SimulatedArena.getInstance()::resetFieldForAuto).andThen(command);
        return command;
    }

    public void configureButtonBindings() {
    }

    public boolean getOperatorConnected() {
        return operatorJoystick.isConnected();
    }

    public boolean getDriverConnected() {
        return driverJoystick.isConnected();
    }

    public void smartDashSetup() {
        autoChooser.addRoutine("Leave", List.of(), autoFactory::getLeaveAuto);

        autoChooser.addRoutine("Characterize", List.of(
                new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase)),
                new AutoQuestion<>("Which Routine",
                        Map.of("Quasistatic Forward", CharacterizationRoutine.QUASISTATIC_FORWARD,
                                "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD, "Dynamic Forward",
                                CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward",
                                CharacterizationRoutine.DYNAMIC_BACKWARD))),
                autoFactory::getCharacterizationRoutine);
    }

    public void displaySimField() {
        if (Robot.isReal() || SimConstants.REPLAY) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        CameraIOSim.addSimPose(new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    }

    public void setIdleMode(boolean isBrakeMode) {
        driveBase.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void periodic() {}
}
