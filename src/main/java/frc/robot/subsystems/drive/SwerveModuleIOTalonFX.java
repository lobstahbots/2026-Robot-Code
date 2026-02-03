// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.tempControl.MonitoredSparkMax;
import frc.robot.util.tempControl.TemperatureMonitor;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final MonitoredSparkMax angleMotor;
    private final TalonFX driveMotor;
    private final AbsoluteEncoder angleAbsoluteEncoder;
    private Rotation2d angularOffset;
    private final int moduleID;
    private final TemperatureMonitor monitor;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Voltage> driveVoltage;
    private final StatusSignal<Current> driveCurrent;

    /**
     * Creates a new SwerveModule for real cases.
     * 
     * @param moduleID             The module number (0-3).
     * @param angleMotorID         The CAN ID of the motor controlling the angle.
     * @param driveMotorID         The CAN ID of the motor controlling drive speed.
     * @param angularOffsetDegrees The offset angle in degrees.
     */
    public SwerveModuleIOTalonFX(int moduleID, String name, int angleMotorID, int driveMotorID,
            double angularOffsetDegrees, boolean inverted) {
        this.moduleID = moduleID;

        this.angleMotor = new MonitoredSparkMax(angleMotorID, MotorType.kBrushless, name + " angle motor");
        this.driveMotor = new TalonFX(driveMotorID);

        driveVelocity = driveMotor.getVelocity();
        drivePosition = driveMotor.getPosition();
        driveVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();

        var driveMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
                                .withStatorCurrentLimit(3 * DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(RobotConstants.DRIVE_GEAR_RATIO))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        driveMotor.getConfigurator().apply(driveMotorConfig);

        var angleMotorConfig = new SparkMaxConfig().smartCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake).voltageCompensation(12).inverted(inverted);
        angleMotorConfig.absoluteEncoder
                .positionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder();

        monitor = new TemperatureMonitor(Arrays.asList(angleMotor));

        this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
        driveMotor.setPosition(0);
        resetEncoders();
    }

    /** Stops the module motors. */
    public void stopMotors() {
        angleMotor.stopMotor();
        driveMotor.stopMotor();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current encoder velocities of the module as a
     *         {@link SwerveModuleState}.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity.getValueAsDouble() * RobotConstants.WHEEL_DIAMETER * Math.PI,
                new Rotation2d(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians()));
    }

    /**
     * Returns the module ID.
     *
     * @return The ID number of the module (0-3).
     */
    public int getModuleID() {
        return moduleID;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current encoder positions position of the module as a
     *         {@link SwerveModulePosition}.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition.getValueAsDouble(),
                new Rotation2d(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians()));
    }

    /**
     * Sets the braking mode of the driving motor.
     * 
     * @param mode the {@link IdleMode} to set motors to.
     */
    public void setDriveIdleMode(IdleMode mode) {
        driveMotor.getConfigurator().apply(new MotorOutputConfigs()
                .withNeutralMode(mode == IdleMode.kBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    }

    /**
     * Sets the braking mode of the turning motor.
     * 
     * @param mode the {@link IdleMode} to set motors to.
     */
    public void setTurnIdleMode(IdleMode mode) {
        angleMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /** Zeroes the drive encoder. */
    public void resetEncoders() {
        driveMotor.setPosition(0);
    }

    /**
     * Sets voltage of driving motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    /**
     * Sets voltage of turn motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    public void setTurnVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePosition = new Rotation2d(drivePosition.getValue());
        inputs.driveVelocityRadPerSec = driveVelocity.getValue().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        inputs.turnAbsolutePosition = angleMotor.getLastError() == REVLibError.kOk
                ? Rotation2d.fromRadians(-angleAbsoluteEncoder.getPosition() - angularOffset.getRadians())
                : inputs.turnPosition;
        inputs.turnPosition = inputs.turnAbsolutePosition;
        inputs.turnVelocityRadPerSec = angleMotor.getLastError() == REVLibError.kOk ? angleAbsoluteEncoder.getVelocity()
                : inputs.turnVelocityRadPerSec;
        inputs.turnAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.turnCurrentAmps = angleMotor.getOutputCurrent();
        inputs.angularOffset = angularOffset;
    }

    public void periodic() {
        monitor.monitor();
        drivePosition.refresh();
        driveVelocity.refresh();
        driveCurrent.refresh();
        driveVoltage.refresh();
    }
}
