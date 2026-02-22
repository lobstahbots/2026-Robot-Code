package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSparkMax implements ShooterIO {
    private final SparkMax flywheelMotor1;
    private final SparkMax flywheelMotor2;
    private final SparkMax flywheelMotor3;
    private final SparkMax hoodMotor;
    private final SparkClosedLoopController flywheelController;
    private final SparkClosedLoopController hoodController;
    private final RelativeEncoder flywheelEncoder;
    private final RelativeEncoder hoodEncoder;

    public ShooterIOSparkMax(int flywheelMotor1ID, int flywheelMotor2ID, int flywheelMotor3ID, int hoodMotorID) {
        this.flywheelMotor1 = new SparkMax(flywheelMotor1ID, MotorType.kBrushless);
        this.flywheelMotor2 = new SparkMax(flywheelMotor2ID, MotorType.kBrushless);
        this.flywheelMotor3 = new SparkMax(flywheelMotor3ID, MotorType.kBrushless);
        this.hoodMotor = new SparkMax(hoodMotorID, MotorType.kBrushless);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT).idleMode(IdleMode.kCoast)
                .inverted(false);
        flywheelConfig.encoder.positionConversionFactor(1 / ShooterConstants.FLYWHEEL_GEAR_RATIO)
                .velocityConversionFactor(60 / ShooterConstants.FLYWHEEL_GEAR_RATIO).quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(10);
        flywheelConfig.closedLoop
                .pid(ShooterConstants.FLYWHEEL_kP, ShooterConstants.FLYWHEEL_kI, ShooterConstants.FLYWHEEL_kD)
                .apply(new FeedForwardConfig().sva(ShooterConstants.FLYWHEEL_kS, ShooterConstants.FLYWHEEL_kV,
                        ShooterConstants.FLYWHEEL_kA))
                .apply(new MAXMotionConfig().maxAcceleration(ShooterConstants.FLYWHEEL_MAX_ACCELERATION));
        flywheelMotor1.configure(flywheelConfig, ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        flywheelConfig.follow(flywheelMotor1);
        flywheelMotor2.configure(flywheelConfig, ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        flywheelMotor3.configure(flywheelConfig, ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        flywheelEncoder = flywheelMotor1.getEncoder();
        flywheelController = flywheelMotor1.getClosedLoopController();

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.smartCurrentLimit(ShooterConstants.HOOD_CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(false);
        hoodConfig.encoder.positionConversionFactor(1 / ShooterConstants.HOOD_GEAR_RATIO)
                .velocityConversionFactor(60 / ShooterConstants.HOOD_GEAR_RATIO);
        hoodConfig.alternateEncoder.positionConversionFactor(1 / ShooterConstants.HERRINGBONE_RATIO)
                .velocityConversionFactor(60 / ShooterConstants.HERRINGBONE_RATIO);
        hoodConfig.closedLoop.pid(ShooterConstants.HOOD_kP, ShooterConstants.HOOD_kI, ShooterConstants.HOOD_kD)
                .apply(new FeedForwardConfig().svacr(ShooterConstants.HOOD_kS, ShooterConstants.HOOD_kV,
                        ShooterConstants.HOOD_kA, ShooterConstants.HOOD_kG, 1))
                .apply(new MAXMotionConfig().cruiseVelocity(ShooterConstants.HOOD_CRUISE_VELOCITY)
                        .maxAcceleration(ShooterConstants.HOOD_MAX_ACCELERATION)
                        .allowedProfileError(ShooterConstants.HOOD_ALLOWED_PROFILE_ERROR));
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        hoodEncoder = hoodMotor.getEncoder();
        hoodController = hoodMotor.getClosedLoopController();
    }

    public void setFlywheelVoltage(double voltage) {
        flywheelMotor1.setVoltage(voltage);
    }

    public void setHoodVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelController.setSetpoint(velocity.in(RotationsPerSecond), ControlType.kMAXMotionVelocityControl);
    }

    public void setHoodPosition(Rotation2d position) {
        hoodController.setSetpoint(position.getRotations(), ControlType.kMAXMotionPositionControl);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocity = RotationsPerSecond.of(flywheelEncoder.getVelocity());
        inputs.flywheelAppliedVoltages[0] = flywheelMotor1.getAppliedOutput() * 12;
        inputs.flywheelAppliedVoltages[1] = flywheelMotor2.getAppliedOutput() * 12;
        inputs.flywheelAppliedVoltages[2] = flywheelMotor3.getAppliedOutput() * 12;
        inputs.flywheelCurrents[0] = flywheelMotor1.getOutputCurrent();
        inputs.flywheelCurrents[1] = flywheelMotor2.getOutputCurrent();
        inputs.flywheelCurrents[2] = flywheelMotor3.getOutputCurrent();
        inputs.flywheelTemperatures[0] = flywheelMotor1.getMotorTemperature();
        inputs.flywheelTemperatures[1] = flywheelMotor2.getMotorTemperature();
        inputs.flywheelTemperatures[2] = flywheelMotor3.getMotorTemperature();

        inputs.hoodPosition = Rotation2d.fromRotations(hoodEncoder.getPosition());
        inputs.hoodVelocity = RotationsPerSecond.of(hoodEncoder.getVelocity());
        inputs.hoodAppliedVoltage = hoodMotor.getAppliedOutput() * 12;
        inputs.hoodCurrent = hoodMotor.getOutputCurrent();
        inputs.hoodTemperature = hoodMotor.getMotorTemperature();
    }
}
