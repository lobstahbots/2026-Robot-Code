package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;

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
    private final Encoder quadEncoder = new Encoder(0, 1);
    private Rotation2d offset = Rotation2d.kZero;
    private boolean useProfile = true;

    public ShooterIOSparkMax(int flywheelMotor1ID, int flywheelMotor2ID, int flywheelMotor3ID, int hoodMotorID) {
        this.flywheelMotor1 = new SparkMax(flywheelMotor1ID, MotorType.kBrushless);
        this.flywheelMotor2 = new SparkMax(flywheelMotor2ID, MotorType.kBrushless);
        this.flywheelMotor3 = new SparkMax(flywheelMotor3ID, MotorType.kBrushless);
        this.hoodMotor = new SparkMax(hoodMotorID, MotorType.kBrushless);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT).idleMode(IdleMode.kCoast)
                .inverted(true);
        flywheelConfig.encoder.positionConversionFactor(1 / ShooterConstants.FLYWHEEL_GEAR_RATIO)
                .velocityConversionFactor(1 / ShooterConstants.FLYWHEEL_GEAR_RATIO).quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(10);
        flywheelConfig.closedLoop
                .pid(ShooterConstants.FLYWHEEL_kP, ShooterConstants.FLYWHEEL_kI, ShooterConstants.FLYWHEEL_kD,
                        ClosedLoopSlot.kSlot0)
                .apply(new FeedForwardConfig().sva(ShooterConstants.FLYWHEEL_kS, ShooterConstants.FLYWHEEL_kV,
                        ShooterConstants.FLYWHEEL_kA, ClosedLoopSlot.kSlot0))
                .apply(new MAXMotionConfig().maxAcceleration(ShooterConstants.FLYWHEEL_MAX_ACCELERATION,
                        ClosedLoopSlot.kSlot0));
        flywheelConfig.closedLoop
                .pid(ShooterConstants.FLYWHEEL_kP, ShooterConstants.FLYWHEEL_kI, ShooterConstants.FLYWHEEL_kD,
                        ClosedLoopSlot.kSlot1)
                .apply(new FeedForwardConfig().sva(ShooterConstants.FLYWHEEL_kS, ShooterConstants.FLYWHEEL_kV,
                        ShooterConstants.FLYWHEEL_kA, ClosedLoopSlot.kSlot1))
                .apply(new MAXMotionConfig().maxAcceleration(ShooterConstants.FLYWHEEL_MAX_ACCELERATION * 10000,
                        ClosedLoopSlot.kSlot1));
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
                .velocityConversionFactor(1 / 60.0 / ShooterConstants.HOOD_GEAR_RATIO);
        hoodConfig.alternateEncoder.positionConversionFactor(1 / ShooterConstants.HERRINGBONE_RATIO)
                .velocityConversionFactor(1 / 60.0 / ShooterConstants.HERRINGBONE_RATIO);
        hoodConfig.closedLoop.pid(ShooterConstants.HOOD_kP, ShooterConstants.HOOD_kI, ShooterConstants.HOOD_kD)
                .apply(new FeedForwardConfig().svacr(ShooterConstants.HOOD_kS, ShooterConstants.HOOD_kV,
                        ShooterConstants.HOOD_kA, ShooterConstants.HOOD_kG, 1))
                .apply(new MAXMotionConfig().cruiseVelocity(ShooterConstants.HOOD_CRUISE_VELOCITY)
                        .maxAcceleration(ShooterConstants.HOOD_MAX_ACCELERATION)
                        .allowedProfileError(ShooterConstants.HOOD_ALLOWED_PROFILE_ERROR));
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        hoodEncoder = hoodMotor.getEncoder();
        hoodController = hoodMotor.getClosedLoopController();

        quadEncoder.setDistancePerPulse(1 / ShooterConstants.HERRINGBONE_RATIO / 2048);
    }

    public void setFlywheelVoltage(double voltage) {
        useProfile = true;
        flywheelMotor1.setVoltage(voltage);
    }

    public void setHoodVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        if (RPM.of(flywheelEncoder.getVelocity()).isNear(velocity, ShooterConstants.TOLERANCE))
            useProfile = false;
        if (!useProfile)
            flywheelController.setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl,
                    ClosedLoopSlot.kSlot1);
        else
            flywheelController.setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl,
                    ClosedLoopSlot.kSlot0);
    }

    public void setHoodPosition(Rotation2d position) {
        hoodController.setSetpoint(position.getRotations(), ControlType.kMAXMotionPositionControl);
    }

    public void resetEncoder(Rotation2d position) {
        hoodEncoder.setPosition(position.getRotations());
        quadEncoder.reset();
        offset = position;
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocity = RPM.of(flywheelEncoder.getVelocity());
        inputs.flywheelSetpoint = RPM.of(flywheelController.getMAXMotionSetpointVelocity());
        inputs.flywheelAppliedVoltages[0] = flywheelMotor1.getAppliedOutput() * flywheelMotor1.getBusVoltage();
        inputs.flywheelAppliedVoltages[1] = flywheelMotor2.getAppliedOutput() * flywheelMotor2.getBusVoltage();
        inputs.flywheelAppliedVoltages[2] = flywheelMotor3.getAppliedOutput() * flywheelMotor3.getBusVoltage();
        inputs.flywheelCurrents[0] = flywheelMotor1.getOutputCurrent();
        inputs.flywheelCurrents[1] = flywheelMotor2.getOutputCurrent();
        inputs.flywheelCurrents[2] = flywheelMotor3.getOutputCurrent();
        inputs.flywheelTemperatures[0] = flywheelMotor1.getMotorTemperature();
        inputs.flywheelTemperatures[1] = flywheelMotor2.getMotorTemperature();
        inputs.flywheelTemperatures[2] = flywheelMotor3.getMotorTemperature();

        inputs.hoodPosition = Rotation2d.fromRotations(hoodEncoder.getPosition());
        inputs.encoderPosition = Rotation2d.fromRotations(quadEncoder.getDistance()).plus(offset);
        inputs.hoodVelocity = RotationsPerSecond.of(hoodEncoder.getVelocity());
        inputs.hoodAppliedVoltage = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.hoodCurrent = hoodMotor.getOutputCurrent();
        inputs.hoodTemperature = hoodMotor.getMotorTemperature();

        inputs.useProfile = useProfile;
    }
}
