package frc.robot.subsystems.shooter

import com.revrobotics.PersistMode
import com.revrobotics.RelativeEncoder
import com.revrobotics.ResetMode
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.FeedForwardConfig
import com.revrobotics.spark.config.MAXMotionConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Encoder
import frc.robot.Constants.*
import com.lobstahbots.units.*
import com.revrobotics.spark.SparkBase.ControlType
import edu.wpi.first.units.Units.RPM

class ShooterIOSparkMax(flywheelMotor1ID: Int, flywheelMotor2ID: Int, flywheelMotor3ID: Int, hoodMotorID: Int) :
    ShooterIO {
    var offset: Rotation2d = Rotation2d.kZero
    var useProfile = true
    val flywheelMotor1: SparkMax = SparkMax(flywheelMotor1ID, MotorType.kBrushless)
    val flywheelMotor2: SparkMax = SparkMax(flywheelMotor2ID, MotorType.kBrushless)
    val flywheelMotor3: SparkMax = SparkMax(flywheelMotor3ID, MotorType.kBrushless)
    val hoodMotor: SparkMax = SparkMax(hoodMotorID, MotorType.kBrushless)
    val flywheelEncoder: RelativeEncoder
    val flywheelController: SparkClosedLoopController
    val hoodEncoder: RelativeEncoder
    val hoodController: SparkClosedLoopController
    val quadEncoder = Encoder(0, 1)

    init {
        val flywheelConfig = SparkMaxConfig()
        flywheelConfig.smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT).idleMode(IdleMode.kCoast)
            .inverted(true)
        flywheelConfig.encoder.positionConversionFactor(1 / ShooterConstants.FLYWHEEL_GEAR_RATIO)
            .velocityConversionFactor(1 / ShooterConstants.FLYWHEEL_GEAR_RATIO).quadratureAverageDepth(10)
            .quadratureMeasurementPeriod(10).uvwAverageDepth(5).uvwMeasurementPeriod(8)
        flywheelConfig.closedLoop.pid(
            ShooterConstants.FLYWHEEL_kP,
            ShooterConstants.FLYWHEEL_kI,
            ShooterConstants.FLYWHEEL_kD,
            ClosedLoopSlot.kSlot0
        ).apply(
            FeedForwardConfig().sva(
                ShooterConstants.FLYWHEEL_kS,
                ShooterConstants.FLYWHEEL_kV,
                ShooterConstants.FLYWHEEL_kA,
                ClosedLoopSlot.kSlot0
            )
        ).apply(
            MAXMotionConfig().maxAcceleration(
                ShooterConstants.FLYWHEEL_MAX_ACCELERATION, ClosedLoopSlot.kSlot0
            )
        )
        flywheelConfig.closedLoop.pid(
            ShooterConstants.FLYWHEEL_kP,
            ShooterConstants.FLYWHEEL_kI,
            ShooterConstants.FLYWHEEL_kD,
            ClosedLoopSlot.kSlot1
        ).apply(
            FeedForwardConfig().sva(
                ShooterConstants.FLYWHEEL_kS,
                ShooterConstants.FLYWHEEL_kV,
                ShooterConstants.FLYWHEEL_kA,
                ClosedLoopSlot.kSlot1
            )
        ).apply(
            MAXMotionConfig().maxAcceleration(
                ShooterConstants.FLYWHEEL_MAX_ACCELERATION * 10000, ClosedLoopSlot.kSlot1
            )
        )
        flywheelMotor1.configure(
            flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )
        flywheelConfig.follow(flywheelMotor1)
        flywheelMotor2.configure(
            flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )
        flywheelMotor3.configure(
            flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )
        flywheelEncoder = flywheelMotor1.encoder
        flywheelController = flywheelMotor1.closedLoopController

        val hoodConfig = SparkMaxConfig()
        hoodConfig.smartCurrentLimit(ShooterConstants.HOOD_CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(false)
        hoodConfig.encoder.positionConversionFactor(1 / ShooterConstants.HOOD_GEAR_RATIO)
            .velocityConversionFactor(1 / 60.0 / ShooterConstants.HOOD_GEAR_RATIO)
        hoodConfig.alternateEncoder.positionConversionFactor(1 / ShooterConstants.HERRINGBONE_RATIO)
            .velocityConversionFactor(1 / 60.0 / ShooterConstants.HERRINGBONE_RATIO)
        hoodConfig.closedLoop.pid(ShooterConstants.HOOD_kP, ShooterConstants.HOOD_kI, ShooterConstants.HOOD_kD).apply(
            FeedForwardConfig().svacr(
                ShooterConstants.HOOD_kS,
                ShooterConstants.HOOD_kV,
                ShooterConstants.HOOD_kA,
                ShooterConstants.HOOD_kG,
                1.0
            )
        ).apply(
            MAXMotionConfig().cruiseVelocity(ShooterConstants.HOOD_CRUISE_VELOCITY)
                .maxAcceleration(ShooterConstants.HOOD_MAX_ACCELERATION)
                .allowedProfileError(ShooterConstants.HOOD_ALLOWED_PROFILE_ERROR)
        )
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        hoodEncoder = hoodMotor.encoder
        hoodController = hoodMotor.closedLoopController

        quadEncoder.distancePerPulse = 1 / ShooterConstants.HERRINGBONE_RATIO / 2048
    }

    override fun setFlywheelVoltage(voltage: Double) {
        flywheelMotor1.setVoltage(voltage)
    }

    override fun setHoodVoltage(voltage: Double) {
        hoodMotor.setVoltage(voltage)
    }

    override fun setFlywheelVelocity(velocity: AngularVelocity) {
        if (flywheelEncoder.velocity.rpm.isNear(velocity, ShooterConstants.TOLERANCE)) useProfile = false
        if (!useProfile) flywheelController.setSetpoint(
            velocity.`in`(RPM), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1
        )
        else flywheelController.setSetpoint(
            velocity.`in`(RPM), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0
        )
    }

    override fun setHoodPosition(position: Rotation2d) {
        hoodController.setSetpoint(position.rotations, ControlType.kMAXMotionPositionControl)
    }

    override fun resetEncoder(position: Rotation2d) {
        hoodEncoder.position = position.rotations
        quadEncoder.reset()
        offset = position
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.flywheelVelocity = flywheelEncoder.velocity.rpm
        inputs.flywheelSetpoint = flywheelController.setpoint.rpm
        inputs.flywheelAppliedVoltages[0] = flywheelMotor1.appliedOutput * flywheelMotor1.busVoltage
        inputs.flywheelAppliedVoltages[1] = flywheelMotor2.appliedOutput * flywheelMotor2.busVoltage
        inputs.flywheelAppliedVoltages[2] = flywheelMotor3.appliedOutput * flywheelMotor3.busVoltage
        inputs.flywheelCurrents[0] = flywheelMotor1.outputCurrent
        inputs.flywheelCurrents[1] = flywheelMotor2.outputCurrent
        inputs.flywheelCurrents[2] = flywheelMotor3.outputCurrent
        inputs.flywheelTemperatures[0] = flywheelMotor1.motorTemperature
        inputs.flywheelTemperatures[1] = flywheelMotor2.motorTemperature
        inputs.flywheelTemperatures[2] = flywheelMotor3.motorTemperature

        inputs.hoodPosition = Rotation2d.fromRotations(hoodEncoder.position)
        inputs.encoderPosition = Rotation2d.fromRotations(quadEncoder.distance + offset.rotations)
        inputs.hoodAppliedVoltage = hoodMotor.appliedOutput * hoodMotor.busVoltage
        inputs.hoodCurrent = hoodMotor.outputCurrent
        inputs.hoodTemperature = hoodMotor.motorTemperature
        inputs.hoodVelocity = hoodEncoder.velocity.rpm

        inputs.useProfile = useProfile
    }
}