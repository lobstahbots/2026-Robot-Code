package frc.robot.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.lobstahbots.units.*
import com.reduxrobotics.sensors.canandmag.Canandmag
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.RobotConstants

class SwerveModuleIOThrifty(
    angleMotorID: Int,
    driveMotorID: Int,
    encoderID: Int,
    angularOffsetDegrees: Double,
    inverted: Boolean
) : SwerveModuleIO {
    private val angleMotor = TalonFX(angleMotorID)
    private val driveMotor = TalonFX(driveMotorID)
    private val absoluteEncoder = Canandmag(encoderID)
    private val angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees)
    private val driveVelocity: StatusSignal<AngularVelocity> = driveMotor.velocity
    private val drivePosition: StatusSignal<Angle> = driveMotor.position
    private val driveOutputVoltage: StatusSignal<Voltage> = driveMotor.motorVoltage
    private val driveCurrent: StatusSignal<Current> = driveMotor.statorCurrent
    private val driveSupply: StatusSignal<Current> = driveMotor.supplyCurrent
    private val driveTemperature: StatusSignal<Temperature> = driveMotor.deviceTemp
    private val angleVelocity: StatusSignal<AngularVelocity> = angleMotor.velocity
    private val anglePosition: StatusSignal<Angle> = angleMotor.position
    private val angleOutputVoltage: StatusSignal<Voltage> = angleMotor.motorVoltage
    private val angleCurrent: StatusSignal<Current> = angleMotor.statorCurrent
    private val angleSupply: StatusSignal<Current> = angleMotor.supplyCurrent
    private val angleTemperature: StatusSignal<Temperature> = angleMotor.deviceTemp

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            200.hertz,
            driveVelocity,
            drivePosition,
            angleVelocity,
            anglePosition,
            driveOutputVoltage,
            angleOutputVoltage
        )
        BaseStatusSignal.setUpdateFrequencyForAll(20.hertz, driveCurrent, driveSupply, angleCurrent, angleSupply)
        BaseStatusSignal.setUpdateFrequencyForAll(4.hertz, driveTemperature, angleTemperature)

        val driveMotorConfig = TalonFXConfiguration().withCurrentLimits(
            CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
                .withStatorCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT * 3.0)
        ).withFeedback(FeedbackConfigs().withSensorToMechanismRatio(RobotConstants.DRIVE_GEAR_RATIO)).withMotorOutput(
            MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        )
        driveMotor.configurator.apply(driveMotorConfig)

        val angleMotorConfig = TalonFXConfiguration().withCurrentLimits(
            CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT)
                .withStatorCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT * 3.0)
        ).withMotorOutput(
            MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(if (inverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive)
        )
        angleMotor.configurator.apply(angleMotorConfig)

        driveMotor.setPosition(0.0)
    }

    /**
     * Sets the braking mode of the driving motor.
     *
     * @param mode the [IdleMode] to set motors to.
     */
    override fun setDriveIdleMode(mode: IdleMode) {
        driveMotor.configurator.apply(
            MotorOutputConfigs()
                .withNeutralMode(if (mode == IdleMode.kBrake) NeutralModeValue.Brake else NeutralModeValue.Coast)
        )
    }

    /**
     * Sets the braking mode of the turning motor.
     *
     * @param mode the [IdleMode] to set motors to.
     */
    override fun setTurnIdleMode(mode: IdleMode) {
        angleMotor.configurator.apply(
            MotorOutputConfigs()
                .withNeutralMode(if (mode == IdleMode.kBrake) NeutralModeValue.Brake else NeutralModeValue.Coast)
        )
    }

    override fun setDriveVoltage(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    override fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    override fun updateInputs(inputs: SwerveModuleIO.ModuleIOInputs) {
        BaseStatusSignal.refreshAll(
            driveVelocity,
            drivePosition,
            driveOutputVoltage,
            driveCurrent,
            driveSupply,
            driveTemperature,
            angleVelocity,
            anglePosition,
            angleOutputVoltage,
            angleCurrent,
            angleSupply,
            angleTemperature
        )

        inputs.drivePosition = drivePosition.value
        inputs.driveVelocity = driveVelocity.value
        inputs.driveAppliedVoltage = driveOutputVoltage.value
        inputs.driveStatorCurrent = driveCurrent.value
        inputs.driveSupplyCurrent = driveSupply.value
        inputs.driveTemperature = driveTemperature.value

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(absoluteEncoder.absPosition).minus(angularOffset)
        inputs.turnPosition = inputs.turnAbsolutePosition
        inputs.turnVelocity = angleVelocity.value
        inputs.turnAppliedVoltage = angleOutputVoltage.value
        inputs.turnCurrent = angleCurrent.value
        inputs.angularOffset = angularOffset
        inputs.turnTemperature = angleTemperature.value
    }
}