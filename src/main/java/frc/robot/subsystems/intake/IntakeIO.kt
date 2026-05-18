package frc.robot.subsystems.intake

import com.lobstahbots.junction.AutoLogKt
import edu.wpi.first.math.geometry.Rotation2d
import com.lobstahbots.units.*

interface IntakeIO {
    @AutoLogKt
    open class IntakeIOInputs {
        /**
         * Velocity of intake arm in rotations/second
         */
        var armVelocity = 0.rotationsPerSecond

        /**
         * Voltage of intake arm motor in volts
         */
        var armAppliedVoltage = 0.volts

        /**
         * Current of intake arm motor in amperes
         */
        var armCurrent = 0.amps

        /**
         * Temperature of intake arm motor in celcius
         */
        var armTemp = 0.celsius

        /**
         * Position of intake arm in rotations
         */
        var armPosition: Rotation2d = Rotation2d.kZero

        /**
         * Velocity of roller motor in rotations/second
         */
        var rollerVelocity = 0.rotationsPerSecond

        /**
         * Voltage of roller motor in volts
         */
        var rollerAppliedVoltage = 0.volts

        /**
         * Current of roller motor in amperes
         */
        var rollerCurrent = 0.amps

        /**
         * Temperature of roller motor in Celsius
         */
        var rollerTemp = 0.celsius

        var isDeployed: Boolean = false
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun stopArmMotor() {}

    fun stopRollerMotor() {}

    fun setArmVoltage(volts: Double) {}

    fun setRollerVoltage(volts: Double) {}

    fun setArmPosition(position: Rotation2d) {}

    fun setRollerSpeed(speed: Double) {}

    fun setArmIdleMode(isBrake: Boolean) {}

    fun setRollerIdleMode(isBrake: Boolean) {}

    fun resetEncoder(position: Rotation2d) {}

    fun periodic() {}
}
