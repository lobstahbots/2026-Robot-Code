package frc.robot.subsystems.shooter

import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import com.lobstahbots.junction.AutoLogKt
import com.lobstahbots.units.*

interface ShooterIO {
    @AutoLogKt
    open class ShooterIOInputs {
        var flywheelVelocity: AngularVelocity = 0.0.rpm
        var flywheelSetpoint: AngularVelocity = 0.0.rpm
        var flywheelAppliedVoltages = doubleArrayOf(0.0, 0.0, 0.0)
        var flywheelCurrents = doubleArrayOf(0.0, 0.0, 0.0)
        var flywheelTemperatures = doubleArrayOf(0.0, 0.0, 0.0)

        var hoodPosition: Rotation2d = Rotation2d.kZero
        var encoderPosition: Rotation2d = Rotation2d.kZero
        var hoodAppliedVoltage = 0.0
        var hoodCurrent = 0.0
        var hoodTemperature = 0.0
        var hoodVelocity: AngularVelocity = 0.0.rpm

        var useProfile = true
    }

    fun setFlywheelVoltage(voltage: Double) {}

    fun setHoodVoltage(voltage: Double) {}

    fun setFlywheelVelocity(velocity: AngularVelocity) {}

    fun setHoodPosition(position: Rotation2d) {}

    fun updateInputs(inputs: ShooterIOInputs) {}

    fun resetEncoder(position: Rotation2d) {}
}