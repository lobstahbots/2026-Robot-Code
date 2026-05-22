package frc.robot.subsystems.shooter

import com.lobstahbots.units.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants.ShooterConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.util.math.ShotData
import org.littletonrobotics.junction.Logger

class Shooter(val io: ShooterIO) : SubsystemBase() {
    private val inputs = ShooterIOInputsAutoLogged()
    private var setpoint = 0.0.rpm
    private val zero = 0.0.rpm

    @JvmField
    val atSpeed: Trigger = Trigger {
        inputs.flywheelVelocity.isNear(setpoint, ShooterConstants.TOLERANCE) && !setpoint.isNear(
            zero, ShooterConstants.TOLERANCE
        )
    }.debounce(0.1)

    /**
     * Get the hood position of this shooter.
     *
     * @return The current hood position
     */
    fun getHoodPosition(): Rotation2d {
        return inputs.hoodPosition
    }

    /**
     * Get the current flywheel velocity
     *
     * @return The current flywheel velocity.
     */
    fun getFlywheelVelocity(): AngularVelocity {
        return inputs.flywheelVelocity
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)
    }

    /**
     * Constructs a command which keeps the hood at an angle specified by a
     * supplier.
     *
     * @param angle A supplier for the angle, in the form of a [Rotation2d].
     * @return The constructed command.
     */
    fun hoodAngle(angle: () -> Rotation2d): Command {
        return run { io.setHoodPosition(angle()) }
    }

    fun velocity(velocity: () -> AngularVelocity): Command {
        return runEnd({ io.setFlywheelVelocity(velocity()) }, { io.setFlywheelVelocity(0.0.rotationsPerSecond) })
    }

    fun voltage(voltage: Double): Command {
        return startEnd({ io.setFlywheelVoltage(voltage) }, { io.setFlywheelVoltage(0.0) })
    }

    /**
     * Constructs a command which keeps the hood at an angle specified by a supplier
     * and the flywheel at a velocity specified by a supplier.
     *
     * @param hoodAngle        A supplier for the hood angle as a [Rotation2d].
     * @param flywheelVelocity A supplier for the rotational velocity of the
     *                         flywheel as a [AngularVelocity] Measure object.
     * @return The constructed command.
     */
    fun operate(hoodAngle: () -> Rotation2d, flywheelVelocity: () -> AngularVelocity): Command {
        return runEnd({
            io.setHoodPosition(hoodAngle())
            io.setFlywheelVelocity(flywheelVelocity())
        }, {
            io.setHoodPosition(inputs.hoodPosition)
            io.setFlywheelVoltage(0.0)
        })
    }

    /**
     * Constructs a command which keeps the hood at an angle and the flywheel at a
     * velocity specified by a supplier.
     *
     * @param shotData The supplier for the data for the shot, which supplies
     *                 [ShotData] objects.
     * @return The constructed command.
     * @see ShotData
     */
    fun operate(shotData: () -> ShotData): Command {
        return operate({ shotData().hoodPosition }, { shotData().flywheelVelocity })
    }

    fun zero(): Command {
        return runEnd({ io.setHoodVoltage(-10.0) }, { io.setHoodVoltage(0.0) }).raceWith(
            Commands.waitSeconds(0.1).andThen(Commands.idle().until { inputs.hoodCurrent > 25 })
        ).andThen(Commands.waitSeconds(0.05).andThen({ io.resetEncoder(ShooterConstants.MIN_ANGLE) }))
    }
}