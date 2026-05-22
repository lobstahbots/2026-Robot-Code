// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.lobstahbots.units.meters
import com.lobstahbots.units.metersPerSecond
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units
import frc.robot.Constants
import frc.robot.util.sysId.CharacterizableSubsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.cos

/**
 * Constructs a SwerveModule. The SwerveModule class contains methods relating
 * to both real and simulated modules, whereas SwerveModuleReal and
 * SwerveModuleSim are implementations of SwerveModule IO that contain methods
 * specific to a case.
 *
 * @param io       the [SwerveModuleIO] that handles loggable fields as
 * @param moduleID The module ID number. ID numbers range from 0-3: FrontLeft,
 * BackLeft, FrontRight, BackRight.
 */
class SwerveModule(
    private val io: SwerveModuleIO,
    /**
     * Returns the module ID.
     * 
     * @return The ID number of the module (0-3).
     */
    val moduleID: Int
) : CharacterizableSubsystem() {
    private val inputs = ModuleIOInputsAutoLogged()

    private val feedforward = SimpleMotorFeedforward(
        Constants.SwerveConstants.KS,
        Constants.SwerveConstants.KV, Constants.SwerveConstants.KA
    )
    private val driveController = PIDController(
        Constants.SwerveConstants.DRIVE_PID_P, Constants.SwerveConstants.DRIVE_PID_I,
        Constants.SwerveConstants.DRIVE_PID_D
    )
    private val angleController = PIDController(
        Constants.SwerveConstants.TURN_PID_P, Constants.SwerveConstants.TURN_PID_I,
        Constants.SwerveConstants.TURN_PID_D
    )

    init {
        angleController.enableContinuousInput(
            Constants.SwerveConstants.TURN_PID_MIN_INPUT,
            Constants.SwerveConstants.TURN_PID_MAX_INPUT
        )
    }

    /**
     * Sets voltage of drive motor and holds angle at 0 for use during
     * characterization ramp routines.
     * 
     * @param voltage The voltage to set the drive motor to.
     * @see CharacterizableSubsystem
     */
    override fun runVolts(voltage: Double) {
        io.setDriveVoltage(voltage)
        io.setTurnVoltage(angleController.calculate(this.angle.radians, 0.0))
    }

    /** Sets the voltages of both motors to 0  */
    fun stop() {
        io.setDriveVoltage(0.0)
        io.setTurnVoltage(0.0)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/Module$moduleID", inputs)
        io.periodic()
    }

    /**
     * Sets the desired state for the module. Optimizes state.
     * 
     * @param desiredState A [SwerveModuleState] with desired speed and angle.
     * @return The optimized SwerveModuleState.
     */
    fun setDesiredState(desiredState: SwerveModuleState, isOpenLoop: Boolean): SwerveModuleState {
        desiredState.optimize(inputs.turnAbsolutePosition)
        io.setTurnVoltage(angleController.calculate(this.angle.radians, desiredState.angle.radians))

        // Update velocity based on turn error
        desiredState.speedMetersPerSecond *= cos(angleController.error)

        // Run drive controller
        val velocityRadPerSec = desiredState.speedMetersPerSecond / (Constants.RobotConstants.WHEEL_DIAMETER.baseUnitMagnitude() / 2)
        io.setDriveVoltage(
            feedforward.calculate(velocityRadPerSec)
                    + driveController.calculate(inputs.driveVelocity.`in`(Units.RadiansPerSecond), velocityRadPerSec)
        )

        return desiredState
    }

    /**
     * Sets whether brake mode is enabled.
     * 
     * @param mode New [IdleMode], applied to both drive and turn motors.
     */
    fun setIdleMode(mode: IdleMode) {
        io.setDriveIdleMode(mode)
        io.setTurnIdleMode(mode)
    }

    val angle: Rotation2d
        /**
         * @return The current turn angle of the module.
         */
        get() = inputs.turnAbsolutePosition

    val absoluteAngle: Rotation2d
        /**
         * @returns The current turn angle of the module.
         */
        get() = inputs.turnAbsolutePosition

    val positionMeters: Double
        /**
         * @returns The current drive position of the module in meters.
         */
        get() = inputs.drivePosition.`in`(Units.Radians) * Math.PI * Constants.RobotConstants.WHEEL_DIAMETER.baseUnitMagnitude()

    val velocityMetersPerSec: Double
        /**
         * @returns The current drive velocity of the module in meters per second.
         */
        get() = inputs.driveVelocity.`in`(Units.RadiansPerSecond) * Constants.RobotConstants.WHEEL_DIAMETER.baseUnitMagnitude() / 2.0

    val position: SwerveModulePosition
        /**
         * @returns The module position (turn angle and drive position).
         */
        get() = SwerveModulePosition(this.positionMeters, this.absoluteAngle)

    val state: SwerveModuleState
        /**
         * @returns The module state (turn angle and drive velocity).
         */
        get() = SwerveModuleState(this.velocityMetersPerSec, this.absoluteAngle)
}