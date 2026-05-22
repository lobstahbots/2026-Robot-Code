// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import frc.robot.Constants.SimConstants
import frc.robot.SimShared
import frc.robot.subsystems.drive.SwerveModuleIO.ModuleIOInputs
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController
import com.lobstahbots.units.*

class SwerveModuleIOSim(
    angularOffsetDegrees: Double,
    /** Creates a new SwerveModuleSim.  */
    private val moduleSimulation: SwerveModuleSimulation, private val id: Int
) : SwerveModuleIO {
    private val driveMotor: GenericMotorController
    private val angleMotor: GenericMotorController

    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0

    private val angularOffset: Rotation2d

    init {
        this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees)
        driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
            .withCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
        angleMotor = moduleSimulation.useGenericControllerForSteer()
            .withCurrentLimit(Constants.DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        if (DriverStation.isDisabled()) {
            setDriveVoltage(0.0)
            setTurnVoltage(0.0)
        }

        inputs.turnAbsolutePosition = moduleSimulation.steerAbsoluteFacing
        inputs.turnPosition = inputs.turnAbsolutePosition
        inputs.drivePosition = moduleSimulation.driveWheelFinalPosition
            .plus(moduleSimulation.driveWheelFinalSpeed.times(SimConstants.LOOP_TIME.seconds))
        inputs.driveVelocity = moduleSimulation.driveWheelFinalSpeed
        inputs.driveAppliedVoltage = driveAppliedVolts.volts
        inputs.driveStatorCurrent = moduleSimulation.driveMotorStatorCurrent
        inputs.driveSupplyCurrent = moduleSimulation.driveMotorSupplyCurrent
        inputs.turnVelocity = moduleSimulation.steerAbsoluteEncoderSpeed
        inputs.turnAppliedVoltage = turnAppliedVolts.volts
        inputs.turnCurrent = moduleSimulation.steerMotorSupplyCurrent
        inputs.angularOffset = angularOffset

        SimShared.powerDistributionSim.setCurrent(
            SimConstants.SWERVE_CHANNELS[2 * id],
            inputs.driveStatorCurrent.baseUnitMagnitude()
        )
        SimShared.powerDistributionSim.setCurrent(
            SimConstants.SWERVE_CHANNELS[2 * id + 1],
            inputs.turnCurrent.baseUnitMagnitude()
        )
    }

    /**
     * Sets voltage of driving motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    override fun setDriveVoltage(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        driveMotor.requestVoltage(driveAppliedVolts.volts)
    }

    /**
     * Sets voltage of turn motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    override fun setTurnVoltage(volts: Double) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        angleMotor.requestVoltage(driveAppliedVolts.volts)
    }
}
