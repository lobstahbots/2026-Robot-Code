// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake

import edu.wpi.first.math.geometry.Rotation2d
import com.lobstahbots.units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.IntakeConstants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d
import kotlin.math.abs

class Intake
/** Creates a new Intake.  */(private val io: IntakeIO) : SubsystemBase() {
    private val inputs = IntakeIOInputsAutoLogged()
    private val mech2d = LoggedMechanism2d(3.feet, 2.feet)

    private val root: LoggedMechanismRoot2d = mech2d.getRoot(
        "Intake",
        1.5.feet.plus(6.5.inches).baseUnitMagnitude(), 8.inches.baseUnitMagnitude()
    )
    private val arm2d: LoggedMechanismLigament2d = root
        .append(
            LoggedMechanismLigament2d(
                "Arm",
                12.inches,
                0.degrees
            )
        )

    val rollerVelocity: Double
        get() = inputs.rollerVelocity.baseUnitMagnitude()

    val position: Rotation2d
        get() = inputs.armPosition

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
        arm2d.setAngle(this.position)
        Logger.recordOutput("IntakeMech2d", mech2d)
    }

    fun spin(): Command {
        return runEnd({ io.setRollerSpeed(1.0) }, io::stopRollerMotor)
    }

    private fun setPosition(setpoint: Rotation2d): Command {
        return startEnd(
            { io.setArmPosition(setpoint) },
            io::stopArmMotor
        ).until {
            abs(
                setpoint.minus(
                    position
                ).rotations
            ) <= IntakeConstants.MAX_ERROR.rotations
        }
    }

    fun deploy(): Command {
        return setPosition(IntakeConstants.DEPLOYED)
    }

    fun stow(): Command {
        return setPosition(IntakeConstants.STOWED)
    }

    fun home(): Command {
        return startEnd({ io.setArmVoltage(-6.0) }, {
            io.stopArmMotor()
            io.resetEncoder(IntakeConstants.STOWED)
        }).until { inputs.armCurrent >= IntakeConstants.ARM_DEPLOY_CURRENT_THRESHOLD.amps }
    }
}
