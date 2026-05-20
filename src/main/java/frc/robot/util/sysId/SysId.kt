package frc.robot.util.sysId

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger
import java.util.function.Consumer

object SysId {
    fun getCharacterizationRoutine(subsystem: CharacterizableSubsystem, routine: CharacterizationRoutine): Command {
        val sysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                null, null, null
            )  // Use default config
            { state: SysIdRoutineLog.State? -> Logger.recordOutput("SysIdTestState", state.toString()) },
            Mechanism(
                { voltage: Voltage? -> subsystem.runVolts(voltage!!.`in`(Units.Volts)) },
                null,  // No log consumer, since data is recorded by AdvantageKit
                subsystem
            )
        )
        return when (routine) {
            CharacterizationRoutine.QUASISTATIC_FORWARD -> sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            CharacterizationRoutine.QUASISTATIC_BACKWARD -> sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
            CharacterizationRoutine.DYNAMIC_FORWARD -> sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
            CharacterizationRoutine.DYNAMIC_BACKWARD -> sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        }
    }

    enum class CharacterizationRoutine {
        QUASISTATIC_FORWARD,
        QUASISTATIC_BACKWARD,
        DYNAMIC_FORWARD,
        DYNAMIC_BACKWARD,
    }
}
