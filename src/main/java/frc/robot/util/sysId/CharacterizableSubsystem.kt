package frc.robot.util.sysId

import edu.wpi.first.wpilibj2.command.SubsystemBase

abstract class CharacterizableSubsystem : SubsystemBase() {
    abstract fun runVolts(volts: Double)
}