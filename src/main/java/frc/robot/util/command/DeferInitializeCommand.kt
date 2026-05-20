// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.command

import edu.wpi.first.wpilibj2.command.Command

class DeferInitializeCommand(private val commandSupplier: () -> Command) : Command() {
    private var command: Command? = null

    init {
        addRequirements(commandSupplier().requirements)
    }

    override fun initialize() {
        command = commandSupplier()
        command!!.initialize()
    }

    override fun execute() = command!!.execute()

    override fun end(interrupted: Boolean) = command!!.end(interrupted)

    override fun isFinished() = command!!.isFinished
}
