// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.command

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * A command composition that runs one of two commands, depending on the value of the given condition when this command
 * is initialized.
 * 
 * 
 * 
 * The rules for command compositions apply: command instances that are passed to it cannot be added to any other
 * composition or scheduled individually, and the composition requires all subsystems its components require.
 * 
 * 
 * 
 * This class is provided by the NewCommands VendorDep
 */
class PeriodicConditionalCommand
/**
 * Creates a new ConditionalCommand.
 *
 * @param onTrue the command to run if the condition is true
 * @param onFalse the command to run if the condition is false
 * @param condition the condition to determine which command to run
 */
    (private val onTrue: Command, private val onFalse: Command, private val condition: () -> Boolean) : Command() {
    private var selectedCommand: Command? = null


    init {
        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse)

        addRequirements(onTrue.requirements)
        addRequirements(onFalse.requirements)
    }

    override fun initialize() {
        selectedCommand = if (condition()) {
            onTrue
        } else {
            onFalse
        }.apply { initialize() }
    }

    override fun execute() {
        if (condition() && selectedCommand != onTrue) {
            selectedCommand = onTrue
            onFalse.cancel()
            selectedCommand!!.initialize()
        } else if (!condition() && selectedCommand != onFalse) {
            selectedCommand = onFalse
            onTrue.cancel()
            selectedCommand!!.initialize()
        }

        selectedCommand!!.execute()
    }

    override fun end(interrupted: Boolean) = selectedCommand!!.end(interrupted)

    override fun isFinished() = selectedCommand!!.isFinished

    override fun runsWhenDisabled(): Boolean = onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled()

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addStringProperty("onTrue", onTrue::getName, null)
        builder.addStringProperty("onFalse", onFalse::getName, null)
        builder.addStringProperty(
            "selected", {
                if (selectedCommand == null) {
                    "null"
                } else {
                    selectedCommand!!.name
                }
            }, null
        )
    }
}
