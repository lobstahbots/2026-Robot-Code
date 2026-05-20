// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.command

import edu.wpi.first.wpilibj2.command.Command

class CycleCommand
/**
 * Creates a new CycleCommand. This command cycles through the list of commands
 * each time it is scheduled. Note that this doesn't run every command every
 * time, but merely runs the next command on the list every time this command is
 * scheduled.
 *
 * @param commands The list of commands to cycle through.
 */(private vararg val commands: Command) : Command() {
    private var index = -1


    init {
        for (command in commands) {
            addRequirements(command.requirements)
        }
    }

    override fun initialize() {
        index = (index + 1) % commands.size
        commands[index].initialize()
    }

    override fun execute() = commands[index].execute()

    override fun end(interrupted: Boolean) = commands[index].end(interrupted)

    override fun isFinished() = commands[index].isFinished
}
