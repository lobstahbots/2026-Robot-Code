// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.command;

import edu.wpi.first.wpilibj2.command.Command;

public class CycleCommand extends Command {
    private final Command[] commands;
    private int index = -1;

    /**
     * Creates a new CycleCommand. This command cycles through the list of commands
     * each time it is scheduled. Note that this doesn't run every command every
     * time, but merely runs the next command on the list every time this command is
     * scheduled.
     * 
     * @param commands The list of commands to cycle through.
     */
    public CycleCommand(Command... commands) {
        this.commands = commands;
        for (Command command : commands) {
            addRequirements(command.getRequirements());
        }
    }

    @Override
    public void initialize() {
        index = (index + 1) % commands.length;
        commands[index].initialize();
    }

    @Override
    public void execute() {
        commands[index].execute();
    }

    @Override
    public void end(boolean interrupted) {
        commands[index].end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return commands[index].isFinished();
    }
}
