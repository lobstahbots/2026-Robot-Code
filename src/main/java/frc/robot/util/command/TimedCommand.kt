package frc.robot.util.command

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

/**
 * A wrapper that runs a command for a certain amount of time.
 */
class TimedCommand
/**
 * Creates a new TimedCommand.
 * 
 * @param command The command to run.
 * @param seconds The amount of time to run the command for.
 */
    (seconds: Double, command: Command) : ParallelDeadlineGroup(WaitCommand(seconds), command)
