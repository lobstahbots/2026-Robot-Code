package frc.robot.util.math

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.jvm.optionals.getOrNull

/**
 * Obtain the time until the hub is active, or -1 if the hub is currently
 * active.
 *
 * @return time in seconds
 */
fun timeToActive(): Double {
    if (DriverStation.isAutonomous()) return -1.0
    val time = DriverStation.getMatchTime()
    if (time > 130 || time <= 30) return -1.0
    val autoWinner = if ("R" == DriverStation.getGameSpecificMessage()) Alliance.Red else Alliance.Blue
    val alliance = DriverStation.getAlliance().getOrNull() ?: return -1.0
    if (alliance == autoWinner) {
        if (time > 105) return time - 105
        else if (time > 80) return -1.0
        else if (time > 55) return time - 55
        return -1.0
    } else {
        if (time > 105) return -1.0
        else if (time > 80) return time - 80
        else if (time > 55) return -1.0
        return time - 30
    }
}

/**
 * Trigger is true when the hub is active according to estimated match time.
 */
val activeHub: Trigger = Trigger { timeToActive() == -1.0 }

/**
 * Trigger is true when the hub will be active soon (now or in the next four
 * seconds) according to estimated match time.
 */
val activeHubSoon: Trigger = Trigger { timeToActive() <= 4 }
