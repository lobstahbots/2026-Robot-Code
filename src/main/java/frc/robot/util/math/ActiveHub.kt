package frc.robot.util.math;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ActiveHub {
    /**
     * Obtain the time until the hub is active, or -1 if the hub is currently
     * active.
     * 
     * @return time in seconds
     */
    public static double timeToActive() {
        if (DriverStation.isAutonomous()) return -1;
        double time = DriverStation.getMatchTime();
        if (time > 130 || time <= 30) return -1;
        Alliance autoWinner = "R".equals(DriverStation.getGameSpecificMessage()) ? Alliance.Red : Alliance.Blue;
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isEmpty()) return -1;
        Alliance alliance = optionalAlliance.get();
        if (alliance == autoWinner) {
            if (time > 105)
                return time - 105;
            else if (time > 80)
                return -1;
            else if (time > 55) return time - 55;
            return -1;
        } else {
            if (time > 105)
                return -1;
            else if (time > 80)
                return time - 80;
            else if (time > 55) return -1;
            return time - 30;
        }
    }

    /**
     * Trigger is true when the hub is active according to estimated match time.
     */
    public static final Trigger activeHub = new Trigger(() -> timeToActive() == -1);
    /**
     * Trigger is true when the hub will be active soon (now or in the next four
     * seconds) according to estimated match time.
     */
    public static final Trigger activeHubSoon = new Trigger(() -> timeToActive() <= 4);
}
