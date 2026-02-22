package frc.robot.util.math;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriverAssistConstants;

public class DriveAssist {
    private static final List<Translation2d> TRENCH_POINTS = List.of(
            FieldConstants.LeftTrench.openingTopLeft.plus(FieldConstants.LeftTrench.openingTopRight).div(2)
                    .toTranslation2d(),
            FieldConstants.RightTrench.openingTopRight.plus(FieldConstants.RightTrench.openingTopLeft).div(2)
                    .toTranslation2d(),
            FieldConstants.LeftTrench.oppOpeningTopLeft.plus(FieldConstants.LeftTrench.oppOpeningTopRight).div(2)
                    .toTranslation2d(),
            FieldConstants.RightTrench.oppOpeningTopRight.plus(FieldConstants.RightTrench.oppOpeningTopLeft).div(2)
                    .toTranslation2d());

    public static ChassisSpeeds trenchAssist(ChassisSpeeds input, Pose2d robotPose) {
        var translation = robotPose.getTranslation();
        var nearest = translation.nearest(TRENCH_POINTS);
        var minus = nearest.minus(translation);
        if (nearest.getDistance(translation) < DriverAssistConstants.TRENCH_ASSIST_RADIUS
                && Math.abs(new Rotation2d(input.vxMetersPerSecond, input.vyMetersPerSecond)
                        .minus(new Rotation2d(minus.getX(), minus.getY())).getRotations()) < 0.25) {
            var assist = minus.times(DriverAssistConstants.TRENCH_ASSIST_STRENGTH);
            return new ChassisSpeeds(assist.getX(), assist.getY(), 0);
        }
        return new ChassisSpeeds();
    }
}
