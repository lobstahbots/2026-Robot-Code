package frc.robot.util.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverAssistConstants;

public class DriveAssist {
    private static final double LEFT_MIDLINE = (FieldConstants.LinesHorizontal.leftTrenchOpenEnd
            + FieldConstants.LinesHorizontal.leftTrenchOpenStart) / 2;
    private static final double RIGHT_MIDLINE = (FieldConstants.LinesHorizontal.rightTrenchOpenEnd
            + FieldConstants.LinesHorizontal.rightTrenchOpenStart) / 2;

    public static ChassisSpeeds trenchAssist(ChassisSpeeds input, Pose2d robotPose) {
        ChassisSpeeds normalized = input.div(DriveConstants.MAX_DRIVE_SPEED);
        double yComponent = 0;
        double xComponent = 0;
        if (robotPose.getY() >= FieldConstants.LinesHorizontal.leftTrenchOpenEnd
                - FieldConstants.LeftTrench.openingWidth * normalized.vyMetersPerSecond * 2)
            yComponent = DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (LEFT_MIDLINE - robotPose.getY());
        else if (robotPose.getY() <= FieldConstants.LinesHorizontal.rightTrenchOpenStart
                - FieldConstants.RightTrench.openingWidth * normalized.vyMetersPerSecond * 2)
            yComponent = DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (RIGHT_MIDLINE - robotPose.getY());
        else
            return new ChassisSpeeds();
        if (FieldConstants.LinesVertical.allianceZone
                - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS <= robotPose.getX()
                && robotPose.getX() <= FieldConstants.LinesVertical.neutralZoneNear && normalized.vxMetersPerSecond > 0)
            xComponent = DriverAssistConstants.TRENCH_ASSIST_STRENGTH
                    * (FieldConstants.LinesVertical.neutralZoneNear - robotPose.getX());
        else if (FieldConstants.LinesVertical.allianceZone <= robotPose.getX()
                && robotPose.getX() <= FieldConstants.LinesVertical.neutralZoneNear
                        - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS
                && normalized.vxMetersPerSecond < 0)
            xComponent = DriverAssistConstants.TRENCH_ASSIST_STRENGTH
                    * (FieldConstants.LinesVertical.allianceZone - robotPose.getX());
        else if (FieldConstants.LinesVertical.neutralZoneFar
                - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS <= robotPose.getX()
                && robotPose.getX() <= FieldConstants.LinesVertical.oppAllianceZone && normalized.vxMetersPerSecond > 0)
            xComponent = DriverAssistConstants.TRENCH_ASSIST_STRENGTH
                    * (FieldConstants.LinesVertical.oppAllianceZone - robotPose.getX());
        else if (FieldConstants.LinesVertical.neutralZoneFar <= robotPose.getX()
                && robotPose.getX() <= FieldConstants.LinesVertical.oppAllianceZone
                        - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS
                && normalized.vxMetersPerSecond < 0)
            xComponent = DriverAssistConstants.TRENCH_ASSIST_STRENGTH
                    * (FieldConstants.LinesVertical.neutralZoneFar - robotPose.getX());
        else
            return new ChassisSpeeds();
        double theta = MathUtil.inputModulus(robotPose.getRotation().getRadians(), 0, 2 * Math.PI);
        double thetaMod = MathUtil.inputModulus(theta, 0, Math.PI / 2);
        return new ChassisSpeeds(xComponent, yComponent, DriverAssistConstants.TRENCH_ASSIST_STRENGTH
                * ((thetaMod < Math.PI / 4 ? theta - thetaMod : theta - thetaMod + Math.PI / 2) - theta));
    }
}
