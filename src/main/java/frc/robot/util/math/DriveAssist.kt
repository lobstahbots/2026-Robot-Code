package frc.robot.util.math

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.Constants
import frc.robot.Constants.DriverAssistConstants
import frc.robot.FieldConstants

private val LEFT_MIDLINE =
    (FieldConstants.LinesHorizontal.leftTrenchOpenEnd + FieldConstants.LinesHorizontal.leftTrenchOpenStart) / 2
private val RIGHT_MIDLINE =
    (FieldConstants.LinesHorizontal.rightTrenchOpenEnd + FieldConstants.LinesHorizontal.rightTrenchOpenStart) / 2

fun trenchAssist(input: ChassisSpeeds, robotPose: Pose2d): ChassisSpeeds {
    val normalized = input.div(Constants.DriveConstants.MAX_DRIVE_SPEED)
    val yComponent: Double =
        if (robotPose.y >= FieldConstants.LinesHorizontal.leftTrenchOpenEnd - FieldConstants.LeftTrench.openingWidth * normalized.vyMetersPerSecond * 2) DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (LEFT_MIDLINE - robotPose.y)
        else if (robotPose.y <= FieldConstants.LinesHorizontal.rightTrenchOpenStart - FieldConstants.RightTrench.openingWidth * normalized.vyMetersPerSecond * 2) DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (RIGHT_MIDLINE - robotPose.y)
        else return ChassisSpeeds()
    val xComponent: Double =
        if ((FieldConstants.LinesVertical.allianceZone - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS <= robotPose.x) && robotPose.x <= FieldConstants.LinesVertical.neutralZoneNear && normalized.vxMetersPerSecond > 0) (DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (FieldConstants.LinesVertical.neutralZoneNear - robotPose.x))
        else if (FieldConstants.LinesVertical.allianceZone <= robotPose.x && (robotPose.x <= FieldConstants.LinesVertical.neutralZoneNear - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS) && normalized.vxMetersPerSecond < 0) (DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (FieldConstants.LinesVertical.allianceZone - robotPose.x))
        else if ((FieldConstants.LinesVertical.neutralZoneFar - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS <= robotPose.x) && robotPose.x <= FieldConstants.LinesVertical.oppAllianceZone && normalized.vxMetersPerSecond > 0) (DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (FieldConstants.LinesVertical.oppAllianceZone - robotPose.x))
        else if (FieldConstants.LinesVertical.neutralZoneFar <= robotPose.x && (robotPose.x <= FieldConstants.LinesVertical.oppAllianceZone - normalized.vxMetersPerSecond * DriverAssistConstants.TRENCH_ASSIST_RADIUS) && normalized.vxMetersPerSecond < 0) (DriverAssistConstants.TRENCH_ASSIST_STRENGTH * (FieldConstants.LinesVertical.neutralZoneFar - robotPose.x))
        else return ChassisSpeeds()
    val theta = MathUtil.inputModulus(robotPose.rotation.radians, 0.0, 2 * Math.PI)
    val thetaMod = MathUtil.inputModulus(theta, 0.0, Math.PI / 2)
    return ChassisSpeeds(
        xComponent,
        yComponent,
        DriverAssistConstants.TRENCH_ASSIST_STRENGTH * ((if (thetaMod < Math.PI / 4) theta - thetaMod else theta - thetaMod + Math.PI / 2) - theta)
    )
}
