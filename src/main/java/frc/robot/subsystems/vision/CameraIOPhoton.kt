package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.robot.Constants.VisionConstants
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget

class CameraIOPhoton(override val cameraName: String) : CameraIO {
    private val camera: PhotonCamera = PhotonCamera(cameraName)
    private val poseEstimator: LobstahPoseEstimator = LobstahPoseEstimator(
        aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
        VisionConstants.CAMERA_TRANSFORMS[cameraName]!!
    )
    private var estimatedPose = LobstahEstimatedRobotPose(
        Pose3d.kZero, Pose3d.kZero, 0.0, 0.0, 0.0, 0.0,
        ArrayList(), VisionConstants.POSE_STRATEGY
    )
    private val disconnectedAlert: Alert = Alert("$cameraName has disconnected.", AlertType.kError)

    init {
        disconnectedAlert.set(false)
    }

    override fun updateInputs(inputs: CameraIOInputs) {
        val poseResults = camera.getAllUnreadResults()
        if (poseResults.isNotEmpty()) {
            val latestResult = poseResults[poseResults.size - 1]
            val estimatedPose = poseEstimator.update(latestResult)
            estimatedPose?.also {
                inputs.updateFrom(estimatedPose)
                inputs.pipelineResult = latestResult
            } ?: run {
                inputs.clearInputs()
            }
        }
        inputs.connected = camera.isConnected()
        disconnectedAlert.set(!camera.isConnected())
    }


    override val trackedTargets: MutableList<PhotonTrackedTarget>
        get() = estimatedPose.targetsUsed

    override val tagPoses: Array<Pose3d>
        get() = trackedTargets.map { target ->
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
        }.toTypedArray()

    companion object {
        private val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark)
    }
}
