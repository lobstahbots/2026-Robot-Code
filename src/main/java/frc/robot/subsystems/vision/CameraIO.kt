package frc.robot.subsystems.vision

import com.lobstahbots.junction.AutoLogKt
import edu.wpi.first.math.geometry.Pose3d
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

interface CameraIO {
    @AutoLogKt
    open class CameraIOInputs {
        /**
         * Estimated robot pose with least reprojection error
         */
        var bestEstimatedPose: Pose3d = Pose3d.kZero

        /**
         * Estimated robot pose with most reprojection error
         */
        var altEstimatedPose: Pose3d = Pose3d.kZero

        /**
         * Reprojection error of the best robot pose
         */
        var bestReprojErr: Double = 0.0

        /**
         * Reprojection error of worst robot pose
         */
        var altReprojErr: Double = 0.0

        /**
         * Multi-tag ambiguity - is equal to `bestReprojError` divided by
         * `altReprojError`
         */
        var ambiguity: Double = 0.0

        /**
         * Estimated time of the pose
         */
        var estimatedPoseTimestamp: Double = 0.0

        /**
         * Visible fiducial IDs
         */
        var visibleFiducialIDs: IntArray = intArrayOf()

        /**
         * Total area of targets as a fraction of the total image area
         */
        var totalArea: Double = 0.0

        /**
         * The photon pipeline result
         */
        var pipelineResult: PhotonPipelineResult = PhotonPipelineResult()

        var connected: Boolean = false

        /**
         * Update from a [LobstahEstimatedRobotPose].
         * 
         * @param estimatedRobotPose pose to get information from
         */
        fun updateFrom(estimatedRobotPose: LobstahEstimatedRobotPose) {
            bestEstimatedPose = estimatedRobotPose.bestEstimatedPose
            altEstimatedPose = estimatedRobotPose.alternateEstimatedPose
            bestReprojErr = estimatedRobotPose.bestReprojError
            altReprojErr = estimatedRobotPose.altReprojError
            ambiguity = estimatedRobotPose.multiTagAmbiguity
            estimatedPoseTimestamp = estimatedRobotPose.timestampSeconds
            visibleFiducialIDs = estimatedRobotPose.fiducialIDsUsed
            totalArea = estimatedRobotPose.totalArea
        }

        /**
         * Set all inputs to their empty value.
         */
        fun clearInputs() {
            bestEstimatedPose = Pose3d.kZero
            altEstimatedPose = Pose3d.kZero
            bestReprojErr = 0.0
            altReprojErr = 0.0
            ambiguity = 0.0
            visibleFiducialIDs = intArrayOf()
            totalArea = 0.0
            pipelineResult = PhotonPipelineResult()
        }
    }

    /**
     * Get a list of the tracked targets.
     * 
     * @return the list of tracked targets
     */
    val trackedTargets: MutableList<PhotonTrackedTarget>

    fun updateInputs(inputs: CameraIOInputs)

    /**
     * Get the name of this camera
     * 
     * @return the name as a string
     */
    val cameraName: String

    val tagPoses: Array<Pose3d>
}
