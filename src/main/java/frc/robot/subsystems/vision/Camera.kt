package frc.robot.subsystems.vision

import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N3
import frc.robot.Constants.VisionConstants
import org.littletonrobotics.junction.Logger
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.pow

class Camera(private val io: CameraIO) {
    private val inputs = CameraIOInputsAutoLogged()

    /**
     * Get this camera's name
     * 
     * @return The camera name, as set by the [CameraIO] passed into the
     * constructor of this lassF
     */
    val name: String = io.cameraName

    /**
     * Get the estimated pose from this camera.
     * 
     * @param odometryPose The current odometry pose
     * @return a [Pose] object representing the estimated pose, standard
     * deviation, and timestamp
     */
    fun getEstimatedPose(odometryPose: Pose2d): Pose {
        if (inputs.visibleFiducialIDs.isEmpty()) return Pose.empty()
        var resolvedPose: Pose3d? = null
        var resolvedReprojErr = 0.0

        if (inputs.ambiguity < VisionConstants.AMBIGUITY_ACCEPTANCE_THRESHOLD) {
            resolvedPose = inputs.bestEstimatedPose
            resolvedReprojErr = inputs.bestReprojErr
        }

        val stdev = VisionConstants.BASE_STDEV.times(
            (resolvedReprojErr.pow(VisionConstants.REPROJ_TO_STDEV_EXP) * 10 * inputs.ambiguity // multiply by ambiguity
                    * exp((1 / inputs.visibleFiducialIDs.size).toDouble()) * inputs.visibleFiducialIDs.size.toDouble()
                .pow(VisionConstants.APRIL_TAG_NUMBER_EXPONENT) * inputs.totalArea.pow(-1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE) * ln(
                2.0
            )) / ln(inputs.totalArea + Math.E) // Multiply by the scaling for the area of the AprilTags
        )

        Logger.recordOutput("Vision/" + this.name + "/ResolvedPose", resolvedPose)
        Logger.recordOutput("Vision/" + this.name + "/stdev", stdev.toString())

        if (resolvedPose != null && resolvedPose.x == 0.0 && resolvedPose.y == 0.0) resolvedPose = null

        return Pose(
            resolvedPose, stdev, inputs.estimatedPoseTimestamp
        )
    }

    val timestamp: Double
        /**
         * Get the timestamp of the pose capture.
         * 
         * @return the latest timestamp.
         */
        get() = inputs.estimatedPoseTimestamp

    val targets: MutableList<PhotonTrackedTarget>
        /**
         * Get the tracked targets from the camera.
         * 
         * @return A list of the [PhotonTrackedTarget]s.
         */
        get() = io.trackedTargets

    val fiducialIDs: IntArray
        /**
         * Get the fiducial IDs of the targets seen by the camera.
         * 
         * @return an array of the IDs
         */
        get() = inputs.visibleFiducialIDs

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Vision/" + this.name, inputs)
        Logger.recordOutput("Vision/$name/tagPoses", *io.tagPoses)
    }

    val robotToCamera: Transform3d?
        /**
         * Get the robot-to-camera transform for this camera.
         * 
         * @return The robot-to-camera transform for this camera, as a
         * [Transform3d].
         */
        get() = VisionConstants.CAMERA_TRANSFORMS[this.name]

    /**
     * Contains a pose, a timestamp, and a stdev estimated from a camera.
     */
    @JvmRecord
    data class Pose(
        /**
         * The estimated robot pose.
         */
        val pose: Pose3d?,
        /**
         * The stdev for the estimated robot pose.
         */
        val stdev: Vector<N3>?,
        /**
         * The timestamp of the estimated pose
         */
        val timestamp: Double?
    ) {
        companion object {
            fun empty(): Pose {
                return Pose(null, null, null)
            }
        }
    }
}
