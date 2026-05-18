package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.robot.Constants.VisionConstants
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs
import org.littletonrobotics.junction.AutoLogOutput
import org.photonvision.PhotonCamera
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionTargetSim
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.function.Consumer
import java.util.stream.Collectors
import kotlin.collections.flatMap

class CameraIOSim
/**
 * Create a new simulated [CameraIO].
 *
 * @param cameraName The name of the camera, as published to NetworkTables.
 */
    (override val cameraName: String) : CameraIO {
    private val camera: PhotonCamera = PhotonCamera(cameraName)
    private val cameraProp = SimCameraProperties()
    private val cameraSim: PhotonCameraSim
    private val poseEstimator: LobstahPoseEstimator
    private var estimatedPose = LobstahEstimatedRobotPose(
        Pose3d.kZero, Pose3d.kZero, 0.0, 0.0, 0.0, 0.0, ArrayList(), VisionConstants.POSE_STRATEGY
    )
    private val cameraToRobot: Transform3d = VisionConstants.CAMERA_TRANSFORMS[cameraName]!!


    init {
        cameraProp.setCalibration(
            VisionConstants.CAMERA_RES_WIDTH,
            VisionConstants.CAMERA_RES_HEIGHT,
            Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEG.toDouble())
        )
        cameraProp.setCalibError(VisionConstants.AVG_ERROR_PX, VisionConstants.ERROR_STDEV_PX)
        cameraProp.setFPS(VisionConstants.FPS)
        cameraProp.setAvgLatencyMs(VisionConstants.CAMERA_AVG_LATENCY_MS)
        cameraProp.setLatencyStdDevMs(VisionConstants.CAMERA_LATENCY_STDEV_MS)
        cameraSim = PhotonCameraSim(camera, cameraProp)
        poseEstimator = LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, cameraToRobot)
    }

    override fun updateInputs(inputs: CameraIOInputs) {
        val allTargets = targetSets.flatMap { it.value }

        // check if this camera is ready to process and get latency
        val optTimestamp = cameraSim.consumeNextEntryTime()
        if (optTimestamp.isEmpty) return

        // when this result "was" read by NT
        val timestampNT = optTimestamp.get()
        // this results processing latency in milliseconds
        val latencyMillis = cameraSim.prop.estLatencyMs()
        // the image capture timestamp
        val timestampCapture = timestampNT - (latencyMillis * 1e3).toLong()

        // use camera pose from the image capture timestamp
        val lateRobotPose: Pose3d = getRobotPose(timestampCapture.toDouble())
        val lateCameraPose = lateRobotPose.plus(cameraToRobot)
        dbgField.getObject(cameraName).pose = lateCameraPose.toPose2d()

        // process a PhotonPipelineResult with visible targets
        var camResult = cameraSim.process(latencyMillis, lateCameraPose, allTargets)
        camResult = PhotonPipelineResult(
            camResult.metadata.sequenceID,
            timestampCapture,
            camResult.metadata.publishTimestampMicros,
            camResult.metadata.timeSinceLastPong,
            camResult.targets,
            camResult.multiTagResult
        )

        // publish this info to NT as estimated timestamp of receive
        cameraSim.submitProcessedFrame(camResult, timestampNT)

        // display debug results
        val visibleTargetPoses: MutableList<Pose2d?> = ArrayList<Pose2d?>()
        for (target in camResult.getTargets()) {
            val transform = target.getBestCameraToTarget()
            if (transform.x == 0.0 && transform.y == 0.0 && transform.z == 0.0) continue
            visibleTargetPoses.add(lateCameraPose.transformBy(transform).toPose2d())
        }
        dbgField.getObject("visibleTargetPoses$cameraName").setPoses(visibleTargetPoses)

        val estimatedPose = poseEstimator.update(camResult)
        estimatedPose?.also {
            inputs.updateFrom(estimatedPose)
            inputs.pipelineResult = camResult
        } ?: run(inputs::clearInputs)
    }

    override val trackedTargets: MutableList<PhotonTrackedTarget>
        get() = estimatedPose.targetsUsed

    override val tagPoses: Array<Pose3d>
        get() = trackedTargets.map { target ->
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
        }.toTypedArray()

    companion object {
        private val aprilTagFieldLayout: AprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
        private val robotPoseBuffer: TimeInterpolatableBuffer<Pose3d> =
            TimeInterpolatableBuffer.createBuffer(VisionConstants.SIM_BUFFER_LENGTH)
        private val targetSets: MutableMap<String, MutableSet<VisionTargetSim>> = HashMap()
        private val dbgField = Field2d()

        /**
         * Adds targets on the field which your vision system is designed to detect. The
         * [PhotonCamera]s simulated from this system will report the location of
         * the camera relative to the subset of these targets which are visible from the
         * given camera position.
         * 
         * 
         * 
         * The AprilTags from this layout will be added as vision targets under the type
         * "apriltag". The poses added preserve the tag layout's current alliance
         * origin. If the tag layout's alliance origin is changed, these added tags will
         * have to be cleared and re-added.
         * 
         * @param tagLayout The field tag layout to get Apriltag poses and IDs from
         */
        fun addAprilTags(tagLayout: AprilTagFieldLayout) {
            for (tag in tagLayout.tags) {
                addVisionTargets(
                    "apriltag", VisionTargetSim(
                        tagLayout.getTagPose(tag.ID).get(),  // preserve alliance rotation
                        TargetModel.kAprilTag36h11, tag.ID
                    )
                )
            }
        }

        /**
         * Adds targets on the field which your vision system is designed to detect. The
         * [PhotonCamera]s simulated from this system will report the location of
         * the camera relative to the subset of these targets which are visible from the
         * given camera position.
         * 
         * @param type    Type of target (e.g. "cargo").
         * @param targets Targets to add to the simulated field
         */
        fun addVisionTargets(type: String, vararg targets: VisionTargetSim) {
            targetSets.computeIfAbsent(type) { HashSet() }
            for (tgt in targets) {
                targetSets[type]!!.add(tgt)
            }
        }

        init {
            addAprilTags(aprilTagFieldLayout)
        }

        /**
         * Get the robot pose in meters saved by the vision system at this timestamp.
         * 
         * @param timestamp Timestamp of the desired robot pose
         */
        fun getRobotPose(timestamp: Double): Pose3d {
            return robotPoseBuffer.getSample(timestamp).orElse(Pose3d())!!
        }

        @JvmStatic
        fun periodic() {
            targetSets.entries.forEach { entry ->
                dbgField.getObject(entry.key)
                    .setPoses(entry.value.map { it.pose.toPose2d() })
            }
        }

        @JvmStatic
        fun addSimPose(robotPose: Pose3d) {
            robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose)
        }
    }
}
