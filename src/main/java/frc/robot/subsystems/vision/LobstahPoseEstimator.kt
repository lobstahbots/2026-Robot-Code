/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.estimation.TargetModel
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.math.abs

/**
 * The LobstahPoseEstimator class, based on PhotonPoseEstimator, filters or
 * combines readings from all the AprilTags visible at a given timestamp on the
 * field to produce a single robot in field pose, using the strategy set below.
 * Example usage can be found in our apriltagExample example project.
 * 
 * Differences from PhotonPoseEstimator: - No multi tag on rio strategy - method
 * signatures are slightly different because no camera calibration data is
 * needed as there is no multi tag on rio strategy
 */
class LobstahPoseEstimator
/**
 * Create a new LobstahPoseEstimator.
 *
 * @param fieldTagsLayout     A WPILib [AprilTagFieldLayout] linking AprilTag
 * IDs to Pose3d objects with respect to the FIRST field
 * using the [Field
 * Coordinate System](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system). Note that setting the origin of
 * this layout object will affect the results from this class.
 * @param strategy      The strategy it should use to determine the best pose.
 * @param robotToCameraTransform [Transform3d] from the center of the robot to the camera mount position (ie, robot -> camera) in the
 * [Robot Coordinate System](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system).
 */(
    fieldTagsLayout: AprilTagFieldLayout, strategy: PoseStrategy, var robotToCameraTransform: Transform3d
) {
    var multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY
        set(value) {
            checkUpdate(field, value)
            if (value == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR || value == PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
                DriverStation.reportWarning(
                    "Fallback cannot be set to MULTI_TAG_PNP! Setting to lowest ambiguity", false
                )
                field = PoseStrategy.LOWEST_AMBIGUITY
            } else field = value
        }

    var fieldTags: AprilTagFieldLayout = fieldTagsLayout
        set(value) {
            checkUpdate(field, value)
            field = value
        }

    var primaryStrategy: PoseStrategy = strategy
        set(value) {
            if (value == PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
                DriverStation.reportError(
                    "LobstahPoseEstimator doesn't support on-rio multi-tag! Keeping at " + this.primaryStrategy.toString(),
                    false
                )
                return
            }
            checkUpdate(this.primaryStrategy, value)
            field = value
        }

    var lastPose: Pose3d? = null
        set(value) {
            checkUpdate(field, value)
            field = value
        }
    var referencePose: Pose3d? = null
        set(value) {
            checkUpdate(field, value)
            field = value
        }
    protected var poseCacheTimestampSeconds: Double = -1.0
    private val reportedErrors: MutableSet<Int?> = HashSet<Int?>()

    init {
        HAL.report(tResourceType.kResourceType_PhotonPoseEstimator, InstanceCount)
        InstanceCount++
    }

    /** Invalidates the pose cache.  */
    private fun invalidatePoseCache() {
        poseCacheTimestampSeconds = -1.0
    }

    private fun checkUpdate(oldObj: Any?, newObj: Any?) {
        if (oldObj != newObj && oldObj != null) {
            invalidatePoseCache()
        }
    }

    /**
     * Updates the estimated position of the robot. Returns empty if:
     * 
     * 
     *  * The timestamp of the provided pipeline result is the same as in the
     * previous call to `update()`.
     *  * No targets were found in the pipeline results.
     * 
     * 
     * @return an [LobstahEstimatedRobotPose] with an estimated pose,
     * timestamp, and targets used to create the estimate.
     */
    fun update(cameraResult: PhotonPipelineResult): LobstahEstimatedRobotPose? {
        // Time in the past -- give up, since the following if expects times > 0
        if (cameraResult.timestampSeconds < 0) {
            return null
        }

        // If the pose cache timestamp was set, and the result is from the same
        // timestamp, return an
        // empty result
        if (poseCacheTimestampSeconds > 0 && abs(poseCacheTimestampSeconds - cameraResult.timestampSeconds) < 1e-6) {
            return null
        }

        // Remember the timestamp of the current result used
        poseCacheTimestampSeconds = cameraResult.timestampSeconds

        // If no targets seen, trivial case -- return empty result
        if (!cameraResult.hasTargets()) {
            return null
        }

        return update(cameraResult, this.primaryStrategy)
    }

    private fun update(cameraResult: PhotonPipelineResult, strat: PoseStrategy): LobstahEstimatedRobotPose? {
        var estimatedPose: LobstahEstimatedRobotPose?
        when (strat) {
            PoseStrategy.LOWEST_AMBIGUITY -> estimatedPose = lowestAmbiguityStrategy(cameraResult)
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE -> estimatedPose =
                closestToReferencePoseStrategy(cameraResult, referencePose)

            PoseStrategy.CLOSEST_TO_LAST_POSE -> {
                referencePose = lastPose
                estimatedPose = closestToReferencePoseStrategy(cameraResult, referencePose)
            }

            PoseStrategy.AVERAGE_BEST_TARGETS -> estimatedPose = averageBestTargetsStrategy(cameraResult)
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR -> estimatedPose = multiTagOnCoprocStrategy(cameraResult)
            else -> {
                DriverStation.reportError("[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false)
                return null
            }
        }

        return estimatedPose
    }

    private fun multiTagOnCoprocStrategy(result: PhotonPipelineResult): LobstahEstimatedRobotPose? {
        if (result.multiTagResult.isPresent) {
            val estimatedPose = result.multiTagResult.get().estimatedPose
            val best_tf = estimatedPose.best
            val best = Pose3d().plus(best_tf) // field-to-camera
                .relativeTo(fieldTags.origin).plus(robotToCameraTransform.inverse()) // field-to-robot
            val alt_tf = estimatedPose.alt
            val alt = Pose3d().plus(alt_tf) // field-to-camera
                .relativeTo(fieldTags.origin).plus(robotToCameraTransform.inverse()) // field-to-robot
            return LobstahEstimatedRobotPose(
                best,
                alt,
                estimatedPose.bestReprojErr,
                estimatedPose.altReprojErr,
                result.timestampSeconds,
                estimatedPose.ambiguity,
                result.getTargets(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            )
        } else {
            return update(result, this.multiTagFallbackStrategy)
        }
    }

    /**
     * Return the estimated position of the robot with the lowest position ambiguity
     * from a List of pipeline results.
     * 
     * @param result pipeline result
     * @return the estimated position of the robot in the FCS and the estimated
     * timestamp of this estimation.
     */
    private fun lowestAmbiguityStrategy(result: PhotonPipelineResult): LobstahEstimatedRobotPose? {
        var lowestAmbiguityTarget: PhotonTrackedTarget? = null

        var lowestAmbiguityScore = 10.0

        result.targets.forEach { target ->
            // Make sure the target is a Fiducial target.
            if (target.poseAmbiguity != -1.0 && target.poseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = target.poseAmbiguity
                lowestAmbiguityTarget = target
            }
        }

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null) return null

        val targetFiducialId = lowestAmbiguityTarget.fiducialId

        val targetPosition = fieldTags.getTagPose(targetFiducialId)

        if (targetPosition.isEmpty) {
            reportFiducialPoseError(targetFiducialId)
            return null
        }

        return LobstahEstimatedRobotPose(
            targetPosition.get().transformBy(lowestAmbiguityTarget.bestCameraToTarget.inverse())
                .transformBy(robotToCameraTransform.inverse()),
            Pose3d(),
            1.0,
            1.0,
            result.timestampSeconds,
            lowestAmbiguityTarget.poseAmbiguity,
            result.targets,
            PoseStrategy.LOWEST_AMBIGUITY
        )

    }

    /**
     * Return the estimated position of the robot using the target with the lowest
     * delta in the vector magnitude between it and the reference pose.
     * 
     * @param result        pipeline result
     * @param referencePose reference pose to check vector magnitude difference
     * against.
     * @return the estimated position of the robot in the FCS and the estimated
     * timestamp of this estimation.
     */
    private fun closestToReferencePoseStrategy(
        result: PhotonPipelineResult, referencePose: Pose3d?
    ): LobstahEstimatedRobotPose? {
        if (referencePose == null) {
            DriverStation.reportError(
                "[PhotonPoseEstimator] Tried to use reference pose strategy without setting the reference!", false
            )
            return null
        }

        var smallestPoseDelta = 10e9
        var lowestDeltaPose: LobstahEstimatedRobotPose? = null

        result.targets.forEach { target ->
            val targetFiducialId = target.getFiducialId()

            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (targetFiducialId == -1) return@forEach

            val targetPosition = fieldTags.getTagPose(target.fiducialId)

            if (targetPosition.isEmpty) {
                reportFiducialPoseError(targetFiducialId)
                return@forEach
            }

            val altTransformPosition = targetPosition.get().transformBy(target.alternateCameraToTarget.inverse())
                .transformBy(robotToCameraTransform.inverse())
            val bestTransformPosition = targetPosition.get().transformBy(target.bestCameraToTarget.inverse())
                .transformBy(robotToCameraTransform.inverse())

            val altDifference = abs(calculateDifference(referencePose, altTransformPosition))
            val bestDifference = abs(calculateDifference(referencePose, bestTransformPosition))

            if (altDifference < smallestPoseDelta) {
                smallestPoseDelta = altDifference
                lowestDeltaPose = LobstahEstimatedRobotPose(
                    altTransformPosition,
                    bestTransformPosition,
                    1.0,
                    0.0,
                    result.timestampSeconds,
                    1.0,
                    result.targets,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE
                )
            }
            if (bestDifference < smallestPoseDelta) {
                smallestPoseDelta = bestDifference
                lowestDeltaPose = LobstahEstimatedRobotPose(
                    bestTransformPosition,
                    altTransformPosition,
                    1.0,
                    0.0,
                    result.timestampSeconds,
                    1.0,
                    result.targets,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE
                )
            }
        }
        return lowestDeltaPose
    }

    /**
     * Return the average of the best target poses using ambiguity as weight.
     * 
     * @param result pipeline result
     * @return the estimated position of the robot in the FCS and the estimated
     * timestamp of this estimation.
     */
    private fun averageBestTargetsStrategy(result: PhotonPipelineResult): LobstahEstimatedRobotPose? {
        val estimatedRobotPoses: MutableList<Pair<PhotonTrackedTarget, Pose3d>> =
            ArrayList<Pair<PhotonTrackedTarget, Pose3d>>()
        var totalAmbiguity = 0.0

        result.targets.forEach { target ->
            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (target.fiducialId == -1) return@forEach

            val targetPosition = fieldTags.getTagPose(target.fiducialId)

            if (targetPosition.isEmpty) {
                reportFiducialPoseError(target.fiducialId)
                return@forEach
            }

            val targetPoseAmbiguity = target.getPoseAmbiguity()

            // Pose ambiguity is 0, use that pose
            if (targetPoseAmbiguity == 0.0) {
                return LobstahEstimatedRobotPose(
                    targetPosition.get().transformBy(target.bestCameraToTarget.inverse())
                        .transformBy(robotToCameraTransform.inverse()),
                    Pose3d(),
                    1.0,
                    0.0,
                    result.timestampSeconds,
                    0.0,
                    result.targets,
                    PoseStrategy.AVERAGE_BEST_TARGETS

                )
            }

            totalAmbiguity += 1.0 / target.poseAmbiguity

            estimatedRobotPoses.add(
                Pair(
                    target,
                    targetPosition.get().transformBy(target.bestCameraToTarget.inverse())
                        .transformBy(robotToCameraTransform.inverse())
                )
            )
        }

        // Take the average
        var translation = Translation3d()
        var rotation = Rotation3d()

        if (estimatedRobotPoses.isEmpty()) return null

        for (pair in estimatedRobotPoses) {
            // Total ambiguity is non-zero confirmed because if it was zero, that pose was
            // returned.
            val weight = (1.0 / pair.getFirst()!!.getPoseAmbiguity()) / totalAmbiguity
            val estimatedPose: Pose3d = pair.getSecond()!!
            translation = translation.plus(estimatedPose.translation.times(weight))
            rotation = rotation.plus(estimatedPose.rotation.times(weight))
        }

        return LobstahEstimatedRobotPose(
            Pose3d(translation, rotation),
            Pose3d(),
            1.0,
            0.0,
            result.timestampSeconds,
            totalAmbiguity,
            result.targets,
            PoseStrategy.AVERAGE_BEST_TARGETS
        )
    }

    /**
     * Difference is defined as the vector magnitude between the two poses
     * 
     * @return The absolute "difference" (>=0) between two Pose3ds.
     */
    private fun calculateDifference(x: Pose3d, y: Pose3d): Double {
        return x.translation.getDistance(y.translation)
    }

    private fun reportFiducialPoseError(fiducialId: Int) {
        if (!reportedErrors.contains(fiducialId)) {
            DriverStation.reportError(
                "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: $fiducialId", false
            )
            reportedErrors.add(fiducialId)
        }
    }

    companion object {
        private var InstanceCount = 0
    }
}
