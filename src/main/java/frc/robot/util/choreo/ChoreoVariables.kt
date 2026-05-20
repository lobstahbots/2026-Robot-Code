package frc.robot.util.choreo

import com.google.gson.JsonElement
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import com.lobstahbots.units.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.Filesystem
import java.io.BufferedReader
import java.io.File
import java.io.FileReader

/**
 * A class to assist in retrieving values from Choreo configuration files.
 */
object ChoreoVariables {
    private var variableCache = mutableMapOf<String, Double>()
    private var poseCache = mutableMapOf<String, Pose2d>()
    private var INITIALIZED = false

    private fun getVal(obj: JsonObject): Double {
        return obj.get("val").asDouble
    }

    private fun getVal(obj: JsonElement): Double {
        return ChoreoVariables.getVal(obj.getAsJsonObject())
    }

    private fun initialize() {
        if (INITIALIZED) return
        val choreoFile = File(Filesystem.getDeployDirectory(), "choreo/2025robot.chor")
        try {
            val reader = BufferedReader(FileReader(choreoFile))
            val str = reader.lines().reduce("") { a: String, b: String -> a + b }
            reader.close()
            val wholeChor = JsonParser().parse(str).getAsJsonObject()
            val variables = wholeChor.get("variables").getAsJsonObject()
            val expressions = variables.get("expressions").getAsJsonObject()
            for (entry in expressions.entrySet()) {
                variableCache[entry.key] = getVal(entry.value.getAsJsonObject().get("var"))
            }
            val poses = variables.get("poses").getAsJsonObject()
            for (entry in poses.entrySet()) {
                val `val` = entry.value.getAsJsonObject()
                poseCache[entry.key] = Pose2d(
                    getVal(`val`.get("x")), getVal(`val`.get("y")),
                    Rotation2d.fromRadians(getVal(`val`.get("heading")))
                )
            }
        } catch (e: Exception) {
            System.err.println(e)
            return
        }
        INITIALIZED = true
    }

    /**
     * Get a pose from Choreo.
     * 
     * @param key The variable name in Choreo
     * @return The [Pose2d] with the given key
     */
    fun getPose(key: String): Pose2d? {
        initialize()
        return poseCache[key]
    }

    /**
     * Get a value as a double-precision floating point number from Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A double in the SI unit for that variable
     */
    fun get(key: String): Double {
        initialize()
        return variableCache[key]!!
    }

    /**
     * Get a value in meters from Choreo. NOTE: this works regardless of
     * whether the value actually is set as a length in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [Distance] object representing the length
     */
    fun getLength(key: String): Distance = get(key).meters

    /**
     * Get a value in meters per second from Choreo. NOTE: this works
     * regardless of whether the value actually is set as a linear velocity in
     * Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [LinearVelocity] object representing the linear velocity
     */
    fun getLinearVelocity(key: String): LinearVelocity = get(key).metersPerSecond

    /**
     * Get a value in meters per second per second from Choreo. NOTE: this
     * works regardless of whether the value actually is set as a linear
     * acceleration in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [LinearAcceleration] object representing the linear
     * acceleration
     */
    fun getLinearAcceleration(key: String): LinearAcceleration = get(key).metersPerSecondPerSecond

    /**
     * Get a value in radians from Choreo. NOTE: this works regardless of
     * whether the value actually is set as an angle in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [Angle] object representing the angle
     */
    fun getAngle(key: String): Angle = get(key).radians

    /**
     * Get a rotation from Choreo. NOTE: this works regardless of whether the value
     * actually is set as an angle in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [Rotation2d] object representing the rotation
     */
    fun getRotation2d(key: String): Rotation2d = Rotation2d.fromRadians(get(key))

    /**
     * Get a value in radians per second from Choreo. NOTE: this works
     * regardless of whether the value actually is set as a rotational velocity in
     * Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [AngularVelocity] object representing the rotational velocity
     */
    fun getAngularVelocity(key: String): AngularVelocity = get(key).radiansPerSecond

    /**
     * Get a value in radians per second per second from Choreo. NOTE: this
     * works regardless of whether the value actually is set as an angular
     * acceleration in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [AngularAcceleration] object representing the angular
     * acceleration
     */
    fun getAngularAcceleration(key: String): AngularAcceleration =get(key).radiansPerSecondPerSecond

    /**
     * Get a value in seconds from Choreo. NOTE: this works regardless of
     * whether the value actually is set as a time in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [Time] object representing the time
     */
    fun getTime(key: String): Time = get(key).seconds

    /**
     * Get a value in kilograms from Choreo. NOTE: this works regardless of
     * whether the value actually is set as a mass in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [Mass] object representing the mass
     */
    fun getMass(key: String): Mass = get(key).kilograms

    /**
     * Get a value in Newton-meters from Choreo. NOTE: this works regardless
     * of whether the value actually is set as a torque in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [Torque] object representing the torque
     */
    fun getTorque(key: String): Torque = get(key).newtonMeters

    /**
     * Get a value in kilogram square meters from Choreo. NOTE: this works
     * regardless of whether the value actually is set as an MOI in Choreo.
     * 
     * @param key The variable name in Choreo
     * @return A [MomentOfInertia] object representing the MOI
     */
    fun getMOI(key: String): MomentOfInertia = get(key).kilogramSquareMeters

    /**
     * Deinitialize this - it will reinitialize later if necessary. This method
     * cleans up memory by removing the variable and pose caches used to retrieve
     * values - when something is read the whole file is read into these hash maps
     * and kept and reused if more variables are read. This resets those hash maps.
     */
    fun deinitialize() {
        variableCache = mutableMapOf()
        poseCache = mutableMapOf()
        INITIALIZED = false
        System.gc()
    }
}
