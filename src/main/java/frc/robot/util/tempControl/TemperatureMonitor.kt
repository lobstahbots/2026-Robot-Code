// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.tempControl

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants.TempConstants

/**
 * Class to monitor the temperature of some items.
 */
class TemperatureMonitor(
    private val motors: List<Monitorable>, private val disableAll: Boolean = true
) {
    /**
     * A monitorable device, likely a motor. Needs to be able to save its disabled state, get
     * its disabled state, and get its temperature and label.
     */
    interface Monitorable {
        /**
         * Sets the disabled state of the monitorable.
         * @param disable Whether or not to disable.
         */
        fun setDisabled(disable: Boolean)

        /**
         * Gets the disabled state of the monitorable.
         * @return Whether or not it is disabled.
         */
        fun getDisabled(): Boolean

        /**
         * Gets the motor temperature.
         * @return The temperature.
         */
        fun getMotorTemperature(): Double

        /**
         * Gets the label for this.
         * @return The label.
         */
        val label: String?
    }

    private var overheatTime = -1.0
    private var disabled = false
    private val alerts: MutableMap<String?, Alert?> =
        HashMap(motors.size, 1f) // Load factor can be 1 because the size will never change

    init {
        alerts.putAll(motors.map { motor ->
            motor.label to Alert(
                "${motor.label} has overheated above ${TempConstants.OVERHEAT_TEMP} C. It and related motors will be disabled until its temperature drops below ${TempConstants.SAFE_TEMP} C.",
                AlertType.kError
            )
        })
    }

    /**
     * Monitor the [Monitorable]s. This method should be called periodically,
     * probably in the `periodic` method of a subsystem.
     */
    fun monitor() {
        var safe = true
        for (motor in motors) {
            val temp = motor.getMotorTemperature()
            if (temp > TempConstants.SAFE_TEMP) safe = false
            else {
                alerts[motor.label]?.set(false)
                if (!disableAll) motor.setDisabled(false) // This motor is at a safe temperature so if we don't disable all motors we know it's safe to enable
            }
            if (temp > TempConstants.OVERHEAT_TEMP && overheatTime == -1.0) overheatTime = Timer.getFPGATimestamp()
            else if (temp > TempConstants.OVERHEAT_TEMP && Timer.getFPGATimestamp() - overheatTime >= 2) {
                disabled = true
                alerts[motor.label]?.set(true)
                motor.setDisabled(true)
            }
            if (disabled && disableAll) motor.setDisabled(true)
            else if (!disabled) motor.setDisabled(false)
        }
        if (safe) {
            disabled = false
            overheatTime = -1.0
        }
    }
}
