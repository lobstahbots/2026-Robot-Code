// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.tempControl

import com.revrobotics.spark.SparkMax

/** A temperature-monitored SPARK MAX motor controller.  */
class MonitoredSparkMax
/**
 * Create a new object to control a SPARK MAX motor Controller with default label "Motor deviceId",
 * where deviceId is the device ID.
 * 
 * @param deviceId The device ID.
 * @param type The motor type connected to the controller. Brushless motor wires must be connected
 * to their matching colors and the hall sensor must be plugged in. Brushed motors must be
 * connected to the Red and Black terminals only.
 */(
    deviceId: Int, type: MotorType, override val label: String = String.format("Motor %d", deviceId)
) : SparkMax(deviceId, type), TemperatureMonitor.Monitorable {
    private var disabled = false


    override fun setDisabled(disable: Boolean) {
        if (disable) super.set(0.0)
        disabled = disable
    }

    override fun getDisabled(): Boolean {
        return disabled
    }

    override fun set(speed: Double) {
        if (!disabled) super.set(speed)
    }

    override fun setVoltage(outputVolts: Double) {
        if (!disabled) super.setVoltage(outputVolts)
    }

    override fun getMotorTemperature(): Double {
        return super.getMotorTemperature()
    }
}
