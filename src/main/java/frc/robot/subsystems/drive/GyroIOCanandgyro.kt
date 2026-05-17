// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.reduxrobotics.sensors.canandgyro.Canandgyro
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs

/** Add your docs here.  */
class GyroIOCanandgyro(id: Int) : GyroIO {
    private val gyro: Canandgyro = Canandgyro(id)
    private var offset = 0.0

    val yaw: Rotation2d
        get() = Rotation2d.fromRotations(gyro.yaw - offset)
            .plus(
                if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red)
                    Rotation2d.kZero
                else
                    Rotation2d.k180deg
            )

    val pitch: Rotation2d
        get() = Rotation2d.fromRotations(gyro.pitch)

    val roll: Rotation2d
        get() = Rotation2d.fromRotations(gyro.roll)

    override fun zeroGyro() {
        offset = gyro.yaw
    }

    override val isCalibrating: Boolean
        get() = gyro.isCalibrating

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = gyro.isConnected
        inputs.rollPosition = roll
        inputs.pitchPosition = pitch
        inputs.yawPosition = yaw
        inputs.isCalibrating = gyro.isCalibrating
    }
}
