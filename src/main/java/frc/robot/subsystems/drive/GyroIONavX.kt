// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.studica.frc.AHRS
import com.studica.frc.AHRS.NavXComType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs

/** Add your docs here.  */
class GyroIONavX : GyroIO {
    private val gyro = AHRS(NavXComType.kMXP_SPI)

    val yaw: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.yaw.toDouble())
            .plus(
                if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red)
                    Rotation2d.kZero
                else
                    Rotation2d.k180deg
            )

    val pitch: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.pitch.toDouble())

    val roll: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.roll.toDouble())

    val yawVelocity: Double
        get() = gyro.velocityZ.toDouble()

    val rollVelocity: Double
        get() = gyro.velocityY.toDouble()

    val pitchVelocity: Double
        get() = gyro.velocityX.toDouble()

    override fun zeroGyro() {
        gyro.reset()
    }

    override val isCalibrating: Boolean
        get() = gyro.isCalibrating

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = gyro.isConnected
        inputs.rollPosition = this.roll
        inputs.pitchPosition = this.pitch
        inputs.yawPosition = this.yaw
        inputs.rollVelocity = this.rollVelocity
        inputs.pitchVelocity = this.pitchVelocity
        inputs.yawVelocity = this.yawVelocity
        inputs.isCalibrating = gyro.isCalibrating
    }
}
