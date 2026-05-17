package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs
import org.ironmaple.simulation.drivesims.GyroSimulation

class GyroIOSim(private val gyroSimulation: GyroSimulation) : GyroIO {
    override fun zeroGyro() {
        gyroSimulation.setRotation(Rotation2d.kZero)
    }

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = true
        inputs.yawPosition = gyroSimulation.gyroReading
        inputs.isCalibrating = false
    }
}
