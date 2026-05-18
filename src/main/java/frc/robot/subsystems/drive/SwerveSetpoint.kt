package frc.robot.subsystems.drive

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState

class SwerveSetpoint(var chassisSpeeds: ChassisSpeeds, var moduleStates: Array<SwerveModuleState>)