// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.pathfinding.Pathfinding
import com.reduxrobotics.canand.CanandEventLoop
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.Constants.SimConstants
import frc.robot.Constants.mode
import frc.robot.subsystems.vision.CameraIOSim.Companion.periodic
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.conduit.ConduitApi
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.littletonrobotics.urcl.URCL
import java.io.File

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null

    private var robotContainer: RobotContainer? = null

    private val canAlert = Alert("CAN Error", AlertType.kError)

    private val powerDistribution = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    override fun robotInit() {
        Pathfinding.setPathfinder(LocalADStarAK())
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
        Logger.recordMetadata("Lobstah Bots", "2025 Robot Code")

        Logger.registerURCL(URCL.startExternal())

        val log = File(Filesystem.getOperatingDirectory(), "log")
        val logPath = log.absolutePath

        if (isReal()) {
            Logger.addDataReceiver(WPILOGWriter()) // Save outputs to a new log
            Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            CanandEventLoop.getInstance()
        } else {
            if (mode == Constants.RobotMode.REPLAY) {
                val replayPath = logPath + "\\" + SimConstants.REPLAY_LOG_PATH
                Logger.setReplaySource(WPILOGReader(replayPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_replay")))
                setUseTiming(false) // Run as fast as possible
            } else {
                Logger.addDataReceiver(WPILOGWriter(logPath)) // Save outputs to a new log
                Logger.addDataReceiver(NT4Publisher())
            }
        }
        DataLogManager.start()
        Logger.start()

        robotContainer = RobotContainer()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated
     * and test.
     * 
     * 
     * 
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
        val canStatus = RobotController.getCANStatus()
        if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
            canAlert.set(true)
            // canAlert.setText(String.format("CAN error: %d receive errors, %d transmit errors, %d%% utilization",
            //         canStatus.receiveErrorCount, canStatus.transmitErrorCount, canStatus.percentBusUtilization));
        } else canAlert.set(false)


        robotContainer?.periodic()

        Logger.recordOutput("conduit total thing", ConduitApi.getInstance().pdpTotalCurrent)
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * [RobotContainer] class.
     */
    override fun autonomousInit() {
        autonomousCommand = robotContainer?.autonomousCommand.also {
            CommandScheduler.getInstance().schedule(autonomousCommand)
        }
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        autonomousCommand?.cancel()
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
        robotContainer?.setIdleMode(false)
    }

    override fun testExit() {
        CommandScheduler.getInstance().cancelAll()
        robotContainer?.setIdleMode(true)
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic()
        robotContainer?.displaySimField()

        periodic()

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(*powerDistribution.allCurrents))
    }

    companion object {
        val isReal: Boolean
            get() = isReal()
        val isSimulation: Boolean
            get() = isSimulation()
    }
}
