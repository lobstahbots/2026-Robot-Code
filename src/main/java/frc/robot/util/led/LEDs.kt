// Adapted from 6328 Mechanical Advantage's 2023 Robot Code
package frc.robot.util.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.LEDConstants
import frc.robot.Constants.LEDConstants.LengthConstants
import frc.robot.util.led.LobstahLEDBuffer.Companion.solid
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class LEDs : SubsystemBase() {
    var led: AddressableLED = AddressableLED(LEDConstants.LED_PORT)

    //#endregion
    //#region STATE VARIABLES
    enum class ConnectionState {
        DISCONNECTED, DS_ONLY, FMS
    }

    var connectionState: ConnectionState? = ConnectionState.DISCONNECTED
    var alliance: Alliance? = Alliance.Red

    enum class RobotMode {
        DISABLED, TELEOP, AUTONOMOUS, ESTOPPED
    }

    var robotMode: RobotMode? = RobotMode.DISABLED
        set(value) {
            if (value == RobotMode.DISABLED && robotMode == RobotMode.AUTONOMOUS && connectionState == ConnectionState.FMS) {
                triggerTeleopCountdown()
            }
            field = value
        }
    var aligned: Boolean = false
    var aligning: Boolean = false
    var readyForIntake: Boolean = false
    var hasCoral: Boolean = false
    var userSignal: Boolean = false

    var debugColor: Color? = null
    var debugLength: LoggedNetworkNumber = LoggedNetworkNumber("/Tuning/LED/DebugLength", 0.0)

    //#endregion
    //#region STATE GETTERS AND SETTERS
    fun setFMSState(value: ConnectionState?) {
        connectionState = value
    }

    fun triggerTeleopCountdown() {}
    fun triggerEndgameSignal() {}


    //#endregion
    override fun periodic() {
        loadingNotifier.stop()

        // Sets driver station state variables to reflect realtiy 
        if (!DriverStation.isDSAttached()) {
            setFMSState(ConnectionState.DISCONNECTED)
        } else if (DriverStation.isFMSAttached()) {
            setFMSState(ConnectionState.FMS)
            if (DriverStation.getAlliance().isPresent) {
                alliance = DriverStation.getAlliance().get()
            }
        } else {
            setFMSState(ConnectionState.DS_ONLY)
        }

        robotMode = if (DriverStation.isEStopped()) {
            RobotMode.ESTOPPED
        } else if (DriverStation.isAutonomousEnabled()) {
            RobotMode.AUTONOMOUS
        } else if (DriverStation.isTeleopEnabled()) {
            RobotMode.TELEOP
        } else {
            RobotMode.DISABLED
        }

        // Updates LEDs
        led.setData(
            LobstahLEDBuffer.layer(
                LengthConstants.TOTAL,
                if (robotMode == RobotMode.DISABLED)
                    if (connectionState == ConnectionState.DISCONNECTED)
                        disconnected() // Disconnected
                    else
                        disabledStandby() //Standby
                else
                    null,
                if (robotMode == RobotMode.AUTONOMOUS) autonomous() else null,  //Auto
                if (aligning) solid(LengthConstants.TOTAL, Color.kBlue).opacity(.5) else null,
                if (aligned) solid(LengthConstants.TOTAL, Color.kLime) else null,
                if (readyForIntake) prideFlagCycle(
                    LengthConstants.TOTAL,
                    8.0
                ) else null,  // hasCoral ? LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kPurple) : null,
                if (userSignal) solid(LengthConstants.TOTAL, Color.kWhite) else null,
                if (debugColor == null) null else solid(LengthConstants.TOTAL, debugColor),  //for testing
                solid(debugLength.get().toInt(), Color.kWhite)
            ).toAdressableLEDBuffer()
        )
    }

    //#region PATTERNS
    //#region INSTANCES OF PATTERN-RELATED OBJECTS
    // Timer possessionSignalTimer = new Timer();
    val loadingNotifier: Notifier = Notifier(Runnable {
        synchronized(this) {
            led.setData(loading()!!.toAdressableLEDBuffer())
        }
    })

    init {
        check(instance == null) { "LEDs already initialized" }
        instance = this

        led.setLength(LengthConstants.TOTAL)
        led.start()

        loadingNotifier.startPeriodic(0.02)
    }

    companion object {
        //#region SINGLETON, SETUP, AND CONSTRUCTOR
        var instance: LEDs? = null
            private set

        //#endregion
        fun segments(
            left: LobstahLEDBuffer?,
            midSegment: LobstahLEDBuffer?,
            right: LobstahLEDBuffer?
        ): LobstahLEDBuffer? {
            return LobstahLEDBuffer.Companion.concat(
                if (left == null) LobstahLEDBuffer(LengthConstants.LEFT) else left.crop(LengthConstants.LEFT),
                if (midSegment == null) LobstahLEDBuffer(LengthConstants.MID) else midSegment.crop(LengthConstants.MID),
                if (right == null) LobstahLEDBuffer(LengthConstants.RIGHT) else right.crop(LengthConstants.RIGHT).flip()
            )
        }

        fun loading(): LobstahLEDBuffer? {
            val opacity = AnimationEasing.sine(System.currentTimeMillis().toDouble(), 1000.0, 0.0)
            val buffer: LobstahLEDBuffer? =
                solid(10, LEDConstants.ColorConstants.LOADING).mask(AlphaBuffer.Companion.sine(10, 20.0, -10.0))
                    .opacity(opacity)
            return segments(buffer, null, buffer)
        }

        fun disconnected(): LobstahLEDBuffer? {
            val bouncyBallLength = 3
            val bouncyBallOffset = (AnimationEasing.sine(Timer.getFPGATimestamp(), 1.5, 0.0) * LengthConstants.MID
                    - bouncyBallLength / 2).toInt()
            val bouncyBall: LobstahLEDBuffer? = solid(3, LEDConstants.ColorConstants.LOADING).shift(
                LengthConstants.MID,
                bouncyBallOffset
            )

            val waveLength = 10
            val waves: LobstahLEDBuffer = solid(waveLength, LEDConstants.ColorConstants.LOADING).mask(
                AlphaBuffer.Companion.sine(
                    waveLength,
                    waveLength.toDouble(),
                    AnimationEasing.sine(Timer.getFPGATimestamp(), 3.0, 0.0) * 10
                )
            )

            return segments(
                waves.tile(LengthConstants.LEFT), bouncyBall,
                waves.cycle(-LengthConstants.RIGHT).tile(LengthConstants.RIGHT)
            )
        }

        fun disabledStandby(): LobstahLEDBuffer? {
            return LobstahLEDBuffer.Companion.solid(LengthConstants.TOTAL, Color("#FF5555"), 0.5)
                .mask(AlphaBuffer.Companion.sine(LengthConstants.TOTAL, 10.0, Timer.getFPGATimestamp() * 20))
                .layerAbove(solid(LengthConstants.TOTAL, Color.kRed))
        }

        fun autonomous(): LobstahLEDBuffer? {
            return LobstahLEDBuffer.Companion.solid(LengthConstants.TOTAL, LEDConstants.ColorConstants.AUTON_1, 0.5)
                .mask(AlphaBuffer.Companion.sine(LengthConstants.TOTAL, 10.0, Timer.getFPGATimestamp() * 10))
                .layerAbove(
                    solid(LengthConstants.TOTAL, LEDConstants.ColorConstants.AUTON_2)
                        .mask(AlphaBuffer.Companion.sine(LengthConstants.TOTAL, 5.0, Timer.getFPGATimestamp() * 7))
                )
                .layerAbove(solid(LengthConstants.TOTAL, LEDConstants.ColorConstants.AUTON_3))
        }

        fun prideFlagCycle(segmentLength: Int, speed: Double): LobstahLEDBuffer? {
            val offset = (Timer.getFPGATimestamp() * speed).toInt()
            return prideFlag(segmentLength)!!.prepend(solid(1, Color.kBlack)).cycle(offset)
        }

        fun prideFlag(segmentLength: Int): LobstahLEDBuffer? {
            return LobstahLEDBuffer.Companion.concat(
                solid(segmentLength, LEDConstants.ColorConstants.PRIDE_RED),
                solid(segmentLength, LEDConstants.ColorConstants.PRIDE_ORANGE),
                solid(segmentLength, LEDConstants.ColorConstants.PRIDE_YELLOW),
                solid(segmentLength, LEDConstants.ColorConstants.PRIDE_GREEN),
                solid(segmentLength, LEDConstants.ColorConstants.PRIDE_BLUE),
                solid(segmentLength, LEDConstants.ColorConstants.PRIDE_PURPLE),
                solid(1, Color.kBlack),
                solid(segmentLength, LEDConstants.ColorConstants.TRANS_TEAL),
                solid(segmentLength, LEDConstants.ColorConstants.TRANS_PINK),
                solid(segmentLength, Color.kWhite),
                solid(segmentLength, LEDConstants.ColorConstants.TRANS_PINK),
                solid(segmentLength, LEDConstants.ColorConstants.TRANS_TEAL)
            ).flip()
        } //#endregion
    }
}