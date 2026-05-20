package frc.robot.util.led

import kotlin.math.max

// TODO: Implement LEDFrameAnimation

@Deprecated("Unimplemented")
open class LEDFrameAnimation(vararg val frames: LobstahLEDBuffer) {
    val loop: Boolean = false

    fun getFrame(index: Int): LobstahLEDBuffer? {
        return if (loop) frames[Math.floorMod(index, frames.size)]
        else if (index < 0 || index >= frames.size) null
        else frames[index]
    }

    val length: Int
        get() = frames.size

    fun getFrame(index: Double): LobstahLEDBuffer { // TODO
        val i = index.toInt()
        val alpha = index - i
        val frameA = frames[i]
        val frameB = frames[i + 1]
        return LobstahLEDBuffer.layer(max(frameA.length, frameB.length), frameA, frameB)
    }
}
