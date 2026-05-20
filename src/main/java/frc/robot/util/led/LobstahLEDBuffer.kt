// Adapted from 6328 Mechanical Advantage's 2023 Robot Code
package frc.robot.util.led

import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color
import kotlin.math.max
import kotlin.math.min

/**
 * Generates and composites LED patterns
 * 
 * Adapted from 6328 Mechanical Advantage's 2023 Robot Code
 */
class LobstahLEDBuffer(val color: AddressableLEDBuffer, val alpha: AlphaBuffer) {
    val length: Int

    init {
        this.length = alpha.buffer.size
        require(color.getLength() == alpha.buffer.size) { "Length of color and alpha must be the same" }
    }

    constructor(
        red: IntArray,
        green: IntArray,
        blue: IntArray,
        alpha: DoubleArray
    ) : this(AddressableLEDBuffer(red.size), AlphaBuffer(alpha)) {
        require(!(red.size != green.size || red.size != blue.size)) { "Length of red, green, and blue must be the same" }
        for (i in 0..<length) {
            color.setRGB(i, red[i], green[i], blue[i])
        }
    }

    @JvmOverloads
    constructor(ledBuffer: AddressableLEDBuffer, alpha: Double = 1.0) : this(
        ledBuffer,
        AlphaBuffer(ledBuffer.length, alpha)
    )

    protected constructor(length: Int, alpha: Double) : this(AddressableLEDBuffer(length), alpha)

    constructor(length: Int) : this(AddressableLEDBuffer(length), AlphaBuffer(length))

    fun toAdressableLEDBuffer(): AddressableLEDBuffer {
        val buffer = AddressableLEDBuffer(length)
        for (i in 0..<length) {
            buffer.setRGB(
                i,
                (color.getRed(i) * alpha.buffer[i]).toInt(),
                (color.getGreen(i) * alpha.buffer[i]).toInt(),
                (color.getBlue(i) * alpha.buffer[i]).toInt()
            )
        }
        return buffer
    }

    fun mask(mask: AlphaBuffer): LobstahLEDBuffer {
        return LobstahLEDBuffer(color, alpha.multiply(mask))
    }

    fun opacity(alpha: Double): LobstahLEDBuffer {
        return mask(AlphaBuffer(length, alpha))
    }

    fun crop(length: Int): LobstahLEDBuffer {
        val cropped = LobstahLEDBuffer(length)
        for (i in 0..<min(length, this.length)) {
            cropped.color.setLED(i, color.getLED(i))
            cropped.alpha.buffer[i] = alpha.buffer[i]
        }
        return cropped
    }

    fun flip(): LobstahLEDBuffer {
        val flipped = LobstahLEDBuffer(length)
        for (i in 0..<length) {
            flipped.color.setLED(i, color.getLED(length - i - 1))
            flipped.alpha.buffer[i] = alpha.buffer[length - i - 1]
        }
        return flipped
    }

    fun tile(length: Int): LobstahLEDBuffer {
        val tiled = LobstahLEDBuffer(length)
        for (i in 0..<this.length) {
            val j = Math.floorMod(i, length)
            tiled.color.setLED(i, color.getLED(j))
            tiled.alpha.buffer[i] = alpha.buffer[j]
        }
        return tiled
    }

    fun repeat(times: Int): LobstahLEDBuffer {
        return tile(times * length)
    }

    fun wrappedShift(outputLength: Int, offset: Int): LobstahLEDBuffer {
        val translated = LobstahLEDBuffer(outputLength)
        for (i in 0..<length) {
            val j = Math.floorMod(i + offset, outputLength)
            translated.color.setLED(j, color.getLED(i))
            translated.alpha.buffer[j] = alpha.buffer[i]
        }
        return translated
    }

    fun cycle(offset: Int): LobstahLEDBuffer {
        return wrappedShift(length, offset)
    }

    fun shift(outputLength: Int, offset: Int): LobstahLEDBuffer {
        val translated = LobstahLEDBuffer(outputLength)
        for (i in max(0, -offset)..<min(length, outputLength - offset)) {
            val j = i + offset
            translated.color.setLED(j, color.getLED(i))
            translated.alpha.buffer[j] = alpha.buffer[i]
        }
        return translated
    }

    fun layerAbove(background: LobstahLEDBuffer): LobstahLEDBuffer {
        return layer(length, background, this)
    }

    fun layerBelow(foreground: LobstahLEDBuffer?): LobstahLEDBuffer {
        return layer(length, this, foreground)
    }

    fun append(other: LobstahLEDBuffer?): LobstahLEDBuffer {
        return concat(this, other)
    }

    fun prepend(other: LobstahLEDBuffer): LobstahLEDBuffer {
        return concat(other, this)
    }

    companion object {
        @JvmOverloads
        fun solid(length: Int, color: Color?, alpha: Double = 1.0): LobstahLEDBuffer {
            val ledBuffer = LobstahLEDBuffer(length, alpha)
            for (i in 0..<ledBuffer.length) {
                ledBuffer.color.setLED(i, color)
            }
            return ledBuffer
        }

        fun layer(outputLength: Int, vararg layers: LobstahLEDBuffer?): LobstahLEDBuffer {
            val output = LobstahLEDBuffer(outputLength)
            for (layer in layers) {
                if (layer == null) continue
                for (i in 0..<min(outputLength, layer.length)) {
                    output.color.setRGB(
                        i,
                        (layer.color.getRed(i) * layer.alpha.buffer[i] + output.color.getRed(i) * output.alpha.buffer[i] * (1 - layer.alpha.buffer[i])).toInt(),
                        (layer.color.getGreen(i) * layer.alpha.buffer[i] + output.color.getGreen(i) * output.alpha.buffer[i] * (1 - layer.alpha.buffer[i])).toInt(),
                        (layer.color.getBlue(i) * layer.alpha.buffer[i] + output.color.getBlue(i) * output.alpha.buffer[i] * (1 - layer.alpha.buffer[i])).toInt()
                    )

                    output.alpha.buffer[i] =
                        output.alpha.buffer[i] + layer.alpha.buffer[i] * (1 - output.alpha.buffer[i])
                }
            }
            return output
        }

        fun concat(outputLength: Int, vararg segments: LobstahLEDBuffer?): LobstahLEDBuffer {
            val output = LobstahLEDBuffer(outputLength)
            var i = 0
            for (segment in segments) {
                if (segment == null) continue
                for (j in 0..<segment.length) {
                    if (i >= outputLength) return output
                    output.color.setLED(i, segment.color.getLED(j))
                    output.alpha.buffer[i] = segment.alpha.buffer[j]
                    i++
                }
            }
            return output
        }

        fun concat(vararg segments: LobstahLEDBuffer?): LobstahLEDBuffer {
            var length = 0
            for (segment in segments) {
                if (segment == null) continue
                length += segment.length
            }
            return concat(length, *segments)
        }
    }
}
