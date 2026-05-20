package frc.robot.util.led

import java.util.*
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min

class AlphaBuffer(// A wrapper for an array of doubles
    val buffer: DoubleArray
) {
    constructor(length: Int) : this(DoubleArray(length))

    constructor(length: Int, alpha: Double) : this(length) {
        Arrays.fill(buffer, alpha)
    }

    fun invert() = AlphaBuffer(buffer.map { 1 - it }.toDoubleArray())

    fun crop(length: Int): AlphaBuffer {
        val result = DoubleArray(length)
        for (i in 0..<min(length, buffer.size)) {
            result[i] = buffer[i]
        }
        return AlphaBuffer(result)
    }

    fun flip() = AlphaBuffer(buffer.reversedArray())

    fun tile(length: Int) = AlphaBuffer(DoubleArray(length) { i -> buffer[Math.floorMod(i, buffer.size)] })

    fun repeat(times: Int) = tile(times * buffer.size)

    fun wrappedShift(length: Int, offset: Int) =
        AlphaBuffer(DoubleArray(length) { i -> buffer[Math.floorMod(i - offset, buffer.size)] })

    fun cycle(offset: Int) = wrappedShift(buffer.size, offset)

    fun shift(outputLength: Int, offset: Int): AlphaBuffer {
        val result = DoubleArray(outputLength)
        for (i in max(0, -offset)..<min(buffer.size, outputLength - offset)) {
            result[i + offset] = buffer[i]
        }
        return AlphaBuffer(result)
    }

    fun multiply(other: AlphaBuffer): AlphaBuffer = multiply(buffer.size, this, other)

    fun layer(other: AlphaBuffer?): AlphaBuffer = layer(buffer.size, this, other)

    fun append(other: AlphaBuffer?): AlphaBuffer = concat(this, other)

    fun prepend(other: AlphaBuffer): AlphaBuffer = concat(other, this)

    companion object {
        fun linear(length: Int) = AlphaBuffer(DoubleArray(length) { i -> i.toDouble() / (length - 1) })

        fun sine(length: Int, period: Double, phase: Double) =
            AlphaBuffer(DoubleArray(length) { i -> (1 - cos(2 * Math.PI / period * (i - phase))) / 2 })

        fun multiply(outputLength: Int, a: AlphaBuffer, b: AlphaBuffer) =
            AlphaBuffer(
                DoubleArray(
                    min(
                        outputLength,
                        min(a.buffer.size, b.buffer.size)
                    )
                ) { i -> a.buffer[i] * b.buffer[i] })

        fun layer(outputLength: Int, vararg layers: AlphaBuffer?): AlphaBuffer {
            val result = DoubleArray(outputLength)
            layers.forEach { layer ->
                layer?.let {
                    for (i in 0..<min(outputLength, layer.buffer.size)) {
                        result[i] = result[i] + layer.buffer[i] * (1 - result[i])
                    }
                }
            }
            return AlphaBuffer(result)
        }

        fun concat(outputLength: Int, vararg segments: AlphaBuffer?): AlphaBuffer {
            val result = DoubleArray(outputLength)
            var i = 0
            segments.forEach { segment ->
                segment?.let {
                    for (j in segment.buffer.indices) {
                        if (i >= outputLength) return AlphaBuffer(result)
                        result[i] = segment.buffer[j]
                        i++
                    }
                }
            }
            return AlphaBuffer(result)
        }

        fun concat(vararg segments: AlphaBuffer?) =
            AlphaBuffer(segments.flatMap({ it?.buffer?.asIterable() ?: listOf() }).toDoubleArray())
    }
}
