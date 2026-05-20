package frc.robot.util.led

import kotlin.math.cos

object AnimationEasing {
    @Deprecated("Work in progress ")
    fun easeIn(t: Double, b: Double, c: Double, d: Double): Double { // TODO: verify implementation
        var t = t
        return c * (d.let { t /= it; t }) * t + b
    }

    @Deprecated("Work in progress ")
    fun easeOut(t: Double, b: Double, c: Double, d: Double): Double { // TODO: verify implementation
        var t = t
        return -c * (d.let { t /= it; t }) * (t - 2) + b
    }

    @Deprecated("Work in progress ")
    fun easeInOut(t: Double, b: Double, c: Double, d: Double): Double { // TODO: verify implementation
        var t = t
        if ((d / 2.let { t /= it; t }) < 1) return c / 2 * t * t + b
        return -c / 2 * ((--t) * (t - 2) - 1) + b
    }

    fun sine(input: Double, period: Double, phaseAngle: Double): Double {
        return (1 - cos(2 * Math.PI * (input / period - phaseAngle))) / 2
    }
}
