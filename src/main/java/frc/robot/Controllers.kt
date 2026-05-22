@file:Suppress("PropertyName")

package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.IOConstants.ControllerIOConstants
import java.util.function.BooleanSupplier

object Controllers {
    val driver: Controller = Controller(ControllerIOConstants.DRIVER_CONTROLLER_PORT)
    val operator: Controller = Controller(ControllerIOConstants.OPERATOR_CONTROLLER_PORT)

    class Controller(port: Int) : Joystick(port) {
        val LTButton: Trigger = Trigger(BooleanSupplier { getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.5 })
        val RTButton: Trigger = Trigger(BooleanSupplier { getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.5 })
        val RBButton: JoystickButton = JoystickButton(this, ControllerIOConstants.RB_BUTTON)
        val LBButton: JoystickButton = JoystickButton(this, ControllerIOConstants.LB_BUTTON)
        val leftPaddle: JoystickButton = JoystickButton(this, ControllerIOConstants.LEFT_PADDLE)
        val rightPaddle: JoystickButton = JoystickButton(this, ControllerIOConstants.RIGHT_PADDLE)
        val AButton: JoystickButton = JoystickButton(this, ControllerIOConstants.A_BUTTON)
        val BButton: JoystickButton = JoystickButton(this, ControllerIOConstants.B_BUTTON)
        val XButton: JoystickButton = JoystickButton(this, ControllerIOConstants.X_BUTTON)
        val YButton: JoystickButton = JoystickButton(this, ControllerIOConstants.Y_BUTTON)
        val dpadUp: POVButton = POVButton(this, 0)
        val dpadDown: POVButton = POVButton(this, 180)
        val dpadLeft: POVButton = POVButton(this, 270)
        val dpadRight: POVButton = POVButton(this, 90)
    }
}
