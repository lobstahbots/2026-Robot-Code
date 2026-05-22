package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants.ControllerIOConstants;

public class Controllers {
    public static class Controller extends Joystick {
        public final Trigger LTButton;
        public final Trigger RTButton;
        public final JoystickButton RBButton;
        public final JoystickButton LBButton;
        public final JoystickButton leftPaddle;
        public final JoystickButton rightPaddle;
        public final JoystickButton AButton;
        public final JoystickButton BButton;
        public final JoystickButton XButton;
        public final JoystickButton YButton;
        public final POVButton dpadUp;
        public final POVButton dpadDown;
        public final POVButton dpadLeft;
        public final POVButton dpadRight;

        public Controller(int port) {
            super(port);
            LTButton = new Trigger(() -> getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.5);
            RTButton = new Trigger(() -> getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.5);
            RBButton = new JoystickButton(this, ControllerIOConstants.RB_BUTTON);
            LBButton = new JoystickButton(this, ControllerIOConstants.LB_BUTTON);
            leftPaddle = new JoystickButton(this, ControllerIOConstants.LEFT_PADDLE);
            rightPaddle = new JoystickButton(this, ControllerIOConstants.RIGHT_PADDLE);
            AButton = new JoystickButton(this, ControllerIOConstants.A_BUTTON);
            BButton = new JoystickButton(this, ControllerIOConstants.B_BUTTON);
            XButton = new JoystickButton(this, ControllerIOConstants.X_BUTTON);
            YButton = new JoystickButton(this, ControllerIOConstants.Y_BUTTON);
            dpadUp = new POVButton(this, 0);
            dpadRight = new POVButton(this, 90);
            dpadDown = new POVButton(this, 180);
            dpadLeft = new POVButton(this, 270);
        }
    }

    public static final Controller driver = new Controller(ControllerIOConstants.DRIVER_CONTROLLER_PORT);
    public static final Controller operator = new Controller(ControllerIOConstants.OPERATOR_CONTROLLER_PORT);
}
