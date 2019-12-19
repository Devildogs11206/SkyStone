package org.firstinspires.ftc.teamcode;

public class TiltController extends RobotController {
    public TiltController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_trigger > 0){
            robot.tilt(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > 0) {
            robot.tilt(-gamepad2.right_trigger);
        } else if(gamepad2.left_stick_button) {
            robot.tilt(0.5, 1.25);
        }else if(gamepad2.right_stick_button) {
            robot.tilt(-0.5, 1.25);
        }else{
            robot.tilt( 0);
        }
    }
}
