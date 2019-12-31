package org.firstinspires.ftc.teamcode;

public class SlideController extends RobotController{
    double power = 0.5;

    public SlideController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute(){
        if (gamepad2.dpad_left) robot.slide(power);
        else if (gamepad2.dpad_right) robot.slide(-power);
        else robot.slide(0);
    }
}
