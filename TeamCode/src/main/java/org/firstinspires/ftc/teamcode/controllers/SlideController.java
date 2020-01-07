package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.FORWARD;

public class SlideController extends RobotController {
    double power = 0.5;

    public SlideController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute(){
        if (gamepad2.dpad_left) robot.slide(power);
        else if (gamepad2.dpad_right) robot.slide(-power);
        else if(gamepad2.left_stick_button && gamepad2.x){robot.slide(0.5,BACK);}
        else if(gamepad2.left_stick_button && gamepad2.b){robot.slide(0.5,FORWARD);}
        else robot.slide(0);
    }
}
