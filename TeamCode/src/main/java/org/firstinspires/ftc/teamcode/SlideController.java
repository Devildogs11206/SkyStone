package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideController extends RobotController{
    public SlideController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute(){

    //THIS MAY BE BACKWARDS.
    //USE CAUTION.

        if (gamepad.dpad_left){robot.slide(0.2);}
        else if (gamepad.dpad_right){robot.slide(-0.2);}
        else {robot.slide(0);}

    }
}
