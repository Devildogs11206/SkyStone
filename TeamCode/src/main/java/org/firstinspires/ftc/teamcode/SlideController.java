package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideController extends RobotController{
    double power = 0.2;

    public SlideController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute(){
        if (gamepad.dpad_left) robot.slide(power);
        else if (gamepad.dpad_right) robot.slide(-power);
        else robot.slide(0);
    }
}
