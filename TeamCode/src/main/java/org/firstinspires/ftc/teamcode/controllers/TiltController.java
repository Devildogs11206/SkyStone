package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class TiltController extends RobotController {
    public TiltController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_trigger > 0) robot.tilt(gamepad2.left_trigger);
        else if (gamepad2.right_trigger > 0) robot.tilt(-gamepad2.right_trigger);
        else robot.tilt( 0);
    }
}
