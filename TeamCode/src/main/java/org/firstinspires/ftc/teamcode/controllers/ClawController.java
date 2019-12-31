package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class ClawController extends RobotController {
    public ClawController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_bumper) robot.closeClaw();
        if (gamepad2.right_bumper) robot.openClaw();
    }
}
