package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltAccel.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltAccel.TILTED;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltAccel.UP;

public class TiltController extends RobotController {
    public TiltController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_trigger > 0) robot.tilt(gamepad2.left_trigger);
        else if (gamepad2.right_trigger > 0) robot.tilt(-gamepad2.right_trigger);
        else if (gamepad2.y) robot.tiltAccel(UP);
        else if (gamepad2.b) robot.tiltAccel(TILTED);
        else if (gamepad2.a) robot.tiltAccel(BACK);
        else robot.tilt((double)0);
    }
}
