package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class LiftController extends RobotController {
    private double liftStrength = 0.5;

    public LiftController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.dpad_up) robot.lift(liftStrength);
        else if (gamepad2.dpad_down) robot.lift(-liftStrength);
        else robot.lift( 0);
    }
}
