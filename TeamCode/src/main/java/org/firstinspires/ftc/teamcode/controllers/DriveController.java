package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DriveController extends RobotController {
    private static final double POWER_LOW = 0.5;
    private static final double POWER_HIGH = 1.0;

    private double power = POWER_LOW;

    public DriveController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        //if (gamepad1.x) power = POWER_LOW;
        //if (gamepad1.y) power = POWER_HIGH;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        robot.drive(drive * power, turn * power);
    }
}
