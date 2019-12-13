package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveController extends RobotController{
    private static final double POWER_LOW = 0.5;
    private static final double POWER_HIGH = 1.0;

    private double power = POWER_LOW;

    public DriveController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        if (gamepad.x) power = POWER_LOW;
        if (gamepad.y) power = POWER_HIGH;

        double drive = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;

        robot.drive(drive * power, turn * power);
    }
}
