package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveController extends RobotController{

    private double driveSpeed = 1.0;

    public DriveController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {

        if(gamepad.x){driveSpeed=0.6;}
        if(gamepad.y){driveSpeed=1.0;}

        double drive = gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;

        double left = driveSpeed*(drive + turn);
        double right = driveSpeed*(drive - turn);

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        robot.drive(left, right);
    }
}
