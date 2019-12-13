package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonmousController extends RobotController {
    public AutonmousController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        driveUnderBridge();
    }

    protected void driveUnderBridge() {
        robot.drive(.5 ,0,8);
        robot.turn(-90,.5);
        robot.drive(.5,0,34);
    }
}
