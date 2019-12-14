package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonmousController extends RobotController {
    private String mode;
    Boolean executed = false;

    public AutonmousController(Telemetry telemetry, Gamepad gamepad, Robot robot, String mode) {
        super(telemetry, gamepad, robot);
        this.mode = mode;
    }

    @Override
    public void execute() {
        if(executed)return;
        robot.slide(0.5,2);
        robot.tilt(0.5,500);
        switch (mode) {
            case "driveUnderBlueBridge1":
                driveUnderBlueBridge1();
                break;
            case "driveUnderBlueBridge2":
                driveUnderBlueBridge2();
                break;
        }
        executed = true;
    }

    protected void driveUnderBlueBridge1() {
    }

    protected void driveUnderBlueBridge2() {
        robot.drive(.5 ,0,8);
        robot.turn(90,.5);
        robot.drive(.5,0,34);
    }
}
