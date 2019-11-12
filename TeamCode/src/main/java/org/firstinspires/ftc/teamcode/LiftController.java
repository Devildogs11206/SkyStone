package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftController extends RobotController {

    public LiftController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        if (gamepad.dpad_up){
            robot.lift(.50);
        } else if (gamepad.dpad_down) {
            robot.lift(-.50);
        } else {
            robot.lift( 0);
        }
    }
}