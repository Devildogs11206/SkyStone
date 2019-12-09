package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftController extends RobotController {
    private double liftStrength = 0.5;

    public LiftController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        if (gamepad.dpad_up){
            robot.lift(liftStrength);
        } else if (gamepad.dpad_down) {
            robot.lift(-liftStrength);
        } else {
            robot.lift( 0);
        }
    }
}