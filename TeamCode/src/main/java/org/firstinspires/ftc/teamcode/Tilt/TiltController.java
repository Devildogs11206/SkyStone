package org.firstinspires.ftc.teamcode.Tilt;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotController;

public class TiltController extends RobotController {
    public TiltController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        if (gamepad.left_trigger >0){
            robot.tilt(gamepad.left_trigger);
        } else if (gamepad.right_trigger >0) {
            robot.tilt(-gamepad.right_trigger);
        } else {
            robot.tilt( 0);
        }
    }
}
