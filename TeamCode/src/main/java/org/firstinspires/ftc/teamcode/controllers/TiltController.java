package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.TILTED;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

public class TiltController extends RobotController {
    public TiltController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_trigger > 0) {
            robot.tilt(gamepad2.left_trigger);
            robot.setLights(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        if (gamepad2.left_trigger < 0) {
            robot.tilt(gamepad2.left_trigger);
            robot.setLights(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        else if (gamepad2.right_trigger > 0) robot.tilt(-gamepad2.right_trigger);

        else if(gamepad2.right_stick_button && gamepad2.y){robot.tilt(UP);}
        else if(gamepad2.right_stick_button && gamepad2.b){robot.tilt(TILTED);}
        else if(gamepad2.right_stick_button && gamepad2.a){robot.tilt(DOWN);}
        else robot.tilt( 0);
    }
}
