package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawController extends RobotController{
    public ClawController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        if (gamepad.left_bumper) robot.openClaw();
        if (gamepad.right_bumper) robot.closeClaw();
    }
}
