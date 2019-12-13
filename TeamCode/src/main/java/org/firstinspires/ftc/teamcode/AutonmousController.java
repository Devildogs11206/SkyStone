package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonmousController extends RobotController {
    public AutonmousController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        super(telemetry, gamepad, robot);
    }

    @Override
    public void execute() {
        if (!gamepad.start) return;

        driveUnderBridge();
    }

    protected void driveUnderBridge() {
        // TODO
    }
}
