package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotController {
    protected OpMode opMode;
    protected Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    protected Robot robot;

    public RobotController(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.gamepad2 = opMode.gamepad1;
        this.gamepad1 = opMode.gamepad2;
        this.robot = opMode.robot;
    }

    public abstract void execute();
}
