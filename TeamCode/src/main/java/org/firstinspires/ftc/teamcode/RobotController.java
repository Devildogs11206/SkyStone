package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotController {
    public Telemetry telemetry;
    public Gamepad gamepad;
    public Robot robot;

    public RobotController(Telemetry telemetry, Gamepad gamepad, Robot robot) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.robot = robot;
    }

    public abstract void execute();
}
