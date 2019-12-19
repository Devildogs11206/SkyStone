package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpMode extends LinearOpMode {
    protected Robot robot;
    private RobotController[] robotControllers;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        robot.init();
        waitForStart();
        execute();
    }

    protected abstract void execute();
}
