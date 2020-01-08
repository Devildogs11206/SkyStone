package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.internal.Robot;
import org.firstinspires.ftc.teamcode.controllers.RobotController;

public abstract class OpMode extends LinearOpMode {
    private boolean calibrate = false;

    public Robot robot;

    private RobotController[] robotControllers;

    public OpMode() {
        this(true);
    }

    public OpMode(boolean calibrate) {
        this.calibrate = calibrate;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.init();
        if (calibrate) robot.calibrate();
        waitForStart();
        robot.start();
        execute();
    }

    public boolean isActive() {
        return opModeIsActive();
    }

    public boolean isContinuing() {
        return !isStopRequested() && !gamepad1.back && !gamepad2.back;
    }

    public void yield() {
        sleep(50);
    }

    protected abstract void execute();
}
