package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.internal.Robot;
import org.firstinspires.ftc.teamcode.controllers.RobotController;

public abstract class OpMode extends LinearOpMode {
    public Robot robot;

    private RobotController[] robotControllers;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.init();
        waitForStart();
        execute();
    }

    public boolean isActive() {
        return opModeIsActive();
    }

    public boolean isContinuing() {
        return isActive() && !gamepad1.back && !gamepad2. back;
    }

    public void yield() {
        sleep(50);
    }

    protected abstract void execute();
}
