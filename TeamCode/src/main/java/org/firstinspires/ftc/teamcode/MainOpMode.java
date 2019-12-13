package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public abstract class MainOpMode extends LinearOpMode {
    protected Robot robot;
    private RobotController[] robotControllers;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);

        robotControllers = getRobotControllers();

        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            for (RobotController controller : robotControllers) {
                controller.execute();
            }

            telemetry.update();
        }
    }

    protected abstract RobotController[] getRobotControllers();

}
