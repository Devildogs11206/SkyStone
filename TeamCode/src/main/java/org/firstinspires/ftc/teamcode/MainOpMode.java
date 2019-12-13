package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainOpMode extends LinearOpMode {
    private Robot robot;
    private RobotController[] robotControllers;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);

        robotControllers = new RobotController[] {
            new AutonmousController(telemetry, gamepad1, robot),
            new DriveController(telemetry, gamepad1, robot),
            new LiftController(telemetry, gamepad2, robot),
            new ClawController(telemetry, gamepad2, robot),
            new SlideController(telemetry, gamepad2, robot),
            new TiltController(telemetry, gamepad2, robot)
        };

        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            for (RobotController controller : robotControllers) {
                controller.execute();
            }

            telemetry.update();
        }
    }
}
