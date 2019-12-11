package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainOpMode extends OpMode {
    private Robot robot;
    private RobotController[] robotControllers;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap);

        robotControllers = new RobotController[] {
            new DriveController(telemetry,gamepad1,robot),
            new LiftController(telemetry, gamepad1,robot),
            new ClawController(telemetry,gamepad1,robot),
            new SlideController(telemetry,gamepad1,robot),
            new TiltController(telemetry, gamepad1, robot)
        };

        robot.init();

    }

    @Override
    public void loop() {
        for(RobotController controller : robotControllers){
            controller.execute();
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
