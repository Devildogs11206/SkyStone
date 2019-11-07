package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainOpMode extends OpMode {
    private Robot robot;
    private RobotController driveController;
    private RobotController liftController;

    @Override
    public void init() {
        driveController = new DriveController(telemetry,gamepad1,robot);
        liftController = new LiftController(telemetry, gamepad1, robot);
    }

    @Override
    public void loop() {
        driveController.execute();
        liftController.execute();
    }
}
