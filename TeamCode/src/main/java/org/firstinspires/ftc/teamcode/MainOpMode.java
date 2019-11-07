package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainOpMode extends OpMode {
    private Robot robot;
    private RobotController robotcontroller;

    @Override
    public void init() {
        robotcontroller = new LiftController(telemetry, gamepad1, robot);
    }

    @Override
    public void loop() {
        robotcontroller.execute();
    }
}
