package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends MainOpMode {
    @Override
    protected RobotController[] getRobotControllers() {
        return new RobotController[] {
            new DriveController(telemetry, gamepad1, robot),
            new LiftController(telemetry, gamepad2, robot),
            new ClawController(telemetry, gamepad2, robot),
            new SlideController(telemetry, gamepad2, robot),
            new TiltController(telemetry, gamepad2, robot)
        };
    }
}
