package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "driveUnderBlueBridge1")
public class DriveUnderBlueBridge1 extends MainOpMode {
    @Override
    protected RobotController[] getRobotControllers() {
        return new RobotController[] {
            new AutonmousController(telemetry, gamepad1, robot, "driveUnderBlueBridge1"),
        };
    }
}
