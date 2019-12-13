package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "driveUnderBlueBridge2")
public class DriveUnderBlueBridge2 extends MainOpMode {
    @Override
    protected RobotController[] getRobotControllers() {
        return new RobotController[] {
            new AutonmousController(telemetry, gamepad1, robot, "driveUnderBlueBridge2"),
        };
    }
}
