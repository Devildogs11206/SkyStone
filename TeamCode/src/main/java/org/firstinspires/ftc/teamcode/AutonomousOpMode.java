package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomousOpMode extends MainOpMode {
    @Override
    protected RobotController[] getRobotControllers() {
        return new RobotController[] {
                new AutonmousController(telemetry, gamepad1, robot),
        };
    }

}
