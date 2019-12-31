package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends OpMode {
    private RobotController[] robotControllers;

    @Override
    protected void execute() {
        robotControllers = new RobotController[]{
            new DriveController(this),
            new LiftController(this),
            new ClawController(this),
            new SlideController(this),
            new TiltController(this),
        };

        while (opModeIsActive()) {
            for (RobotController controller : robotControllers) {
                controller.execute();
            }

            robot.addTelemetry();

            telemetry.update();
        }
    }
}
