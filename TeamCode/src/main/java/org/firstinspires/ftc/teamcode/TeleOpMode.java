package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public boolean isActive() {
        return opModeIsActive();
    }

    public boolean isContinue() {
        return isActive() &&
            (!gamepad1.left_stick_button || !gamepad1.right_stick_button) &&
            (!gamepad2.left_stick_button || !gamepad2.right_stick_button);
    }
}
