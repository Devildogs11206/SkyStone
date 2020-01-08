package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.controllers.DriveController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.RobotController;
import org.firstinspires.ftc.teamcode.controllers.SlideController;
import org.firstinspires.ftc.teamcode.controllers.StickController;
import org.firstinspires.ftc.teamcode.controllers.TiltController;

@TeleOp
public class TeleOpMode extends OpMode {
    private RobotController[] robotControllers;

    public TeleOpMode() {
        super(false);
    }

    @Override
    protected void execute() {
        robotControllers = new RobotController[]{
            new DriveController(this),
            new LiftController(this),
            new ClawController(this),
            new SlideController(this),
            new TiltController(this),
            new StickController(this)
        };

        while (isActive()) {
            for (RobotController controller : robotControllers) {
                controller.execute();
            }

            robot.addTelemetry();

            telemetry.update();
        }
    }
}
