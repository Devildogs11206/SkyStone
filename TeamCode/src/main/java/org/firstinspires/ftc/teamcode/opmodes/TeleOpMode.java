package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.DriveController;
import org.firstinspires.ftc.teamcode.controllers.RecorderController;
import org.firstinspires.ftc.teamcode.controllers.RobotController;
import org.firstinspires.ftc.teamcode.internal.Alliance;

import static org.firstinspires.ftc.teamcode.internal.Alliance.UNKNOWN;

@TeleOp
public class TeleOpMode extends OpMode {
    private RobotController[] robotControllers;

    public TeleOpMode() {
        super(false);
    }

    protected Alliance getAlliance() {
        return UNKNOWN;
    }

    @Override
    protected void execute() {
        robotControllers = new RobotController[]{
            new RecorderController(this),
            new DriveController(this)
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
