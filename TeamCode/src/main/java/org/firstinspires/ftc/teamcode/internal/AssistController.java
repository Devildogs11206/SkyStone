package org.firstinspires.ftc.teamcode.internal;

import org.firstinspires.ftc.teamcode.controllers.RobotController;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class AssistController extends RobotController {
    public AssistController(OpMode opMode) {
        super(opMode);
    }
    @Override
    public void execute() {
        if(gamepad1.x){robot.pickUpStone();}
    }
}
