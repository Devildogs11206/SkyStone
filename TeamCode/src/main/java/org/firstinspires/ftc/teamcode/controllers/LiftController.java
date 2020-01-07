package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.LVL_ONE;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.LVL_THREE;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.LVL_TWO;

public class LiftController extends RobotController {
    private double liftStrength = 0.5;

    public LiftController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.dpad_up) robot.lift(liftStrength);
        else if (gamepad2.dpad_down) robot.lift(-liftStrength);
        else if(gamepad2.start && gamepad2.y){robot.lift(0.5,LVL_THREE);}
        else if(gamepad2.start && gamepad2.b){robot.lift(0.5,LVL_TWO);}
        else if(gamepad2.start && gamepad2.a){robot.lift(0.5,DOWN);}
        else if(gamepad2.start && gamepad2.x){robot.lift(0.5,LVL_ONE);}
        else robot.lift( 0);
    }
}
