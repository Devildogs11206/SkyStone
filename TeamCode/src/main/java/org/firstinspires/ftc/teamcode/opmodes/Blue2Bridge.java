package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Blue2Bridge extends OpMode {
    @Override
    protected void execute() {
        robot.drive(.5,0,4);
        robot.drive(.5,45,12 );
        robot.drive(.5,0,20);
        robot.tilt(UP);
        robot.drive(-.5,0,30);
        robot.turn(.5,90);
        robot.tilt(BACK);
        robot.turn(.5,-45);
        robot.drive(.5,-45,30);
    }
}
