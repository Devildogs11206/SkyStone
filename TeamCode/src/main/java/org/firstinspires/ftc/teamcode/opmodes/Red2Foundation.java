package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Red2Foundation extends RedOpMode {
    @Override
    protected void execute() {
        double power = 0.5;
        robot.drive(power,0,4);
        robot.drive(power,-60,15 );
        robot.drive(power,0,12);
        robot.drive(-0.1,0,1);
        robot.tilt(UP);
        robot.drive(-power,0,42);
        robot.tilt(BACK);
        robot.drive(-power,-90,48);
    }
}
