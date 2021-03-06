package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.FOUNDATION;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Red2Foundation extends RedOpMode {
    @Override
    protected void execute() {
        double power = 0.5;
        robot.drive(power,0,4);
        robot.drive(power,-60,16);
        robot.drive(power,0,14);
        robot.drive(-0.1,0,1);
        robot.tilt(FOUNDATION);
        robot.drive(-power,0,18);
        robot.turn(power,-90);
        robot.tilt(BACK);
        robot.drive(-power,-90,42);
    }
}
