package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Blue2Foundation extends BlueOpMode {
    @Override
    protected void execute() {
        double power = 0.75;
        robot.drive(power,0,4);
        robot.drive(power,45,12 );
        robot.drive(power,0,12);
        robot.tilt(UP);
        robot.drive(-power,0,24);
        robot.turn(power,90);
        robot.tilt(BACK);
        robot.drive(-power,45,18);
        robot.drive(-power,90,48);
    }
}
