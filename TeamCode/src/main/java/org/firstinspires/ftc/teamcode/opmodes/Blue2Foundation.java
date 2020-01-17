package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Blue2Foundation extends BlueOpMode {
    @Override
    protected void execute() {
        double power = 0.5;
        robot.drive(power,0,4);
        robot.drive(power,45,12 );
        robot.drive(power,0,12);
        robot.drive(-0.1,0,1);
        robot.tilt(UP);
        robot.drive(-power,0,36);
        robot.tilt(BACK);
        robot.drive(-power,90,36);
    }
}
