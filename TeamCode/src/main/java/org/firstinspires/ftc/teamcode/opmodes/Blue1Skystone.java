package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.OPEN;
import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.OUT;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Blue1Skystone extends OpMode {
    @Override
    protected void execute() {
        robot.slide(OUT);
        robot.tilt(UP);
        robot.claw(OPEN);

        robot.drive(0.5,0,32);

        robot.claw(CLOSE);
        robot.tilt(BACK);

        robot.drive(-0.5,0,12);

        robot.drive(0.5,90,64);
        robot.claw(OPEN);

        robot.drive(-0.5,90,32);
    }
}
