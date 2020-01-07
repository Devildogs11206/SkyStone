package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Red1Skystone extends OpMode {
    @Override
    protected void execute() {

        robot.slide(0.5,FORWARD);
        robot.tilt(0.5,UP);
        robot.openClaw();

        robot.drive(0.5,0,32);

        robot.closeClaw();
        robot.tilt(-0.5,BACK);

        robot.drive(-0.5,0,24);

        robot.drive(0.5,-90,64);
        robot.openClaw();

        robot.drive(-0.5,-90,32);
    }
}
