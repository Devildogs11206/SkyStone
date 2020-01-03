package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

@Autonomous
public class Red1PickUpSkystoneOpMode extends OpMode {
    @Override
    protected void execute() {
        robot.slide(0.5,1);
        robot.tilt(0.5,1.25);
        robot.openClaw();

        robot.drive(0.5,0,32);

        robot.closeClaw();
        robot.tilt(-0.5,1.3);

        robot.drive(-0.5,0,24);

        robot.drive(0.5,-90,64);
        robot.openClaw();

        robot.drive(-0.5,-90,32);
    }
}
