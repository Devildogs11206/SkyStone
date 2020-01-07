package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

@Autonomous
public class Red1DriveUnderBridgeOpMode extends OpMode {
    @Override
    protected void execute() {
        robot.drive(.5 ,0,8);
        robot.drive(.5,-90,34);
    }
}
