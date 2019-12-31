package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

@Autonomous
public class Red2DriveUnderBridgeOpMode extends OpMode {
    @Override
    protected void execute() {
        robot.drive(.5 ,0,8);
        robot.turn(-90,.5);
        robot.drive(.5,0,34);
    }
}
