package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Red2Bridge extends OpMode {
    @Override
    protected void execute() {
        robot.drive(.5 ,0,4);
        robot.drive(.5,90,34);
    }
}
