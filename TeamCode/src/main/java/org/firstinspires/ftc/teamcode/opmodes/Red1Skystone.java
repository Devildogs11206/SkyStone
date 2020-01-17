package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.OPEN;
import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.OUT;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.TILTED;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Red1Skystone extends RedOpMode {
    @Override
    protected void execute() {
        double power = 0.5;

        robot.drive(power,0,12);

        robot.turn(power, 30);

        while (isContinuing() &&
            !robot.isSkystoneVisible() &&
            robot.findNearestStone(true) == null &&
            robot.getOrientation().firstAngle > -10)
            robot.drive(0, 0.1);

        robot.drive(0, 0);

        if (robot.isSkystoneVisible()) {
            robot.pickUpSkystone();
        } else if (robot.findNearestStone(true) != null) {
            robot.pickUpStone(true);
        } else {
            robot.claw(OPEN);
            robot.tilt(UP);
            robot.drive(power, 0, 12);
            robot.claw(CLOSE);
            robot.tilt(TILTED);
        }

        robot.drive(-power, robot.getOrientation().firstAngle, 8);
        robot.drive(power, -90, 64);
        robot.claw(OPEN);
        robot.drive(-power, -90, 32);
    }
}
