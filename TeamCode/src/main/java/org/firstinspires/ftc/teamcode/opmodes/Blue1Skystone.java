package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.AQUA;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.OPEN;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.TILTED;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

@Autonomous
public class Blue1Skystone extends BlueOpMode {
    @Override
    protected void execute() {
        double power = 0.5;

        robot.drive(power,0,12);

        robot.turn(power, 10);

        while (isContinuing() &&
            robot.findNearestStone(true) == null &&
            robot.getOrientation().firstAngle > -45)
            robot.drive(0, 0.1);

        robot.drive(0, 0);

        if (robot.findNearestStone(true) != null) {
            robot.pickUpStone(true);
        } else {
            robot.claw(OPEN);
            robot.tilt(UP);
            robot.drive(power, 0, 12);
            robot.claw(CLOSE);
            robot.tilt(TILTED);
        }

        robot.drive(-power, robot.getOrientation().firstAngle, 8);
        robot.drive(power, 90, 32);
        robot.claw(OPEN);
        robot.drive(-power, 90, 16);
    }
}
