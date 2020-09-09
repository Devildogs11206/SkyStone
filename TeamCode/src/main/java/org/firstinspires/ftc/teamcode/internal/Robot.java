package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class Robot {
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 1120 / INCHES_PER_ROTATION;

    private OpMode opMode;

    private BNO055IMU imu;

    private DcMotor left_drive;
    private DcMotor right_drive;

    //Creating motors for mechanum wheels
    private DcMotor mec_lf;
    private DcMotor mec_lb;
    private DcMotor mec_rf;
    private DcMotor mec_rb;

    //so they can all be "initialized" at once
    private DcMotor[] mecs = {mec_lf,mec_lb,mec_rf,mec_rb};

    public Robot(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(Alliance alliance) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        left_drive = hardwareMap.get(DcMotor.class, "left_front");
        left_drive.setDirection(FORWARD);

        right_drive = hardwareMap.get(DcMotor.class,"right_front");
        right_drive.setDirection(REVERSE);

        //defining all mechanum wheel motors from hardwareMap
        for(int i=0;i<mecs.length;i++){
            mecs[i] = hardwareMap.get(DcMotor.class,mecs[i].toString());
        }

    }

    public void drive(double power, double turn) {
        if (!opMode.isContinuing()) return;

        double left = power + turn;
        double right = power - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        left_drive.setPower(left);
        right_drive.setPower(right);
    }

    //mecanum drive method
    public void mecanumDrive(double lf, double lb, double rf, double rb){

        //not doing much because everything done in MecanumController
        mec_lf.setPower(lf);
        mec_lb.setPower(lb);
        mec_rf.setPower(rf);
        mec_rb.setPower(rb);

    }

    public void addTelemetry(){
        Telemetry telemetry = opMode.telemetry;

        //orientation = getOrientation();

        telemetry.addData("Drive","%.2f Pow", opMode.gamepad2.left_stick_y);
        telemetry.addData("Turn","%.2f Pow", opMode.gamepad2.right_stick_x);
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_drive.getPower(), left_drive.getCurrentPosition());
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_drive.getPower(), left_drive.getCurrentPosition());
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_drive.getPower(), left_drive.getCurrentPosition());


        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_drive.getPower(), left_drive.getCurrentPosition());
        telemetry.addData("Drive (RF)","%.2f Pow, %d Pos", right_drive.getPower(), right_drive.getCurrentPosition());

    }

    private double clamp(double min, double max, double value) {
        return value >= 0 ?
            Math.min(max, Math.max(min, value)) :
            Math.min(-min, Math.max(-max, value));
    }

    private void sleep(double seconds) {
        try {
            Thread.sleep((long)(1000 * seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}