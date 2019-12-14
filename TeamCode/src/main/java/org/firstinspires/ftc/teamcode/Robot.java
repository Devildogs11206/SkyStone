package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private static final double TICKS_PER_INCH = 1120 / (3.95 * Math.PI);
    private static final double TICKS_PER_DEGREE = 1120 / 90;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private DcMotor left_front;
    private DcMotor left_rear;
    private DcMotor right_front;
    private DcMotor right_rear;
    private DcMotor slide;
    private DcMotor lift;
    private DcMotor tilt;

    private Servo claw_left;
    private Servo claw_right;

    private DigitalChannel slide_limit_front;
    private DigitalChannel slide_limit_rear;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setDirection(DcMotor.Direction.FORWARD);

        left_rear = hardwareMap.get(DcMotor.class,"left_rear");
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setDirection(DcMotor.Direction.FORWARD);

        right_front = hardwareMap.get(DcMotor.class,"right_front");
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setDirection(DcMotor.Direction.REVERSE);

        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setDirection(DcMotor.Direction.REVERSE);

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.REVERSE);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.REVERSE);

        tilt = hardwareMap.get(DcMotor.class, "tilt");
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw_left = hardwareMap.get(Servo.class, "claw_left");
        claw_right = hardwareMap.get(Servo.class, "claw_right");

        slide_limit_front = hardwareMap.get(DigitalChannel.class, "slide_limit_front");
        slide_limit_front.setMode(DigitalChannel.Mode.INPUT);

        slide_limit_rear = hardwareMap.get(DigitalChannel.class, "slide_limit_rear");
        slide_limit_rear.setMode(DigitalChannel.Mode.INPUT);
    }

    public void drive(double drive, double turn) {
        double left = drive + turn;
        double right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        left_front.setPower(left);
        left_rear.setPower(left);
        right_front.setPower(right);
        right_rear.setPower(right);
    }

    public void drive(double drive, double turn, double inches) {
        resetEncoders();

        int position = 0;

        drive(drive, turn);

        while (position < inches * TICKS_PER_INCH) {
            position = (
                left_front.getCurrentPosition() +
                left_rear.getCurrentPosition() +
                right_front.getCurrentPosition() +
                right_rear.getCurrentPosition()
            ) /4;

            Thread.yield();
        }

        this.drive(0,0);
    }

    public void turn(double degrees, double power) {
        resetEncoders();

        int position = 0;

        drive(0,degrees / Math.abs(degrees) * power);

        while (position < Math.abs((degrees) * TICKS_PER_DEGREE)) {
            position = (
                Math.abs(left_front.getCurrentPosition()) +
                Math.abs(left_rear.getCurrentPosition()) +
                Math.abs(right_front.getCurrentPosition()) +
                Math.abs(right_rear.getCurrentPosition())
            ) /4;

            Thread.yield();
        }
        this.drive(0, 0);
    }

    public void slide(double power) {
        // If the digital channel returns true it's HIGH and the button is unpressed, so we are going to ...
        boolean limitFront = !slide_limit_front.getState();
        boolean limitRear = !slide_limit_rear.getState();

        if ((power > 0 && limitRear) || (power < 0 && limitFront)) {
            slide.setPower(0);
        } else {
            slide.setPower(power);
        }
    }

    public void slide(double power, double seconds) {

        slide(power);
        sleep(seconds);
        slide(0);

    }

    public void tilt(double power) {
        tilt.setPower(power);
    }

    public void tilt(double power, int position){

        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt.setTargetPosition(position);
        tilt.setPower(power);
        while(tilt.isBusy()) {
            Thread.yield();
        }

    }

    public void tilt(double power, double seconds) {

        tilt(power);
        sleep(seconds);
        tilt(0);

    }

    public void lift(double power){
        int minPos = 0;
        int maxPos = 8000;

        int position = lift.getCurrentPosition();

        if((power > 0 && position < maxPos) || (power < 0 && position > minPos)){
            lift.setPower(power);
        } else {
            lift.setPower(0);
        }


    }

    public void openClaw() {
        claw_left.setPosition(1);
        claw_right.setPosition(0.25);
    }

    public void closeClaw() {
        claw_left.setPosition(0.15);
        claw_right.setPosition(0.85);
    }

    public void addTelemetry(){

        telemetry.addLine().addData("Left Front","%.2f Pow, %d Pos",left_front.getPower(),left_front.getCurrentPosition());
        telemetry.addLine().addData("Left Rear","%.2f Pow, %d Pos",left_rear.getPower(),left_rear.getCurrentPosition());
        telemetry.addLine().addData("Right Front","%.2f Pow, %d Pos",right_front.getPower(),right_front.getCurrentPosition());
        telemetry.addLine().addData("Right Rear","%.2f Pow, %d Pos",right_rear.getPower(),right_rear.getCurrentPosition());
        telemetry.addLine().addData("Slide","%.2f Pow, %d Pos",slide.getPower(),slide.getCurrentPosition());
        telemetry.addLine().addData("Tilt","%.2f Pow, %d Pos",tilt.getPower(),tilt.getCurrentPosition());
        telemetry.addLine().addData("Lift","%.2f Pow, %d Pos",lift.getPower(),lift.getCurrentPosition());

    }

    private void resetEncoders() {
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void sleep(double seconds) {
        try {
            Thread.sleep((long) (1000*seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
}
