package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

public class Robot {
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 1120 / INCHES_PER_ROTATION;
    private static final double INCHES_PER_DEGREE = INCHES_PER_ROTATION / 120;

    private OpMode opMode;

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

    public WebcamName webcamName;
    public int cameraMonitorViewId;
    public int tfodMonitorViewId;

    BNO055IMU imu;

    public Position position = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public Orientation orientation = new Orientation();
    public List<Recognition> recognitions = null;

    private VisionThread visionThread;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        HardwareMap hardwareMap = opMode.hardwareMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


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

        webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());

        visionThread = new VisionThread(opMode,this);
        visionThread.start();
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

        int targetPosition = (int)(inches * TICKS_PER_INCH);
        int position = 0;

        drive(drive, turn);

        while (opMode.isContinuing() && targetPosition - position > 0) {
            position = (
                Math.abs(left_front.getCurrentPosition()) +
                Math.abs(left_rear.getCurrentPosition()) +
                Math.abs(right_front.getCurrentPosition()) +
                Math.abs(right_rear.getCurrentPosition())
            ) / 4;

            opMode.yield();
        }

        this.drive(0,0);
    }

    public void turn(double degrees, double power) {
        double turn = degrees / Math.abs(degrees) * power;
        double inches = Math.abs(degrees * INCHES_PER_DEGREE);
        drive(0, turn, inches);
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
/*
    public void slide(double power, double seconds) {
        slide(power);
        sleep(seconds);
        slide(0);
    }
*/

    public void slide(double power, int position){
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setTargetPosition(position);
        slide.setPower(power);

        while(opMode.isContinuing() && slide.isBusy()) {
            opMode.yield();
        }
    }

    public static class SlidePosition{
        public static final int BACK = 0;
        public static final int FORWARD = 3000;
    }

    public void tilt(double power) {
        tilt.setPower(power);
    }

    public static class TiltPosition{
        public static final int BACK = 0;
        public static final int TILTED = 1500;
        public static final int UP = 3000;
    }

    public void tilt(int position){
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt.setTargetPosition(position);
        tilt.setPower(0.5);

        while(opMode.isContinuing() && tilt.isBusy()) {
            opMode.yield();
        }
    }

    public static class TiltAccel{
        public static final double BACK = 10;
        public static final double TILTED = 9;
        public static final double UP = 0.5;
    }

    public void tiltAccel(double accel){}
/*
    public void tilt(double power, double seconds) {
        tilt(power);
        sleep(seconds);
        tilt(0);
    }
*/
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

    public void lift(double power, int position){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setTargetPosition(position);
        lift.setPower(power);

        while(opMode.isContinuing() && lift.isBusy()) {
            opMode.yield();
        }
    }

    public static class LiftPosition{
        public static final int DOWN = 0;
        public static final int LVL_ONE = 1000;
        public static final int LVL_TWO = 2000;
        public static final int LVL_THREE = 3000;
    }

    public void openClaw() {
        claw_left.setPosition(1);
        claw_right.setPosition(0.25);
        sleep(0.25);
    }

    public void closeClaw() {
        claw_left.setPosition(0.15);
        claw_right.setPosition(0.85);
        sleep(0.25);
    }

    public void addTelemetry(){
        Telemetry telemetry = opMode.telemetry;

        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());
        telemetry.addData("Drive (LR)","%.2f Pow, %d Pos", left_rear.getPower(), left_rear.getCurrentPosition());
        telemetry.addData("Drive (RF)","%.2f Pow, %d Pos", right_front.getPower(), right_front.getCurrentPosition());
        telemetry.addData("Drive (RR)","%.2f Pow, %d Pos", right_rear.getPower(), right_rear.getCurrentPosition());
        telemetry.addData("Slide","%.2f Pow, %d Pos", slide.getPower(), slide.getCurrentPosition());
        telemetry.addData("Tilt","%.2f Pow, %d Pos", tilt.getPower(), tilt.getCurrentPosition());
        telemetry.addData("Lift","%.2f Pow, %d Pos", lift.getPower(), lift.getCurrentPosition());
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Orientation", orientation);
        telemetry.addData("Target", visionThread.targetVisible);
        telemetry.addData("Position (in)", position);


        if (recognitions != null) {

            telemetry.addData("Recognitions", recognitions.size());

            for (Recognition recognition : recognitions) {
                telemetry.addData("  left,top", "%.3f , %.3f", recognition.getLeft(), recognition.getTop());
                telemetry.addData("  right,bottom", "%.3f , %.3f", recognition.getRight(), recognition.getBottom());
                telemetry.addData("  angle", "%.3f", recognition.estimateAngleToObject(DEGREES));
                telemetry.addData("  area", "%.3f", (recognition.getRight() - recognition.getLeft()) * (recognition.getBottom() - recognition.getTop()));
            }
        }

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
            Thread.sleep((long)(1000*seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
