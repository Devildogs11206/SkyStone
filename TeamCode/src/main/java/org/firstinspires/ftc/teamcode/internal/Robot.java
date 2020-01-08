package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
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
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.OPEN;
import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.IN;
import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.OUT;

public class Robot {
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 1120 / INCHES_PER_ROTATION;

    private OpMode opMode;

    private BNO055IMU imu;

    private DcMotor left_front;
    private DcMotor left_rear;
    private DcMotor right_front;
    private DcMotor right_rear;

    private DcMotor slide;
    private DigitalChannel slide_limit_front;
    private DigitalChannel slide_limit_rear;

    private DcMotor tilt;
    private DigitalChannel tilt_limit;
    private ModernRoboticsI2cCompassSensor tilt_accelerometer;

    private DcMotor lift;

    private Servo claw_left;
    private Servo claw_right;

    private Servo stick;

    private VisionThread visionThread;

    public WebcamName webcamName;
    public int cameraMonitorViewId;
    public int tfodMonitorViewId;

    public Position position = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public Orientation orientation = new Orientation();
    public List<Recognition> recognitions = null;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        HardwareMap hardwareMap = opMode.hardwareMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear = hardwareMap.get(DcMotor.class,"left_rear");
        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front = hardwareMap.get(DcMotor.class,"right_front");
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setDirection(DcMotor.Direction.REVERSE);

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide_limit_front = hardwareMap.get(DigitalChannel.class, "slide_limit_front");
        slide_limit_front.setMode(DigitalChannel.Mode.INPUT);
        slide_limit_rear = hardwareMap.get(DigitalChannel.class, "slide_limit_rear");
        slide_limit_rear.setMode(DigitalChannel.Mode.INPUT);

        tilt = hardwareMap.get(DcMotor.class, "tilt");
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt_limit = hardwareMap.get(DigitalChannel.class, "tilt_limit");
        tilt_accelerometer = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "tilt_accelerometer");

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        claw_left = hardwareMap.get(Servo.class, "claw_left");
        claw_right = hardwareMap.get(Servo.class, "claw_right");

        stick = hardwareMap.get(Servo.class, "stick");

        webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());

        visionThread = new VisionThread(opMode,this);
        visionThread.start();
    }

    public void calibrate() {
        slide(IN);
    }

    public void start() {
        slide(OUT);
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

    public void drive(double drive, double heading, double inches) {
        turn(drive, heading);
        resetEncoders();

        int targetPosition = (int)(inches * TICKS_PER_INCH);
        int position = 0;

        double remainder, turn;

        while (opMode.isContinuing() && targetPosition - position > 0) {
            remainder = getRemainderLeftToTurn(heading);
            turn = remainder / 50;
            drive(drive, turn);

            position = (
                Math.abs(left_front.getCurrentPosition()) +
                Math.abs(left_rear.getCurrentPosition()) +
                Math.abs(right_front.getCurrentPosition()) +
                Math.abs(right_rear.getCurrentPosition())
            ) / 4;
        }

        this.drive(0,0);
    }

    public void turn(double drive, double heading) {
        double remainder, turn;

        do {
            remainder = getRemainderLeftToTurn(heading);
            turn = remainder / Math.abs(remainder) * drive;
            drive(0, turn);
        } while ((remainder < -1 || remainder > 1) && opMode.isActive());

        drive(0,0);
    }

    public enum SlidePosition{ IN, OUT }

    public void slide(SlidePosition position) {
        final double power = 0.5;

        if (position == OUT) {
            while(opMode.isContinuing() && slide_limit_rear.getState()) {
                slide.setPower(power);
            }
        }

        if (position == IN) {
            while(opMode.isContinuing() && slide_limit_front.getState() && tilt_accelerometer.getAcceleration().yAccel > 9) {
                slide.setPower(-power);
            }
        }

        slide.setPower(0);
    }

    public void tilt(double power) {
        tilt.setPower(power);
    }

    public static class TiltPosition {
        public static final int BACK = 0;
        public static final int TILTED = 1000;
        public static final int UP = 3000;
    }

    public void tilt(int position) {
        tilt.setTargetPosition(position);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setPower(0.5);

        while(tilt.isBusy() && opMode.isContinuing());
    }

    public static class TiltAccel {
        public static final double BACK = 10;
        public static final double TILTED = 9;
        public static final double UP = 0.5;
    }

    public void tiltAccel(double accel) {

    }

    public void lift(double power) {
        int minPos = 0;
        int maxPos = 10000;

        int position = lift.getCurrentPosition();

        if((power > 0 && position < maxPos) || (power < 0 && position > minPos)){
            lift.setPower(power);
        } else {
            lift.setPower(0);
        }
    }

    public void stickToggle(){
        if (stick.getPosition() > 0.5) {
            stick.setPosition(0.25);
        } else {
            stick.setPosition(0.75);
        }
    }

    public enum ClawPosition { OPEN, CLOSE }

    public void claw(ClawPosition position) {
        claw_left.setPosition(position == OPEN ? 1 : 0.15);
        claw_right.setPosition(position == CLOSE ? 0.25 : 0.85);
        sleep(0.25);
    }

    public void addTelemetry(){
        Telemetry telemetry = opMode.telemetry;

        orientation = getOrientation();

        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());
        telemetry.addData("Drive (LR)","%.2f Pow, %d Pos", left_rear.getPower(), left_rear.getCurrentPosition());
        telemetry.addData("Drive (RF)","%.2f Pow, %d Pos", right_front.getPower(), right_front.getCurrentPosition());
        telemetry.addData("Drive (RR)","%.2f Pow, %d Pos", right_rear.getPower(), right_rear.getCurrentPosition());
        telemetry.addData("Slide","%.2f Pow, %d Pos", slide.getPower(), slide.getCurrentPosition());
        telemetry.addData("Tilt","%.2f Pow, %d Pos", tilt.getPower(), tilt.getCurrentPosition());
        telemetry.addData("Tilt Limit", tilt_limit.getState());
        telemetry.addData("Tilt Accelerometer", tilt_accelerometer.getAcceleration());
        telemetry.addData("Lift","%.2f Pow, %d Pos", lift.getPower(), lift.getCurrentPosition());
        telemetry.addData("Orientation", orientation);
        telemetry.addData("Target", visionThread.targetVisible);
        telemetry.addData("Position (in)", position);

        if (recognitions != null) {
            telemetry.addData("Recognitions", recognitions.size());

            for (Recognition recognition : recognitions) {
                telemetry.addData(" label", recognition.getLabel());
                telemetry.addData("  left,top", "%.3f , %.3f", recognition.getLeft(), recognition.getTop());
                telemetry.addData("  right,bottom", "%.3f , %.3f", recognition.getRight(), recognition.getBottom());
                telemetry.addData("  height,width", "%.3f , %.3f", recognition.getBottom() - recognition.getTop(), recognition.getRight() - recognition.getLeft());
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

    private double getRemainderLeftToTurn(double heading) {
        double remainder;
        orientation = getOrientation();
        remainder = orientation.firstAngle - heading;
        if (remainder > 180) remainder -= 360;
        if (remainder < -180) remainder += 360;
        return remainder;
    }

    private Orientation getOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void sleep(double seconds) {
        try {
            Thread.sleep((long)(1000 * seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
