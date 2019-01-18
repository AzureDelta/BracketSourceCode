package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

public class AutonMap {
    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;

    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV * (80/120) / (WHEEL_DIAMETER_INCHES * Math.PI)); //calculates counts per inch
    static final double FEET = 12 * INCHES;
    double OFFSET = 0;
    public static final double M = (2 / Math.sqrt(2));
    public static final double DRIVE_SPEED = 0.5;

    public static final double MID_SERVO = 0.5;//legacy code, can be removed
    //public ColorSensor colorSensor;//legacy code, can be removed


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* Constructor */
    public AutonMap() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFR = hwMap.get(DcMotor.class, "fr");
        motorFL = hwMap.get(DcMotor.class, "fl");
        motorRR = hwMap.get(DcMotor.class, "rr");
        motorRL = hwMap.get(DcMotor.class, "rl");

        motorFR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if motors are facing outward
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if motors are facing outward
        motorRR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if motors are facing outward
        motorRL.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if motors are facing outward

        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);

        // Sets zero power behavior to brake for more precise movement
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Resets encoder values to prevent the robot from freaking out as soon as we init
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void drive(double speed, double distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets motors to run to a given encoder value
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        // Determines new target position, and pass to motor controller
        targetFL = motorFL.getCurrentPosition() + (int) (distance);
        targetFR = motorFR.getCurrentPosition() + (int) (distance);
        targetRL = motorRL.getCurrentPosition() + (int) (distance);
        targetRR = motorRR.getCurrentPosition() + (int) (distance);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);
        motorRL.setTargetPosition(targetRL);
        motorRR.setTargetPosition(targetRR);

        // Motors are set to run at a certain speed until one reaches its target position
        while (motorFL.isBusy() && motorFR.isBusy() && motorRL.isBusy() && motorRR.isBusy()) {
            motorFL.setPower(Math.abs(speed));
            motorFR.setPower(Math.abs(speed));
            motorRL.setPower(Math.abs(speed));
            motorRR.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
    }

    public void strafe(double speed, double distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets motors to run to a given encoder value
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        // Determines new target position, and pass to motor controller
        targetFL = motorFL.getCurrentPosition() + (int) (distance);
        targetFR = motorFR.getCurrentPosition() + (int) (distance);
        targetRL = motorRL.getCurrentPosition() + (int) (distance);
        targetRR = motorRR.getCurrentPosition() + (int) (distance);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);
        motorRL.setTargetPosition(targetRL);
        motorRR.setTargetPosition(targetRR);

        // Motors are set to run at a certain speed until one reaches its target position
        while (motorFL.isBusy() && motorFR.isBusy() && motorRL.isBusy() && motorRR.isBusy()) {
            motorFL.setPower(Math.abs(-speed));
            motorFR.setPower(Math.abs(speed));
            motorRL.setPower(Math.abs(speed));
            motorRR.setPower(Math.abs(-speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
    }

    /*    public void actuate(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.actuator.setPower(speed);
        }
    }*/

    /*public void intake(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.intake.setPower(speed);
        }
    }*/
}