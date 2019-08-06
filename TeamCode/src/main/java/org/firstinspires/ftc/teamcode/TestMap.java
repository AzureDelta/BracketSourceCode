package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestMap {
    /* Public OpMode members. */
    public DcMotor motorFR = null;

    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 2.0/3.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV * (80/120) / (WHEEL_DIAMETER_INCHES * Math.PI)); //calculates counts per inch
    static final double FEET = 12 * INCHES;
    //59.41785 base inches (no mec compensation)
    //84.02952 per inch
    int OFFSET = 0;
    public static final double M = (2 / Math.sqrt(2));
    public static final double DRIVE_SPEED = 0.5;

    public static final double MID_SERVO = 0.5;//legacy code, can be removed
    //public ColorSensor colorSensor;//legacy code, can be removed

    //YellowJacket for Close
    //NeverRest for Open

    //NeverRest 40 motor: Diameter = 0.85 inches
    //NeverREST: 120 rpm
    //Circumference = 2.669 inches
    //NeverRest: IPM: 320.28 inches/min
    //GoBilda Yellow Jacket motor: Diameter = 1 inch
    //GoBilda: 84 rpm
    //Circumference: 3.14 inches
    //GoBilda: IPM: 263.76 inches/min
    //YellowJacket is close
    //NeverRest is open
    //Run NeverRest at 82.35% to get same power

    /* local OpMode members. */

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* Constructor */
    public TestMap() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFR = hwMap.get(DcMotor.class, "test");


        motorFR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if motors are facing outward


        // Sets zero power behavior to brake for more precise movement
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Resets encoder values to prevent the robot from freaking out as soon as we init
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Set all motors to zero power
        motorFR.setPower(0);

    }



    public void drive(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;
        // Determines new target position, and pass to motor controller
        targetFR = motorFR.getCurrentPosition() + distance;
        motorFR.setTargetPosition(targetFR);


        // Sets motors to run to a given encoder value
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Motors are set to run at a certain speed until one reaches its target position
        while (motorFR.isBusy()) {
            motorFR.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        motorFR.setPower(0);
        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
    }

    public void strafe(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;
        // Determines new target position, and pass to motor controller

        targetFR = motorFR.getCurrentPosition() - distance;
        motorFR.setTargetPosition(targetFR);


        // Sets motors to run to a given encoder value
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors are set to run at a certain speed until one reaches its target position
        while (motorFR.isBusy()) {
            motorFR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        motorFR.setPower(0);

        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
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