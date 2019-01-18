package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonMap {
    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;

    public static final double MID_SERVO = 0.5;//legacy code, can be removed
    //public ColorSensor colorSensor;//legacy code, can be removed


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();

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
}