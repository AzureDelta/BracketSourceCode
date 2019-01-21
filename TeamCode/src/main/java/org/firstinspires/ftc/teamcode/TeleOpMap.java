package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOpMap {
    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;
    public DcMotor open = null;
    public DcMotor close = null;
    public DcMotor slide = null;
    public DcMotor intake = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TeleOpMap() {
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
        open = hwMap.get(DcMotor.class, "open");
        close = hwMap.get(DcMotor.class, "close");
        slide = hwMap.get(DcMotor.class, "s");
        intake = hwMap.get(DcMotor.class, "ir");

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.REVERSE);
        open.setDirection(DcMotor.Direction.FORWARD);//requires testing
        close.setDirection(DcMotor.Direction.FORWARD);//requires testing
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);
        slide.setPower(0);
        open.setPower(0);
        close.setPower(0);
        intake.setPower(0);

        //set zero power behavior
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        open.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        close.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        open.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        close.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Not using encoders for non drive train to allow for more direct control of power.
        //Arm uses encoders to make sure motors stay in sync
        //same with intake
    }
}