package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

@Autonomous(name="FART NoAbstraction", group="Auto")

/* Declare OpMode members. */


public class FullAutonRewriteNoAbstraction extends LinearOpMode {

    AutonMap robot = new AutonMap();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 80/120;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_ROTATION * 0.5) / (WHEEL_DIAMETER_INCHES * Math.PI); //calculates counts per inch
    public static final double M = (2 / Math.sqrt(2));
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        waitForStart();


        // Play the audio
        //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

        //drive, strafe
        robot.drive(0.5, 12 * INCHES * M);
        sleep(500);
        robot.drive(0.5, -12 * INCHES * M);
        sleep(500);
        //positive dist. is right
        //negative dist. is left
        //first right 3 inches
        robot.strafe(0.5, 12 * INCHES * M);
        sleep(500);
        //then left 3 inches
        robot.strafe(0.5, -12 * INCHES * M);
        sleep(500);

    }

}