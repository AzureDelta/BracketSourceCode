package org.firstinspires.ftc.teamcode;

public class formatting {
    // Combine drive and turn for blended motion.
    FLvalue =  - Y + X + R;
    FRvalue =  + Y + X + R;
    RLvalue =  - Y - X+ R;
    RRvalue =  + Y - X + R;
    armPower = (gamepad1.right_trigger-gamepad1.left_trigger);

            if(gamepad1.a=true){
        robot.intakeL.setPower(0.75);
        robot.intakeR.setPower(0.75);
    }

    //maxes the values at 1
    FLvalue = Range.clip(FLvalue, -1, 1 );
    FRvalue = Range.clip(FRvalue, -1, 1 );
    RLvalue = Range.clip(RLvalue, -1, 1 );
    RRvalue = Range.clip(RRvalue, -1, 1 );
    armPower = Range.clip(armPower, -1, 1);

}
