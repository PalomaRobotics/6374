
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;



public class EncoderClass {

    private DcMotor motorVar;

    public EncoderClass (DcMotor motorVar)
    {
        this.motorVar = motorVar;
        this.motorVar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunToEncoderValue(int encVal, double speed)
    {
        //telemetry.addData("MOTOR POS:", motorVar.getCurrentPosition());

        motorVar.setTargetPosition(encVal);
        motorVar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorVar.setPower(speed);
        //telemetry.update();

    }



    public static void RunToEncoderDegree(DcMotor motorVar, int degree, double speed)
    {
        //telemetry.addData("RIGHT MOTOR POS:", right.getCurrentPosition());
        degree = (int)(degree * (1120 / 360));
        //motorVar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVar.setTargetPosition(degree);
        motorVar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorVar.setPower(speed);
    }


}
