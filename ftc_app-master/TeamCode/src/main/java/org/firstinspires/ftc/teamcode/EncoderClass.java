package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EncoderClass {

    private EncoderClass ()
    {

    }

    public static void RunToEncoderValue(DcMotor motorVar, int encVal, double speed)
    {
                //telemetry.addData("RIGHT MOTOR POS:", right.getCurrentPosition());
                //motorVar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorVar.setTargetPosition(encVal);
                motorVar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorVar.setPower(speed);
    }

    //public static void RunCycle (DcMotor motorVar, double speed)
    //{
    //    double startingPosition = motorVar.getCurrentPosition();
    //}

    public static void RunToEncoderDegree(DcMotor motorVar, int degree, double speed)
    {
        //telemetry.addData("RIGHT MOTOR POS:", right.getCurrentPosition());
        degree = (int)(degree * (1120 / 360));
        motorVar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVar.setTargetPosition(degree);
        motorVar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorVar.setPower(speed);

        while (motorVar.isBusy())
        {

        }
        motorVar.setPower(0.0);
    }


    public static void RunToEncoderDegree(DcMotor motorVar, int degree, double speed, Telemetry telemetry)
    {
        //telemetry.addData("RIGHT MOTOR POS:", right.getCurrentPosition());
        degree = (int)(degree * (1120 / 360));
        motorVar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVar.setTargetPosition(degree);
        motorVar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorVar.setPower(speed);

        while (motorVar.isBusy())
        {
            telemetry.addData("RIGHT MOTOR POS:", motorVar.getCurrentPosition());
            telemetry.update();
        }
        motorVar.setPower(0.0);
    }

}
