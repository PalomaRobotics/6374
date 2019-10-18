package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class EncoderClass {

    enum MotorType
    {
        NeveRest40,
        NeveRest60
    }

    public static void RunToEncoderValue(DcMotor motorObject, int encValue, double speed, boolean holdWhenFinished)
    {
        //telemetry.addData("RIGHT MOTOR POS:", right.getCurrentPosition());
        motorObject.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObject.setTargetPosition(encValue);
        motorObject.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorObject.setPower(speed);

        while (motorObject.isBusy()) //wait while the motor moves to the desired position
        {
        }

        if(!holdWhenFinished) //if you don't want me to hold the position...
            motorObject.setPower(0.0); //stop the motor when we arrive
    }

    public static void RunToEncoderValueAsync(final DcMotor motorObject, final int encValue, final double speed, final boolean holdWhenFinished)
    {
        //This method calls the regular RunToEncoderValue method asynchronously so other code continues to run while the motor moves to the desired position
        new Thread(new Runnable() {
            public void run() {
                RunToEncoderValue(motorObject, encValue, speed, holdWhenFinished);
            }
        }).start();
    }

    //public static void RunCycle (DcMotor motorVar, double speed)
    //{
    //    double startingPosition = motorVar.getCurrentPosition();
    //}

    public static void RunToEncoderDegree(DcMotor motorObject, MotorType motorType, int degree, double speed, boolean holdWhenFinished)
    {
        //telemetry.addData("RIGHT MOTOR POS:", right.getCurrentPosition());
        switch(motorType)
        {
            case NeveRest40:
                degree = (int)(degree * (1120 / 360));
                break;
            case NeveRest60:
                degree = (int)(degree * (1680 / 360));
                break;
        }

        motorObject.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObject.setTargetPosition(degree);
        motorObject.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorObject.setPower(speed);

        while (motorObject.isBusy()) //wait while the motor moves to the desired position
        {
        }

        if(!holdWhenFinished) //if you don't want me to hold the position...
            motorObject.setPower(0.0); //stop the motor when we arrive


    }

    public static void RunToEncoderDegreeAsync(final DcMotor motorObject, final MotorType motorType, final int degree, final double speed, final boolean holdWhenFinished)
    {
        //This method calls the regular RunToEncoderDegree method asynchronously so other code continues to run while the motor moves to the desired position
        new Thread(new Runnable() {
            public void run() {
                RunToEncoderDegree(motorObject, motorType, degree, speed, holdWhenFinished);
            }
        }).start();
    }


}
