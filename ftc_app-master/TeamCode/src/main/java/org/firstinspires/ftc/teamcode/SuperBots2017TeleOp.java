package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robotics on 11/7/2016.
 */
@TeleOp (name = "SuperBots2017TeleOp", group = "TeleOp")
@Disabled
public class SuperBots2017TeleOp extends OpMode
{

    //variables here

    DcMotor driveLeft; //the left wheel
    DcMotor driveRight; //the right wheel

    DcMotor shooterA;//one part of the outtake
    DcMotor shooterB;//the other part of the outtake

    DcMotor treadIntake;//the intake tread

    public SuperBots2017TeleOp() {

    }//leave empty

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init()
    {
        driveLeft = hardwareMap.dcMotor.get("left");
        driveRight = hardwareMap.dcMotor.get("right");
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterA = hardwareMap.dcMotor.get("shootL");
        shooterB = hardwareMap.dcMotor.get("shootR");
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        treadIntake = hardwareMap.dcMotor.get("tread");

    }//initialize

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop()
    {
        //Methods.Drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        //motorClass myMotor = new motorClass(42);

        //if(gamepad1.x)
            //myMotor.setBooYeah(79);

        //telemetry.addData("Text", myMotor.getBooYeah());
        if(((-gamepad1.left_stick_y) * 1.5)>1)
            driveLeft.setPower(1);
         else if(((-gamepad1.left_stick_y) * 1.5)<-1)
            driveLeft.setPower(-1);
        else
        driveLeft.setPower((-gamepad1.left_stick_y) * 1.5);

        if(((-gamepad1.right_stick_y) * 1.5)>1)
            driveRight.setPower(1);
        else if(((-gamepad1.right_stick_y) * 1.5)<-1)
            driveRight.setPower(-1);
        else
        driveRight.setPower((-gamepad1.right_stick_y) * 1.5);

        if(gamepad1.left_bumper)
        {
            treadIntake.setPower(-1);
        }
        if(gamepad1.right_bumper)
        {
            treadIntake.setPower(1);
        }
        if(gamepad1.y)
        {
            treadIntake.setPower(0);
        }

        if(gamepad1.a)
        {
            shooterA.setPower(-1);
            shooterB.setPower(1);
        }
        if(gamepad1.b)
        {
            shooterA.setPower(0);
            shooterB.setPower(0);
        }



    }//end main loop

    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    //DO NOT TOUCH
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}

