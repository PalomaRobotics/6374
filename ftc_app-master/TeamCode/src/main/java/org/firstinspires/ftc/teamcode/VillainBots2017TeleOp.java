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
@TeleOp (name = "VillainBots2017TeleOp", group = "TeleOp")
@Disabled
public class VillainBots2017TeleOp extends OpMode {

    //variables here

    DcMotor driveFL;
    DcMotor driveFR;
    DcMotor driveBL;
    DcMotor driveBR;

    //DcMotor lift;

    //DcMotor armL;
    //DcMotor armR;

    //Servo als;
    //Servo ars;



    public VillainBots2017TeleOp() {

    }//leave empty

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        driveFL = hardwareMap.dcMotor.get("FL");
        driveFR = hardwareMap.dcMotor.get("FR");
        driveBL = hardwareMap.dcMotor.get("BL");
        driveBR = hardwareMap.dcMotor.get("BR");
        driveFR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBR.setDirection(DcMotorSimple.Direction.REVERSE);

        //lift = hardwareMap.dcMotor.get("lift");

        /*armL = hardwareMap.dcMotor.get("armL");
        armR = hardwareMap.dcMotor.get("armR");
        armR.setDirection(DcMotorSimple.Direction.REVERSE);

        als = hardwareMap.servo.get("als");
        ars = hardwareMap.servo.get("ars");
        als.setPosition(-1);
        ars.setPosition(1);
        */

    }//initialize

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        //DRIVE
        driveFL.setPower(-gamepad1.left_stick_y);
        driveFR.setPower(-gamepad1.right_stick_y);
        driveBL.setPower(-gamepad1.left_stick_y);
        driveBR.setPower(-gamepad1.right_stick_y);



        //LIFT
        /*
            if (gamepad1.left_bumper)//raise the lift!
            {
                lift.setPower(.3);
            } else if(gamepad1.right_bumper)
            {
                lift.setPower(-.3);
            }else //don't raise the lift
            {
                lift.setPower(0);
            }

        if(gamepad1.a) {
            als.setPosition(Range.clip(als.getPosition() + .01, als.MIN_POSITION, als.MAX_POSITION));
            ars.setPosition(Range.clip(ars.getPosition() - .01, ars.MIN_POSITION, ars.MAX_POSITION));
        }

        if(gamepad1.b) {
            als.setPosition(Range.clip(als.getPosition() - .01, als.MIN_POSITION, als.MAX_POSITION));
            ars.setPosition(Range.clip(ars.getPosition() + .01, ars.MIN_POSITION, ars.MAX_POSITION));
        }

        //GET THAT BALL
            if (gamepad1.dpad_left)//will swing arms back to pick up ball, do before you lift ><
            {
                armR.setPower(-.25);
                armL.setPower(-.25);
            }else if(gamepad1.dpad_right)
            {
                armR.setPower(.25);
                armL.setPower(.25);
            }else
            {
                armR.setPower(0);
                armL.setPower(0);
            }
        */

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

