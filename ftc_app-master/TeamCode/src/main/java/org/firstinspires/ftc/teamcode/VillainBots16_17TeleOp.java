package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by robotics on 11/7/2016.
 */
@TeleOp (name = "VillainBots16_17TeleOp", group = "TeleOp")
@Disabled
public class VillainBots16_17TeleOp extends OpMode {

    //variables here

    //these are the wheels
    DcMotor driveLeft;
    DcMotor driveRight;

    DcMotor flicker;

    DcMotor lift;

    DcMotor intakeArm;
    DcMotor intakeWheel;

    DcMotor armL;
    DcMotor armR;

    Servo ramp;

    int timer;//for a timer
    boolean aL = false;//arm lock
    boolean eG = false;//end game

    public VillainBots16_17TeleOp() {

    }//leave empty

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        driveLeft = hardwareMap.dcMotor.get("left");
        driveRight = hardwareMap.dcMotor.get("right");
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);//this is setting the motor to run in the
                                                                //opposite direction, so that the robot will drive
                                                                //forward, instead of spinning forever like a top or something

       //flicker = hardwareMap.dcMotor.get("flicker");

       // lift = hardwareMap.dcMotor.get("lift");

      //  intakeArm = hardwareMap.dcMotor.get("intakeArm");
      //  intakeWheel = hardwareMap.dcMotor.get("intakeWheel");

      //  armL = hardwareMap.dcMotor.get("armL");
        //armR = hardwareMap.dcMotor.get("armR");
        //armR.setDirection(DcMotorSimple.Direction.REVERSE);//again with the direction reversing, this time though
                                                            //its more of an experiment to see if this is going to
                                                            //keep the arms from slamming into the robot
                                                            //'cause that would be bad


        ramp = hardwareMap.servo.get("ramp");//this is a servo
    }//initialize

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        //DRIVE

        driveLeft.setPower(-gamepad1.left_stick_y);
        driveRight.setPower(-gamepad1.right_stick_y);

        //INTAKE
        if(gamepad1.left_trigger > 0)//move arm down
        {
            intakeArm.setPower(.1);
        }else if(gamepad1.right_trigger > 0)//move arm up
        {
            intakeArm.setPower(-.1);
        }
        else//dont move
        {
            intakeArm.setPower(0);
        }

        if(gamepad1.x)//run wheel
        {
            intakeWheel.setPower(.25);
        }else if(!gamepad1.x)//dont run wheel
        {
            intakeWheel.setPower(0);
        }


        //LAUNCH BALL

        if(gamepad1.b)//to fire the ball
        {
            flicker.setPower(-.5);//FIRE!!!!
        } else
        {
            flicker.setPower(0);//no fire
        }//end to fire the ball


        //ENDGAME ENDGAME ENDGAME ENDGAME

        //LIFT
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

