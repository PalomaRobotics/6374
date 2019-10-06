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
@TeleOp (name = "SuperBots16_17TeleOp", group = "TeleOp")
@Disabled
public class SuperBots16_17TeleOp extends OpMode {

    //variables here

    //these are the wheels
    DcMotor driveLeft;
    DcMotor driveRight;

    DcMotor flicker;

    //DcMotor liftL;
    //DcMotor liftR;

    //DcMotor armL;
    DcMotor armR;

    Servo ramp;
    Servo scoop;

    // int timer;//for a timer
    //boolean aL = false;//arm lock
    // boolean eG = false;//end game

    public SuperBots16_17TeleOp() {

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

        flicker = hardwareMap.dcMotor.get("flicker");

        //liftL = hardwareMap.dcMotor.get("liftL");
        //liftR = hardwareMap.dcMotor.get("liftR");
        //liftR.setDirection(DcMotorSimple.Direction.REVERSE);//once again, reversing the direction the motor spins
                                                            //except this time, its so the lift will go up, instead of
                                                            //one side going up and the other side going down.

        //armL = hardwareMap.dcMotor.get("armL");
        armR = hardwareMap.dcMotor.get("armR");
        armR.setDirection(DcMotorSimple.Direction.REVERSE);//again with the direction reversing, this time though
                                                            //its more of an experiment to see if this is going to
                                                            //keep the arms from slamming into the robot
                                                            //'cause that would be bad


        ramp = hardwareMap.servo.get("ramp");//this is a servo
        scoop = hardwareMap.servo.get("scoop"); // other servo
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


        //LAUNCH BALL

        if(gamepad1.y)//to fire the ball
        {
            flicker.setPower(.75);//FIRE!!!!
        } else
        {
            flicker.setPower(0);//no fire
        }//end to fire the ball


        //ARMS for intake

        if(gamepad1.right_trigger > 0)//to go out
        {
            armR.setPower(-.15);
            //armL.setPower(0);
        }
        else if(gamepad1.left_trigger > 0)//to go in
        {
            armR.setPower(.15);
            //armL.setPower(0);
        }
        else//if not supposed to move
        {
            armR.setPower(0);
            //armL.setPower(0);
        }

        if(gamepad1.right_bumper)//move ramp up?
        {
            //ramp.setPosition(1);
            ramp.setPosition(Range.clip(ramp.getPosition() + .01, ramp.MIN_POSITION, ramp.MAX_POSITION));
        }
        else if(gamepad1.left_bumper)// move back down?
        {
            ramp.setPosition(Range.clip(ramp.getPosition() - .01, ramp.MIN_POSITION, ramp.MAX_POSITION));
        }

        if(gamepad1.x)//move scoop in?
        {
            scoop.setPosition(Range.clip(scoop.getPosition() + .01, scoop.MIN_POSITION, scoop.MAX_POSITION));
        }
        else if(gamepad1.b)//move scoop out?
        {
            scoop.setPosition(Range.clip(scoop.getPosition() - .01, scoop.MIN_POSITION, scoop.MAX_POSITION));
        }
        //ENDGAME ENDGAME ENDGAME ENDGAME

        //LIFT
           /* if (gamepad1.left_bumper)//raise the lift!
            {
                liftR.setPower(.3);
                liftL.setPower(.3);
            } else if(gamepad1.right_bumper)
            {
                liftR.setPower(-.3);
                liftL.setPower(-.3);
            }else //don't raise the lift
            {
                liftL.setPower(0);
                liftR.setPower(0);
            }*/



        //GET THAT BALL
           /* if (gamepad1.dpad_left)//will swing arms back to pick up ball, do before you lift ><
            {
                armR.setPower(-.25);
               // armL.setPower(-.25);
            }else if(gamepad1.dpad_right)
            {
                armR.setPower(.25);
                //armL.setPower(.25);
            }else
            {
                armR.setPower(0);
                armL.setPower(0);
            }*/

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

