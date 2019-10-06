package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Superbots17/18", group="Competition")
@Disabled
//Created by Brendan Conwell
public class Superbots17_18TeleOp extends LinearOpMode {

    //region Variable Declaration
    // Misc. Variables
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    boolean lastDUpState, lastDDownState, lastYState, lastXState, lastAState, lastBackState, lastLeftStickState, lastDRightState, lastDLeftState, lastRightStickState;
    boolean driverControl = true, endGame = false;

    //Gyro Variables
    ModernRoboticsI2cGyro gyro;

    //Drive Variables
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    int throttle = 1; //used by lb/rb to run drive motors at half speed

    //Arm Variables
    DcMotor armExtendMotor;
    DcMotor armLiftMotor;
    double armUpSpeed = 0.4, armDownSpeed = -0.4, armExtendSpeed = 1;
    int armLiftPosition = 0;
    int armLiftMax = 975, armExtendMax = 5700, armMidpoint = 4500;
    boolean armLifted = false, armFullExtend = false, armSlightExtend = false;

    //Claw Variables
    Servo claw;
    Servo tail;
    private boolean clawOpen = true, clawFullClosed = false, clawClosed = false;

    //Platform Arm Variables
    DcMotor platformArm;
    boolean platformArmDown = false, platformArmOut = false;
    //endregion



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //region object instantiation
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.dcMotor.get("front_left_drive");
        frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        backRightDrive = hardwareMap.dcMotor.get("back_right_drive");
        //gyro is labled sensor_gyro
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        //////////////////////////////////////////////////////////////////
        armExtendMotor = hardwareMap.dcMotor.get("arm_extend");         //
        armLiftMotor = hardwareMap.dcMotor.get("arm_lift");
        platformArm = hardwareMap.dcMotor.get("platform_arm");//
        armExtendMotor.setDirection(DcMotorSimple.Direction.FORWARD);   //
        armLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        platformArm.setDirection(DcMotorSimple.Direction.REVERSE);      //
        claw = hardwareMap.servo.get("servo_claw");                     //
        tail = hardwareMap.servo.get("servo_tail");
        //////////////////////////////////////////////////////////////////

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //endregion

        //Calibrate the Gyro for use
        gyro.calibrate(); //calibrate the gyro. DON'T MOVE THE ROBOT OR SENSOR UNTIL THIS IS DONE!

        //Displays the status of the gyro
        if(gyro.isCalibrating())  {
            telemetry.addData("Gyro Status: ", "Calibrating...");
        }
        else
        {
            telemetry.addData("Gyro Status: ", "Calibrated");
            gyro.resetZAxisIntegrator(); //reset the heading. The sensor only returns a heading for the Z axis
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //encoder.RunToEncoderValue(armExtendMotor, 25, 0.25); //reset to home position
        claw.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

                //region drive movement
                if (gamepad1.right_trigger > 0.5 || gamepad1.right_trigger < -0.5) //if the Left Trigger or Right Trigger is pressed...
                {
                    throttle = 3; //run at full speed speed
                } else {
                    throttle = 1; //else run at a lesser speed
                }

                //Strafing
                //Forward Right
                //if (gamepad1.left_stick_y < -0.85 && gamepad1.left_stick_x > 0.85) {
                    ///*
                //    frontLeftDrive.setPower(gamepad1.left_stick_x / throttle);
                //    frontRightDrive.setPower(0);
                //    backLeftDrive.setPower(0);
                //    backRightDrive.setPower(gamepad1.left_stick_x / throttle);
                    //*/
                    //telemetry.addData("strafe fwd/R ran","");
                //}
                //Forward Left
                //else if (gamepad1.left_stick_y < -0.85 && gamepad1.left_stick_x < -0.85) {
                    ///*
                //    frontLeftDrive.setPower(0);
                //    frontRightDrive.setPower(-gamepad1.left_stick_x / throttle);
                //    backLeftDrive.setPower(-gamepad1.left_stick_x / throttle);
                //    backRightDrive.setPower(0);
                    //*/
                    //telemetry.addData("strafe fwd/L ran","");
                //}
                //Backward Left
                //else if (gamepad1.left_stick_y > 0.85 && gamepad1.left_stick_x < -0.85) {
                    ///*
                //    frontLeftDrive.setPower(gamepad1.left_stick_x / throttle);
                //    frontRightDrive.setPower(0);
                //    backLeftDrive.setPower(0);
                //    backRightDrive.setPower(gamepad1.left_stick_x / throttle);
                    //*/
                    //telemetry.addData("strafe bk/L ran","");
                //}
                //Backward Right
                //else if (gamepad1.left_stick_y > 0.85 && gamepad1.left_stick_x > 0.85) {
                    ///*
                //    frontLeftDrive.setPower(0);
                //    frontRightDrive.setPower(-gamepad1.left_stick_x / throttle);
                //    backLeftDrive.setPower(-gamepad1.left_stick_x / throttle);
                //    backRightDrive.setPower(0);
                    //*/
                    //telemetry.addData("strafe bk/R ran","");
                //}

                //Forward/Backward Movement
                //if(gamepad1.left_stick_x < 0.4 || gamepad1.left_stick_x > -0.4) { //if x is zeroed...
                if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0) {//if up/down exceeds threshold
                    ///*
                    frontLeftDrive.setPower(-gamepad1.left_stick_y / throttle); //move based on controllers y axis values
                    frontRightDrive.setPower(-gamepad1.left_stick_y / throttle);
                    backLeftDrive.setPower(-gamepad1.left_stick_y / throttle);
                    backRightDrive.setPower(-gamepad1.left_stick_y / throttle);
                    //*/
                    //telemetry.addData("forward/backward ran","");
                }

                //Rotate Left Movement
                else if (gamepad1.right_stick_x > 0.5) {//if up/down exceeds threshold
                    ///*
                    frontLeftDrive.setPower(gamepad1.right_stick_x / throttle); //move based on controllers y axis values
                    frontRightDrive.setPower(-gamepad1.right_stick_x / throttle);
                    backLeftDrive.setPower(gamepad1.right_stick_x / throttle);
                    backRightDrive.setPower(-gamepad1.right_stick_x / throttle);
                    telemetry.addData("Left Trigger Status: ", gamepad1.left_bumper);
                    //*/
                    //telemetry.addData("forward/backward ran","");
                }

                //Rotate Right Movement
                else if (gamepad1.right_stick_x < -0.5) {//if up/down exceeds threshold
                    ///*
                    frontLeftDrive.setPower(gamepad1.right_stick_x/ throttle); //move based on controllers y axis values
                    frontRightDrive.setPower(-gamepad1.right_stick_x / throttle);
                    backLeftDrive.setPower(gamepad1.right_stick_x / throttle);
                    backRightDrive.setPower(-gamepad1.right_stick_x / throttle);
                    telemetry.addData("Right Trigger Status: ", gamepad1.right_bumper);
                    //*/
                    //telemetry.addData("forward/backward ran","");
                }

                //Left/Right Movement
                //else if(gamepad1.left_stick_y < 0.4 || gamepad1.left_stick_y > -0.4) { //if Y is zeroed...
                else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < -0) { //else if left/right exceeds threshold
                    ///*
                    frontLeftDrive.setPower(gamepad1.left_stick_x / throttle); //move based on controller x axis values
                    frontRightDrive.setPower(-gamepad1.left_stick_x / throttle);
                    backLeftDrive.setPower(-gamepad1.left_stick_x / throttle);
                    backRightDrive.setPower(gamepad1.left_stick_x / throttle);
                    //*/
                    //telemetry.addData("rotation ran","");
                } else//else just stop
                {
                    ///*
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    //*/
                    //telemetry.addData("stop ran","");
                }

                //telemetry.addData("Controller", "Y (%.2f), X (%.2f)", gamepad1.left_stick_y, gamepad1.left_stick_x);
                //endregion

                //region Arm Lift Controls
                //if ((gamepad1.dpad_down || gamepad1.dpad_up)) //if you are attempting to lower/rise arm and it's not in the park position...
                //{
                //   ParkArm(); //park it first
                //} else {
                    if (gamepad1.right_stick_y > 0.5 && armExtendMotor.getCurrentPosition() > 50) //if the down button is pressed...
                    {
                        armLiftMotor.setPower(armDownSpeed/2);
                    }
                    else if (gamepad1.right_stick_y > 0.5) //if the down button is pressed...
                    {
                        armLiftMotor.setPower(armDownSpeed);
                    }
                    else if (gamepad1.right_stick_y < -0.5 && armExtendMotor.getCurrentPosition() < 50) //if the down button is pressed...
                    {
                        armLiftMotor.setPower(armUpSpeed);
                    }
            else //else just stop the motor
                    {
                        armLiftMotor.setPower(0);
                    }
            /*
            boolean positionChanged = false;
            if(gamepad1.dpad_up && armLiftPosition < 4 && lastDUpState != gamepad1.dpad_up) //if the up button is pressed now and wasn't pressed on last update...
            {
                armLiftPosition++;
                positionChanged = true;
            }
            else if(gamepad1.dpad_down && armLiftPosition > 0 && lastDDownState != gamepad1.dpad_down) //if the down button is pressed now and wasn't pressed on the last update...
            {
                armLiftPosition--;
                positionChanged = true;
            }
            if(positionChanged) { //only run motors if position changed to reduce stress from vibration
                switch (armLiftPosition) {
                    case 0:
                        encoder.RunToEncoderValue(armLiftMotor, 0, armLiftSpeed);
                        armLifted = false;
                        //telemetry.addData("ran:0", "");
                        break;
                    case 1:
                        encoder.RunToEncoderValue(armLiftMotor, (int) (armLiftMax * .25), armLiftSpeed); //brilliant!!
                        armLifted = true;
                        //telemetry.addData("ran:1", "");
                        break;
                    case 2:
                        encoder.RunToEncoderValue(armLiftMotor, (int) (armLiftMax * .5), armLiftSpeed);
                        armLifted = true;
                        //telemetry.addData("ran:2", "");
                        break;
                    case 3:
                        encoder.RunToEncoderValue(armLiftMotor, (int) (armLiftMax * .75), armLiftSpeed);
                        armLifted = true;
                        //telemetry.addData("ran:3", "");
                        break;
                    case 4:
                        encoder.RunToEncoderValue(armLiftMotor, armLiftMax, armLiftSpeed);
                        armLifted = true;
                        //telemetry.addData("ran:4", "");
                        break;
                }
            }
            lastDUpState = gamepad1.dpad_up; //remember what the dpad status was on this update to debounce next iteration
            lastDDownState = gamepad1.dpad_down;
            telemetry.addData("armLiftPosition",  armLiftPosition);
            */
                //endregion

                //region Arm Extension Controls
                //Full extension for endgame

                if (gamepad1.b) //check for cancel: if b is pressed...
                {
                    armExtendMotor.setPower(0); //stop the motor
                }

                if (gamepad1.y && gamepad1.y != lastYState) //if Y is pressed now, but wasn't pressed on the last update...
                {
                    armFullExtend = !armFullExtend;
                    armSlightExtend = false;
                    if (armFullExtend) {
                        encoder.RunToEncoderValue(armExtendMotor, armExtendMax, armExtendSpeed); //full extension
                        telemetry.update();
                    }
                }
                //Slight extension for placing blocks
                if (gamepad1.x && gamepad1.x != lastXState) //if X is pressed now, but wasn't pressed on the last update...
                {
                    armSlightExtend = !armSlightExtend;
                    armFullExtend = false;
                    if (armSlightExtend) {
                        encoder.RunToEncoderValue(armExtendMotor, armMidpoint, armExtendSpeed); //slightly extend arm
                        telemetry.update();
                    }
                }
                //Allow extension of the arm with the right joy-stick
                if (armSlightExtend == false && armFullExtend == false) //if arm is neither fully nor partially extended...
                {
                    ParkArm();
                }
                lastYState = gamepad1.y; //update last pad state to debounce buttons
                lastXState = gamepad1.x;


                //endregion

                //region Platform Arm Controls
                if(gamepad1.left_stick_button && armLiftMotor.getCurrentPosition() > 250 && lastLeftStickState != gamepad1.left_stick_button)
                {
                    platformArmOut = !platformArmOut;
                    if(platformArmOut)
                    {
                        encoder.RunToEncoderValue(platformArm,0,0.2);
                    }
                    else if(platformArmOut == false)
                    {
                        encoder.RunToEncoderValue(platformArm, 850,0.2);
                    }
                }
                lastLeftStickState = gamepad1.left_stick_button;
                //endregion

                //region Claw Controls
                if (gamepad1.a && gamepad1.a != lastAState && gamepad1.right_bumper) //if A is pressed not and it wasn't pressed on last update...
                {
                    clawOpen = !clawOpen;

                    if (clawOpen == false) {
                        claw.setPosition(0.08);
                    }
                    else if (clawOpen) {
                        claw.setPosition(0.5);
                    }
                }
                else if (gamepad1.a && gamepad1.a != lastAState)
                {
                    clawOpen = !clawOpen;

                    if (clawOpen == false) {
                        claw.setPosition(0.13);
                    }
                    else if (clawOpen) {
                        claw.setPosition(0.5);
                    }
                }
                lastAState = gamepad1.a;
                //endregion

                //region Reset robot to starting position
                if (gamepad1.back && gamepad1.back != lastBackState) //if the back button is pressed now and it was't pressed on the last update...
                {
                    //Reset motor's power
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);

                    //Reset all arm and claw movement
                    armFullExtend = false;
                    armSlightExtend = false;
                    ParkArm();
                    if (armExtendMotor.getCurrentPosition() == 30) {
                        encoder.RunToEncoderValue(armLiftMotor, 50, armDownSpeed);
                    }
                    clawOpen = false;

                }
                lastBackState = gamepad1.back;
                //endregion

                //region telemetry
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Drive Motors", "Front Left (%.2f), Front Right (%.2f), Back Left (%.2f), Back Right (%.2f)", frontLeftDrive.getPower(), frontRightDrive.getPower(), backLeftDrive.getPower(), backRightDrive.getPower());
            //telemetry.addData("Arm Lift","Status, Position", armLifted, armLiftPosition);
            //telemetry.addData("Arm Extention", "Slight, Full", armSlightExtend, armFullExtend);
            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
            telemetry.addData("Front Left Wheel: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Wheel: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Wheel: ", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Wheel: ", backRightDrive.getCurrentPosition());
            telemetry.addData("dpad up/dn=Arm up/dn","");
            telemetry.addData("Y=full extend toggle","");
            telemetry.addData("X=midpoint toggle","");
            telemetry.addData("B=extend cancel","");
            telemetry.addData("A=claw toggle","");
            telemetry.addData("LB/RB=turbo","");

            telemetry.update();
            //endregion
        }
    }

    public void ParkArm()
    {
        if (armExtendMotor.getCurrentPosition() > 30)
        {
            encoder.RunToEncoderValue(armExtendMotor, 30, armExtendSpeed); //reset to home position
        }
    }

}

