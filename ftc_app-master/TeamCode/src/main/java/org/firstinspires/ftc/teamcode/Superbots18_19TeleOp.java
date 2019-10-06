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
//@Disabled
//Created by Brendan Conwell
public class Superbots18_19TeleOp extends LinearOpMode {

    //region Variable Declaration
    // Misc. Variables
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    boolean lastDUpState, lastDDownState, lastYState, lastXState, lastAState, lastBackState, lastLeftStickState, lastDRightState, lastDLeftState, lastRightStickState;
    boolean driverControl = true, endGame = false;

    //Gyro Variables
    //ModernRoboticsI2cGyro gyro;

    //Drive Variables
    DcMotor LeftDrive;
    DcMotor RightDrive;
    int throttle = 1; //used by lb/rb to run drive motors at half speed

    //Outtake & Intake Variable
    DcMotor ArmRightOuttake;
    DcMotor ArmLeftOuttake;
    double armUpSpeed = 0.4, armDownSpeed = -0.4, armExtendSpeed = 1;
    int armLiftPosition = 0;
    boolean armLifted = false, armFullExtend = false, armSlightExtend = false;
    //Lift Variables
    DcMotor BackLift;
    boolean platformArmDown = false, platformArmOut = false;
    DcMotor Intake;
    //endregion


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //region object instantiation
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //DriveMotors
        RightDrive = hardwareMap.dcMotor.get("Right_Drive");
        LeftDrive = hardwareMap.dcMotor.get("Left_Drive");
        //////////////////////////////////////////////////////////////////
        //Intake & Outtake
        Intake = hardwareMap.dcMotor.get("Intake");
        ArmLeftOuttake = hardwareMap.dcMotor.get("Arm_Left_Outtake");
        ArmRightOuttake = hardwareMap.dcMotor.get("Arm_Right_Outtake");
        //////////////////////////////////////////////////////////////////
        //Lift
        BackLift = hardwareMap.dcMotor.get("Back_Lift");
        BackLift.setDirection(DcMotorSimple.Direction.FORWARD);   ////
        //////////////////////////////////////////////////////////////////

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //endregion
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //encoder.RunToEncoderValue(armExtendMotor, 25, 0.25); //reset to home position
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //region drive movement
            if (gamepad1.right_trigger > 0.5 || gamepad1.right_trigger < -0.5) //if the Left Trigger or Right Trigger is pressed...
            {
                throttle = 3; //run at full speed speed
            } else {
                throttle = 1; //else run at a lesser speed
            }

            //Forward/Backward Movement
            //if(gamepad1.left_stick_x < 0.4 || gamepad1.left_stick_x > -0.4) { //if x is zeroed...
            if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0) {//if up/down exceeds threshold
                ///*
                RightDrive.setPower(-gamepad1.left_stick_y / throttle); //move based on controllers y axis values
                LeftDrive.setPower(-gamepad1.left_stick_y / throttle);
                //*/
                //telemetry.addData("forward/backward ran","");
            }

            //Rotate Left Movement
            else if (gamepad1.right_stick_x > 0.5) {//if up/down exceeds threshold
                ///*
                RightDrive.setPower(gamepad1.right_stick_x / throttle); //move based on controllers y axis values
                LeftDrive.setPower(-gamepad1.right_stick_x / throttle);
                telemetry.addData("Left Trigger Status: ", gamepad1.left_bumper);
                //*/
                //telemetry.addData("forward/backward ran","");
            }

            //Rotate Right Movement
            else if (gamepad1.right_stick_x < -0.5) {//if up/down exceeds threshold
                ///*
                RightDrive.setPower(gamepad1.right_stick_x / throttle); //move based on controllers y axis values
                telemetry.addData("Right Trigger Status: ", gamepad1.right_bumper);
                //*/
                //telemetry.addData("forward/backward ran","");
            }

            //Left/Right Movement
            //else if(gamepad1.left_stick_y < 0.4 || gamepad1.left_stick_y > -0.4) { //if Y is zeroed...
            else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < -0) { //else if left/right exceeds threshold
                ///* //move based on controller x axis values
                LeftDrive.setPower(-gamepad1.left_stick_x / throttle);
                //*/
                //telemetry.addData("rotation ran","");
            } else//else just stop
            {
                ///*
                LeftDrive.setPower(0);
                RightDrive.setPower(0);
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
            //if (gamepad1.right_stick_y > 0.5 && armExtendMotor.getCurrentPosition() > 50) //if the down button is pressed...
            //{
            //   armLiftMotor.setPower(armDownSpeed/2);
            // }
            //else if (gamepad1.right_stick_y > 0.5) //if the down button is pressed...
            //{
            //    armLiftMotor.setPower(armDownSpeed);
            //}
            //else if (gamepad1.right_stick_y < -0.5 && armExtendMotor.getCurrentPosition() < 50) //if the down button is pressed...
            //{
            //   armLiftMotor.setPower(armUpSpeed);
            //}
            // else //else just stop the motor
            //{
            //   armLiftMotor.setPower(0);
            //}

            //region Arm Extension Controls
            //Full extension for endgame

            //if (gamepad1.b) //check for cancel: if b is pressed...
            //{
            //  Intake.setPower(0); //stop the motor
            //}

            //if (gamepad1.y && gamepad1.y != lastYState) //if Y is pressed now, but wasn't pressed on the last update...
            //{
            //  armFullExtend = !armFullExtend;
            //armSlightExtend = false;
            // if (armFullExtend) {
            //   encoder.RunToEncoderValue(armExtendMotor, armExtendMax, armExtendSpeed); //full extension
            //     telemetry.update();
            // }
            //}
            //Slight extension for placing blocks
            //if (gamepad1.x && gamepad1.x != lastXState) //if X is pressed now, but wasn't pressed on the last update...
            //{
            //  armSlightExtend = !armSlightExtend;
            //  armFullExtend = false;
            //  if (armSlightExtend) {
            //      encoder.RunToEncoderValue(armExtendMotor, armMidpoint, armExtendSpeed); //slightly extend arm
            //      telemetry.update();
            //  }
            //}
            //Allow extension of the arm with the right joy-stick
            //if (armSlightExtend == false && armFullExtend == false) //if arm is neither fully nor partially extended...
            //{
            //  ParkArm();
            //}
            //lastYState = gamepad1.y; //update last pad state to debounce buttons
            //lastXState = gamepad1.x;


            //endregion

            //region Platform Arm Controls
            //if(gamepad1.left_stick_button && armLiftMotor.getCurrentPosition() > 250 && lastLeftStickState != gamepad1.left_stick_button)
            //{
            //  platformArmOut = !platformArmOut;
            //  if(platformArmOut)
            //  {
            //      encoder.RunToEncoderValue(platformArm,0,0.2);
            //  }
            //  else if(platformArmOut == false)
            //  {
            //      encoder.RunToEncoderValue(platformArm, 850,0.2);
            ////  }
            // }
            lastLeftStickState = gamepad1.left_stick_button;
            //endregion

            //region Claw Controls
            // if (gamepad1.a && gamepad1.a != lastAState && gamepad1.right_bumper) //if A is pressed not and it wasn't pressed on last update...
            // {
//                clawOpen = !clawOpen;
//
            //              if (clawOpen == false) {
            //      claw.setPosition(0.08);
            //  }
            //  else if (clawOpen) {
            ////      claw.setPosition(0.5);
            // }
            //}
            //else if (gamepad1.a && gamepad1.a != lastAState)
            //{
            //  clawOpen = !clawOpen;

            //if (clawOpen == false) {
            //      claw.setPosition(0.13);
            //  }
            //  else if (clawOpen) {
            //      claw.setPosition(0.5);
            //  }
            //}
            //lastAState = gamepad1.a;
            //endregion

            //region Reset robot to starting position
            if (gamepad1.back && gamepad1.back != lastBackState) //if the back button is pressed now and it was't pressed on the last update...
            {
                //Reset motor's power
                LeftDrive.setPower(0);
                RightDrive.setPower(0);

                //if (armExtendMotor.getCurrentPosition() == 30) {
                // encoder.RunToEncoderValue(armLiftMotor, 50, armDownSpeed);
                // }
                //clawOpen = false;

            }
            lastBackState = gamepad1.back;
            //endregion

            //region telemetry
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Drive Motors", "Front Left (%.2f), Front Right (%.2f), Back Left (%.2f), Back Right (%.2f)", frontLeftDrive.getPower(), frontRightDrive.getPower(), backLeftDrive.getPower(), backRightDrive.getPower());
            //telemetry.addData("Arm Lift","Status, Position", armLifted, armLiftPosition);
            //telemetry.addData("Arm Extention", "Slight, Full", armSlightExtend, armFullExtend);
            telemetry.addData(" Left Wheel: ", LeftDrive.getCurrentPosition());
            telemetry.addData(" Right Wheel: ", RightDrive.getCurrentPosition());
            telemetry.addData("dpad up/dn=Arm up/dn", "");
            telemetry.addData("Y=full extend toggle", "");
            telemetry.addData("X=midpoint toggle", "");
            telemetry.addData("B=extend cancel", "");
            telemetry.addData("A=claw toggle", "");
            telemetry.addData("LB/RB=turbo", "");

            telemetry.update();
            //endregion
        }
    }
}

