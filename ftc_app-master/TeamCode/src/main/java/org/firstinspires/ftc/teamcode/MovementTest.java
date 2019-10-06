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

@TeleOp(name="MovementTest", group="test")
@Disabled
//Created by Brendan Conwell
public class MovementTest extends LinearOpMode {

    //region Variable Declaration
    // Misc. Variables
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    boolean lastDUpState, lastDDownState, lastYState, lastXState, lastAState, lastBackState;

    //Drive Variables
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    int throttle = 1; //used by lb/rb to run drive motors at half speed

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //region drive movement
            if(gamepad1.right_trigger > 0.5 || gamepad1.right_trigger < -0.5) //if the Left Trigger or Right Trigger is pressed...
            {
                throttle = 1; //run at full speed speed
            }
            else
            {
                throttle = 6; //else run at a lesser speed
            }
            //Strafing
            //Forward Right
            if(gamepad1.right_stick_y < -0.3 && gamepad1.right_stick_x > 0.3)
            {
                ///*
                frontLeftDrive.setPower(gamepad1.right_stick_x / throttle);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(gamepad1.right_stick_x / throttle);
                //*/
                //telemetry.addData("strafe fwd/R ran","");
            }
            //Forward Left
            else if(gamepad1.right_stick_y < -0.3 && gamepad1.right_stick_x < -0.3)
            {
                ///*
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(-gamepad1.right_stick_x / throttle);
                backLeftDrive.setPower(-gamepad1.right_stick_x / throttle);
                backRightDrive.setPower(0);
                //*/
                //telemetry.addData("strafe fwd/L ran","");
            }
            //Backward Left
            else if(gamepad1.right_stick_y > 0.3 && gamepad1.right_stick_x < -0.3)
            {
                ///*
                frontLeftDrive.setPower(gamepad1.right_stick_x / throttle);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(gamepad1.right_stick_x / throttle);
                //*/
                //telemetry.addData("strafe bk/L ran","");
            }
            //Backward Right
            else if(gamepad1.right_stick_y > 0.3 && gamepad1.right_stick_x > 0.3)
            {
                ///*
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(-gamepad1.right_stick_x / throttle);
                backLeftDrive.setPower(-gamepad1.right_stick_x / throttle);
                backRightDrive.setPower(0);
                //*/
                //telemetry.addData("strafe bk/R ran","");
            }

            //Forward/Backward Movement
            //if(gamepad1.left_stick_x < 0.4 || gamepad1.left_stick_x > -0.4) { //if x is zeroed...
            else if(gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0) {//if up/down exceeds threshold
                ///*
                frontLeftDrive.setPower(-gamepad1.left_stick_y / throttle); //move based on controllers y axis values
                frontRightDrive.setPower(-gamepad1.left_stick_y / throttle);
                backLeftDrive.setPower(-gamepad1.left_stick_y / throttle);
                backRightDrive.setPower(-gamepad1.left_stick_y / throttle);
                //*/
                //telemetry.addData("forward/backward ran","");
            }

            //Rotate Left Movement
            else if(gamepad1.left_bumper) {//if up/down exceeds threshold
                ///*
                frontLeftDrive.setPower(-0.8 / throttle); //move based on controllers y axis values
                frontRightDrive.setPower(0.8 / throttle);
                backLeftDrive.setPower(-0.8 / throttle);
                backRightDrive.setPower(0.8 / throttle);
                telemetry.addData("Left Trigger Status: ", gamepad1.left_bumper);
                //*/
                //telemetry.addData("forward/backward ran","");
            }

            //Rotate Right Movement
            else if(gamepad1.right_bumper) {//if up/down exceeds threshold
                ///*
                frontLeftDrive.setPower(0.8 / throttle); //move based on controllers y axis values
                frontRightDrive.setPower(-0.8 / throttle);
                backLeftDrive.setPower(0.8 / throttle);
                backRightDrive.setPower(-0.8 / throttle);
                telemetry.addData("Right Trigger Status: ", gamepad1.right_bumper);
                //*/
                //telemetry.addData("forward/backward ran","");
            }

            //Left/Right Movement
            //else if(gamepad1.left_stick_y < 0.4 || gamepad1.left_stick_y > -0.4) { //if Y is zeroed...
            else if(gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < -0){ //else if left/right exceeds threshold
                ///*
                frontLeftDrive.setPower(gamepad1.left_stick_x / throttle); //move based on controller x axis values
                frontRightDrive.setPower(-gamepad1.left_stick_x / throttle);
                backLeftDrive.setPower(-gamepad1.left_stick_x / throttle);
                backRightDrive.setPower(gamepad1.left_stick_x / throttle);
                //*/
                //telemetry.addData("rotation ran","");
            }

            else //else just stop
            {
                ///*
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                //*/
                //telemetry.addData("stop ran","");
            }

            //region telemetry
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Drive Motors", "Front Left (%.2f), Front Right (%.2f), Back Left (%.2f), Back Right (%.2f)", frontLeftDrive.getPower(), frontRightDrive.getPower(), backLeftDrive.getPower(), backRightDrive.getPower());
            //telemetry.addData("Arm Lift","Status, Position", armLifted, armLiftPosition);
            //telemetry.addData("Arm Extention", "Slight, Full", armSlightExtend, armFullExtend);
            telemetry.addData("dpad up/dn=Arm up/dn","");
            telemetry.addData("Y=full extend toggle","");
            telemetry.addData("X=midpoint toggle","");
            telemetry.addData("B=extend cancel","");
            telemetry.addData("A=claw toggle","");
            telemetry.addData("LB/RB=turbo","");
            //telemetry.addData("Servo Position",claw.getPosition());
            telemetry.update();
            //endregion
        }
    }


}
