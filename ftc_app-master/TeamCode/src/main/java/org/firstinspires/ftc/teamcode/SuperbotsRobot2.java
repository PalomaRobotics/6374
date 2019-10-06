/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class SuperbotsRobot2 extends OpMode {

	final static double Hold_Ir_Signal_Strenght = .20;

	DcMotorController controller;
	DcMotor treadRight;
	DcMotor treadLeft;
	DcMotor liftRight;
	DcMotor liftLeft;
	DcMotor intake;
	TouchSensor touch;
	Servo flapA;
	Servo flapB;
	IrSeekerSensor ir;
    int liftPosition = 0;

	public void Lift(int position, int destination)
	{
        //liftLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
       // liftRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        if (destination == 0) {
            if (position == 1) {
                if (liftRight.getCurrentPosition() > 4000 && liftLeft.getCurrentPosition() < -4000) {
                    liftRight.setPower(-.3);
                    liftLeft.setPower(-.3);
                }
            }
            if (position == 2) {
                if (liftLeft.getCurrentPosition() > 8000 && liftRight.getCurrentPosition() < -8000) {
                    liftRight.setPower(-.3);
                    liftLeft.setPower(-.3);
                }
            }
            liftLeft.setPower(0);
            liftRight.setPower(0);
            return;
        }
		if (destination == 1) {
			if (position == 0) {
				if (liftRight.getCurrentPosition() < 4000 && liftLeft.getCurrentPosition() > -4000) {
					liftRight.setPower(.3);
					liftLeft.setPower(.3);
				}
			}
			if (position == 2) {
				if (liftLeft.getCurrentPosition() > 4000 && liftRight.getCurrentPosition() < -4000) {
					liftRight.setPower(-.3);
					liftLeft.setPower(-.3);
				}
			}
            liftLeft.setPower(0);
            liftRight.setPower(0);
            return;
		}

		if (destination == 2) {
			if (position == 0) {
				if (liftLeft.getCurrentPosition() > -8000 && liftRight.getCurrentPosition() < 8000) {
                    liftRight.setPower(.3);
                    liftLeft.setPower(.3);
                }
			}
			if (position == 1) {
				if (liftLeft.getCurrentPosition() > -4000 && liftRight.getCurrentPosition() < 4000) {
					liftRight.setPower(-.3);
					liftLeft.setPower(-.3);
				}
			}
				liftRight.setPower(0);
				liftLeft.setPower(0);
            return;

		}
        liftPosition = destination;
	}

		//DcMotor spinner;// Variable that Controls the turntable
	//Servo climberRight;
	//Servo climberLeft;

	/**
	 * Constructor
	 */
	public SuperbotsRobot2() {

	}

	/*
	 * Code to run when the op mode is initialized goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
	@Override
	public void init() {


		controller = hardwareMap.dcMotorController.get("controller");

		treadRight = hardwareMap.dcMotor.get("treadRight");
		treadLeft = hardwareMap.dcMotor.get("treadLeft");
		treadLeft.setDirection(DcMotor.Direction.REVERSE);


		liftRight = hardwareMap.dcMotor.get("liftRight");
		liftLeft = hardwareMap.dcMotor.get("liftLeft");
		liftRight.setDirection(DcMotor.Direction.REVERSE);

		//liftLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//liftRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

		intake = hardwareMap.dcMotor.get("intake");
		touch = hardwareMap.touchSensor.get("touch");
		//spinner = hardwareMap.dcMotor.get("spinner");

		flapA = hardwareMap.servo.get("flapA");
		flapB = hardwareMap.servo.get("flapB");
		flapA.setPosition(-1);
		flapB.setPosition(1);

		ir = hardwareMap.irSeekerSensor.get("ir");
		
		//climberRight = hardwareMap.servo.get("climberRight");
		//climberLeft = hardwareMap.servo.get("climberLeft");

		// assign the starting position of the wrist and claw
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {


		//move section

		//if(gamepad1.left_stick_y<-35 || -gamepad1.left_stick_y>35)
		treadLeft.setPower(-gamepad1.left_stick_y);
		//-----
		// ---if(gamepad1.right_stick_y<-35 || -gamepad1.right_stick_y>35)
		treadRight.setPower(-gamepad1.right_stick_y);


		//intake section

		if (gamepad1.right_bumper && !gamepad1.left_bumper) {
			intake.setPower(.8);
		} else {
			if (!gamepad1.right_bumper && gamepad1.left_bumper) {
				intake.setPower(-.8);
			} else {
				intake.setPower(0);
			}
		}

		//lift don't kill section


		if (gamepad1.left_trigger > 0) {
			if (touch.isPressed()) {
				liftRight.setPower(.6);
				liftLeft.setPower(.6);
			}
			liftLeft.setPower(-.3);
			liftRight.setPower(-.3);
		} else {
			if (gamepad1.right_trigger > 0) {

				liftLeft.setPower(.6);
				liftRight.setPower(.6);
			} else {
				liftRight.setPower(0);
				liftLeft.setPower(0);
			}
		}

		//servo section

		if (gamepad1.a) {
			flapA.setPosition(Range.clip(flapA.getPosition() + .01, flapA.MIN_POSITION, flapA.MAX_POSITION));
			flapB.setPosition(Range.clip(flapB.getPosition() - .01, flapB.MIN_POSITION, flapB.MAX_POSITION));
		}
		if (gamepad1.b) {
			flapA.setPosition(Range.clip(flapA.getPosition() - .01, flapA.MIN_POSITION, flapA.MAX_POSITION));
			flapB.setPosition(Range.clip(flapA.getPosition() + .01, flapB.MIN_POSITION, flapB.MAX_POSITION));
		}


		//ir section (Auto)

		double angle;
		double strength;

		if (ir.signalDetected()) {
			angle = ir.getAngle();
			strength = ir.getStrength();

			if (angle > 0) {
				//move to the right
				treadLeft.setPower(.15);
				treadRight.setPower(-.15);
			}
			if (angle < 0) {
				//move to the left
				treadLeft.setPower(-.15);
				treadRight.setPower(.15);
			} else if (strength < Hold_Ir_Signal_Strenght) {
				//move to the signal
				treadLeft.setPower(.2);
				treadRight.setPower(.2);
			} else {
				treadLeft.setPower(0);
				treadRight.setPower(0);
			}
		}
		else {
			treadRight.setPower(0);
			treadLeft.setPower(0);
		}


		//lift to score(auto)
		//this all need to go into a METHOD



	}


	//if(gamepad1.dpad_right)
	//{
	//spinner.setPower(-.5);
	//}
	//else{
	//if(gamepad1.dpad_left)
	//{
	//spinner.setPower(.5);
	//}
	//else {
	//spinner.setPower(0);
	//}
	//}


	// telemetry.addData("Text", "*** Robot Data***");


	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
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
