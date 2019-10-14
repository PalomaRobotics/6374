

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class BoysTeamTeleOp_wilson extends OpMode {

	DcMotor rightF; // Front Right Drive Motor
	DcMotor leftF; // Front Left Drive Motor
	DcMotor rightB; // Back Right Drive Motor
	DcMotor leftB; // Back Left Drive Motor
	int liftPosition = 0; // position set
	long delay = 1000;
	//DcMotor intake;
	//DcMotor pivot;
	//Servo gateR;
	//Servo gateL;

	IrSeekerSensor ir;
	LightSensor lightSensor;
	//double lightValue;
	//boolean lightColor;
double power = .5;
boolean InertialDamper(double counter, boolean joystick)
{
	SystemClock.sleep(2);
	if(counter>0)
	{
		if(joystick)
		{
			rightF.setPower(counter);
			rightB.setPower(counter);
			return InertialDamper((counter - .01), joystick);
		}
		else
		{
			leftF.setPower(counter);
			leftB.setPower(counter);
			return InertialDamper((counter - .01), joystick);
		}
	}
	return true;
}

	/**
	 * Constructor
	 */
	public BoysTeamTeleOp_wilson() {

	}

	/*
	 * Code to run when the op mode is initialized goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
	@Override
	public void init() {


		//light = hardwareMap.lightSensor.get("light");
		rightF = hardwareMap.dcMotor.get("rightF");
		leftF = hardwareMap.dcMotor.get("leftF");
		leftF.setDirection(DcMotor.Direction.REVERSE); //reverse direction so you have only positive values

		//leftF.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
		//rightF.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//leftF.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

		rightB = hardwareMap.dcMotor.get("rightB");
		leftB = hardwareMap.dcMotor.get("leftB");
		leftB.setDirection(DcMotor.Direction.REVERSE);


		lightSensor = hardwareMap.lightSensor.get("lightSensor");
		//intake = hardwareMap.dcMotor.get("intake");
		//pivot = hardwareMap.dcMotor.get("pivot");
		//gateL = hardwareMap.servo.get("gateL");
		//gateR = hardwareMap.servo.get("gateR");
		//gateL.setPosition(-1);
		//gateR.setPosition(1);

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		if(gamepad1.left_stick_x < 0.2 || gamepad1.left_stick_x > 0.2) { //if not turning...
			rightF.setPower(gamepad1.left_stick_y); //move all in unison
			rightB.setPower(gamepad1.left_stick_y);
			leftF.setPower(gamepad1.left_stick_y);
			leftB.setPower(gamepad1.left_stick_y);
		}
		else //else if turning
		{
			rightF.setPower(gamepad1.left_stick_x);
			rightB.setPower(gamepad1.left_stick_x);
			leftF.setPower(-gamepad1.left_stick_x);
			leftB.setPower(-gamepad1.left_stick_x);
		}


		/*if(Math.abs(gamepad1.right_stick_y)<.2) {
			rightF.setPower(rightF.getPower()/2);
			SystemClock.sleep(20);
		}
		if(Math.abs(gamepad1.left_stick_y)<.2)
			leftF.setPower(leftF.getPower() / 2);
		SystemClock.sleep(20);

		rightF.setPower(-gamepad1.left_stick_y);
		rightB.setPower(-gamepad1.left_stick_y);
		leftF.setPower(-gamepad1.right_stick_y);
		leftB.setPower(-gamepad1.right_stick_y);


		if(gamepad1.a)//if a is pressed......
		{
			while(lightSensor.getLightDetected() == 3)//while on the line
			{
				leftB.setPower(.3);//move forward
				rightF.setPower(.3);
			}//on line

			if(lightSensor.getLightDetected() != 3)//if off of the line.....
			{
				rightF.setTargetPosition(rightF.getCurrentPosition() + 4000);//moves one way to find line
				leftB.setTargetPosition(leftB.getCurrentPosition() + 4000);
				leftB.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
				rightF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
				//then.....
				rightF.setTargetPosition(rightF.getCurrentPosition() - 8000);//move back and beyond to find line
				leftB.setTargetPosition(leftB.getCurrentPosition() - 8000);
				leftB.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
				rightF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
			}//off the line
		}//pressing a*/

/*
		if(gamepad1.a) {
			rightF.setPower(power);

			rightF.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
			if(rightF.getCurrentPosition() <= 1120) {
				rightF.setPower(.1);
			}
			else
				rightF.setPower(0);

		}
		if(gamepad1.dpad_down)
			power -= .1;
		if(gamepad1.dpad_up)
			power += .1;
			*/
		/*
		else{
			rightF.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
			rightF.setPower(0);
			/*
			if(rightF.getCurrentPosition()!=0) {
				rightF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
				rightF.setTargetPosition(0);
			}
			else
				rightF.setPower(0);
				//

		}
	*/
		/*
		if(gamepad1.right_bumper )
		{

			rightF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
			leftF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
				rightF.setPower(-.3);
				leftF.setPower(-.3);
			rightF.setTargetPosition(4000);
			leftF.setTargetPosition(4000);

		}
		if(gamepad1.left_bumper && liftPosition>0)
		{
			Lift(liftPosition, liftPosition-1);
		}
		if(gamepad1.x)
		{
			Lift(liftPosition, 0);
		}



*/

		/*
        else if(gamepad1.back)
        {
            if(gamepad1.x)
            {
                rightF.setPower(-.7);
                leftB.setPower(-.7);
            }
            if (gamepad1.y)
            {
            rightB.setPower(-.7);
            leftF.setPower(-.7);
            }
        }
        else if(gamepad1.y)
        {
            rightB.setPower(.7);
            leftF.setPower(.7);
        }
        else if(gamepad1.x)
        {
            rightB.setPower(.7);
            leftF.setPower(.7);
        }
		else
		{


			if (Math.abs(gamepad1.right_stick_y) > 0.1) // If Moving Forward
			{
				rightB.setPower((2 * gamepad1.right_stick_y) / 3);
				rightF.setPower((2 * gamepad1.right_stick_y) / 3);
			} else // If Not Moving F/B
			{

			}
			if (Math.abs(gamepad1.right_stick_x) > 0.1) // If Moving Forward
			{
				leftF.setPower((2 * gamepad1.right_stick_x) / 3);
				leftB.setPower((2 * gamepad1.right_stick_x) / 3);
			} else // If Not Moving F/B
			{
				rightB.setPower(0);
				rightF.setPower(0);
				leftB.setPower(0);
				leftF.setPower(0);
			}
		}
		*/

/*
		if(gamepad1.right_bumper && !gamepad1.left_bumper)
		{
			intake.setPower(.8);
		}
		else
		{
			if(!gamepad1.right_bumper && gamepad1.left_bumper)
			{
				intake.setPower(-.8);
			}
			else
			{
				intake.setPower(0);
			}
		}
		if(gamepad1.dpad_right && !gamepad1.dpad_left)
		{
			pivot.setPower(-.2);
		}
		else
		{
			if(!gamepad1.dpad_right && gamepad1.dpad_left)
			{
				pivot.setPower(.2);
			}
			else
			{
				pivot.setPower(0);
			}
		}
		if(gamepad1.x)
		gateL.setPosition(Range.clip((gateL.getPosition()+.01),gateL.MIN_POSITION, gateL.MAX_POSITION));
		if(gamepad1.b)
			gateR.setPosition(Range.clip((gateR.getPosition()-.01),gateL.MIN_POSITION, gateL.MAX_POSITION));
		if(gamepad1.a) {
			gateL.setPosition(-1);
			gateR.setPosition(1);
		}
		telemetry.addData("ServoData: ", gateR.getPosition() + gateR.MIN_POSITION + gateR.MAX_POSITION);


*/
		telemetry.addData("LightSensor: ", lightSensor.getLightDetected());
		//telemetry.addData("Encoder Left", leftF.getCurrentPosition());

		//telemetry.addData("Robot Y", -gamepad1.right_stick_y);
		//telemetry.addData("Robot x", gamepad1.left_stick_x);
       // telemetry.addData("Text", "*** Robot Data***");
	}//end loop
	public void Lift(int position, int destination)
	{
		telemetry.addData("Yes","");
		//leftF.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
		//rightF.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
		//leftF.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//rightF.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		if (position == 0) {
			if (destination == 1) {
				if (Math.abs(rightF.getCurrentPosition()) < 4000 && Math.abs(leftF.getCurrentPosition()) < 4000) {
					rightF.setPower(-.3);
					leftF.setPower(-.3);
				}

			}
			if (destination == 2) {
				if (Math.abs(rightF.getCurrentPosition()) < 8000 && Math.abs(leftF.getCurrentPosition()) < 8000) {
					rightF.setPower(-.3);
					leftF.setPower(-.3);
				}

			}
			leftF.setPower(0);
			rightF.setPower(0);

		}
		if (position == 1) {
			if (destination == 0) {
				if (Math.abs(rightF.getCurrentPosition()) < 4000 && Math.abs(leftF.getCurrentPosition()) < 4000) {
					rightF.setPower(.3);
					leftF.setPower(.3);
				}

			}
			if (destination == 2) {
				if (Math.abs(rightF.getCurrentPosition()) < 4000 && Math.abs(leftF.getCurrentPosition()) < 4000) {
					rightF.setPower(-.3);
					leftF.setPower(-.3);
				}

			}
			leftF.setPower(0);
			rightF.setPower(0);

		}

		if (position == 2) {
			if (destination == 0) {
				if (Math.abs(rightF.getCurrentPosition()) < 8000 && Math.abs(leftF.getCurrentPosition()) < 8000) {
					rightF.setPower(.3);
					leftF.setPower(.3);
				}

			}
			if (destination == 1) {
				if (Math.abs(rightF.getCurrentPosition()) < 4000 && Math.abs(leftF.getCurrentPosition()) < 4000) {
					rightF.setPower(-.3);
					leftF.setPower(-.3);
				}
			}
			rightF.setPower(0);
			leftF.setPower(0);


		}
		liftPosition = destination;
	}
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
