package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GyroClass{

    private DcMotor lf, rf, lb, rb;
    private double leftPwr = 0.25;
    private double rightPwr = 0.25;
    private double remLPwr, remRPwr;
    private int heading = 0;              // Gyro integrated heading
    private boolean allStop = false;
    private int prevLeftValue, prevRightValue;
    private ModernRoboticsI2cGyro gyro;
    private double adjustmentSpeed = 0.50;
    private double speed = 0.0;
    private final int MAJOR_ERROR_THRESHOLD = 10, MINOR_ERROR_CORRECTION = 3;

    Telemetry telemetry;

    public GyroClass(ModernRoboticsI2cGyro gyroObject, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, double speed, Telemetry telemetry)
    {
        //You will need to create motor and gyro objects from your main class and pass them as parameters to this constructor
        //The turn method uses a loop that ties up the robot for a while which could result in a "stuck in loop" exception.
        //To fix this, you can increase the watchdog timer with this command from your init:
        //super.msStuckDetectLoop = 30000;
        this.telemetry = telemetry;

        this.lf = frontLeftMotor; //move the parameter objects into their respective class-level variables
        this.rf = frontRightMotor;
        this.lb = backLeftMotor;
        this.rb = backRightMotor;
        this.speed = speed;
        this.leftPwr = speed; //0.25
        this.rightPwr = speed; //0.25

        this.gyro = gyroObject; //THIS IS AN I2C DEVICE. Find a Gyro Sensor on the robot named gyroSens. gyroSens is the name that appears in the phones configuration

        gyro.calibrate(); //calibrate the gyro. DON'T MOVE THE ROBOT OR SENSOR UNTIL THIS IS DONE!

        // This loop ties up the robot until gyro calibration is complete
        while (gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        ResetHeading(); //reset the heading. The sensor only returns a heading for the Z axis
    }

    private double AdjustPower(double currentValue, double amountOfAdjustment)
    {
        //method prevents a motor value from dipping below 50% of the desired speed
        double start = Math.abs(currentValue);
        if(Math.abs(currentValue + amountOfAdjustment) > 0.5 * speed && Math.abs(currentValue + amountOfAdjustment) <= 1.0 )
        {
            return currentValue + amountOfAdjustment;
        }
        else
        {
            return currentValue;
        }
    }

    public void StraightLineUpdate()
    {
        if(allStop) //if I'm stopped due to a major error....
        {
            ErrorCorrection(); //fix it first
        }
        else //else proceed straight
        {
            if (GetHeading() >= MINOR_ERROR_CORRECTION) //if trending RIGHT
            {
                leftPwr = AdjustPower(leftPwr,-0.01); //move right by increasing power to left motor
                rightPwr = AdjustPower(rightPwr,0.01); //and decreasing power on right
                telemetry.addData("trending", "RIGHT");
            }
            else if (GetHeading() <= -MINOR_ERROR_CORRECTION) //if trending LEFT
            {
                leftPwr = AdjustPower(leftPwr,0.01); //move right by increasing power to left motor
                rightPwr = AdjustPower(rightPwr,-0.01); //and decreasing power on right
                telemetry.addData("trending", "LEFT");
            }
            else
            {
                telemetry.addData("Normal Operation", ":)");
            }

            //if major error to LEFT
            if (GetHeading() > MAJOR_ERROR_THRESHOLD || GetHeading() <= -MAJOR_ERROR_THRESHOLD) //if major error...
            {
                remLPwr = leftPwr; //remember what power we were at prior to the problem
                remRPwr = rightPwr;
                leftPwr = 0.0; //Stop until corrected
                rightPwr = 0.0;
                allStop = true;
                telemetry.addData("Major err", "ALL STOP");
            }

            telemetry.addData("LFpwr", leftPwr);
            telemetry.addData("RFpwr", rightPwr);
            lf.setPower(leftPwr); //drive the FRONT motors at the calculated speeds
            rf.setPower(rightPwr);
            lb.setPower(0.10);
            rb.setPower(0.10);
        }


        telemetry.addData("heading", GetHeading());
        telemetry.update();
    }

    private void ErrorCorrection()
    {
        while(GetHeading() > MINOR_ERROR_CORRECTION) //if too far RIGHT, move motors until you're back on course
        {
            lf.setPower(-0.10);
            rf.setPower(0.10);
            lb.setPower(0.0);
            rb.setPower(0.0);
            telemetry.addData("ERROR!!", "TO RIGHT");
            telemetry.addData("heading", GetHeading());
            telemetry.addData("LFpwr", leftPwr);
            telemetry.addData("RFpwr", rightPwr);
            telemetry.update();
        }

        while(GetHeading() < -MINOR_ERROR_CORRECTION) //if too far LEFT, move motors until you're back on course
        {
            lf.setPower(0.10);
            rf.setPower(-0.10);
            lb.setPower(0.0);
            rb.setPower(0.0);
            telemetry.addData("ERROR!!", "TO LEFT");
            telemetry.addData("heading", GetHeading());
            telemetry.addData("LFpwr", leftPwr);
            telemetry.addData("RFpwr", rightPwr);
            telemetry.update();
        }

        allStop = false; //note that we are done with the error correction
        leftPwr = remLPwr; //resume normal speed
        rightPwr = remRPwr;
        telemetry.addData("problem corrected!", ":)");
        telemetry.addData("heading", GetHeading());
        telemetry.addData("LFpwr", leftPwr);
        telemetry.addData("RFpwr", rightPwr);
        telemetry.update();
    }

    private void RunStraight()
    {

    }

    public void Turn(int newHeading)
    {

    }


    /*

    public void StraightLineUpdate()
    {

        heading = gyro.getHeading(); //get the heading info. The Modern Robotics' gyro sensor keeps track of the current heading for the Z axis only.

        if(heading > 180 && heading < 359 && allStop == false) { //if trending left...
            //note these are reverse what you would think because the left motors are reversed
            leftPwr -= 0.01; //move right by increasing power to left motor
            rightPwr += 0.01; //and decreasing power on right
            telemetry.addData("Minor correct", "TO RIGHT");
        }

        if(heading < 180 && heading > 1 && allStop == false) { //if trending right...
            leftPwr += 0.01; //turn left by decreasing power on left motor
            rightPwr -= 0.01;
            telemetry.addData("Minor correct", "TO LEFT");
        }

        if((heading > 180 && heading < 350) || (heading < 180 && heading > 10))  //if major trend to left OR right...
        {
            if(allStop == false) //if not already stopped
            {
                leftPwr = 0.0; //Stop until corrected
                rightPwr = 0.0;
                allStop = true;
                telemetry.addData("Major err", "ALL STOP");
            }
        }

        lf.setPower(leftPwr); //drive the FRONT motors at the calculated speeds
        rf.setPower(rightPwr);

        telemetry.addData("pwr", leftPwr);

        /////////////////////////////////////////////////////////////RECOVER FROM ALL STOP: if too far LEFT...
        if (heading > 180 && heading < 358 && allStop == true)
        {
            try //attempt to correct for major off course
            {
                telemetry.addData("Major correct", "to right");
                telemetry.update();


                EncoderClass.RunToEncoderDegreeAsync(rf, EncoderClass.MotorType.NeveRest60,20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                EncoderClass.RunToEncoderDegreeAsync(lf, EncoderClass.MotorType.NeveRest60,-20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions


                //EncoderClass.RunToEncoderDegreeAsync(lb, EncoderClass.MotorType.NeveRest60,-20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                //EncoderClass.RunToEncoderDegreeAsync(rb, EncoderClass.MotorType.NeveRest60,20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions

                while (rf.isBusy() || rb.isBusy() || lf.isBusy() || lb.isBusy()) //wait while the motor moves to the desired position
                {

                }

            }
            catch(NullPointerException ex)
            {
                sleep(20); //wait a little bit if there is an error and try again on the next pass
            }
            finally {
                heading = gyro.getHeading(); //update the heading no matter what
                sleep(10);
            }
        }
        else if(heading > 180 && heading >= 358 && allStop == true) { //if you're mostly correct again...
            allStop = false;
            RunStraight();
        }
        /////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////RECOVER FROM ALL STOP: if too far RIGHT...
        if (heading < 180 && heading > 2 && allStop == true)
        {
            try //attempt to correct for major off course
            {

                //the following two lines errored out with modifications to EncoderClass
                //EncoderClass.RunToEncoderDegree(right, 20, 0.25); //use encoders to move the wheels 20 degrees at a time in opposite directions
                //EncoderClass.RunToEncoderDegree(left, -20, 0.25);

                telemetry.addData("Major correct", "to left");
                telemetry.update();

                EncoderClass.RunToEncoderDegreeAsync(rf, EncoderClass.MotorType.NeveRest60,-20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                EncoderClass.RunToEncoderDegreeAsync(lf, EncoderClass.MotorType.NeveRest60,20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions

                //EncoderClass.RunToEncoderDegreeAsync(lb, EncoderClass.MotorType.NeveRest60,20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                //EncoderClass.RunToEncoderDegreeAsync(rb, EncoderClass.MotorType.NeveRest60,-20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
            }
            catch(NullPointerException ex)
            {
                sleep(20); //wait a little bit if there is an error and try again on the next pass
            }
            finally {
                heading = gyro.getHeading(); //update the heading no matter what
                sleep(10);
            }
        }
        else if(heading < 180 && heading <= 2 && allStop == true) { //if you're mostly correct again...
            allStop = false;
            RunStraight();
        }
        /////////////////////////////////////////////////////////////

        if(heading >= 359 || heading <= 1) { //if on course...
            allStop = false;
            RunStraight();
        }
        telemetry.update();
    }

    private void RunStraight()
    {
    //  left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //disable encoder for running straight
    //  right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("RUN STRAIGHT", speed);

        leftPwr = speed; //resume nominal power
        rightPwr = speed;
        allStop = false;

    //    left.setPower(leftPwr);
    //    right.setPower(rightPwr);
    }



    public void Turn(int newHeading)
    {
        int upperThreshold, lowerThreshold;

        if(newHeading <= 0) //prevent user from turning to 0 which would simply mean "continue driving straight"
        {
            throw new IllegalArgumentException("newHeading parameter cannot be <= zero");
        }
        else
        {
            lowerThreshold = newHeading - 1;
        }

        if (newHeading >= 359) //prevent user from turning to 0 which would simply mean "continue driving straight"
        {
            throw new IllegalArgumentException("newHeading parameter cannot be >= 359");
        }
        else
        {
            upperThreshold = newHeading + 1;
        }

        leftPwr = 0.0; //Stop until corrected
        rightPwr = 0.0;
        ResetHeading(); //set the direction I'm facing to zero

        while(true) {
            heading = gyro.getHeading(); //get the heading info. The Modern Robotics' gyro sensor keeps track of the current heading for the Z axis only.

            /////////////////////////////////////////////////////////////RECOVER FROM ALL STOP: if newHeading is to my LEFT...
            if ((upperThreshold >= 180 || lowerThreshold >= 180) && (heading > upperThreshold || heading <= 2)) //if I need to rotate LEFT...
            {
                try //attempt to correct for major off course
                {

                    //the following two lines errored out with modifications to EncoderClass
                    //EncoderClass.RunToEncoderDegree(right, 20, 0.25); //use encoders to move the wheels 20 degrees at a time in opposite directions
                    //EncoderClass.RunToEncoderDegree(left, -20, 0.25);
                    EncoderClass.RunToEncoderDegreeAsync(rf, EncoderClass.MotorType.NeveRest60,20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                    EncoderClass.RunToEncoderDegreeAsync(rb, EncoderClass.MotorType.NeveRest60,20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions

                    EncoderClass.RunToEncoderDegreeAsync(lf, EncoderClass.MotorType.NeveRest60,-20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                    EncoderClass.RunToEncoderDegreeAsync(lb, EncoderClass.MotorType.NeveRest60,-20, adjustmentSpeed, false); //use encoders to move the wheels 20 degrees at a time in opposite directions
                } catch (NullPointerException ex) {
                    sleep(20); //wait a little bit if there is an error and try again on the next pass
                } finally {
                    heading = gyro.getHeading(); //update the heading no matter what
                    sleep(10);
                }
            }

            /////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////RECOVER FROM ALL STOP: if new heading is to my RIGHT...
            if ((upperThreshold < 180 || lowerThreshold < 180) && (heading < lowerThreshold || heading >= 358)) //if I need to rotate RIGHT...
            {
                try //attempt to correct for major off course
                {
                    //the following two lines errored out with modifications to EncoderClass
                    //EncoderClass.RunToEncoderDegree(right, -20, 0.25); //use encoders to move the wheels 20 degrees at a time in opposite directions
                    //EncoderClass.RunToEncoderDegree(left, 20, 0.25);
                } catch (NullPointerException ex) {
                    sleep(20); //wait a little bit if there is an error and try again on the next pass
                } finally {
                    heading = gyro.getHeading(); //update the heading no matter what
                    sleep(10);
                }
            }

            /////////////////////////////////////////////////////////////

            if (heading <= upperThreshold && heading >= lowerThreshold) { //if you're mostly correct again...
                ResetHeading(); //make sure the new direction we are facing is zero
                return; //escape the loop and return to main
            }

        }

    }
    */



    public void ResetHeading()
    {
        gyro.resetZAxisIntegrator(); //reset the heading. The sensor only returns a heading for the Z axis
    }

    public int GetHeading()
    {
        //return gyro.getHeading();
        return -gyro.getIntegratedZValue();
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

}
