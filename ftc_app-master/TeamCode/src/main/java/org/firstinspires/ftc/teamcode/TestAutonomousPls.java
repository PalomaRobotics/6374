package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TestAutonomousPls {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private double speed;
    private boolean autonomousPeriod = true;

    private ModernRoboticsI2cGyro gyro;

    private int motorState = 0;
    private int lastMotorState = 0;

    public TestAutonomousPls(ModernRoboticsI2cGyro gyroObject, DcMotor frontLeftMotorObject, DcMotor frontRightMotorObject, DcMotor backLeftMotorObect, DcMotor backRightMotorObject , double inputSpeed) {
        //move the parameters objects into their respective class-level variables

        this.frontLeftDrive = frontLeftMotorObject;
        this.frontRightDrive = frontRightMotorObject;
        this.backLeftDrive = backLeftMotorObect;
        this.backRightDrive = backRightMotorObject;
        this.speed = speed;

        //region Gyro Initialization
        this.gyro = gyroObject; //THIS IS AN I2C DEVICE. Find a Gyro Sensor on the robot named gyroSens. gyroSens is the name that appears in the phones configuration

        gyro.calibrate(); //calibrate the gyro. DON'T MOVE THE ROBOT OR SENSOR UNTIL THIS IS DONE!

        // This loop ties up the robot until gyro calibration is complete
        while (gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        gyro.resetZAxisIntegrator(); //reset the heading. The sensor only returns a heading for the Z axis
        //endregion
    }

    public void Autonomous(boolean waitForStart){

            motorState = 1;

            //region ChangeMovement

            //using a switch and the variable "motorState" we can change the movement of the robot to another predetermined value.
            //it can be upgraded to accept gyro movement commands, butI'm not really sure how to do that.

            if (lastMotorState != motorState)
            {
                lastMotorState = motorState;//set the last motor state to the current motor state

                        switch (motorState)
                        {
                            case 0://stop the robot
                                frontLeftDrive.setPower(0);
                                frontRightDrive.setPower(0);
                                backLeftDrive.setPower(0);
                                backRightDrive.setPower(0);
                                break;
                            case 1://move the robot forward
                                frontLeftDrive.setPower(speed);
                                frontRightDrive.setPower(speed);
                                backLeftDrive.setPower(speed);
                                backRightDrive.setPower(speed);
                                break;
                            case 2://move the robot backwards
                                frontLeftDrive.setPower(-speed);
                                frontRightDrive.setPower(-speed);
                                backLeftDrive.setPower(-speed);
                                backRightDrive.setPower(-speed);
                                break;
                            case 3://turnLeft
                                frontLeftDrive.setPower(-speed);
                                frontRightDrive.setPower(speed);
                                backLeftDrive.setPower(0);
                                backRightDrive.setPower(0);
                                break;
                            case 4://turnRight
                                frontLeftDrive.setPower(speed);
                                frontRightDrive.setPower(-speed);
                                backLeftDrive.setPower(0);
                                backRightDrive.setPower(0);
                                break;
                        }
            }//endregion

        //here step by step movement instructions should be written

            //region EndLoop

            //as long as ReturnAutonomousIncomplete() is true, loop this entire method

            //endregion

    }

    public boolean ReturnAutonomousIncomplete()
    {
        //if returns true, Autonomous method must be looped further
        return autonomousPeriod;
    }

    //region Nescessary Methods for Gyro?
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
    //endregion
}
