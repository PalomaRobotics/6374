package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;


public class GyroControlClass {

    private DcMotor left, right;
    private double leftPwr = 0.25;
    private double rightPwr = 0.25;
    private int heading = 0;              // Gyro integrated heading
    private boolean allStop = false;
    private int prevLeftValue, prevRightValue;
    private ModernRoboticsI2cGyro gyro;
    public int driveStraitTimer = 0;
    public int turnTimer = 0;
    private boolean turnDirection = true;
    private int turnPower = 1;


    public GyroControlClass(ModernRoboticsI2cGyro gyroObject)
    {

        this.gyro = gyroObject; //THIS IS AN I2C DEVICE. Find a Gyro Sensor on the robot named gyroSens. gyroSens is the name that appears in the phones configuration

        gyro.calibrate(); //calibrate the gyro. DON'T MOVE THE ROBOT OR SENSOR UNTIL THIS IS DONE!

        // This loop ties up the robot until gyro calibration is complete
        while (gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        gyro.resetZAxisIntegrator(); //reset the heading. The sensor only returns a heading for the Z axis
    }

    public void FollowWhiteLine(DcMotor leftDrive, DcMotor rightDrive, double speed, double sensorValue) {
        if (sensorValue > .2f) {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
            if (driveStraitTimer > 1)
                turnDirection = !turnDirection;
            driveStraitTimer = 0;
            turnTimer = 0;
            turnPower = 1;
        } else if (driveStraitTimer < 120) {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
        } else if (turnTimer < 5000 * turnPower) {
            if (turnDirection == true)
            {
                leftDrive.setPower(-speed/3);
                rightDrive.setPower(speed/3);
            }
            else
            {
                leftDrive.setPower(speed/3);
                rightDrive.setPower(-speed/3);
            }
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        driveStraitTimer++;
        turnTimer++;
        if (turnTimer > 5000 * turnPower) {
            turnPower = turnPower + 1;
            turnDirection = !turnDirection;
        }
    }

    public void DriveStraight(DcMotor leftDrive, DcMotor rightDrive, double speed)
    {

    }
    public void FollowWhiteLine2(DcMotor leftDrive, DcMotor rightDrive, double speed, double sensorValue)
    {
        if (sensorValue > .2f) //check the sensor value
        {
            leftDrive.setPower(speed);//drive
            rightDrive.setPower(speed);
            if (driveStraitTimer > 0)
                turnDirection = !turnDirection;
            driveStraitTimer = 0;//reset timers
            turnTimer = 0;
            turnPower = 1;
        }else if (driveStraitTimer < 20) {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
        }else if (turnTimer < 5000 * turnPower) {
            if (turnDirection == false)
                speed = speed * -1;//invert direction
            leftDrive.setPower(-speed/3);
            rightDrive.setPower(speed/3);
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        driveStraitTimer++;//increase timers
        turnTimer++;

        if (turnTimer > 5000 * turnPower) {//if the timer is greater than 5000*power
            turnPower = turnPower*2 + 3;//increase turn power
            turnDirection = !turnDirection;//invert turndirection
        }
    }

    public void FollowXLine(double x, DcMotor leftDrive, DcMotor rightDrive, double speed, double sensorValue, float range)
    {
        if (Math.abs(x - sensorValue) < range/2) //check the sensor value
        {
            leftDrive.setPower(speed);//drive
            rightDrive.setPower(speed);
            if (driveStraitTimer > 0)
                turnDirection = !turnDirection;
            driveStraitTimer = 0;//reset timers
            turnTimer = 0;
            turnPower = 1;
        }else if (driveStraitTimer < 20) {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
        }else if (turnTimer < 5000 * turnPower) {
            if (turnDirection == false)
                speed = speed * -1;//invert direction
            leftDrive.setPower(-speed/3);
            rightDrive.setPower(speed/3);
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        driveStraitTimer++;//increase timers
        turnTimer++;

        if (turnTimer > 5000 * turnPower) {//if the timer is greater than 5000*power
            turnPower = turnPower*2 + 3;//increase turn power
            turnDirection = !turnDirection;//invert turndirection
        }
    }

    public void Rotate(DcMotor leftDrive, DcMotor rightDrive, double speed, int target)
    {
        if(target <= 180) {
            //If the gyro is close to the target...
            if (gyro.getHeading() < target - 5) {
                rightDrive.setPower(speed);
                leftDrive.setPower(-speed);
            }
            //If the gyro passed the target...
            else if (gyro.getHeading() > target + 5) {
                rightDrive.setPower(-speed);
                leftDrive.setPower(speed);
            } else {
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                return;
            }
        }
        else if(target > 180)
        {
            //If the gyro is close to the target...
            if (gyro.getHeading() < target - 5) {
                rightDrive.setPower(-speed);
                leftDrive.setPower(speed);
            }
            //If the gyro passed the target...
            else if (gyro.getHeading() > target + 5) {
                rightDrive.setPower(speed);
                leftDrive.setPower(-speed);
            } else {
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                return;
            }
        }
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
