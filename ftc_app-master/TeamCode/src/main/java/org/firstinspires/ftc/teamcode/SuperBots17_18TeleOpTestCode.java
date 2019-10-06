package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by robotics on 11/7/2016.
 */
@TeleOp (name = "SuperBots16_17TeleOp", group = "TeleOp")
@Disabled
public class SuperBots17_18TeleOpTestCode extends OpMode {

    //variables here



    //these are the wheels
    static DcMotor FLDrive;
    static DcMotor BLDrive;
    static DcMotor FRDrive;
    static DcMotor BRDrive;

    //this is the lift for the arm
    static DcMotor arm;

    //this is the horizontal slide motor
    static DcMotor slide;

    //these are the servos
    static Servo flicker;
    static Servo claw;

    CompassSensor compass;


    public SuperBots17_18TeleOpTestCode() {

    }//leave empty

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        FLDrive = hardwareMap.dcMotor.get("FLeft");
        BLDrive = hardwareMap.dcMotor.get("BLeft");//left set of wheels

        FRDrive = hardwareMap.dcMotor.get("FRight");
        BRDrive = hardwareMap.dcMotor.get("BRight");//right set of wheels

        FRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BRDrive.setDirection(DcMotorSimple.Direction.REVERSE);//reverse direction of right set of wheels to not spin

        arm = hardwareMap.dcMotor.get("Lift");//this is the arm lift

        slide = hardwareMap.dcMotor.get("Slide");

        flicker = hardwareMap.servo.get("flicker");//this is for the jewel flicker on the back
        claw = hardwareMap.servo.get("claw"); // this is for the claw mechanism

        compass = hardwareMap.compassSensor.get("compass");//compass?
    }//initialize

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop()
    {
        Drive(-gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad1.right_stick_y);

        Claw(gamepad1.dpad_right, gamepad1.dpad_left);

        Arm(gamepad1.dpad_up, gamepad1.dpad_down);

        Slide(gamepad1.left_bumper, gamepad1.right_bumper);



    }//end main loop

    public static void Drive(float m1, float m2, float m3, float m4)
    {
        FLDrive.setPower(m1);//control front left wheel
        BLDrive.setPower(m2);//control back left wheel

        FRDrive.setPower(m3);//control front right wheel
        BRDrive.setPower(m4);//control back right wheel
    }

    public static void Claw(boolean cb1, boolean cb2)
    {
        if(cb1 == true)
        {
            claw.setPosition(Range.clip(claw.getPosition() + .01, claw.MIN_POSITION, claw.MAX_POSITION));//open claw
        }
        if(cb2)
        {
            claw.setPosition(Range.clip(claw.getPosition() - .01, claw.MIN_POSITION, claw.MAX_POSITION));//open claw
        }
    }

    public static void Arm(boolean ab1, boolean ab2)
    {
        if(ab1)
        {
            arm.setPower(.5);//arm up
        }

        if(ab2)
        {
            arm.setPower(-.5);//arm down
        }
    }

    public static void Slide(boolean sb1, boolean sb2)
    {
        if(sb1)
        {
            slide.setPower(-.5);
        }

        if(sb2)
        {
            slide.setPower(.5);
        }
    }

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

