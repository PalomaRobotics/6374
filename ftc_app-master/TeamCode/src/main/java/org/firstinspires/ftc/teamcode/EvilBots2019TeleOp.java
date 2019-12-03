package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import java.nio.channels.Pipe;

@TeleOp(name = "EvilBots19-20", group = "TeleOp")

public class EvilBots2019TeleOp extends OpMode {
    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;
    private DcMotor CLAW = null;
    private DcMotor PIVOT = null;
    private DcMotor SLIDE = null;
    private int maxClaw;
    private int minClaw;
    private int maxPivot;
    private int minPivot;
    private int maxSlide;
    private int minSlide;

    public void init(){

     //map hardware

      LFD = hardwareMap.get(DcMotor.class,"FL");
      RFD = hardwareMap.get(DcMotor.class,"FR");
      LBD = hardwareMap.get(DcMotor.class,"BL");
      RBD = hardwareMap.get(DcMotor.class,"BR");
      CLAW = hardwareMap.get(DcMotor.class,"CLAW");
      PIVOT = hardwareMap.get(DcMotor.class,"PIVOT");
      SLIDE = hardwareMap.get(DcMotor.class,"SLIDE");

      CLAW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      PIVOT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      SLIDE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //get init values for min and max
      maxClaw = 350;
      minClaw = -30;
      maxPivot = PIVOT.getCurrentPosition()+0;
      minPivot = PIVOT.getCurrentPosition();
      maxSlide = SLIDE.getCurrentPosition()+0;
      minSlide = SLIDE.getCurrentPosition();


    }
    public void loop() {

        //get input
        double inx = (double)gamepad1.left_stick_x;
        double iny = (double)gamepad1.left_stick_y;
        double inRotation = (double)gamepad1.right_stick_x;

        //
        //DRIVING
        //

        //calculate powers
        double[] dirs = HolonomicDrive.RoboMoveXY(inx,iny);
        double[] rotToAdd = HolonomicDrive.RoboRotate(inRotation);

        //average values
        for(int i=0;i<rotToAdd.length;i++)
        {
            dirs[i]=(dirs[i]+rotToAdd[i])/2;
        }
        //decreases power if fine control btn is presses
        if(gamepad1.b)
        {
            for(int i=0;i<dirs.length;i++)
            {
                dirs[i]=dirs[i]/4;
            }
        }

        // find biggest dir
        double maxValue = 0;
        int maxIndex = 0;
        for (int i = 0 ;i<dirs.length;i++)
        {
           if( Math.abs(dirs[i]) > maxValue )
           {
               maxValue = Math.abs(dirs[i]);
               maxIndex = i;
           }
        }

        for (int i = 0; i<dirs.length;i++)
        {
            if (maxValue!=0)
            {
                dirs[i] = dirs[i] / maxValue;

            }
            else
            {
                dirs[i]=0;
            }
        }


        //set power for motors
        LFD.setPower(dirs[0]);
        RFD.setPower(dirs[1]);
        LBD.setPower(dirs[2]);
        RBD.setPower(dirs[3]);

        //telemetry
        telemetry.addData("Stick X", gamepad1.left_stick_x);
        telemetry.addData("Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Powers", (String.valueOf(Math.round(dirs[0]*100)/100)+String.valueOf(Math.round(dirs[1]*100)/100)+String.valueOf(Math.round(dirs[2]*100)/100)+String.valueOf(Math.round(dirs[3]*100)/100)));
        telemetry.addData("Direction",(HolonomicDrive.XYtoDeg((float)inx,(float)iny)));

        //
        //MANIPULATION
        //

        //get inputs
        double inClaw = (double)gamepad2.right_stick_x;
        boolean inPivotUp = gamepad2.dpad_up;
        boolean inPivotDown = gamepad2.dpad_down;
        double inSlide = gamepad2.left_stick_y;




        SLIDE.setPower(inSlide/3);

        if (CLAW.getCurrentPosition() <= maxClaw && CLAW.getCurrentPosition() >= minClaw) {
            CLAW.setPower(inClaw / 8);
        }
        else if (CLAW.getCurrentPosition()>maxClaw) {
            CLAW.setPower(-0.2);
        }
        else if(CLAW.getCurrentPosition()<minClaw) {
            CLAW.setPower(0.2);
        }
        else {
              CLAW.setPower(0);
        }
        if(inPivotUp){
            PIVOT.setPower(0.2);
        }else if(inPivotDown){
            PIVOT.setPower(-0.2);
        }else{
            PIVOT.setPower(0);
        }
        //telemetry
        telemetry.addData("Claw pos",CLAW.getCurrentPosition());
        telemetry.addData("Slide pos",SLIDE.getCurrentPosition());
        telemetry.addData("Pivot pos",PIVOT.getCurrentPosition());
    }


}
