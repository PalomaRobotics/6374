package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by robotics on 4/18/2017.
 */
@TeleOp (name ="AbrahamTestRobots" , group ="TeleOp" )
@Disabled
public class AbrahamTestRobots extends OpMode {

    private DcMotorController MC1;
    private DcMotor Dleft; //DRIVE MOTOR LEFT
    private DcMotor Dright; //DRIVE MOTOR RIGHT
    private DcMotor INW; // INTAKE WHEEL MOTOR
    private DcMotor DOORM; //DOOR MOTOR
    private DcMotor CATM; //CATAPULT MOTOR





    public AbrahamTestRobots(){
    }

    @Override
    public void init() {
        //MC1 = hardwareMap.dcMotorController.get("MC1");
        Dleft = hardwareMap.dcMotor.get("Dleft");
        Dright = hardwareMap.dcMotor.get("Dright");
        DOORM = hardwareMap.dcMotor.get("DOORM");
         CATM = hardwareMap.dcMotor.get("CATM");

        INW = hardwareMap.dcMotor.get("INW");



    }

    @Override
    public void loop(){
        Boolean A = false;
        Boolean in = false;
        Boolean out = false;
        if(gamepad1.y== true){
            CATM.setPower(1);
        }
        else{CATM.setPower(0);}


        // Dleft.setPower(gamepad1.left_stick_y);//BASIC MOVEMENT
        // Dright.setPower(-gamepad1.right_stick_y);

        if(gamepad1.x==true){
            INW.setPower(1.0);



        }
    else{INW.setPower(0);}


        if(gamepad1.a==true){
            DOORM.setPower(1);
        }
        else{DOORM.setPower(0);}
        if(gamepad1.b==true){
            DOORM.setPower(-1);
        }
        else{DOORM.setPower(0);}
        Dleft.setPower(gamepad1.left_stick_y);
        Dright.setPower(-gamepad1.right_stick_y);

        }



      /*  if(gamepad1.a)

            A = true;

        if(A == true){        //PRIMER

            //EncoderClass.RunToEncoderDegree(CATM, 180 , -0.5); //VAR IS SUBJECT TO CHANGE
            EncoderClass.RunToEncoderDegree(DOORM, 60 , -0.5);//VAR IS SUBJECT TO CHANGE



        }

        if(gamepad1.b){

            // EncoderClass.RunToEncoderDegree(CATM, 180 , -0.5);
            EncoderClass.RunToEncoderDegree(DOORM, -4 , 0.5);
            A = false;
        }*/
    }








