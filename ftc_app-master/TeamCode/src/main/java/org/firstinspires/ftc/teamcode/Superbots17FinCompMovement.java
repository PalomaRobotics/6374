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
@TeleOp (name ="Superbots17FinCompMovement" , group ="TeleOp" )
@Disabled
public class Superbots17FinCompMovement extends OpMode {

    private DcMotorController MC1;
    private DcMotorController MC3;
    private DcMotor Dleft;
    private DcMotor Dright;
    private DcMotor INW;
    private DcMotor DOORM;
    private DcMotor CATM;
    private DcMotor AEM;


    public Superbots17FinCompMovement() {
    }

    @Override
    public void init() {
        MC1 = hardwareMap.dcMotorController.get("MC1");
        MC1 = hardwareMap.dcMotorController.get("MC3");
        Dleft = hardwareMap.dcMotor.get("Dleft");
        Dright = hardwareMap.dcMotor.get("Dright");
         DOORM = hardwareMap.dcMotor.get("DOORM");
         CATM = hardwareMap.dcMotor.get("CATM");
        AEM = hardwareMap.dcMotor.get("AEM");
        INW = hardwareMap.dcMotor.get("INW");


    }

    @Override
    public void loop() {
       // Boolean A = false;



        DOORM.setPower(gamepad1.left_trigger);
        DOORM.setPower(-gamepad1.right_trigger);

        Dleft.setPower(-gamepad1.left_stick_y); // main controls
        Dright.setPower(gamepad1.right_stick_y);

        if(gamepad1.left_bumper){

            INW.setPower(1.0);
            telemetry.addData("forward","");
        }else{

            if(gamepad1.right_bumper){
                INW.setPower(-1.0);
                telemetry.addData("reverse","");
            }else{
                INW.setPower(0.0);
                telemetry.addData("stop","");
            }
        }




            if (gamepad1.a) {//primer

                CATM.setPower(1.0);


            }else{
                CATM.setPower(0.0);
            }


            if(gamepad1.b){

                EncoderClass.RunToEncoderValue(AEM, 10, 1.0);
            }
        }




    }


