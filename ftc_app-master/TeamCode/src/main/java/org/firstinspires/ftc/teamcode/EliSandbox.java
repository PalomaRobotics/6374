package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Eli Sandbox Auto", group = "Auto")
public class EliSandbox extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private void Ewait(int ms) {
        try {
            wait(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    double thiswaittime;


    public void runOpMode() {
        telemetry.addData("Status", "opMode activated");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "DLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "DRight");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        thiswaittime = runtime.time()+3.5;
        while (thiswaittime > runtime.time() && opModeIsActive()) {
            leftDrive.setPower(1);
            rightDrive.setPower(1);
            telemetry.addData("Status", "driving for" + (thiswaittime - runtime.time()) + "seconds"  );
            telemetry.update();
        }

        thiswaittime = runtime.time()+1;
        while (thiswaittime > runtime.time() && opModeIsActive()) {
            leftDrive.setPower(1);
            rightDrive.setPower(-1);
            telemetry.addData("Status", "turning for" + (thiswaittime - runtime.time()) + "seconds");
            telemetry.update();
        }

        thiswaittime = runtime.time()+6.5;
        while (thiswaittime > runtime.time() && opModeIsActive()) {
            leftDrive.setPower(1);
            rightDrive.setPower(1);
            telemetry.addData("Status", "driving for" + (thiswaittime - runtime.time()) + "seconds"  );
            telemetry.update();
        }

    }
}
