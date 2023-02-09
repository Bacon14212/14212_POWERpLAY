package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Dariuskiwi3 (Blocks to Java)")
public class Dariuskiwi3 extends LinearOpMode {

    private DcMotor W1;
    private DcMotor W2;
    private DcMotor W3;
    private DcMotor spinner;
    private Servo IntakeL;
    private Servo IntakeR;
    private Servo claw;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float x;
        float y;
        double r;

        W1 = hardwareMap.get(DcMotor.class, "W1");
        W2 = hardwareMap.get(DcMotor.class, "W2");
        W3 = hardwareMap.get(DcMotor.class, "W3");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        IntakeL = hardwareMap.get(Servo.class, "IntakeL");
        IntakeR = hardwareMap.get(Servo.class, "IntakeR");
        claw = hardwareMap.get(Servo.class, "claw");


        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    spinner.setPower(1);
                } else if (gamepad1.left_bumper) {
                    spinner.setPower(-1);
                } else {
                    spinner.setPower(0);
                }


                if (gamepad1.a) {
                    claw.setPosition(0.5);
                } else if (gamepad1.b) {
                    claw.setPosition(0);
                }

                if (gamepad1.dpad_up) {
                    IntakeL.setPosition(0.5);
                    IntakeR.setPosition(0.5);
                } else if (gamepad1.b) {
                    IntakeL.setPosition(0);
                    IntakeR.setPosition(1);
                }


                // Put loop blocks here.
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                r = 0.69 * gamepad1.right_stick_x;
                W1.setPower((-0.5 * x - (Math.sqrt(3) / 2) * y) - r);
                W2.setPower((-0.5 * x + (Math.sqrt(3) / 2) * y) - r);
                W3.setPower(x - r);
                telemetry.update();
            }
        }
    }
}