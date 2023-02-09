package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//Declare TeleOp    
@TeleOp
@Disabled
public class ScrimOp extends LinearOpMode {
    // Declaring Motors
    public DcMotor Lift       = null;
    public DcMotor Lift2      = null;
    public DcMotor frontLeft  = null;
    public DcMotor backLeft   = null;
    public DcMotor frontRight = null;
    public DcMotor backRight  = null;
    public DcMotor claw       = null;

    public Servo   Intake     = null;




    @Override
    public void runOpMode(){
        //set variables up
        frontLeft  = hardwareMap.dcMotor.get("leftFront");
        backLeft   = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight  = hardwareMap.dcMotor.get("rightRear");
        Lift  = hardwareMap.dcMotor.get("Lift1");
        Lift2 = hardwareMap.dcMotor.get("Lift2");
        claw = hardwareMap.dcMotor.get("Claw");
        Intake = hardwareMap.servo.get("Intake");

        //Most mechanum wheels will need to have wheels reversed
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        //setting up break behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Intake.setPosition(0);



        //Code will for start to be pressed
        waitForStart();
        resetRuntime();

        //Loop for TeleOp
        while (opModeIsActive()) {

            //Code for* lift right bumper extends claw up and sets power to 0 if not being pressed
            if (gamepad1.right_bumper) {
                Lift.setPower(0.5);
            }
            else if (gamepad1.left_bumper){
                Lift.setPower(-0.5);
            }

            else {
                Lift.setPower(0);
            }

            if (gamepad1.a){
                Intake.setPosition(1);
            }
            else if (gamepad1.b){
                Intake.setPosition(0);

            }

            if(gamepad2.left_bumper) {
               claw.setPower(1);
            }
            else if(gamepad1.right_bumper){
                claw.setPower(-1);
            }



            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            //divide by value greater than 1 to make it slower
            frontLeft.setPower(-frontLeftPower / 1);
            backLeft.setPower(backLeftPower / 1);
            frontRight.setPower(-frontRightPower / 1);
            backRight.setPower(backRightPower / 1);

            telemetry.addData("Status", "Run Time: ");
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}

