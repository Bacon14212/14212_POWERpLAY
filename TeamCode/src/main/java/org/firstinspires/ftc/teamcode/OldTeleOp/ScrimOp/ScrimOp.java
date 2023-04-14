package org.firstinspires.ftc.teamcode.OldTeleOp.ScrimOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Official_TeleOp.Champs_TeleOp;

@Disabled
@TeleOp
public class ScrimOp extends LinearOpMode {
    //SLIDER ENCODER CALCULATIONS
    static final double COUNTS_PER_MOTOR_REV = 145.1;        // TICKS PER REVOLUTION FOR 5203 1620RPM MOTOR
    static final double DRIVE_GEAR_REDUCTION = 1.0;         // This is < 1.0 if geared UP
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;  // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 1;
    static final double FIRST_LEVEL_INCHES = 3;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 12;      // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_LEVEL_INCHES = 22;      // HEIGHT FOR THIRD LEVEL IN INCHES



    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;
    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);

    //FINITE STATE MACHINE SETUP
    public enum LiftState {
        RESET,GRIP_DOWN,GRIP_UP,OPEN_CLAW,ORIGINAL,FRONT
    }

    Champs_TeleOp.LiftState liftState = Champs_TeleOp.LiftState.RESET;

    //DECLARE MOTORS FOR DRIVETRAIN
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //DECLARE MOTORS AND SERVOS FOR SLIDE
    public DcMotorEx LIFT;// MOTOR FOR THE SLIDE
    public DcMotorEx LIFT2;
    public Servo pivotL;
    public Servo pivotR;

    final double CLAW_OPEN = 0.5;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 0;    // SERVO POSITION TO CLOSE CLAW
    final double WRIST_UP = 0.59;
    final double WRIST_DOWN = 0.03;

    int liftTarget = 0;

    ElapsedTime dropTime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //set variables up
        frontLeft  = hardwareMap.dcMotor.get("leftFront");
        backLeft   = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight  = hardwareMap.dcMotor.get("rightRear");

        LIFT = hardwareMap.get(DcMotorEx.class, "Lift");
        LIFT2 = hardwareMap.get(DcMotorEx.class, "Lift2");

        pivotL = hardwareMap.servo.get("pivotL");
        pivotR = hardwareMap.servo.get("pivotR");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //setting up break behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LIFT2.setDirection(DcMotorSimple.Direction.REVERSE);

        //SETS THE ENCODERS ON THE SLIDE TO DEFAULT VALUES
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LIFT2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LIFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LIFT2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotL.setPosition(1);
        pivotR.setPosition(1);


        //Code will for start to be pressed
        waitForStart();
        resetRuntime();

        //Loop for TeleOp
        while (opModeIsActive()) {
            LIFT.setTargetPosition(liftTarget);
            LIFT2.setTargetPosition(liftTarget);
            LIFT.setPower(ARM_SPEED);
            LIFT2.setPower(ARM_SPEED);
            LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.a){
                pivotL.setPosition(1);
                pivotR.setPosition(1);
            }
            if (gamepad1.b){
                pivotL.setPosition(0.1);
                pivotR.setPosition(0.1);
            }
            if (gamepad1.y){
                pivotL.setPosition(0.45);
                pivotR.setPosition(0.45);
            }
            if (gamepad1.dpad_up){
                liftTarget = LIFT_LEVEL_ONE;
            }
            else if (gamepad1.dpad_right){
                liftTarget = LIFT_LEVEL_TWO;
            }
            else if (gamepad1.dpad_down){
                liftTarget = LIFT_LEVEL_THREE;
            }
            else if (gamepad1.dpad_left){
                liftTarget = LIFT_LEVEL_ORIGINAL;
            }



            double x = -gamepad1.left_stick_x* 1.1; // Remember, this is reversed!
            double y = gamepad1.left_stick_y ; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


            telemetry.addData("Status", "Run Time: ");
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}


