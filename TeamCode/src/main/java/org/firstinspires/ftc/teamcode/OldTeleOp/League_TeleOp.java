package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class League_TeleOp extends OpMode {

    //SLIDER ENCODER CALCULATIONS
    static final double COUNTS_PER_MOTOR_REV = 103.8;        // TICKS PER REVOLUTION FOR 5203 435RPM MOTOR
    static final double DRIVE_GEAR_REDUCTION = 1.0;          // This is < 1.0 if geared UP
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 1;
    static final double FIRST_LEVEL_INCHES  = 14;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 24;        // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_LEVEL_INCHES  = 34;        // HEIGHT FOR THIRD LEVEL IN INCHES


    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;
    final int LIFT_LEVEL_ZERO =0;
    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);

    //FINITE STATE MACHINE SETUP
    public enum LiftState {
        LIFT_START, LIFT_EXTEND_ZERO, LIFT_IDLE, /* LIFT_EXTEND_ONE,
        LIFT_EXTEND_TWO, LIFT_EXTEND_THREE,*/ LIFT_DROP, LIFT_EXTEND_ORIGINAL
    }
    LiftState liftState = LiftState.LIFT_START;

    //DECLARE MOTORS FOR DRIVETRAIN
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //DECLARE MOTORS AND SERVOS FOR SLIDE
    public DcMotor Lift1;// MOTOR FOR THE SLIDE
    public DcMotor Lift2;
    public Servo   clawL;// SERVO FOR THE CLAW
    public Servo   clawR;
    public Servo   claw;

    final double CLAW_OPEN =  0.65;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 1;    // SERVO POSITION TO CLOSE CLAW

    public void init() {
        //INITIALIZES ALL MOTORS AND DEFAULTS SETTINGS
        frontLeft  = hardwareMap.dcMotor.get("leftFront");
        backLeft   = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight  = hardwareMap.dcMotor.get("rightRear");

        Lift1 = hardwareMap.dcMotor.get("rightlift");
        Lift2 = hardwareMap.dcMotor.get("leftlift");

        claw   = hardwareMap.servo.get("claw");
        clawL  = hardwareMap.servo.get("clawL");
        clawR  = hardwareMap.servo.get("clawR");

        //REVERSE MOTORS IF NECESSARY
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        //SETS THE ENCODERS ON THE SLIDE TO DEFAULT VALUES
        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //SETS THE CLAW IN DEFAULT POSITION
        claw.setPosition(CLAW_OPEN);

        //MAKES DRIVETRAIN MORE PRECISE (FORCES MOTORS TO BRAKE WHEN STOPPED)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop() {



        //CODE FOR DRIVE TRAIN
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //divide by value greater than 1 to make it slower
        frontLeft.setPower(-frontLeftPower/1.2);
        backLeft.setPower(-backLeftPower/1.2);
        frontRight.setPower(-frontRightPower/1.2);
        backRight.setPower(-backRightPower/1.2);
    }
}