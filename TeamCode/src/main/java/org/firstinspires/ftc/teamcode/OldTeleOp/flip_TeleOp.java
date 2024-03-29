package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class flip_TeleOp extends OpMode {

    //SLIDER ENCODER CALCULATIONS
    static final double COUNTS_PER_MOTOR_REV = 103.8;        // TICKS PER REVOLUTION FOR 5203 1620RPM MOTOR
    static final double DRIVE_GEAR_REDUCTION = 1.0;          // This is < 1.0 if geared UP
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 0.5;
    static final double FIRST_LEVEL_INCHES = 14;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 24;        // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_LEVEL_INCHES = 34;
    static final double STACK_FIVE = 5;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double STACK_FOUR = 4;        // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double STACK_THREE = 3;      // HEIGHT FOR THIRD LEVEL IN INCHES


    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;
    final int LIFT_LEVEL_ZERO = 0;
    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);
    final int STACK5 = (int) (STACK_FIVE * COUNTS_PER_INCH);
    final int STACK4 = (int) (STACK_FOUR * COUNTS_PER_INCH);
    final int STACK3 = (int) (STACK_THREE * COUNTS_PER_INCH);

    //FINITE STATE MACHINE SETUP
    public enum LiftState {
        LIFT_START, LIFT_EXTEND_ZERO, LIFT_IDLE, LIFT_EXTEND_ONE,
        LIFT_EXTEND_TWO, LIFT_EXTEND_THREE,LIFT_DROP, LIFT_EXTEND_ORIGINAL
    }

    LiftState liftState = LiftState.LIFT_START;

    //DECLARE MOTORS FOR DRIVETRAIN
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //DECLARE MOTORS AND SERVOS FOR SLIDE
    public DcMotor LIFT;// MOTOR FOR THE SLIDE
    public DcMotor LIFT2;
    public Servo claw;// SERVO FOR THE CLAW
    public Servo pivotL;
    public Servo pivotR;
    public Servo wrist;

    final double CLAW_OPEN = 0.5;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 1;    // SERVO POSITION TO CLOSE CLAW
    final double PIVOT_DOWN = 0;
    final double PIVOT_UP = 0.7;
    final double WRIST_UP = 0.3;
    final double WRIST_DOWN = 1;


    ElapsedTime dropTime = new ElapsedTime();

    public void init() {
        //INITIALIZES ALL MOTORS AND DEFAULTS SETTINGS
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        backLeft = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight = hardwareMap.dcMotor.get("rightRear");

        LIFT = hardwareMap.dcMotor.get("Lift");
        LIFT2 = hardwareMap.dcMotor.get("Lift2");
        claw = hardwareMap.servo.get("claw");
        pivotL = hardwareMap.servo.get("pivotL");
        pivotR = hardwareMap.servo.get("pivotR");
        wrist = hardwareMap.servo.get("wrist");


        //REVERSE MOTORS IF NECESSARY
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        LIFT2.setDirection(DcMotorSimple.Direction.REVERSE);
        //SETS THE ENCODERS ON THE SLIDE TO DEFAULT VALUES
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LIFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LIFT2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LIFT2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //SETS THE CLAW IN DEFAULT POSITION
        claw.setPosition(CLAW_OPEN);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //MAKES DRIVETRAIN MORE PRECISE (FORCES MOTORS TO BRAKE WHEN STOPPED)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void loop() {
        switch (liftState) {
            case LIFT_START:
                if (gamepad1.left_bumper) {
                    claw.setPosition(CLAW_CLOSE);
                    liftState = LiftState.LIFT_EXTEND_ZERO;
                    telemetry.addData("Lift: ", LIFT.getCurrentPosition());

                }
                break;
            case LIFT_EXTEND_ZERO:
                if (Math.abs(LIFT.getCurrentPosition() - LIFT_LEVEL_ZERO) < 10) {
                    LIFT.setPower(0);
                    LIFT2.setPower(0);
                    liftState = LiftState.LIFT_IDLE;
                    telemetry.addData("Lift: ", LIFT.getCurrentPosition());
                }
                break;
            case LIFT_IDLE:
                if (gamepad1.a) {
                    wrist.setPosition(WRIST_UP);
                    pivotL.setPosition(0.3);
                    pivotR.setPosition(PIVOT_UP);
                    LIFT.setTargetPosition(LIFT_LEVEL_ONE);
                    LIFT2.setTargetPosition(LIFT_LEVEL_ONE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftState = LiftState.LIFT_DROP;
                }

                if (gamepad1.b) {
                    wrist.setPosition(WRIST_UP);
                    pivotL.setPosition(0.3);
                    pivotR.setPosition(PIVOT_UP);
                    LIFT.setTargetPosition(LIFT_LEVEL_TWO);
                    LIFT2.setTargetPosition(LIFT_LEVEL_TWO);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    liftState = LiftState.LIFT_DROP;
                }

                if (gamepad1.y) {
                    wrist.setPosition(WRIST_UP);
                    pivotL.setPosition(0.3);
                    pivotR.setPosition(PIVOT_UP);
                    LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                    LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftState = LiftState.LIFT_DROP;
                }
                break;

            case LIFT_DROP:
                if (gamepad1.right_bumper) {
                    claw.setPosition(CLAW_OPEN);


                    liftState = LiftState.LIFT_EXTEND_ONE;
                }

                if (gamepad1.a) {
                    pivotL.setPosition(PIVOT_UP);
                    pivotR.setPosition(PIVOT_UP);
                    wrist.setPosition(WRIST_UP);
                    LIFT.setTargetPosition(LIFT_LEVEL_ONE);
                    LIFT2.setTargetPosition(LIFT_LEVEL_ONE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    liftState = LiftState.LIFT_DROP;
                }

                if (gamepad1.b) {
                    pivotL.setPosition(PIVOT_UP);
                    pivotR.setPosition(PIVOT_UP);
                    wrist.setPosition(WRIST_UP);
                    LIFT.setTargetPosition(LIFT_LEVEL_TWO);
                    LIFT2.setTargetPosition(LIFT_LEVEL_TWO);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    liftState = LiftState.LIFT_DROP;
                }

                if (gamepad1.y) {
                    pivotL.setPosition(PIVOT_UP);
                    pivotR.setPosition(PIVOT_UP);
                    wrist.setPosition(WRIST_UP);
                    LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                    LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftState = LiftState.LIFT_DROP;
                }
                break;

            case LIFT_EXTEND_ORIGINAL:
                if (Math.abs(LIFT.getCurrentPosition() - LIFT_LEVEL_ORIGINAL) < 10) {
                    LIFT.setPower(0);
                    LIFT2.setPower(0);
                    liftState = LiftState.LIFT_START;
                    telemetry.addData("Lift: ", LIFT.getCurrentPosition());
                }

            case LIFT_EXTEND_ONE: {


                liftState = LiftState.LIFT_EXTEND_TWO;
            }

            case LIFT_EXTEND_TWO:
                if (claw.equals(CLAW_OPEN)){
                pivotR.setPosition(0);
                LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                LIFT.setPower(ARM_SPEED / 1.5);
                LIFT2.setPower(ARM_SPEED / 1.5);
                LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(WRIST_DOWN);

                liftState = LiftState.LIFT_EXTEND_ORIGINAL;
            }


                break;

            default:
                liftState = LiftState.LIFT_START;

        }


        //SAFETY BUTTON TO RESET THE SYSTEM BACK TO THE START
        if (gamepad1.x && liftState != LiftState.LIFT_START) {
            claw.setPosition(CLAW_OPEN);
            pivotL.setPosition(1);
            pivotR.setPosition(0);
            LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
            LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
            LIFT.setPower(ARM_SPEED);
            LIFT2.setPower(ARM_SPEED);
            LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(WRIST_DOWN);

        }

        //CODE FOR DRIVE TRAIN
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //divide by value greater than 1 to make it slower
        frontLeft.setPower(-frontLeftPower/1.1);
        backLeft.setPower(-backLeftPower/1.1);
        frontRight.setPower(-frontRightPower/1.1);
        backRight.setPower(-backRightPower/1.1);

        telemetry.addData("State",liftState);
        telemetry.update();

    }

}
