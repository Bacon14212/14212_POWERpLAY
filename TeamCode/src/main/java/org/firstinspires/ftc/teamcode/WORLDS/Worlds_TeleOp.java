
package org.firstinspires.ftc.teamcode.WORLDS;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "Worlds TeleOp")
public class Worlds_TeleOp extends OpMode {

    //SLIDER ENCODER CALCULATIONS
    static final double COUNTS_PER_MOTOR_REV = 145.1;        // TICKS PER REVOLUTION FOR 5203 1620RPM MOTOR
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;  // For figuring circumference
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 1;                   // MOTOR SPEED
    static final double FIRST_LEVEL_INCHES = 4.5;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 14.5;      // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_LEVEL_INCHES = 24.5;      // HEIGHT FOR THIRD LEVEL IN INCHES


    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;
    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);

    //FINITE STATE MACHINE SETUP
    public enum LiftState {
        Intake_Normal, Gripped_Normal, Driving_Around, Lift_Wait, Ready_To_Score, Lift_Lower, Open_Claw,
        Intake_Down_1, Intake_Down_2, Gripped_Down_1, Gripped_Down_2, Raising_Wrist, RESET
    }

    LiftState liftState = LiftState.Intake_Normal;

    boolean FRONT_INTAKE = true;
    boolean dpadDownLastUpdate = false;

    //DECLARE MOTORS FOR DRIVETRAIN
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //DECLARE MOTORS AND SERVOS FOR SLIDE
    public DcMotorEx LIFT;  // LIFT 1
    public DcMotorEx LIFT2;// LIFT 2
    public Servo pivotL;  // PIVOT SERVO LEFT
    public Servo pivotR; // PIVOT SERVO RIGHT
    public Servo wrist; // SERVO FOR WRIST LEFT RIGHT
    public Servo flip; // SERVO FOR WRIST UP DOWN
    public Servo claw; // SERVO FOR THE CLAW

    final double CLAW_OPEN = 0.38;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 0.24;    // SERVO POSITION TO CLOSE CLAW
    final double PIVOT_DOWN_FRONT = 1;
    final double PIVOT_DOWN_BACK = 0.08;
    final double PIVOT_UP_FRONT = 0.47;
    final double PIVOT_UP_BACK = 0.65;
    final double TWIST_UP_BACK = 0.3;
    final double TWIST_UP_FRONT = 0.3;
    final double TWIST_DOWN_FRONT = 0.98;
    final double TWIST_DOWN_BACK = 0.3;
    final double FLIP_UP_FRONT = 0.25;
    final double FLIP_UP_BACK = 0.29;
    final double FLIP_DOWN_FRONT = 0.35;
    final double FLIP_DOWN_BACK = 0.62;
    final double PIVOT_MID = 0.75;

    int liftTarget = 0;

    ElapsedTime dropTime = new ElapsedTime();
//    RevBlinkinLedDriver lights;

    public void init() {
        //INITIALIZES ALL MOTORS AND DEFAULTS SETTINGS
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        backLeft = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight = hardwareMap.dcMotor.get("rightRear");

        LIFT = hardwareMap.get(DcMotorEx.class, "Lift");
        LIFT2 = hardwareMap.get(DcMotorEx.class, "Lift2");

        claw = hardwareMap.servo.get("claw");
        pivotL = hardwareMap.servo.get("pivotL");
        pivotR = hardwareMap.servo.get("pivotR");
        wrist = hardwareMap.servo.get("wrist");
        flip = hardwareMap.servo.get("flip");
//        lights = hardwareMap.get(RevBlinkinLedDriver.class, "Lights");
//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        //REVERSE MOTORS IF NECESSARY
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


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //MAKES DRIVETRAIN MORE PRECISE (FORCES MOTORS TO BRAKE WHEN STOPPED)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SETS THE CLAW IN DEFAULT POSITION
        pivotL.setPosition(PIVOT_DOWN_FRONT);
        pivotR.setPosition(PIVOT_DOWN_FRONT);
        claw.setPosition(CLAW_OPEN);
        flip.setPosition(FLIP_DOWN_FRONT);
        wrist.setPosition(TWIST_DOWN_FRONT);

    }
    double PivotL_Position = 0;
    double PivotR_Position = 0;
    double claw_Position = 0;
    double flip_Position = 0;
    double wrist_Position = 0;

    boolean FirstUpdate = true;
    ElapsedTime scoreTimer = new ElapsedTime();

    public void loop() {


        /*
        if (liftState == LiftState.RESET){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        if(liftState == LiftState.GRIP_DOWN){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        if(liftState == LiftState.GRIP_UP){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }
        if(liftState == LiftState.OPEN_CLAW){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
*/

        //Constant Loop for LIFT
        LIFT.setTargetPosition(liftTarget);
        LIFT2.setTargetPosition(liftTarget);
        LIFT.setPower(ARM_SPEED);
        LIFT2.setPower(ARM_SPEED);
        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Math.abs(LIFT.getCurrentPosition() - LIFT_LEVEL_ORIGINAL) < 10) {
            LIFT.setPower(0);
            LIFT2.setPower(0);
        }

        switch (liftState) {
            //Drops Lift, claw open, pivot down
            case Intake_Normal:
                liftTarget = (LIFT_LEVEL_ORIGINAL);
                PivotL_Position = PIVOT_DOWN_FRONT;
                PivotR_Position = PIVOT_DOWN_FRONT;
                claw_Position = CLAW_OPEN;
                flip_Position = FLIP_DOWN_FRONT;
                wrist_Position = TWIST_DOWN_FRONT;

                //Sets Lift motors to 0 power

                //Grab a cone
                if (gamepad1.left_bumper) {
                    scoreTimer.reset();
                    liftState = LiftState.Gripped_Normal;
                    FirstUpdate = true;
                }
                if (gamepad1.dpad_left) {
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Down_2;
                    FirstUpdate = true;
                }
                if (gamepad1.dpad_right) {
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Down_1;
                    FirstUpdate = true;
                }
                if (gamepad1.left_stick_button) {

                }

                FirstUpdate = false;
                break;
            case Gripped_Normal:
                claw_Position = CLAW_CLOSE;
                if (gamepad1.left_trigger > 0.1) {
                    scoreTimer.reset();
                    liftState = LiftState.Driving_Around;
                    FirstUpdate = true;

                }
                //Sets arm up after 0.5 seconds
                if (scoreTimer.milliseconds() >= 500) {
                    flip_Position = 0.2;
                    wrist_Position = TWIST_UP_FRONT;
                    PivotL_Position = PIVOT_MID;
                    PivotR_Position = PIVOT_MID;
                }
                if (gamepad1.right_bumper) {
                    wrist_Position = TWIST_DOWN_FRONT;
                    scoreTimer.reset();
                    liftState = LiftState.RESET;
                    FirstUpdate = true;
                }

                FirstUpdate = false;
                break;

            case RESET:
                if (scoreTimer.milliseconds() >= 200) {
                    flip_Position = 0.2;
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Normal;
                    FirstUpdate = true;
                }
                FirstUpdate = false;
                break;

            case Driving_Around:
                if (gamepad1.a || gamepad1.b || gamepad1.y) {
                    liftState = LiftState.Ready_To_Score;
                    flip_Position = FLIP_UP_FRONT;
                    wrist_Position = TWIST_UP_FRONT;
                    FirstUpdate = true;
                    scoreTimer.reset();
                }

                FirstUpdate = false;
                break;

            case Ready_To_Score:
                if (gamepad1.a) {
                    liftTarget = LIFT_LEVEL_ONE;
                }
                if (gamepad1.b) {
                    liftTarget = LIFT_LEVEL_TWO;
                }
                if (gamepad1.y) {
                    liftTarget = LIFT_LEVEL_THREE;
                }
                PivotL_Position = PIVOT_UP_FRONT;
                PivotR_Position = PIVOT_UP_FRONT;
                flip_Position = FLIP_UP_FRONT;
                if (gamepad1.right_bumper) {
                    scoreTimer.reset();
                    liftState = LiftState.Lift_Lower;
                    FirstUpdate = true;
                }
                if (gamepad1.dpad_left) {
                    wrist_Position = TWIST_DOWN_FRONT;
                }
                if (gamepad1.dpad_right) {
                    wrist_Position = TWIST_UP_FRONT;
                }

                FirstUpdate = false;
                break;

            case Lift_Lower:
                liftTarget = (LIFT.getCurrentPosition() - 100);
                flip_Position = 0.28;
                PivotL_Position = 0.4;
                PivotR_Position = 0.4;

                if (scoreTimer.milliseconds() >= 200) {
                    liftState = LiftState.Open_Claw;
                    scoreTimer.reset();
                    FirstUpdate = true;
                }

                FirstUpdate = false;
                break;

            case Open_Claw:
                claw_Position = CLAW_OPEN;
                if (scoreTimer.milliseconds() >= 200 && gamepad1.left_stick_y > 0.15) {
                    wrist_Position = TWIST_DOWN_FRONT;
                    liftState = LiftState.Intake_Normal;
                    scoreTimer.reset();
                    FirstUpdate = true;
                }
                FirstUpdate = false;
                break;

            case Intake_Down_1:
                PivotL_Position = 0.8;
                PivotR_Position = 0.8;
                claw_Position = CLAW_OPEN;
                flip_Position = 0.94;
                wrist_Position = TWIST_DOWN_FRONT;
                if (gamepad1.left_bumper) {
                    flip_Position = 0.85;
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Down_2;
                    FirstUpdate = true;
                }
                if (gamepad1.right_bumper) {
                    flip_Position = FLIP_DOWN_FRONT;
                    scoreTimer.reset();
                    liftState = LiftState.RESET;
                    FirstUpdate = true;

                }

                FirstUpdate = false;
                break;

            case Intake_Down_2:
                if (scoreTimer.milliseconds() >= 100) {
                    PivotL_Position = 0.85;
                    PivotR_Position = 0.85;
                    claw_Position = CLAW_CLOSE;
                }
                if (scoreTimer.milliseconds() >= 500) {
                    wrist_Position = TWIST_UP_FRONT;
                    liftState = LiftState.Ready_To_Score;
                    scoreTimer.reset();
                    FirstUpdate = true;
                }
                FirstUpdate = false;
                break;

            default:
                liftState = LiftState.Intake_Normal;

        }

        double PivotL_Position_Mirrored = PIVOT_DOWN_FRONT - (PivotL_Position - PIVOT_DOWN_BACK);
        double PivotR_Position_Mirrored = PIVOT_DOWN_FRONT - (PivotR_Position - PIVOT_DOWN_BACK);
        double wrist_Position_Mirrored = TWIST_DOWN_FRONT - (wrist_Position - TWIST_DOWN_BACK);
        double flip_Position_Mirrored = FLIP_DOWN_BACK - (flip_Position - FLIP_DOWN_FRONT);

        if (FRONT_INTAKE) {
            pivotL.setPosition(PivotL_Position);
            pivotR.setPosition(PivotR_Position);
            wrist.setPosition(wrist_Position);
            flip.setPosition(flip_Position);
        } else {
            pivotL.setPosition(PivotL_Position_Mirrored);
            pivotR.setPosition(PivotR_Position_Mirrored);
            wrist.setPosition(wrist_Position_Mirrored);
            flip.setPosition(flip_Position_Mirrored);
        }

        claw.setPosition(claw_Position);


        //SAFETY BUTTON TO RESET THE SYSTEM BACK TO THE START

        //CODE FOR DRIVE TRAIN
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x / 1.2;
        if (!FRONT_INTAKE) {
            y *= -1; // Remember, this is reversed!
            x *= -1;
            rx *= 1;
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //divide by value greater than 1 to make it slower
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        if (Math.abs(LIFT.getCurrentPosition() - LIFT_LEVEL_ORIGINAL) > 50) {
            frontLeft.setPower(frontLeftPower / 1.2);
            backLeft.setPower(backLeftPower / 1.2);
            frontRight.setPower(frontRightPower / 1.2);
            backRight.setPower(backRightPower / 1.2);
        }

        telemetry.addData("State", liftState);
        telemetry.addData("Lift Position, Motor 1", LIFT.getCurrentPosition());
        telemetry.addData("Lift Position, Motor 2", LIFT2.getCurrentPosition());
        telemetry.addData("Wrist", wrist_Position);
        telemetry.addData("PivotL", PivotL_Position);
        telemetry.addData("PivotR", PivotR_Position);
        telemetry.addData("flip", flip_Position);
        telemetry.addData("INtake", FRONT_INTAKE);

        telemetry.addLine();
        telemetry.update();
        getRuntime();

        if(!dpadDownLastUpdate && gamepad1.dpad_down) {
            if (FRONT_INTAKE) {
                FRONT_INTAKE = false;
            } else {
                FRONT_INTAKE = true;
            }
        }
        dpadDownLastUpdate = gamepad1.dpad_down;

    }


}

