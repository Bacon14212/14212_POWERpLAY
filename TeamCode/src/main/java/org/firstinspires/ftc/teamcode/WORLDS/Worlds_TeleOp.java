
package org.firstinspires.ftc.teamcode.WORLDS;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        Intake_Normal,Gripped_Normal,Driving_Around,Lift_Wait,Ready_To_Score,Lift_Lower,Open_Claw,
        Intake_Down_1,Intake_Down_2,Gripped_Down_1,Gripped_Down_2,Raising_Wrist
    }

    LiftState liftState = LiftState.Intake_Normal;

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
    final double PIVOT_DOWN_BACK = 0.1;
    final double PIVOT_UP_FRONT = 0.46;
    final double PIVOT_UP_BACK = 0.65;
    final double TWIST_UP_BACK = 0.3;
    final double TWIST_UP_FRONT = 0.28;
    final double TWIST_DOWN_FRONT = 0.98;
    final double TWIST_DOWN_BACK = 0;
    final double FLIP_UP_FRONT = 0.28;
    final double FLIP_UP_BACK = 0.29;
    final double FLIP_DOWN_FRONT = 0.37;
    final double FLIP_DOWN_BACK = 0;
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

    boolean FirstUpdate = true;
    double startTime = 0;
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

        switch (liftState) {
            //Drops Lift, claw open, pivot down
            case Intake_Normal:
                liftTarget = (LIFT_LEVEL_ORIGINAL);
                pivotL.setPosition(PIVOT_DOWN_FRONT);
                pivotR.setPosition(PIVOT_DOWN_FRONT);
                claw.setPosition(CLAW_OPEN);
                flip.setPosition(FLIP_DOWN_FRONT);
                wrist.setPosition(TWIST_DOWN_FRONT);

                //Sets Lift motors to 0 power
                if (Math.abs(LIFT.getCurrentPosition() - LIFT_LEVEL_ORIGINAL) < 5) {
                    LIFT.setPower(0);
                    LIFT2.setPower(0);
                }

                //Grab a cone
                if(gamepad1.left_bumper){
                    scoreTimer.reset();
                    liftState = LiftState.Gripped_Normal;
                    FirstUpdate = true;
                }
                if(gamepad1.dpad_left){
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Down_2;
                    FirstUpdate = true;
                }
                if(gamepad1.dpad_right){
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Down_1;
                    FirstUpdate = true;
                }

                FirstUpdate=false;
                break;
            case Gripped_Normal:
                if (gamepad1.left_trigger > 0.1){
                    scoreTimer.reset();
                    liftState = LiftState.Driving_Around;
                    FirstUpdate = true;

                }
                //Sets arm up after 0.5 seconds
                claw.setPosition(CLAW_CLOSE);
                if (scoreTimer.milliseconds() >= startTime + 500) {
                    flip.setPosition(0.2);
                    wrist.setPosition(TWIST_UP_FRONT);
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                }
                if (gamepad1.right_bumper){
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    if (scoreTimer.milliseconds() >= startTime + 500) {
                        scoreTimer.reset();
                        liftState = LiftState.Intake_Normal;
                        FirstUpdate = true;
                    }
                }

                FirstUpdate = false;
                break;

            case Driving_Around:
                claw.setPosition(CLAW_CLOSE);
                if (gamepad1.a || gamepad1.b || gamepad1.y) {
                    liftState = LiftState.Ready_To_Score;
                    wrist.setPosition(TWIST_UP_BACK);
                    flip.setPosition(FLIP_UP_BACK);
                    FirstUpdate = true;
                    scoreTimer.reset();
                }

                FirstUpdate = false;
                break;

            case Ready_To_Score:
                if(gamepad1.a){
                    liftTarget = LIFT_LEVEL_ONE;
                }
                if(gamepad1.b){
                    liftTarget = LIFT_LEVEL_TWO;
                }
                if(gamepad1.y){
                    liftTarget = LIFT_LEVEL_THREE;
                }
                pivotL.setPosition(PIVOT_UP_FRONT);
                pivotR.setPosition(PIVOT_UP_FRONT);
                flip.setPosition(FLIP_UP_FRONT);
                if(gamepad1.right_bumper){
                    scoreTimer.reset();
                    liftState = LiftState.Lift_Lower;
                    FirstUpdate = true;
                }
                if (gamepad1.dpad_left){
                    wrist.setPosition(TWIST_DOWN_FRONT);
                }
                if (gamepad1.dpad_right){
                    wrist.setPosition(TWIST_UP_FRONT);
                }

                FirstUpdate = false;
                break;

            case Lift_Lower:
                liftTarget = (LIFT.getCurrentPosition() - 100);
                flip.setPosition(0.3);
                pivotL.setPosition(0.35);
                pivotR.setPosition(0.35);

                if (scoreTimer.milliseconds() >= startTime + 200) {
                liftState = LiftState.Open_Claw;
                scoreTimer.reset();
                FirstUpdate = true;
            }

                FirstUpdate = false;
                break;

            case Open_Claw:
                claw.setPosition(CLAW_OPEN);
                if (scoreTimer.milliseconds() >= startTime + 200) {
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    liftState = LiftState.Intake_Normal;
                    scoreTimer.reset();
                    FirstUpdate = true;
                }
                FirstUpdate = false;
                break;

            case Intake_Down_1:
                pivotL.setPosition(0.8);
                pivotR.setPosition(0.8);
                claw.setPosition(CLAW_OPEN);
                flip.setPosition(0.92);
                wrist.setPosition(TWIST_DOWN_FRONT);
                if (gamepad1.left_bumper){
                    flip.setPosition(0.86);
                    scoreTimer.reset();
                    liftState = LiftState.Intake_Down_2;
                    FirstUpdate = true;
                }
                if (gamepad1.right_bumper) {
                    flip.setPosition(FLIP_DOWN_FRONT);
                    if (scoreTimer.milliseconds() >= startTime + 1000) {
                        scoreTimer.reset();
                        liftState = LiftState.Intake_Normal;
                        FirstUpdate = true;
                    }
                }


                FirstUpdate = false;
                break;

            case Intake_Down_2:
                if (scoreTimer.milliseconds() >= startTime + 100) {
                    pivotL.setPosition(0.87);
                    pivotR.setPosition(0.87);
                    claw.setPosition(CLAW_CLOSE);
                }
                if (scoreTimer.milliseconds() >= startTime + 500) {
                    wrist.setPosition(TWIST_UP_FRONT);
                    liftState = LiftState.Ready_To_Score;
                    scoreTimer.reset();
                    FirstUpdate = true;
                }
                FirstUpdate = false;
                break;






               /* if (gamepad1.a || gamepad1.b || gamepad1.y) {
                    liftState = LiftState.GRIP_UP;
                    wrist.setPosition(TWIST_UP_BACK);
                    flip.setPosition(FLIP_UP_BACK);
                    FirstUpdate = true;
                }
                while (liftState == LiftState.BACK) {
                    if (gamepad1.a || gamepad1.b || gamepad1.y) {
                        liftState = LiftState.GRIP_UP;
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        FirstUpdate = true;
                    }
                }
                //Safety button that resets everything
                if(gamepad1.x){
                    liftState = LiftState.RESET;
                    FirstUpdate = true;
                }
                FirstUpdate=false;
                break;

            case GRIP_UP:

                if(gamepad1.a){
                    liftTarget = LIFT_LEVEL_ONE;
                }
                if(gamepad1.b){
                    liftTarget = LIFT_LEVEL_TWO;
                }
                if(gamepad1.y){
                    liftTarget = LIFT_LEVEL_THREE;
                }

                while (liftState == LiftState.FRONT) {
                        liftState = LiftState.GRIP_UP;
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        FirstUpdate = true;
                }
                //Sets arm up to an angle to use the aligner
                pivotL.setPosition(0.25);
                pivotR.setPosition(0.75);

                //Opens Claw
                if(gamepad1.right_bumper){
                    scoreTimer.reset();
                    liftState = LiftState.OPEN_CLAW;
                    FirstUpdate = true;
                }


                FirstUpdate = false;
                break;

            case ORIGINAL:
                break;
            case FRONT:
                claw.setPosition(CLAW_CLOSE);
                pivotL.setPosition(1);
                pivotR.setPosition(0);
                wrist.setPosition(WRIST_DOWN);
                if(gamepad1.left_bumper){
                    scoreTimer.reset();
                    liftState = LiftState.GRIP_UP;
                    FirstUpdate = true;
                }
                if(gamepad1.dpad_down){
                    claw.setPosition(CLAW_OPEN);
                    liftState = LiftState.FRONT;
                    FirstUpdate = true;
                }
                if(gamepad1.right_bumper){
                    scoreTimer.reset();
                    liftTarget = LIFT_LEVEL_ORIGINAL;
                    liftState = LiftState.RESET;
                    FirstUpdate = true;
                }
                if(gamepad1.a){
                    liftTarget = LIFT_LEVEL_ONE + 400;
                }
                if(gamepad1.b){
                    liftTarget = LIFT_LEVEL_TWO + 400;
                }
                if(gamepad1.y){
                    liftTarget = LIFT_LEVEL_THREE + 400;
                }

                FirstUpdate = false;
                break;

            case OPEN_CLAW:
                liftTarget = LIFT.getCurrentPosition() - 50;
                wrist.setPosition(WRIST_UP);
                pivotL.setPosition(PIVOTL_UP);
                pivotR.setPosition(PIVOTR_UP);
                if (scoreTimer.milliseconds() >= startTime + 100) {
                    claw.setPosition(CLAW_OPEN);
                }
                if (scoreTimer.milliseconds() >= startTime + 500) {
                    scoreTimer.reset();
                    liftState = LiftState.RESET;
                    liftTarget = LIFT_LEVEL_ORIGINAL;
                    FirstUpdate = true;
                }

                FirstUpdate = false;
                break;

            case BACK:*/

            default:
                liftState = LiftState.Intake_Normal;

        }

        //SAFETY BUTTON TO RESET THE SYSTEM BACK TO THE START

        //CODE FOR DRIVE TRAIN
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x / 1.2;

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
            frontLeft.setPower(frontLeftPower/1.2);
            backLeft.setPower(backLeftPower/1.2);
            frontRight.setPower(frontRightPower/1.2);
            backRight.setPower(backRightPower/1.2);
        }

        telemetry.addData("State",liftState);
        telemetry.addData("Lift Position, Motor 1", LIFT.getCurrentPosition());
        telemetry.addData("Lift Position, Motor 2", LIFT2.getCurrentPosition());
        telemetry.addLine();
        telemetry.update();
        getRuntime();

    }

}

