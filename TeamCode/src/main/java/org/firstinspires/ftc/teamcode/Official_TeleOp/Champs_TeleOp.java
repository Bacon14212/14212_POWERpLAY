package org.firstinspires.ftc.teamcode.Official_TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(group = " Main TeleOP")
public class Champs_TeleOp extends OpMode {

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

    LiftState liftState = LiftState.RESET;

    //DECLARE MOTORS FOR DRIVETRAIN
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //DECLARE MOTORS AND SERVOS FOR SLIDE
    public DcMotorEx LIFT;// MOTOR FOR THE SLIDE
    public DcMotorEx LIFT2;
    public Servo claw;// SERVO FOR THE CLAW
    public Servo pivotL;
    public Servo pivotR;
    public Servo wrist;

    final double CLAW_OPEN = 0.3;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 0.08;    // SERVO POSITION TO CLOSE CLAW
    final double PIVOTR_UP = 0.85;
    final double PIVOTL_UP = 0.15;
    final double WRIST_UP = 0.59;
    final double WRIST_DOWN = 0.03;

    int liftTarget = 0;

    ElapsedTime dropTime = new ElapsedTime();
    RevBlinkinLedDriver lights;

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
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "Lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

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
    boolean FirstUpdate = true;
    double startTime = 0;
    ElapsedTime scoreTimer = new ElapsedTime();

    public void loop() {
        if (liftState == LiftState.RESET){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        if(liftState == liftState.GRIP_DOWN){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        if(liftState == liftState.GRIP_UP){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }
        if(liftState == liftState.OPEN_CLAW){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        //Constant Loop for LIFT
        LIFT.setTargetPosition(liftTarget);
        LIFT2.setTargetPosition(liftTarget);
        LIFT.setPower(ARM_SPEED);
        LIFT2.setPower(ARM_SPEED);
        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        switch (liftState) {
            //Drops Lift, claw open, pivot down
            case RESET:
                pivotL.setPosition(1);
                pivotR.setPosition(0);
                claw.setPosition(CLAW_OPEN);
                if (scoreTimer.milliseconds() >= startTime + 100) {
                    wrist.setPosition(WRIST_DOWN);
                }

               //Sets Lift motors to 0 power
                if (Math.abs(LIFT.getCurrentPosition() - LIFT_LEVEL_ORIGINAL) < 5) {
                    LIFT.setPower(0);
                    LIFT2.setPower(0);
                }


                //Grab a cone
               if(gamepad1.left_bumper){
                   scoreTimer.reset();
                   liftState = LiftState.GRIP_DOWN;
                   FirstUpdate=true;
               }
                if(gamepad1.dpad_up){
                    claw.setPosition(CLAW_CLOSE);
                    liftState = LiftState.FRONT;
                    FirstUpdate = true;
                }
                FirstUpdate=false;
                break;
            case GRIP_DOWN:
                if(gamepad1.a){
                    liftTarget = LIFT_LEVEL_ONE;
                }
                if(gamepad1.b){
                    liftTarget = LIFT_LEVEL_TWO;
                }
                if(gamepad1.y){
                    liftTarget = LIFT_LEVEL_THREE;
                }
                //Close Claw
                claw.setPosition(CLAW_CLOSE);
                wrist.setPosition(WRIST_DOWN);
                //Sets arm up after 0.5 seconds
                if (scoreTimer.milliseconds() >= startTime + 500) {
                    pivotL.setPosition(0.5);
                    pivotR.setPosition(0.5);
                }

                if(gamepad1.a || gamepad1.b || gamepad1.y){
                    liftState = LiftState.GRIP_UP;
                    wrist.setPosition(WRIST_UP);
                    FirstUpdate=true;
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
                //Sets arm up to an angle to use the aligner
                wrist.setPosition(WRIST_UP);
                    pivotL.setPosition(0.25);
                    pivotR.setPosition(0.75);

                //Opens Claw
                if(gamepad1.right_bumper){
                    scoreTimer.reset();
                    liftState = LiftState.OPEN_CLAW;
                    FirstUpdate = true;
                }

                if(gamepad1.dpad_up){
                    liftState = LiftState.FRONT;
                    liftTarget = LIFT.getCurrentPosition() + 400;
                    FirstUpdate = true;
                }
                if(gamepad1.dpad_down){
                    claw.setPosition(CLAW_OPEN);
                    liftState = LiftState.GRIP_UP;
                    FirstUpdate = true;
                }
                if(gamepad1.dpad_left){
                    wrist.setPosition(WRIST_DOWN);
                    liftState = LiftState.GRIP_UP;
                    FirstUpdate = true;
                }
                if(gamepad1.dpad_right){
                    wrist.setPosition(WRIST_UP);
                    liftState = LiftState.GRIP_UP;
                    FirstUpdate = true;
                }

                FirstUpdate = false;
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

            default:
                liftState = LiftState.RESET;

        }

        //SAFETY BUTTON TO RESET THE SYSTEM BACK TO THE START

        //CODE FOR DRIVE TRAIN
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x / 1.3;

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
            frontLeft.setPower(frontLeftPower/1.5);
            backLeft.setPower(backLeftPower/1.5);
            frontRight.setPower(frontRightPower/1.5);
            backRight.setPower(backRightPower/1.5);
        }



        telemetry.addData("State",liftState);
        telemetry.addData("Lift Position, Motor 1", LIFT.getCurrentPosition());
        telemetry.addData("Lift Position, Motor 2", LIFT2.getCurrentPosition());
        telemetry.addLine();
        telemetry.update();
        getRuntime();

    }

}
