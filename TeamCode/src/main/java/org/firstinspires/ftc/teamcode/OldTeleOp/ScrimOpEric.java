package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class ScrimOpEric extends OpMode {

    //SLIDER ENCODER CALCULATIONS
    static final double COUNTS_PER_MOTOR_REV = 2786.2;        // TICKS PER REVOLUTION FOR 5203 435RPM MOTOR
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
        LIFT_START, LIFT_DROP_TIMER, LIFT_LEVEL, LIFT_EXTEND_LEVEL, LIFT_DROP, LIFT_RETRACT
    }
    LiftState liftState = LiftState.LIFT_START;

    // Declaring Motors
    public DcMotor Lift1       = null;
    public DcMotor Lift2       = null;
    public DcMotor frontLeft   = null;
    public DcMotor backLeft    = null;
    public DcMotor frontRight  = null;
    public DcMotor backRight   = null;
    public Servo   clawL       = null;
    public Servo   clawR       = null;
    public Servo   Intake      = null;


    double clawOffset = 0;
    double liftOffset = 0;

    public static final double MID_LIFT        =0.02;
    public static final double MID_SERVO       =  1 ;
    public static final double CLAW_SPEED      =  1 ;       // sets rate to move servo

    ElapsedTime dropTimer = new ElapsedTime();
    final double dropTime = 0.5;

    public void init(){
        //set variables up
        frontLeft  = hardwareMap.dcMotor.get("leftFront");
        backLeft   = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight  = hardwareMap.dcMotor.get("rightRear");
        Lift1  = hardwareMap.dcMotor.get("Lift1");
        Lift2 = hardwareMap.dcMotor.get("Lift2");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        Intake = hardwareMap.servo.get("Intake");

        //Most mechanum wheels will need to have wheels reversed
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        clawR.setDirection(Servo.Direction.REVERSE);

        //setting up break behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Intake.setPosition(0);
        clawL.setPosition(0);
        clawR.setPosition(0);
    }

    public void loop() {
        switch(liftState) {
            case LIFT_START:
                if(gamepad1.a) {
                    clawL.setPosition(0.5); //drop servo for intake
                    clawR.setPosition(0.5); //drop servo for intake
                    dropTimer.reset();
                    liftState = LiftState.LIFT_DROP_TIMER;
                }
                break;
            case LIFT_DROP_TIMER:
                if (dropTimer.seconds() >= dropTime) {
                    Intake.setPosition(1);
                    clawL.setPosition(0.25);
                    clawR.setPosition(0.25);
                    liftState = LiftState.LIFT_LEVEL;
                }
                break;
            case LIFT_LEVEL:
                if(gamepad1.b) {
                    Lift1.setTargetPosition(LIFT_LEVEL_ONE);
                    Lift2.setTargetPosition(LIFT_LEVEL_ONE);
                    Lift1.setPower(ARM_SPEED);
                    Lift2.setPower(ARM_SPEED);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT_EXTEND_LEVEL;
                }
                break;
            case LIFT_EXTEND_LEVEL:
                if ( (Math.abs(Lift1.getCurrentPosition() - LIFT_LEVEL_ONE) < 10) &&
                        (Math.abs(Lift2.getCurrentPosition() - LIFT_LEVEL_ONE) < 10) ) {
                    Lift1.setPower(0);
                    Lift2.setPower(0);
                    liftState = LiftState.LIFT_DROP;
                }
                break;
            case LIFT_DROP:
                if(gamepad1.x) {
                    Intake.setPosition(0);
                    clawL.setPosition(0);
                    clawR.setPosition(0);

                    Lift1.setTargetPosition(LIFT_LEVEL_ONE);
                    Lift2.setTargetPosition(LIFT_LEVEL_ONE);
                    Lift1.setPower(ARM_SPEED);
                    Lift2.setPower(ARM_SPEED);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if ( (Math.abs(Lift1.getCurrentPosition() - LIFT_LEVEL_ZERO) < 10) &&
                        (Math.abs(Lift2.getCurrentPosition() - LIFT_LEVEL_ZERO) < 10) ) {
                    Lift1.setPower(0);
                    Lift2.setPower(0);
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                liftState = LiftState.LIFT_START;
        }
        //safety button
        //here
    }
}
