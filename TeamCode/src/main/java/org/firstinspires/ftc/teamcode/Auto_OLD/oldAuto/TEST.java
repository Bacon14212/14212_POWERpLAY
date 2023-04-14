package org.firstinspires.ftc.teamcode.Auto_OLD.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto_OLD.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name ="LEFT Test")
public class TEST extends LinearOpMode {

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    private DcMotor LIFT;
    private Servo claw;

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV_LIFT = 103.8;// Gobilda 19.2:1 ratio motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double PULLEY_HUB_DIAMETER_INCHES  = 1.4;
    static final double     COUNTS_PER_INCH_PULLEY  = (COUNTS_PER_MOTOR_REV_LIFT * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double FIRST_LEVEL_INCHES  = 5;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 4;        // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_LEVEL_INCHES  = 34;

    final int LIFT_LEVEL_ORIGINAL = -1;
    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH_PULLEY);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH_PULLEY);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH_PULLEY);


    String webcamName = "Webcam 1";

    private int fLPos;
    private int fRPos;
    private int bLPos;
    private int bRPos;

    final double INCH = 45;
    final double WHEEL_DIAMETER = 4; //Inches
    final double PULES_PER_ROTATION = 537.7; //Gobilda 312 rpm motor

    @Override
    public void runOpMode() {
        fL = hardwareMap.get(DcMotor.class, "leftFront");
        fR = hardwareMap.get(DcMotor.class, "rightFront");
        bL = hardwareMap.get(DcMotor.class, "leftRear");
        bR = hardwareMap.get(DcMotor.class, "rightRear");
        LIFT = hardwareMap.get(DcMotor.class, "Lift");
        claw = hardwareMap.servo.get("claw");

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LIFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(432, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();

        }

        waitForStart();
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.CENTER)) {
            claw.setPosition(1);
            drive(1500, 1500, 1500, 1500, 0.8);
            sleep(500);

        }
        waitForStart();
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.LEFT)) {
            claw.setPosition(1);
            drive(2160, 2160, 2160, 2160, 0.8);
            sleep(500);
            drive(445,-445,445,-445,0.8);
            sleep(500);
            LIFT.setTargetPosition(LIFT_LEVEL_THREE);
            LIFT.setPower(1);
            LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            drive(550, 550, 550, 550, 0.8);
            sleep(500);
            claw.setPosition(0.6);
            sleep(500);
            drive(-345, -345, -345, -345, 0.8);
            sleep(500);
            drive(-1345,1345,-1345,1345,0.8);
            sleep(500);
            LIFT.setTargetPosition(LIFT_LEVEL_ONE);
            LIFT.setPower(1);
            LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive(1230, 1230, 1230, 1230, 0.8);
            sleep(500);
            claw.setPosition(1);
            sleep(500);
            LIFT.setTargetPosition(LIFT_LEVEL_THREE);
            LIFT.setPower(1);
            LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            drive(-2045, -2045, -2045, -2045, 0.7);
            sleep(500);
            drive(450,-450,450,-450,0.8);
            sleep(500);
            drive(380,380,380,380,0.8);
            sleep(500);
            claw.setPosition(0.6);
            sleep(500);
            drive(-500,-500,-500,-500,0.8);
            sleep(500);
            drive(-500,500,-500,500,0.8);
            sleep(500);
            drive(2160, 2160, 2160, 2160, 0.8);
            LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
            LIFT.setPower(1);
            LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            LIFT.setPower(0);
        }

        telemetry.addData("fL: ", fL.getCurrentPosition());
        telemetry.addData("fR: ", fR.getCurrentPosition());
        telemetry.addData("bL: ", bL.getCurrentPosition());
        telemetry.addData("bR: ", bR.getCurrentPosition());
            telemetry.update();

            sleep(1000);

    }
    private void drive(int fLTarget, int fRTarget, int bLTarget, int bRTarget, double speed) {

        //45 ticks per inch
        fLPos += fLTarget;
        fRPos += fRTarget;
        bLPos += bLTarget;
        bRPos += bRTarget;

        fL.setTargetPosition(fLPos);
        fR.setTargetPosition(fRPos);
        bL.setTargetPosition(bLPos);
        bR.setTargetPosition(bRPos);

        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fL.setPower(1);
        fR.setPower(1);
        bL.setPower(1);
        bR.setPower(1);

        while (opModeIsActive() &&
                (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d", fLTarget,  fRTarget, bLTarget, bRTarget);
            telemetry.addData("Currently at",  " at %7d :%7d",
                    fL.getCurrentPosition(), fR.getCurrentPosition(),bL.getCurrentPosition(), bR.getCurrentPosition());
            telemetry.update();
        }

        sleep(500);   // optional pause after each move.
        }
    }
