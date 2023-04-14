package org.firstinspires.ftc.teamcode.Auto_OLD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(group = "2 ODO AUTO")
public class FLIPAUTO extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 103.8;        // TICKS PER REVOLUTION FOR 5203 435RPM MOTOR
    static final double DRIVE_GEAR_REDUCTION = 1.0;          // This is < 1.0 if geared UP
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 0.7;
    static final double FIRST_LEVEL_INCHES = 6;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 5;// HEIGHT FOR SECOND LEVEL IN INCHES
    static final double FOURTH_LEVEL_INCHES = 4;
    static final double FIFTH_LEVEL_INCHES = 3;
    static final double THIRD_LEVEL_INCHES = 26;// HEIGHT FOR THIRD LEVEL IN INCHES
    static final double DOWN_LEVEL_INCHES = 20;


    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;

    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_FOUR = (int) (FOURTH_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_FIVE = (int) (FIFTH_LEVEL_INCHES * COUNTS_PER_INCH);
    final int DOWN = (int) (DOWN_LEVEL_INCHES * COUNTS_PER_INCH);


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

    final double CLAW_OPEN = 0.55;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 0.8;    // SERVO POSITION TO CLOSE CLAW
    final double PIVOT_DOWN = 0;
    final double PIVOT_UP = 0.8;
    final double WRIST_UP = 0.3;
    final double WRIST_DOWN = 1;


    SleeveDetection sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";


    @Override
    public void runOpMode() throws InterruptedException {
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
        claw.setPosition(CLAW_CLOSE);
        wrist.setPosition(WRIST_DOWN);
        pivotL.setPosition(0.6);
        pivotR.setPosition(0.4);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //MAKES DRIVETRAIN MORE PRECISE (FORCES MOTORS TO BRAKE WHEN STOPPED)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        Pose2d startPose = new Pose2d(-35, -66, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = null;
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.RIGHT)) {
            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    //First Cone
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        claw.setPosition(CLAW_CLOSE);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(-32, -13.5), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ONE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ONE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Second Cone
                    .lineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60, -14), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -13), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_TWO);
                        LIFT2.setTargetPosition(LIFT_LEVEL_TWO);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Third Cone
                    .lineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -14), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -13), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_FOUR);
                        LIFT2.setTargetPosition(LIFT_LEVEL_FOUR);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Fourth Cone
                    .lineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -14), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -13), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_FIVE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_FIVE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Fifth Cone
                    .lineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -14), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -13), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Last Cone
                    .lineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -14), Math.toRadians(180))


                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -13), Math.toRadians(45))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    .lineToSplineHeading(new Pose2d(-10, -17, Math.toRadians(270)))
                    .waitSeconds(5)
                    .build();
        }

        TrajectorySequence trajSeq2 = null;
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.CENTER)) {
            trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                    //First Cone
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        claw.setPosition(CLAW_CLOSE);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(-32, -13.5), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ONE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ONE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Second Cone
                    .lineToSplineHeading(new Pose2d(-45, -15, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60, -15), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14.5), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_TWO);
                        LIFT2.setTargetPosition(LIFT_LEVEL_TWO);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Third Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -17, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_FOUR);
                        LIFT2.setTargetPosition(LIFT_LEVEL_FOUR);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Fourth Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_FIVE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_FIVE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Fifth Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Last Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))


                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .waitSeconds(5)
                    .build();

        }

        TrajectorySequence trajSeq3 = null;
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.LEFT)) {
            trajSeq3 = drive.trajectorySequenceBuilder(startPose)

                    //First Cone
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        claw.setPosition(CLAW_CLOSE);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(-32, -13.5), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ONE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ONE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Second Cone
                    .lineToSplineHeading(new Pose2d(-45, -15, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60, -15), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14.5), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_TWO);
                        LIFT2.setTargetPosition(LIFT_LEVEL_TWO);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Third Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -17, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_FOUR);
                        LIFT2.setTargetPosition(LIFT_LEVEL_FOUR);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Fourth Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_FIVE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_FIVE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Fifth Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    //Last Cone
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -16), Math.toRadians(180))


                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-40, -16, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-32, -14), Math.toRadians(45))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        claw.setPosition(CLAW_OPEN);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        pivotL.setPosition(1);
                        pivotR.setPosition(0);
                        wrist.setPosition(WRIST_DOWN);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .waitSeconds(0.3)
                    .lineToSplineHeading(new Pose2d(-45, -16, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -15), Math.toRadians(180))
                    .waitSeconds(5)
                    .build();
        }


            waitForStart();



            while (opModeIsActive() && !isStopRequested()) {
                drive.update();
                drive.followTrajectorySequence(trajSeq);
                drive.followTrajectorySequence(trajSeq2);
                drive.followTrajectorySequence(trajSeq3);
            }
        }
    }

