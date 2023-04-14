package org.firstinspires.ftc.teamcode.AUTOS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(group = "LEFT", name = "ONE + Six")
public class One_And_Six extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 145.1;        // TICKS PER REVOLUTION FOR 5203 435RPM MOTOR
    static final double DRIVE_GEAR_REDUCTION = 1.0;          // This is < 1.0 if geared UP
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 0.9;
    static final double FIRST_LEVEL_INCHES = 5;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 4;// HEIGHT FOR SECOND LEVEL IN INCHES
    static final double FOURTH_LEVEL_INCHES = 3;
    static final double FIFTH_LEVEL_INCHES = 2;
    static final double THIRD_LEVEL_INCHES = 21.5;// HEIGHT FOR THIRD LEVEL IN INCHES
    static final double DOWN_LEVEL_INCHES = 15;


    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;

    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_FOUR = (int) (FOURTH_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_FIVE = (int) (FIFTH_LEVEL_INCHES * COUNTS_PER_INCH);
    final int DOWN = (int) (DOWN_LEVEL_INCHES * COUNTS_PER_INCH);


    RevBlinkinLedDriver lights;
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

    final double CLAW_OPEN = 0.3;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 0.08;    // SERVO POSITION TO CLOSE CLAW
    final double PIVOT_UP = 0.75;
    final double PIVOT_UP_L = 0.25;
    final double WRIST_UP = 0.59;
    final double WRIST_DOWN = 0.03;




    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 552.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //tag ID's of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;


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

        lights = hardwareMap.get(RevBlinkinLedDriver.class, " Lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);


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
        pivotL.setPosition(0.61);
        pivotR.setPosition(0.39);

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

        Pose2d startPose = new Pose2d(-35, -65.2, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        TrajectorySequence trajSeq = null;
        if (tagOfInterest == null || tagOfInterest.id == right) {
            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    //First Cone
                    //
                    //
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        claw.setPosition(CLAW_CLOSE);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -9), Math.toRadians(45),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    //Grab
                    //
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(-30, -7), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .waitSeconds(0.3) .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .waitSeconds(0.3) .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                             SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .lineToSplineHeading(new Pose2d(-11, -12, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(61.5, -12, Math.toRadians(0)))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(50, -13, Math.toRadians(0)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(32, -7.5), Math.toRadians(135))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(0.8);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(60, -11.5, Math.toRadians(0)))
                    .waitSeconds(10)
                    .build();
        }

        TrajectorySequence trajSeq2 = null;
        if (tagOfInterest.id == middle) {
            trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                    //First Cone
                    //
                    //
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        claw.setPosition(CLAW_CLOSE);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -9), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    //Grab
                    //
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(-30, -7), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .waitSeconds(0.3) .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .waitSeconds(0.3) .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .lineToSplineHeading(new Pose2d(-11, -12, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(61.5, -12, Math.toRadians(0)))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(50, -13, Math.toRadians(0)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(32, -7.5), Math.toRadians(135))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(0.8);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .lineToSplineHeading(new Pose2d(35, -21, Math.toRadians(270)))
                    .waitSeconds(10)
                    .build();

        }

        TrajectorySequence trajSeq3 = null;
         if (tagOfInterest.id == left) {
            trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                    //First Cone
                    //
                    //
                    .lineToSplineHeading(new Pose2d(-35, -21, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        claw.setPosition(CLAW_CLOSE);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -9), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    //Grab
                    //
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(-30, -7), Math.toRadians(45))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .waitSeconds(0.3) .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .waitSeconds(0.3) .splineTo(new Vector2d(-45, -12), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -12, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(
                            new Vector2d(-30, -7), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.15);
                        pivotR.setPosition(0.85);;
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .lineToSplineHeading(new Pose2d(-11, -12, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(61.5, -12, Math.toRadians(0)))

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out

                    .waitSeconds(0.2)

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.5);
                        pivotR.setPosition(0.5);

                    })
                    .lineToLinearHeading(new Pose2d(50, -13, Math.toRadians(0)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_L);
                        pivotR.setPosition(PIVOT_UP);
                        LIFT.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT2.setTargetPosition(LIFT_LEVEL_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                        wrist.setPosition(WRIST_UP);
                    })
                    .splineTo(new Vector2d(32, -7.5), Math.toRadians(135))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(0.2);
                        pivotR.setPosition(0.8);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
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
                    .lineToSplineHeading(new Pose2d(10, -15, Math.toRadians(270)))
                    .waitSeconds(10)
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


    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}




