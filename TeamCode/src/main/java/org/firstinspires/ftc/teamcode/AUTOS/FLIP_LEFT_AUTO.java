package org.firstinspires.ftc.teamcode.AUTOS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@Autonomous(name = "Left SIde Auto")
public class FLIP_LEFT_AUTO extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 145.1;        // TICKS PER REVOLUTION FOR 5203 1620RPM MOTOR
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;  // For figuring circumference
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 1;                   // MOTOR SPEED
    static final double FIRST_CONE_INCHES = 4.7;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_CONE_INCHES = 3.5;      // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_CONE_INCHES = 2.5;      // HEIGHT FOR THIRD LEVEL IN INCHES
    static final double FOURTH_CONE_INCHES = 1.5;      // HEIGHT FOR THIRD LEVEL IN INCHES
    static final double FIFTH_CONE_INCHES = 0;      // HEIGHT FOR THIRD LEVEL IN INCHES
    static final double UP_INCHES = 24.5;
    static final double DOWN_INCHES = 14;// HEIGHT FOR THIRD LEVEL IN INCHES

    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES
    final int LIFT_LEVEL_ORIGINAL = 0;
    final int CONE_ONE = (int) (FIRST_CONE_INCHES * COUNTS_PER_INCH);
    final int CONE_TWO = (int) (SECOND_CONE_INCHES * COUNTS_PER_INCH);
    final int CONE_THREE = (int) (THIRD_CONE_INCHES * COUNTS_PER_INCH);
    final int CONE_FOUR = (int) (FOURTH_CONE_INCHES * COUNTS_PER_INCH);
    final int CONE_FIVE = (int) (FIFTH_CONE_INCHES * COUNTS_PER_INCH);
    final int DOWN = (int) (DOWN_INCHES * COUNTS_PER_INCH);
    final int UP = (int) (UP_INCHES * COUNTS_PER_INCH);

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

    final double CLAW_OPEN = 0.42;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 0.27;    // SERVO POSITION TO CLOSE CLAW
    final double PIVOT_DOWN_FRONT = 1;
    final double PIVOT_DOWN_BACK = 0.08;
    final double PIVOT_UP_FRONT = 0.47;
    final double PIVOT_UP_BACK = 0.65;
    final double TWIST_UP_BACK = 0.3;
    final double TWIST_UP_FRONT = 0.3;
    final double TWIST_DOWN_FRONT = 0.98;
    final double TWIST_DOWN_BACK = 0;
    final double FLIP_UP_FRONT = 0.25;
    final double FLIP_UP_BACK = 0.29;
    final double FLIP_DOWN_FRONT = 0.35;
    final double FLIP_DOWN_BACK = 0;
    final double PIVOT_MID = 0.35;

    int liftTarget = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
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
        flip.setPosition(0.13);
        claw.setPosition(CLAW_CLOSE);
        wrist.setPosition(0.3);
        pivotL.setPosition(0.26);
        pivotR.setPosition(0.26);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        while (!isStarted() && !isStopRequested()) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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

        }
        TrajectorySequence trajSeq = null;
        if (tagOfInterest.id == left) {
            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    //First Cone
                    //
                    //
                    .lineToSplineHeading(new Pose2d(-35, -18, Math.toRadians(270)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                        pivotL.setPosition(PIVOT_UP_FRONT);
                        pivotR.setPosition(PIVOT_UP_FRONT);
                        claw.setPosition(CLAW_CLOSE);
                        flip.setPosition(FLIP_UP_FRONT);
                        wrist.setPosition(TWIST_UP_FRONT);
                        LIFT.setTargetPosition(UP);
                        LIFT2.setTargetPosition(UP);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(
                            new Vector2d(-27, -6), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        pivotL.setPosition(PIVOT_DOWN_FRONT);
                        pivotR.setPosition(PIVOT_DOWN_FRONT);
                        wrist.setPosition(TWIST_DOWN_FRONT);
                        flip.setPosition(FLIP_DOWN_FRONT);
                        LIFT.setTargetPosition(CONE_ONE);
                        LIFT2.setTargetPosition(CONE_ONE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -13, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                    })
                    .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_FRONT);
                        pivotR.setPosition(PIVOT_UP_FRONT);
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        LIFT.setTargetPosition(UP);
                        LIFT2.setTargetPosition(UP);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(
                            new Vector2d(-28.5, -5), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        pivotL.setPosition(PIVOT_DOWN_FRONT);
                        pivotR.setPosition(PIVOT_DOWN_FRONT);
                        wrist.setPosition(TWIST_DOWN_FRONT);
                        flip.setPosition(FLIP_DOWN_FRONT);
                        LIFT.setTargetPosition(CONE_TWO);
                        LIFT2.setTargetPosition(CONE_TWO);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -13, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                    })
                    .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_FRONT);
                        pivotR.setPosition(PIVOT_UP_FRONT);
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        LIFT.setTargetPosition(UP);
                        LIFT2.setTargetPosition(UP);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(
                            new Vector2d(-28.5, -5), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        pivotL.setPosition(PIVOT_DOWN_FRONT);
                        pivotR.setPosition(PIVOT_DOWN_FRONT);
                        wrist.setPosition(TWIST_DOWN_FRONT);
                        flip.setPosition(FLIP_DOWN_FRONT);
                        LIFT.setTargetPosition(CONE_THREE);
                        LIFT2.setTargetPosition(CONE_THREE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -13, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                    })
                    .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_FRONT);
                        pivotR.setPosition(PIVOT_UP_FRONT);
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        LIFT.setTargetPosition(UP);
                        LIFT2.setTargetPosition(UP);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(
                            new Vector2d(-28.5, -5), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        pivotL.setPosition(PIVOT_DOWN_FRONT);
                        pivotR.setPosition(PIVOT_DOWN_FRONT);
                        wrist.setPosition(TWIST_DOWN_FRONT);
                        flip.setPosition(FLIP_DOWN_FRONT);
                        LIFT.setTargetPosition(CONE_FOUR);
                        LIFT2.setTargetPosition(CONE_FOUR);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -13, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                    })
                    .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_FRONT);
                        pivotR.setPosition(PIVOT_UP_FRONT);
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        LIFT.setTargetPosition(UP);
                        LIFT2.setTargetPosition(UP);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(
                            new Vector2d(-28.5, -5), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        pivotL.setPosition(PIVOT_DOWN_FRONT);
                        pivotR.setPosition(PIVOT_DOWN_FRONT);
                        wrist.setPosition(TWIST_DOWN_FRONT);
                        flip.setPosition(FLIP_DOWN_FRONT);
                        LIFT.setTargetPosition(CONE_FIVE);
                        LIFT2.setTargetPosition(CONE_FIVE);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                    .lineToLinearHeading(
                            new Pose2d(-59.5, -13, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )

                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                    })
                    .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                        pivotL.setPosition(PIVOT_UP_FRONT);
                        pivotR.setPosition(PIVOT_UP_FRONT);
                        wrist.setPosition(TWIST_UP_FRONT);
                        flip.setPosition(FLIP_UP_FRONT);
                        LIFT.setTargetPosition(UP);
                        LIFT2.setTargetPosition(UP);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(
                            new Vector2d(-28.5, -5), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        pivotL.setPosition(PIVOT_MID);
                        pivotR.setPosition(PIVOT_MID);
                        LIFT.setTargetPosition(DOWN);
                        LIFT2.setTargetPosition(DOWN);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        claw.setPosition(CLAW_OPEN);
                    })
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        pivotL.setPosition(PIVOT_DOWN_FRONT);
                        pivotR.setPosition(PIVOT_DOWN_FRONT);
                        wrist.setPosition(TWIST_DOWN_FRONT);
                        flip.setPosition(0.1);
                        LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                        LIFT.setPower(ARM_SPEED);
                        LIFT2.setPower(ARM_SPEED);
                        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .lineToSplineHeading(new Pose2d(-45, -13, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-59, -13), Math.toRadians(180))
                    .waitSeconds(10)
                    .build();
        }

         TrajectorySequence trajSeq2 = null;
            if(tagOfInterest.id ==middle) {
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                //First Cone
                //
                //
                .lineToSplineHeading(new Pose2d(-35, -18, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    claw.setPosition(CLAW_CLOSE);
                    flip.setPosition(FLIP_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-27, -6), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_ONE);
                    LIFT2.setTargetPosition(CONE_ONE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_TWO);
                    LIFT2.setTargetPosition(CONE_TWO);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_THREE);
                    LIFT2.setTargetPosition(CONE_THREE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_FOUR);
                    LIFT2.setTargetPosition(CONE_FOUR);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_FIVE);
                    LIFT2.setTargetPosition(CONE_FIVE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(0.1);
                    LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                    LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToSplineHeading(new Pose2d(-34, -25, Math.toRadians(270)))
                .waitSeconds(10)
                .build();

    }

         TrajectorySequence trajSeq3 = null;
        if(tagOfInterest.id ==right){
        trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                //First Cone
                //
                //
                .lineToSplineHeading(new Pose2d(-35, -18, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    claw.setPosition(CLAW_CLOSE);
                    flip.setPosition(FLIP_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-27, -6), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_ONE);
                    LIFT2.setTargetPosition(CONE_ONE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_TWO);
                    LIFT2.setTargetPosition(CONE_TWO);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_THREE);
                    LIFT2.setTargetPosition(CONE_THREE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_FOUR);
                    LIFT2.setTargetPosition(CONE_FOUR);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(FLIP_DOWN_FRONT);
                    LIFT.setTargetPosition(CONE_FIVE);
                    LIFT2.setTargetPosition(CONE_FIVE);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(-59.5, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                })
                .lineToLinearHeading(new Pose2d(-50, -13, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    pivotL.setPosition(PIVOT_UP_FRONT);
                    pivotR.setPosition(PIVOT_UP_FRONT);
                    wrist.setPosition(TWIST_UP_FRONT);
                    flip.setPosition(FLIP_UP_FRONT);
                    LIFT.setTargetPosition(UP);
                    LIFT2.setTargetPosition(UP);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(
                        new Vector2d(-28.5, -5), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    pivotL.setPosition(PIVOT_MID);
                    pivotR.setPosition(PIVOT_MID);
                    LIFT.setTargetPosition(DOWN);
                    LIFT2.setTargetPosition(DOWN);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    claw.setPosition(CLAW_OPEN);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    pivotL.setPosition(PIVOT_DOWN_FRONT);
                    pivotR.setPosition(PIVOT_DOWN_FRONT);
                    wrist.setPosition(TWIST_DOWN_FRONT);
                    flip.setPosition(0.1);
                    LIFT.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                    LIFT2.setTargetPosition(LIFT_LEVEL_ORIGINAL);
                    LIFT.setPower(ARM_SPEED);
                    LIFT2.setPower(ARM_SPEED);
                    LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LIFT2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToSplineHeading(new Pose2d(-25, -15, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-9, -18, Math.toRadians(-90)))
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


