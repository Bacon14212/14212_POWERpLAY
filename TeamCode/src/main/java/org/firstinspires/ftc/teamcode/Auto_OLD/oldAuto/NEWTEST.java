package org.firstinspires.ftc.teamcode.Auto_OLD.oldAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto_OLD.SleeveDetection;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(group = "NEWTEST")
public class NEWTEST extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 103.8;        // TICKS PER REVOLUTION FOR 5203 435RPM MOTOR
    static final double DRIVE_GEAR_REDUCTION = 1.0;          // This is < 1.0 if geared UP
    static final double PULLEY_HUB_DIAMETER_INCHES = 1.4;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double ARM_SPEED = 1;
    static final double FIRST_LEVEL_INCHES = 4.5;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 3;// HEIGHT FOR SECOND LEVEL IN INCHES
    static final double FOURTH_LEVEL_INCHES = 2;
    static final double FIFTH_LEVEL_INCHES = 1;
    static final double THIRD_LEVEL_INCHES = 34;        // HEIGHT FOR THIRD LEVEL IN INCHES


    //CALCULATED NUMBER OF TICKS USED TO MOVE THE SLIDE 'X' INCHES

    final int LIFT_LEVEL_ZERO = 0;
    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_FOUR = (int) (FOURTH_LEVEL_INCHES * COUNTS_PER_INCH);
    final int LIFT_LEVEL_FIVE = (int) (FIFTH_LEVEL_INCHES * COUNTS_PER_INCH);

    public DcMotor Lift;               // MOTOR FOR THE SLIDE
    public Servo claw;

    final double CLAW_OPEN = 0.6;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 1;    // SERVO POSITION TO CLOSE CLAW

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    public enum DriveState {
        START,
        LEFT,      //LEFT
        CENTER,   // Then, follow a lineTo() trajectory
        RIGHT,   // Then we want to do a point turn
        IDLE    // Our bot will enter the IDLE state when do
    }


    // We define the current state we're on
    // Default to IDLE
    DriveState driveState = DriveState.START;


    @Override
    public void runOpMode() throws InterruptedException {
        Lift = hardwareMap.dcMotor.get("Lift");

        claw = hardwareMap.servo.get("claw");

        //REVERSE MOTORS IF NECESSARY

        //SETS THE ENCODERS ON THE SLIDE TO DEFAULT VALUES
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //SETS THE CLAW IN DEFAULT POSITION
        claw.setPosition(CLAW_CLOSE);

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


        Pose2d startPose = new Pose2d(-35, -63.7, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();

        }

            TrajectorySequence trajSeq = null;
            if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.RIGHT)) {
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-35, -21), Math.toRadians(90))
                        .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                            claw.setPosition(CLAW_CLOSE);
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_ONE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_TWO);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_FOUR);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_FIVE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))

                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))
                        .waitSeconds(0.3)
                        .lineToSplineHeading(new Pose2d(-32, -8, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_ZERO);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(2)
                    .build();

        }
            TrajectorySequence trajSeq2 = null;
            if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.CENTER)) {
                trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-35, -21), Math.toRadians(90))
                        .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                            claw.setPosition(CLAW_CLOSE);
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_ONE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_TWO);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_FOUR);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_FIVE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))

                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))
                        .waitSeconds(0.3)
                        .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_ZERO);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                    .lineToLinearHeading(new Pose2d(-32, -24, Math.toRadians(90)))
                    .build();
        }

            TrajectorySequence trajSeq3 = null;
            if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.LEFT)) {
                trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-35, -21), Math.toRadians(90))
                        .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                            claw.setPosition(CLAW_CLOSE);
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_ONE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_TWO);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_FOUR);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))


                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))   // Push servo out

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-33, -14, Math.toRadians(45)))
                        .lineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_FIVE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }) // Lower servo
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_CLOSE))   // Push servo out
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_THREE);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .waitSeconds(0.4)
                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-29, -5, Math.toRadians(45)))

                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> claw.setPosition(CLAW_OPEN))
                        .waitSeconds(0.3)
                        .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(45)))
                        .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                            Lift.setTargetPosition(LIFT_LEVEL_ZERO);
                            Lift.setPower(ARM_SPEED);
                            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(0)))
                        .waitSeconds(2)
                        .build();

            }
            waitForStart();
            /*
            switch (driveState) {
                case START:
                    if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.LEFT)) {
                        drive.followTrajectorySequenceAsync(trajSeq3);
                        driveState = DriveState.CENTER;
                    }
                    break;
                case CENTER:
                    if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.CENTER)) {
                        drive.followTrajectorySequenceAsync(trajSeq2);
                        driveState = DriveState.RIGHT;
                    }
                    break;
                case RIGHT:
                    if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.RIGHT)) {
                        drive.followTrajectorySequenceAsync(trajSeq);
                        driveState = DriveState.START;
                    }
            }
*/
            while (opModeIsActive() && !isStopRequested()) {
                drive.update();
                drive.followTrajectorySequence(trajSeq);
                drive.followTrajectorySequence(trajSeq2);
                drive.followTrajectorySequence(trajSeq3);
            }
        }
    }



