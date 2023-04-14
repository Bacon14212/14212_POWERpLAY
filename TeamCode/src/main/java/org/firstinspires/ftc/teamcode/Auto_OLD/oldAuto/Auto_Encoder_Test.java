
package org.firstinspires.ftc.teamcode.Auto_OLD.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto_OLD.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class Auto_Encoder_Test extends LinearOpMode {
    private com.qualcomm.robotcore.hardware.DcMotor frontLeft = null;
    private com.qualcomm.robotcore.hardware.DcMotor frontRight = null;
    private com.qualcomm.robotcore.hardware.DcMotor backLeft = null;
    private com.qualcomm.robotcore.hardware.DcMotor backRight = null;
    private com.qualcomm.robotcore.hardware.DcMotor Lift = null;
    private com.qualcomm.robotcore.hardware.Servo Claw = null;
    private ElapsedTime runtime = new ElapsedTime();

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV_DT =537.7 ;
    static final double COUNTS_PER_MOTOR_REV_LIFT = 103.8;// Gobilda 19.2:1 ratio motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_RADIUS_INCHES         = 1.88976;     // For figuring circumference
    static final double PULLEY_HUB_DIAMETER_INCHES  = 1.4;
    static final double     COUNTS_PER_INCH_PULLEY  = (COUNTS_PER_MOTOR_REV_LIFT * DRIVE_GEAR_REDUCTION) /
            (PULLEY_HUB_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH         = 44;
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.6;
    static final double     STRAFE_SPEED            = 0.6;
    static final double FIRST_LEVEL_INCHES  = 5;         // HEIGHT FOR FIRST LEVEL IN INCHES
    static final double SECOND_LEVEL_INCHES = 4;        // HEIGHT FOR SECOND LEVEL IN INCHES
    static final double THIRD_LEVEL_INCHES  = 34;


    final int LIFT_LEVEL_ONE = (int) (FIRST_LEVEL_INCHES * COUNTS_PER_INCH_PULLEY);
    final int LIFT_LEVEL_TWO = (int) (SECOND_LEVEL_INCHES * COUNTS_PER_INCH_PULLEY);
    final int LIFT_LEVEL_THREE = (int) (THIRD_LEVEL_INCHES * COUNTS_PER_INCH_PULLEY);


    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        backLeft = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight = hardwareMap.dcMotor.get("rightRear");
        Lift = hardwareMap.dcMotor.get("Lift");
        Claw = hardwareMap.servo.get("claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                Lift.getCurrentPosition());
        telemetry.update();


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

        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.LEFT)) {
            Claw.setPosition(1);
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 10);  // S1: Forward 24 Inches with 2 Sec timeout
            encoderDrive(TURN_SPEED, 4.5, -4.5, 4.5, -4.5, 2);  // S2: Turn Right 3.5 Inches with 2 Sec timeout
            //Set Lift to Position
            Lift.setTargetPosition(LIFT_LEVEL_THREE);
            Lift.setPower(1);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            encoderDrive(DRIVE_SPEED, 3.5, 3.5, 3.5, 3.5, 1);  // S3: Forward 4 inches with 1 sec timeout
            Claw.setPosition(0.6);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -4.5, -4.5, -4.5, -4.5, 1); // S4: Reverse 4.5 inches with 1 sec timeout
            encoderDrive(TURN_SPEED, -13.5, 13.5, -13.5, 13.5, 2);
            Lift.setTargetPosition(LIFT_LEVEL_ONE);
            Lift.setPower(1);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            encoderDrive(DRIVE_SPEED, 13, 13, 13, 13, 2);
            Claw.setPosition(1);
            sleep(1000);
            Lift.setTargetPosition(LIFT_LEVEL_THREE);
            Lift.setPower(1);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            encoderDrive(DRIVE_SPEED, -14, -14, -14, -14, 2);
            encoderDrive(TURN_SPEED, 12.5, -12.5, 12.5, -12.5, 2);
            Lift.setPower(1);
            sleep(700);
            encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 1);
            Claw.setPosition(0.6);
            sleep(1000);
            encoderDrive(DRIVE_SPEED, -4.5, -4.5, -4.5, -4.5, 1);


        }


        //encoderDrive(DRIVE_SPEED, 12,12,12,12,1);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    //*  Method to perform a relative move, based on encoder counts.
     //*  Encoders are not reset as the move is based on the current position.
     //*  Move will stop if any of three conditions occur:
     //*  1) Move gets to the desired position
     //*  2) Move runs out of time
     //*  3) Driver stops the opmode running.

    public void encoderDrive(double speed,double leftFInches, double rightFInches, double leftBInches, double rightBInches,
                             double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controllleftFIncheser
            newLeftFTarget = frontLeft.getCurrentPosition() + (int)(leftFInches * COUNTS_PER_INCH);
            newRightFTarget = frontRight.getCurrentPosition() + (int)(rightFInches * COUNTS_PER_INCH);
            newLeftBTarget = frontLeft.getCurrentPosition() + (int)(leftBInches * COUNTS_PER_INCH);
            newRightBTarget = frontRight.getCurrentPosition() + (int)(rightBInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newLeftFTarget);
            frontRight.setTargetPosition(newRightFTarget);
            backLeft.setTargetPosition(newLeftBTarget);
            backRight.setTargetPosition(newRightBTarget);


            // Turn On RUN_TO_POSITION
            frontLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFTarget,  newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION

            frontLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }
}