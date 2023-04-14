package org.firstinspires.ftc.teamcode.Auto_OLD.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto_OLD.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous(name = "Auto")
public class VisionTest extends LinearOpMode {

    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor Lift = null;
    public Servo   claw;               // SERVO FOR THE CLAW

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    private int fLPos;
    private int fRPos;
    private int bLPos;
    private int bRPos;

    final double WHEEL_DIAMETER = 4; //Inches
    final double PULSES_PER_ROTATION = 384.5; //Gobilda 435 rpm motor
    final double CLAW_OPEN =  0.65;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 1;       // SERVO POSITION TO CLOSE CLAW


    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    public void runOpMode() throws InterruptedException {
        frontLeft  = hardwareMap.dcMotor.get("leftFront");
        backLeft   = hardwareMap.dcMotor.get("leftRear");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backRight  = hardwareMap.dcMotor.get("rightRear");
        Lift = hardwareMap.dcMotor.get("Lift");
        claw = hardwareMap.servo.get("claw");


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
        telemetry.addData("Path", "Start");
        telemetry.addData("fL: ", frontLeft.getCurrentPosition());
        telemetry.addData("fR: ", frontRight.getCurrentPosition());
        telemetry.addData("bL: ", backLeft.getCurrentPosition());
        telemetry.addData("bR: ", backRight.getCurrentPosition());
        telemetry.update();
        sleep(1000);
        claw.setPosition(CLAW_CLOSE);
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.RIGHT)) {
            //Strafe Right
            frontLeft.setPower(-0.5);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(1000);
            frontLeft.setPower(-0.5);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(0.5);
            sleep(1000);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            Lift.setPower(0);
        }

        else if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.CENTER)) {
            frontLeft.setPower(-.5);
            backLeft.setPower(.5);
            frontRight.setPower(.5);
            backRight.setPower(.5);
            sleep(1000);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
        //Left
        else {
            frontLeft.setPower(0.5);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(-0.5);
            sleep(1000);
            frontLeft.setPower(-0.5);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(0.5);
            sleep(1000);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }

        telemetry.addData("Path", "Complete");
        telemetry.addData("fL: ", frontLeft.getCurrentPosition());
        telemetry.addData("fR: ", frontRight.getCurrentPosition());
        telemetry.addData("bL: ", backLeft.getCurrentPosition());
        telemetry.addData("bR: ", backRight.getCurrentPosition());
        telemetry.update();


        sleep(1000);


    }
}