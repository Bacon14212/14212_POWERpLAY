package org.firstinspires.ftc.teamcode.Auto_OLD.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto_OLD.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Left Auto")
public class League_Auto extends LinearOpMode {

    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor Lift = null;
    public Servo   claw;               // SERVO FOR THE CLAW
    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    final double CLAW_OPEN =  0.65;     // SERVO POSITION TO OPEN CLAW
    final double CLAW_CLOSE = 1;       // SERVO POSITION TO CLOSE CLAW


    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
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
        claw.setPosition(CLAW_CLOSE);
        if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.RIGHT)) {
            frontLeft.setPower(-0.44);
            backLeft.setPower(-0.44);
            frontRight.setPower(-0.44);
            backRight.setPower(0.44);
            sleep(1000);
            frontLeft.setPower(-0.5);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(0.5);
            sleep(1000);
            frontLeft.setPower(0.1);
            backLeft.setPower(-0.1);
            frontRight.setPower(0.1);
            backRight.setPower(0.1);
            sleep(1000);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            Lift.setPower(0);
        }

        else if (sleeveDetection.getPosition().equals(SleeveDetection.ParkingPosition.CENTER)) {
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
        //Left
        else {
            frontLeft.setPower(0.45);
            backLeft.setPower(0.45);
            frontRight.setPower(0.45);
            backRight.setPower(-0.45);
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
        telemetry.update();

        sleep(1000);


    }
}
