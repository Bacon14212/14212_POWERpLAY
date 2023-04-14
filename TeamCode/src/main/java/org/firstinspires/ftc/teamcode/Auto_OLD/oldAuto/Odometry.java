package org.firstinspires.ftc.teamcode.Auto_OLD.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name="Odometry Test")
public class Odometry extends LinearOpMode {
    private double x;
    private double y;
    private double heading;

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Constructor
    public Odometry() {
        x = 0;
        y = 0;
        heading = 0;
    }

    // This method is called when the op mode is first enabled
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftRear");
        rightBack = hardwareMap.dcMotor.get("rightRear");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update the odometry information
            update();

            // Get the current odometry information
            List<Double> odometry = getOdometry();

            // Print the odometry information to the log
            telemetry.addData("x", odometry.get(0));
            telemetry.addData("y", odometry.get(1));
            telemetry.addData("heading", odometry.get(2));
            telemetry.update();

            // Sleep for a short time to allow other processes to run
            Thread.sleep(100);
        }
    }

    // This method updates the odometry information based on the distance traveled by the left and right wheels
    public void update() {
        double leftDistance = leftFront.getCurrentPosition();
        double rightDistance = rightFront.getCurrentPosition();

        double deltaD = (leftDistance + rightDistance) / 2;
        double deltaTheta = (rightDistance - leftDistance) / 2;
        double dx = deltaD * Math.cos(heading + deltaTheta / 2);
        double dy = deltaD * Math.sin(heading + deltaTheta / 2);

        x += dx;
        y += dy;
        heading += deltaTheta;
    }

    // This method returns the current odometry information as a list
    public List<Double> getOdometry() {
        List<Double> odometry = new ArrayList<>();
        odometry.add(x);
        odometry.add(y);
        odometry.add(heading);
        return odometry;
    }
}