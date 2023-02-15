package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

@Config
@TeleOp

public class DoubleLinearSlideTesting extends OpMode {

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public PIDController liftController;


    public static double slideP = 0.0;
    public static double slideI = 0;
    public static double slideD = 0;
    public static double slideKg = 0.22;

    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.71 / 103.8;



    public static double targetPosition = 0;


    @Override
    public void init(){
        liftController = new PIDController(slideP, slideI, slideD);

        liftMotorOne = hardwareMap.get(DcMotorEx.class, "Lift");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "Lift2");

        liftMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);



    }

    @Override
    public void loop(){
        liftController.setPID(slideP, slideI, slideD);

        double liftPosition = liftMotorOne.getCurrentPosition() * SLIDE_TICKS_PER_INCH;
        double liftPosition2 = liftMotorTwo.getCurrentPosition() * SLIDE_TICKS_PER_INCH;

        double pid = liftController.calculate(liftPosition, targetPosition);

        double liftPower = pid + slideKg;

        liftMotorOne.setPower(liftPower);
        liftMotorTwo.setPower(liftPower);



        telemetry.addData("Lift Position, Motor 1", liftPosition);
        telemetry.addData("Lift Position, Motor 2", liftPosition2);
        telemetry.addLine();
        telemetry.addData("Lift Target", targetPosition);
        telemetry.addData("Lift Power", liftPower);
        telemetry.update();



    }

}