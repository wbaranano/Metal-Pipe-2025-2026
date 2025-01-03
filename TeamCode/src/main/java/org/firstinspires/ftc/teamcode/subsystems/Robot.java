package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Field;



@Config
public class Robot {
    public static IntakeSubsystem.COLOR enemyColor= IntakeSubsystem.COLOR.red;
    // ==============================================================
    // +                           CONFIG                           =
    // ==============================================================
    // note that these are made public just in case, but you should really go through .hardware to read/write
    // (also setting up a whole ass hierarchy would be very painful)

    // drivetrain
    public static DcMotorEx leftFront, leftRear, rightFront, rightRear;


    // lift
    public static DcMotorEx liftLeft, liftRight;

    public static DcMotorEx liftEncoder,intake;

    public static double intakeDistance=0.0;
    // intake

    public static CRServo flipper;

    public static DcMotorEx intakeMotor;
    public static float[] hsvIntake={0f,0f,0f};
    public static Servo droptakeRight, droptakeLeft,intakeWrist1,intakeWrist2;

    // arm
    public static Servo armYellow, armGreen,arm1,arm2,wrist1,wrist2,turn;

    // wrist
    public static Servo wristRed, wristBlue;

    // claw
    public static Servo clawBlack, clawWhite;
    public static Telemetry telemetry;
    // drone
    public static CRServo drone;

    // ==============================================================
    // +                       SHARED VALUES                        =
    // ==============================================================
    public static int level = 0; // this is automatically controlled by GoToHeight, so dont touch it!
    public static boolean forwardIsForward = true;
    public static boolean pastTruss = false;
    public static double distanceBetween = 96.0 * 2;
    public static double t = 0.0;
    public static boolean isBusy = false;
    public static DcMotorEx extendo;
    public static final double width = 412.7; // mm, drivetrain plate to other drivetrain plate
    public static final double length = 440.63; // mm, intake roller to back odo pod u channel
    public static boolean onlyLogImportant = false;
    public static boolean hasCachedLiftValues = false;
    public static double cachedLiftTick = 0;
    public static int cachedLiftRobotLevel = 0;
    public static double startingBatteryVoltage = 0;

    // ==============================================================
    // +                         SUBSYSTEMS                         =
    // ==============================================================

    public static HardwareMap hardwareMap;
    public static RevColorSensorV3 intakeColorSensor;

    public static LiftSubsystem liftSubsystem;

    public static boolean pidControl = false;

    public static Thread hardwareThread, mathThread, writeThread;

    public static void robotInit(HardwareMap hardwareMap) {
        intakeColorSensor=hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        telemetry=new MultipleTelemetry();
        //liftSubsystem = _lift;


        Robot.hardwareMap = hardwareMap;

        // drivetrain
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        extendo=hardwareMap.get(DcMotorEx.class,"extendo");
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake=hardwareMap.get(DcMotorEx.class,"intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipper=hardwareMap.crservo.get("flipper");
        intakeWrist1=hardwareMap.servo.get("iw1");
        intakeWrist2=hardwareMap.servo.get("iw2");
        //leftPod = new OdometryModule(hardwareMap.dcMotor.get("intake"));
        //rightPod = new OdometryModule(hardwareMap.dcMotor.get("rightPodWrapper"));
        //backPod = new OdometryModule(hardwareMap.dcMotor.get("leftRear"));



        //leftPod.reverse();
        //rightPod.reverse();
        //backPod.reverse();



        // lift
        liftEncoder = hardwareMap.get(DcMotorEx.class, "lf");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
//
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // intake
//        bottomRoller = hardwareMap.crservo.get("bottomRoller");
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        droptakeRight = hardwareMap.servo.get("droptakeRight");
//        droptakeLeft = hardwareMap.servo.get("droptakeLeft");
//
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // arm
//        armYellow = Robot.hardwareMap.servo.get("armYellow");
//        armGreen = Robot.hardwareMap.servo.get("armGreen");
//
//        // wrist
//        wristRed = Robot.hardwareMap.servo.get("wristRed");
//        wristBlue = Robot.hardwareMap.servo.get("wristBlue");
//
//        // claw
//        clawWhite = hardwareMap.servo.get("clawWhite");
//        clawBlack = hardwareMap.servo.get("clawBlack");
//
//        // drone
//        drone = hardwareMap.crservo.get("drone");

        // threads


        // reset static variables where needed
        forwardIsForward = true;
        onlyLogImportant = false;
        isBusy = false;


        // replace default distance sensors with custom



        // distance sensors

    }
    public static void sensorUpdate(){
        try {
            Color.RGBToHSV((int) (intakeColorSensor.red() * 255),
                (int) (intakeColorSensor.green() * 255),
                (int) (intakeColorSensor.blue() * 255),
                hsvIntake);
            intakeDistance=intakeColorSensor.getDistance(DistanceUnit.CM);

        }
        catch (Exception e){
            //Noghi
        }
    }
}
