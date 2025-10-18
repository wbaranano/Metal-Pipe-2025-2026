package org.firstinspires.ftc.teamcode.tele_op;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Main")
public class drive extends OpMode {

    // Odometry
    GoBildaPinpointDriver odo;

    // Drive motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    // Intake motor
    private DcMotor ActiveIntake;

    // Color sensor
    private NormalizedColorSensor colorSensor;

    // Intake toggle
    private boolean intakeOn = true;
    private boolean lastAPressed = false;

    // Dashboard
    FtcDashboard dashboard;

    // Webcam
    OpenCvCamera webcam;

    // Field constants
    final double FIELD_MM = 3657.6;
    final double CANVAS_SIZE = 60;
    final double SCALE = FIELD_MM / CANVAS_SIZE;

    final double ROBOT_WIDTH_MM = 20;
    final double ROBOT_LENGTH_MM = 20;

    @Override
    public void init() {
        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Initialize motors
        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");

        // Initialize intake
        ActiveIntake = hardwareMap.get(DcMotor.class, "ActiveIntake");

        // Initialize color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Reverse left motors
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure odometry
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset odometry and IMU
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize webcam (Dashboard-only stream)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1")); // null = no on-robot preview

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void handleIntake() {
        boolean aPressed = gamepad1.a;

        if (aPressed && !lastAPressed) {
            intakeOn = !intakeOn;
        }
        lastAPressed = aPressed;

        if (intakeOn) {
            ActiveIntake.setPower(1.0);
        } else {
            ActiveIntake.setPower(0.0);
        }

        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
    }

    public String detectBallColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        float total = red + green + blue;
        if (total == 0) return "Unknown";
        red /= total;
        green /= total;
        blue /= total;

        if (green > 0.48 && green > red && green > blue * 1.1) {
            return "Green";
        } else if ((red > 0.1 && blue > 0.1) && green < 0.35) {
            return "Purple";
        } else {
            return "Unknown";
        }
    }

    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = forward + strafe + rotate;  // Front Left
        wheelSpeeds[1] = forward - strafe - rotate;  // Front Right
        wheelSpeeds[2] = forward - strafe + rotate;  // Back Left
        wheelSpeeds[3] = forward + strafe - rotate;  // Back Right

        double max = Math.max(1.0, Math.abs(wheelSpeeds[0]));
        for (int i = 1; i < 4; i++) max = Math.max(max, Math.abs(wheelSpeeds[i]));
        for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;

        FLeft.setPower(wheelSpeeds[0]);
        FRight.setPower(wheelSpeeds[1]);
        BLeft.setPower(wheelSpeeds[2]);
        BRight.setPower(wheelSpeeds[3]);

        Pose2D pos = odo.getPosition();
        TelemetryPacket packet = new TelemetryPacket();

        String colorDetected = detectBallColor();

        packet.put("Robot X (mm)", pos.getX(DistanceUnit.MM));
        packet.put("Robot Y (mm)", pos.getY(DistanceUnit.MM));
        packet.put("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
        packet.put("Intake", intakeOn ? "ON" : "OFF");
        packet.put("Ball Color", colorDetected);

        telemetry.addData("Detected Color", colorDetected);

        packet.fieldOverlay().clear();

        double x = pos.getY(DistanceUnit.MM) / SCALE;
        double y = pos.getX(DistanceUnit.MM) / SCALE;

        packet.fieldOverlay().strokeRect(
                x - ROBOT_WIDTH_MM / 2.0,
                y - ROBOT_LENGTH_MM / 2.0,
                ROBOT_WIDTH_MM,
                ROBOT_LENGTH_MM
        );

        double headingRad = pos.getHeading(AngleUnit.RADIANS) - Math.PI / 2.0;
        double lineLength = ROBOT_LENGTH_MM / 2.0;
        packet.fieldOverlay().strokeLine(
                x,
                y,
                x + lineLength * Math.cos(headingRad),
                y + lineLength * Math.sin(headingRad)
        );

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        moveRobot();
        handleIntake();
        odo.update();
        telemetry.update();
    }
}
