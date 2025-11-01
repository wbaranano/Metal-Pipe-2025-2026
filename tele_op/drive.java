package org.firstinspires.ftc.teamcode.tele_op;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// Pedro Pathing imports
import com.pedro.library.Path;
import com.pedro.library.PathFollower;
import com.pedro.library.PathPoint;

@TeleOp(name="Main with Pedro")
public class drive extends OpMode {

    // Odometry
    GoBildaPinpointDriver odo;

    // Drive motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private Limelight3A limelight;

    // Intake motor
    private DcMotor ActiveIntake;

    // Color sensor
    private NormalizedColorSensor colorSensor;

    // Servo for spinning
    private Servo spinnerServo;

    // Intake toggle
    private boolean intakeOn = true;
    private boolean lastAPressed = false;

    // Pathing
    private PathFollower pathFollower;
    private boolean pathMode = false;
    private boolean lastBPressed = false;

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        // Initialize intake
        ActiveIntake = hardwareMap.get(DcMotor.class, "ActiveIntake");

        // Initialize color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Initialize servo
        spinnerServo = hardwareMap.get(Servo.class, "spinnerServo");
        spinnerServo.setPosition(0);

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
                hardwareMap.get(WebcamName.class, "Webcam 1"));

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

        // Initialize Pedro Path
        Path path = new Path();
        path.addPoint(new PathPoint(0, 0));
        path.addPoint(new PathPoint(1000, 0));
        path.addPoint(new PathPoint(1000, 1000));
        path.addPoint(new PathPoint(0, 1000));
        pathFollower = new PathFollower(odo, path);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void handleIntake() {
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAPressed) intakeOn = !intakeOn;
        lastAPressed = aPressed;
        ActiveIntake.setPower(intakeOn ? 1.0 : 0.0);
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
    }

    public void handleSpinner() {
        double trigger = gamepad1.right_trigger;
        spinnerServo.setPosition(trigger);
        telemetry.addData("Spinner Servo", trigger);
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
        if (green > 0.48 && green > red && green > blue * 1.1) return "Green";
        else if ((red > 0.1 && blue > 0.1) && green < 0.35) return "Purple";
        else return "Unknown";
    }

    @Override
    public void start() {
        limelight.start();
    }

    public void moveRobot() {
        // Toggle path mode with B
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBPressed) pathMode = !pathMode;
        lastBPressed = bPressed;

        if (pathMode) {
            // Follow Pedro Path
            double[] powers = pathFollower.update();
            FLeft.setPower(powers[0]);
            FRight.setPower(powers[1]);
            BLeft.setPower(powers[2]);
            BRight.setPower(powers[3]);
        } else {
            // Regular teleop drive
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
        }

        // Telemetry
        Pose2D pos = odo.getPosition();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Robot X (mm)", pos.getX(DistanceUnit.MM));
        packet.put("Robot Y (mm)", pos.getY(DistanceUnit.MM));
        packet.put("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
        packet.put("Path Mode", pathMode ? "ON" : "OFF");
        packet.put("Ball Color", detectBallColor());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        moveRobot();
        handleIntake();
        handleSpinner();
        odo.update();

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        } else {
            telemetry.addData("Status", "No tag detected");
        }
        telemetry.update();
    }
}
