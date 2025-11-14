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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="Main_SmoothSpindexer_TurretManual")
public class drive extends OpMode {

    // Odometry
    GoBildaPinpointDriver odo;

    // Drive motors
    private DcMotor BLeft, BRight, FLeft, FRight;

    // Limelight
    private Limelight3A limelight;

    // Sensors + Servos
    private NormalizedColorSensor colorSensor;
    private Servo spinnerServo;
    private Servo Spindexer;
    private Servo turretServo;

    // Intake toggle
    private boolean intakeOn = true;
    private boolean lastAPressed = false;

    // Pathing toggle
    private boolean pathMode = false;
    private boolean lastBPressed = false;

    // Spindexer storage (3 slots)
    private String[] spindexerSlots = {"Empty", "Empty", "Empty"};
    private int nextEmptySlot = 0;

    // Field constants
    final double BLUE_BASKET_X = -610;
    final double BLUE_BASKET_Y = 915;
    final double RED_BASKET_X = 610;
    final double RED_BASKET_Y = -915;

    // Smooth servo parameters
    private double spindexerTarget = 0;
    private double spindexerSpeed = 0.02; // adjust for speed of movement

    // Turret manual override parameters
    private double turretSpeed = 0.01; // servo increment per loop for D-pad
    private boolean manualTurret = false;

    @Override
    public void init() {
        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0));

        // Initialize motors
        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        // Initialize servos
        spinnerServo = hardwareMap.get(Servo.class, "spinnerServo");
        spinnerServo.setPosition(0);
        Spindexer = hardwareMap.get(Servo.class, "Spindexer");
        Spindexer.setPosition(0);
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(0.5);

        // Color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    // Intake toggle
    public void handleIntake() {
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAPressed) intakeOn = !intakeOn;
        lastAPressed = aPressed;
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");

        // Detect balls and store them if intake is on
        if (intakeOn) {
            String color = detectBallColor();
            if (!color.equals("Unknown") && nextEmptySlot < 3) {
                spindexerSlots[nextEmptySlot] = color;
                nextEmptySlot++;
            }
        }
    }

    // Spinner control
    public void handleSpinner() {
        double trigger = gamepad1.right_trigger;
        spinnerServo.setPosition(trigger);
    }

    // Detect ball color
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

    // Drive control with precision mode
    public void moveRobot() {
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBPressed) pathMode = !pathMode;
        lastBPressed = bPressed;

        if (!pathMode) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double speedScale = gamepad1.right_bumper ? 0.5 : 1.0;

            double[] wheelSpeeds = new double[4];
            wheelSpeeds[0] = (forward + strafe + rotate) * speedScale;
            wheelSpeeds[1] = (forward - strafe - rotate) * speedScale;
            wheelSpeeds[2] = (forward - strafe + rotate) * speedScale;
            wheelSpeeds[3] = (forward + strafe - rotate) * speedScale;

            double max = 1.0;
            for (double s : wheelSpeeds) if (Math.abs(s) > max) max = Math.abs(s);
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;

            FLeft.setPower(wheelSpeeds[0]);
            FRight.setPower(wheelSpeeds[1]);
            BLeft.setPower(wheelSpeeds[2]);
            BRight.setPower(wheelSpeeds[3]);
        }
    }

    // Auto-aim turret to blue basket
    private void aimTurret(double robotX, double robotY, double headingRad) {
        // Check manual override
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            manualTurret = true;
            double currentPos = turretServo.getPosition();
            if (gamepad1.dpad_left) currentPos -= turretSpeed;
            if (gamepad1.dpad_right) currentPos += turretSpeed;
            currentPos = Math.max(0.0, Math.min(1.0, currentPos));
            turretServo.setPosition(currentPos);
        } else {
            manualTurret = false;
        }

        // Auto-aim only if not manually overriding
        if (!manualTurret) {
            double dx = BLUE_BASKET_X - robotX;
            double dy = BLUE_BASKET_Y - robotY;
            double targetAngle = Math.atan2(dy, dx);
            double relativeAngle = targetAngle - headingRad;

            while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

            double servoPos = (relativeAngle / Math.toRadians(180)) + 0.5;
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            turretServo.setPosition(servoPos);
        }
    }

    // Spindexer controls with smooth servo movement
    private void handleSpindexer() {
        // Set target based on triggers
        if (gamepad1.right_trigger > 0.1) { // Purple
            for (int i = 0; i < 3; i++) {
                if (spindexerSlots[i].equals("Purple")) {
                    spindexerTarget = 1.0;
                    spindexerSlots[i] = "Empty";
                    nextEmptySlot = Math.max(nextEmptySlot - 1, 0);
                    break;
                }
            }
        } else if (gamepad1.left_trigger > 0.1) { // Green
            for (int i = 0; i < 3; i++) {
                if (spindexerSlots[i].equals("Green")) {
                    spindexerTarget = 1.0;
                    spindexerSlots[i] = "Empty";
                    nextEmptySlot = Math.max(nextEmptySlot - 1, 0);
                    break;
                }
            }
        } else {
            spindexerTarget = 0.0;
        }

        // Move servo gradually towards target
        double currentPos = Spindexer.getPosition();
        if (Math.abs(spindexerTarget - currentPos) > spindexerSpeed) {
            if (spindexerTarget > currentPos) currentPos += spindexerSpeed;
            else currentPos -= spindexerSpeed;
            Spindexer.setPosition(currentPos);
        } else {
            Spindexer.setPosition(spindexerTarget);
        }
    }

    @Override
    public void loop() {
        moveRobot();
        handleIntake();
        handleSpinner();
        handleSpindexer();
        odo.update();

        Pose2D pos = odo.getPosition();
        double robotX = pos.getX(DistanceUnit.MM);
        double robotY = pos.getY(DistanceUnit.MM);
        double headingRad = pos.getHeading(AngleUnit.RADIANS);
        double headingDeg = pos.getHeading(AngleUnit.DEGREES);

        // Distances to baskets
        double distBlue = Math.hypot(BLUE_BASKET_X - robotX, BLUE_BASKET_Y - robotY);
        double distRed = Math.hypot(RED_BASKET_X - robotX, RED_BASKET_Y - robotY);

        // Auto/manual turret
        aimTurret(robotX, robotY, headingRad);

        // limelight april tag y axis x axis and area
        //need to add the detect ball green ball is stored on pipeline 9
        //april tag is on pipeline 8
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }

      //print out the data
        telemetry.addData("Robot X (mm)", robotX);
        telemetry.addData("Robot Y (mm)", robotY);
        telemetry.addData("Heading (deg)", headingDeg);
        telemetry.addData("Distance to Blue Basket", distBlue);
        telemetry.addData("Distance to Red Basket", distRed);
        telemetry.addData("Detected Ball", detectBallColor());
        telemetry.addData("Spindexer Slots", spindexerSlots[0] + ", " + spindexerSlots[1] + ", " + spindexerSlots[2]);
        telemetry.addData("Precision Mode", gamepad1.right_bumper ? "ON" : "OFF");
        telemetry.addData("Turret Mode", manualTurret ? "Manual" : "Auto");
        telemetry.update();
    }
}
