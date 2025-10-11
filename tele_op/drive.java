package org.firstinspires.ftc.teamcode.tele_op;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Odometry Mecanum Fixed Field Orientation")
public class drive extends OpMode {

    // Odometry
    GoBildaPinpointDriver odo;

    // Drive motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    // Dashboard
    FtcDashboard dashboard;

    // Field constants
    final double FIELD_MM = 3657.6; // 12 ft field
    final double CANVAS_SIZE = 60; // Dashboard canvas

    final double SCALE = FIELD_MM/CANVAS_SIZE;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void moveRobot() {
        // Gamepad inputs
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        // Mecanum calculations
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = forward + strafe + rotate;  // Front Left
        wheelSpeeds[1] = forward - strafe - rotate;  // Front Right
        wheelSpeeds[2] = forward - strafe + rotate;  // Back Left
        wheelSpeeds[3] = forward + strafe - rotate;  // Back Right

        // Normalize
        double max = Math.max(1.0, Math.abs(wheelSpeeds[0]));
        for (int i = 1; i < 4; i++) max = Math.max(max, Math.abs(wheelSpeeds[i]));
        for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;

        // Set motors
        FLeft.setPower(wheelSpeeds[0]);
        FRight.setPower(wheelSpeeds[1]);
        BLeft.setPower(wheelSpeeds[2]);
        BRight.setPower(wheelSpeeds[3]);

        // Get position
        Pose2D pos = odo.getPosition();

        // Create telemetry packet
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Robot X (mm)", pos.getX(DistanceUnit.MM));
        packet.put("Robot Y (mm)", pos.getY(DistanceUnit.MM));
        packet.put("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));

        // Field overlay
        packet.fieldOverlay().clear();

        // Swap X/Y for dashboard to match field orientation
        double x = pos.getY(DistanceUnit.MM) / SCALE; // lateral → canvas X
        double y = pos.getX(DistanceUnit.MM) / SCALE; // forward → canvas Y

        // Draw robot rectangle with fixed size
        packet.fieldOverlay().strokeRect(
                x - ROBOT_WIDTH_MM / 2.0,
                y - ROBOT_LENGTH_MM / 2.0,
                ROBOT_WIDTH_MM,
                ROBOT_LENGTH_MM
        );

        // Draw heading line (adjust for swapped axes)
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
        odo.update();
        telemetry.update();
    }
}
