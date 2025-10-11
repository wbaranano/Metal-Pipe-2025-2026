package org.firstinspires.ftc.teamcode.tele_op;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name="Odometry")

public class odometry extends OpMode {
    GoBildaPinpointDriver odo; //declare odometry

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");


        //reverse the motor direction

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //odometry computer configure
        odo.setOffsets(-84.0,-168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odometry starting position
        odo.resetPosAndIMU();
        Pose2D startingpostion = new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingpostion);

        telemetry.addData("Status", "Intialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.update();
    }
    public void moveRobot(){
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) -heading);
        double sinAngle = Math.sin((Math.PI / 2) -heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalforward = -forward * cosAngle + strafe * cosAngle;

        double[] newWheelSpeeds = new double [4];

        newWheelSpeeds[0] = globalforward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalforward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalforward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalforward + globalStrafe - rotate;

        FLeft.setPower(newWheelSpeeds[0]);
        FRight.setPower(newWheelSpeeds[1]);
        BLeft.setPower(newWheelSpeeds[2]);
        BRight.setPower(newWheelSpeeds[3]);
        telemetry.addData("RobotXpos:", pos.getX(DistanceUnit.MM));
        telemetry.addData("RobotYpos:", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading", heading);
        telemetry.addData( "Forward Speed", globalforward);
        telemetry.addData("Strafe Speed", globalStrafe);

        telemetry.addData( "Forward Speed", globalforward);
        telemetry.addData("Strafe Speed", globalStrafe);
    }
    @Override
    public void loop(){
        moveRobot();

        Pose2D pos = odo.getPosition();
        telemetry.addData("Robot x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot X", pos.getHeading(AngleUnit.DEGREES));

        odo.update();


    }
}
