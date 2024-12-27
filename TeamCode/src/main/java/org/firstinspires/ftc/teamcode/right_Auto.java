package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // called from the start
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.config.Config;

import java.lang.Math;


@Config
@Autonomous(name = "right_Auto")
public class right_Auto extends LinearOpMode { // extends means inherits from linear op mode


    int up_specimen_hang = 1907;

    int up_specimen_hang2 = 500 ;

    int Up_specimen_hang1 = 1313;
    double outtake_servo_hang = 0.30;

    double last_time = 0;
    double gravity_power_tune = 0.001;
    private ElapsedTime runtime = new ElapsedTime();

    // We need to create classes for each definition of hardware that isn't part of our drivetrain (I think this is for organization)
    // Here we make 6 classes, one for viper slide and one for misumi slide, and their claws and wrists and that one sensor (I'm too scared to combine them)

    public class Elevator { // We made nested classes, First class is Elevator with all the methods that involve elevator
        private final DcMotorEx up;

        public Elevator(HardwareMap Hardwaremap) { // Class for Dx motor Up initilize
            up = hardwareMap.get(DcMotorEx.class, "up");
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setDirection(DcMotorSimple.Direction.REVERSE);
            up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class Elevator_Up_Move implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {// why this parameter?
                if (!initialized) {
                    up.setPower(0.5);
                    initialized = true;
                }
                double pos = up.getCurrentPosition();
                if (pos <= Up_specimen_hang1) {
                    telemetry.addData("Up Pos", pos);
                    telemetry.addData("velocity", up.getVelocity());
                    telemetry.update();
                    return true;
                } else {
                    up.setPower(gravity_power_tune);
                    telemetry.addData("Up Pos", pos);
                    telemetry.addData("velocity", up.getVelocity());
                    telemetry.update();
                    sleep(2000);
                    return false;

                }
            }
        }


        public Action elevator_up_move() {
            return new Elevator_Up_Move();
        }
        public class Elevator_Up_Move1 implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {// why this parameter?
                if (!initialized) {
                    up.setPower(-0.4);
                    initialized = true;
                }
                double pos = up.getCurrentPosition();
                if (pos > up_specimen_hang2) {
                    telemetry.addData("Up Pos", pos);
                    telemetry.addData("velocity", up.getVelocity());
                    telemetry.update();
                    return true;
                } else {
                    up.setPower(gravity_power_tune);
                    telemetry.addData("Up Pos", pos);
                    telemetry.addData("velocity", up.getVelocity());
                    telemetry.update();
                    sleep(2000);
                    return false;

                }
            }
        }


        public Action elevator_up_move1() {
            return new Elevator_Up_Move1();
        }
        public class wait_Dont_Kys implements Action {
            private boolean time_spent = false;

            double time = runtime.seconds();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                while (time < 15) {
                telemetry.addData("waiting for sigma", time);
                }
                time_spent = true;
                return true;

            }
        }
        public Action wait_dont_kys() {
            return new wait_Dont_Kys();
        }

        public class Elevator_Down_Move implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {// why this parameter?
                if (!initialized) {
                    up.setPower(-0.5);
                    initialized = true;
                }
                double pos = up.getCurrentPosition();
                telemetry.addData("pos", pos);
                telemetry.addData("velocity", up.getVelocity());
                telemetry.update();
                if (pos > up_specimen_hang2) {
                    return true;
                } else {
                    up.setPower(gravity_power_tune);
                    sleep(2000);
                    return false;


                }
            }
        }

        public Action elevator_down_move() {
            return new Elevator_Down_Move();
        }
        public class Elevator_Down_Move1 implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {// why this parameter?
                if (!initialized) {
                    up.setPower(-0.2);
                    initialized = true;
                }
                double pos = up.getCurrentPosition();
                telemetry.addData("pos", pos);
                telemetry.addData("velocity", up.getVelocity());
                telemetry.update();
                if (pos > Up_specimen_hang1) {
                    return true;
                } else {
                    up.setPower(gravity_power_tune);
                    sleep(2000);
                    return false;


                }
            }
        }
        public Action elevator_down_move1() {
            return new Elevator_Down_Move1();
        }
    }

    public class Induction {
           private DcMotorEx out;

           public Induction(HardwareMap hardwareMap) {
               out = hardwareMap.get(DcMotorEx.class, "out");
               out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               out.setTargetPosition(0);
               out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }
    }

    public class Elevator_claw {
        private Servo servo_outtake;

        public Elevator_claw(HardwareMap hardwaremap) {
            servo_outtake = hardwareMap.get(Servo.class, "outtakeWrist");
        }


        // Class Action to Move viper slide elevator
        // Implements inherits action into Elevator_Up_Move --- more stuff we can do i think
        public class Elevator_Claw_Move implements Action {
            private boolean initilized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initilized) {
                    servo_outtake.setPosition(0.45);
                    initilized = true;
                    return true;
                } else {
                    sleep(1000);
                    return false;
                }

            }
        }
        public Action elevator_claw_move() {
            return new Elevator_Claw_Move();
        }
    }






    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -70, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // make a Claw instance
        Elevator up = new Elevator(hardwareMap);
        // make a Lift instance
        Elevator_claw claw = new Elevator_claw(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        Action drive_forward = drive.actionBuilder(/*start position*/new Pose2d(0.0, -70, 90.0)) // tells the robot where it's going to start?
                .strafeToLinearHeading(new Vector2d(0, -40), Rotation2d.fromDouble(Math.toRadians(90)))
                //first place
                .build();
        Action hang = drive.actionBuilder(new Pose2d(0.0,-40,90))
                .waitSeconds(2)
                .build();
        Action rest = drive.actionBuilder(new Pose2d(0.0, -33, 90))
                .lineToY(-44) //back up

                .strafeTo(new Vector2d(34, -44)) //don't run into sub
                .strafeTo(new Vector2d(34, -5)) //don't run into sample!
                //first sample push
                .strafeTo(new Vector2d(45,-5))
                .waitSeconds(.1)
                .lineToY(-50)
                .lineToY(-15)
                //second sample push
                .strafeTo(new Vector2d(54,-12))
                .waitSeconds(.1)
                .lineToY(-50)
                .lineToY(-15)
                //third sample push
                .strafeTo(new Vector2d(62,-12))
                .waitSeconds(.1)
                .lineToY(-50)
                .waitSeconds(.1)
                .setTangent(Math.toRadians(180)) //rotation positioning
                .lineToXSplineHeading(36,Math.toRadians(270)) //line up for pickup
                .setTangent(Math.toRadians(90))
                .lineToY(-62)
                //pick up first
                .strafeToLinearHeading(new Vector2d(10,-32), Rotation2d.fromDouble(Math.toRadians(90)))
                //place first
                .waitSeconds(1.25)
                .strafeToLinearHeading(new Vector2d(36,-52), Rotation2d.fromDouble(Math.toRadians(270))) //line up for pickup
                .waitSeconds(.1)
                //pick up second
                .lineToY(-62)
                .strafeToLinearHeading(new Vector2d(6,-32), Rotation2d.fromDouble(Math.toRadians(90)))
                //place second
                .waitSeconds(1.25)
                .build();
        Action park =drive.actionBuilder(new Pose2d(0.0, -33, 90))
                .strafeTo(new Vector2d(50,-62)) //parking

                .build();
        Action drive_forward2 = drive.actionBuilder(new Pose2d(0,0,90))
                        .lineToY(-45)
                                .build();






        waitForStart();

        if(isStopRequested()) return;



        // actions that need to happen on init; for instance, a claw tightening.
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("runtime", runtime);

        telemetry.update();
        if (isStopRequested()) return;
        Action drive_forward_build;
        drive_forward_build= drive_forward;
        Action drive_forward_build1;
        drive_forward_build1=drive_forward2;

        Actions.runBlocking(
                new SequentialAction(
                        drive_forward,
                        new ParallelAction(
                                claw.elevator_claw_move(),
                             up.elevator_up_move()
                        ),
                       up.elevator_down_move()


                        //up.elevator_down_move1(),
                        //drive_forward_build1,
                        //up.elevator_down_move()

                        //new ParallelAction(
                        //up.elevator_down_move(), claw.elevator_Claw_Move()
                        //)
                )// it moves but theres like drift itkidk
        );
        telemetry.update();
        sleep(10000);
    }
}