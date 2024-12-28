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

    int up_specimen_hang2 = 500 ; // Todo: Calibrate this position to fit the after hang height, beware this might rip the claw off if its too off. Set these perhaps in opmode first

    int Up_specimen_hang1 = 1313; // TODO :  Calibrate these to fit exactly between specimen hanger and claw
    double outtake_servo_hang = 0.30;

    double last_time = 0;
    double gravity_power_tune = 0.001;
    private ElapsedTime runtime = new ElapsedTime();

    // We need to create classes for each definition of hardware that isn't part of our drivetrain (I think this is for organization)
    // Here we make 6 classes, one for viper slide and one for misumi slide, and their claws and wrists and that one sensor (I'm too scared to combine them)
    public class Elevator { // We made nested classes, First class is Elevator with all the methods that involve elevator
        private final DcMotorEx up;

        public Elevator(HardwareMap Hardwaremap) { // Class for Dx motor Up initialize
            up = hardwareMap.get(DcMotorEx.class, "up");
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setDirection(DcMotorSimple.Direction.REVERSE);
            up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class Elevator_Up_Move_specimen implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {// why this parameter?
                if (!initialized) {
                    up.setPower(0.8); // TODO: speed this up if possible
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
                    return false;

                }
            }
        }

        public Action elevator_up_move_specimen() {
            return new Elevator_Up_Move_specimen();
        }

        public class Elevator_Down_Move_Specimen implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {// why this parameter?
                if (!initialized) {
                    up.setPower(-1); // Todo: Calibrate this if its too fast
                    initialized = true;
                }
                double pos = up.getCurrentPosition();
                telemetry.addData("pos", pos);
                telemetry.addData("velocity", up.getPower());
                telemetry.update();
                if (pos >= up_specimen_hang2) {
                    return true;
                } else {
                    up.setPower(gravity_power_tune);
                    telemetry.addData("pos", pos);
                    telemetry.addData("velocity", up.getPower());
                    telemetry.update();
                    return false;


                }
            }
        }

        public Action elevator_down_move_specimen() {
            return new Elevator_Down_Move_Specimen();
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

    public class Outtake_claw_wrist {
        private Servo servo_outtake_wrist;

        public Outtake_claw_wrist(HardwareMap hardwaremap) {
            servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        }


        // Class Action to Move viper slide elevator
        // Implements inherits action into Elevator_Up_Move_specimen --- more stuff we can do i think
        public class Elevator_Claw_Wrist_Move_Specimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo_outtake_wrist.setPosition(0.45);
                    initialized = true;
                    return true;
                } else {
                    return false;
                }

            }
        }
        public Action elevator_claw_wrist_move_specimen() {
            return new Elevator_Claw_Wrist_Move_Specimen();
        }
    }
    public class Outtake_claw {
        private final CRServo servo_outtake;

        public Outtake_claw(HardwareMap hardwaremap) {
            servo_outtake = hardwaremap.get(CRServo.class,"outtake");
        }

        public class Elevator_Claw_Specimen implements Action {
            @Override
            public boolean run(TelemetryPacket packet) { // not sure how this runs
                runtime.reset();
                telemetry.addData("Time", runtime);
                while (runtime.seconds() < 1) {
                        telemetry.addData("Running Outtake", servo_outtake.getPower());
                        telemetry.addData("Time", runtime);
                        telemetry.update();
                        servo_outtake.setPower(-1); // Todo : set this to the correct power
                    }
                    return true;
            }
        }
        public Action elevator_claw_move_specimen() {
            return new Elevator_Claw_Specimen();
        }
    }






    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -70, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // make a Claw instance
        Elevator up = new Elevator(hardwareMap);
        // make a Lift instance
        Outtake_claw_wrist Outtake_claw_wrist = new Outtake_claw_wrist(hardwareMap);
        // makes claw instance
        Outtake_claw Outtake_claw = new Outtake_claw(hardwareMap);


        // trajectory plans
        Action drive_forward_hang_specimen = drive.actionBuilder(initialPose) // tells the robot where it's going to start?
                .strafeToLinearHeading(new Vector2d(0, -40), Rotation2d.fromDouble(Math.toRadians(90))) // TODO : Tune this so it stops ready to hang pos
                .build();
        Action hang_specimen = drive.actionBuilder(new Pose2d(0.0,-40,90))
                .waitSeconds(3)
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




        // actions that need to happen on init; for instance, a claw_wrist tightening.
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("READY TO RUMBLE", initialPose);
            telemetry.update();
        }
        waitForStart();

        if(isStopRequested()) return;





        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("runtime", runtime);

        telemetry.update();
        if (isStopRequested()) return;

        Actions.runBlocking(
                //new ParallelAction(
                    new SequentialAction(
                            new ParallelAction(
                                    up.elevator_up_move_specimen(),
                                    Outtake_claw_wrist.elevator_claw_wrist_move_specimen()
                            ),
                            drive_forward_hang_specimen // CALIBRATE THIS
                            //up.elevator_down_move_specimen()  // calibrate this safely
                            //Outtake_claw.elevator_claw_move_specimen()


                //)
                )
        );
        telemetry.update();
        sleep(10000);
    }
}