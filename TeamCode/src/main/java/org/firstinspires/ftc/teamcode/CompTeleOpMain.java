package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.EdgeButton;

@TeleOp(name="Comp TeleOp Main", group="Competition")
public class CompTeleOpMain extends OpMode {

    RobotHardware robot;

    DriveSubsystem drive;
    IntakeSubsystem intake;
    ShooterSubsystem shooter;
    TurretSubsystem turret;
    SpindexerSubsystem spindexer;

    @Override
    public void init() {

        robot = new RobotHardware();
        robot.init(hardwareMap);

        drive = new DriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        spindexer = new SpindexerSubsystem(robot);
        shooter = new ShooterSubsystem(robot, spindexer);
        turret = new TurretSubsystem(robot);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        drive.update(gamepad1);

        intake.update(gamepad1);

        shooter.update(gamepad2);

        turret.update(gamepad2);

        telemetry.addData("Turret Deg", turret.getTurretDegrees());
        telemetry.addData("Flywheel Target", shooter.flywheelTargetPower);
        telemetry.addData("Intake Target", intake.intakeTargetPower);
        telemetry.update();
    }
}
