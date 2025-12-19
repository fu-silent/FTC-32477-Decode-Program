package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto_Blue_Far_Leave_3_0", group = "Autonomous")
public class Auto_Blue_Far_Leave_3_0 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    // 自动程序专用参数
    static final double MAX_VEL_TICKS = 2500;
    static final double KP_DRIVE = 0.005;
    static final double KP_TURN = 0.045;
    static final double KP_CORRECTION = 0.06;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // 1. 配置自动程序专用的底盘 PIDF（不影响手动）
        PIDFCoefficients chassisPIDF = new PIDFCoefficients(15, 3, 0.5, 12);
        robot.lf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);
        robot.rf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);
        robot.lb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);
        robot.rb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);

        setChassisRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2. 配置自动程序专用的发射器 PIDF
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(250, 0, 100, 14);
        robot.s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        robot.s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        waitForStart();

        // --- 动作序列 ---
        // 前进 100mm, 锁定 0 度, 侧向补偿 0.22 (针对 12.7 度偏移)
        moveStraightVelocity(100, 0, 0.5, 0.22, 2.0);

        // 旋转 18 度瞄准
        rotateVelocity(18, 0.3, 2.5);

        // 发射 (1675 RPM, 持续 3 秒)
        shootVelocity(1675, 3.0);

        sleep(15000);

        // 归位并离开
        rotateVelocity(0, 0.3, 2.0);
        moveStraightVelocity(200, 0, 0.7, 0.22, 4.0);

        sleep(100);
    }

    // --- 自动程序私有控制方法 ---

    public void moveStraightVelocity(double mm, double targetAngle, double speed, double strafeComp, double timeout) {
        int targetTicks = (int)(mm * robot.TICKS_PER_MM);
        resetChassisEncoders();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeout) {
            int currentPos = (Math.abs(robot.lf.getCurrentPosition()) + Math.abs(robot.rf.getCurrentPosition())) / 2;
            double error = targetTicks - currentPos;
            if (Math.abs(error) < 15) break;

            double drive = Range.clip(error * KP_DRIVE, -speed, speed);
            double turn = getHeadingError(targetAngle) * KP_CORRECTION;

            // 使用 Velocity 闭环控制
            setVelocities(drive + strafeComp - turn, drive - strafeComp + turn,
                    drive - strafeComp - turn, drive + strafeComp + turn);
        }
        setVelocities(0,0,0,0);
    }

    public void rotateVelocity(double targetAngle, double speed, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeout) {
            double error = getHeadingError(targetAngle);
            if (Math.abs(error) < 1.0) break;
            double turn = Range.clip(error * KP_TURN, -speed, speed);
            if (Math.abs(turn) < 0.15) turn = Math.signum(turn) * 0.15;
            setVelocities(-turn, turn, -turn, turn);
        }
        setVelocities(0,0,0,0);
    }

    public void shootVelocity(int rpm, double duration) {
        double ticksPerSec = (rpm * robot.SHOOTER_TICKS) / 60.0;
        robot.s1.setVelocity(ticksPerSec);
        robot.s2.setVelocity(ticksPerSec);
        robot.intake.setPower(0.95);
        sleep(1500); // 等待转速稳定
        robot.load.setPower(0.85);
        sleep((long)(duration * 1000));
        robot.s1.setVelocity(0); robot.s2.setVelocity(0);
        robot.intake.setPower(0); robot.load.setPower(0);
    }

    private void setVelocities(double vLF, double vRF, double vLB, double vRB) {
        robot.lf.setVelocity(vLF * MAX_VEL_TICKS);
        robot.rf.setVelocity(vRF * MAX_VEL_TICKS);
        robot.lb.setVelocity(vLB * MAX_VEL_TICKS);
        robot.rb.setVelocity(vRB * MAX_VEL_TICKS);
    }

    private double getHeadingError(double target) {
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - heading;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    private void resetChassisEncoders() {
        setChassisRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setChassisRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setChassisRunMode(DcMotor.RunMode mode) {
        robot.lf.setMode(mode); robot.rf.setMode(mode);
        robot.lb.setMode(mode); robot.rb.setMode(mode);
    }
}