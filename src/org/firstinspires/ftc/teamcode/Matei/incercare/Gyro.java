package org.firstinspires.ftc.teamcode.Matei.incercare;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro extends LinearOpMode {
    private final DcMotor lf, rf, lr, rr;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double global = 0;

    public Gyro(DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr) {
        this.lf = lf;
        this.rf = rf;
        this.lr = lr;
        this.rr = rr;
    }

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double delta = angles.firstAngle - lastAngles.firstAngle;

        if (delta < -180) {
            delta += 360;
        } else if (delta > 180) {
            delta -= 360;
        }

        global += delta;
        lastAngles = angles;

        return global;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        global = 0;
    }

    public void rotire(int grade, double power) {
        double left, right, powerSlow, gradeSlow;
        resetAngle();


        if (grade < 0) {
            left = power;
            right = -power;
        } else if (grade > 0) {
            left = -power;
            right = power;
        } else return;


        setMotorsPower(left, left, right, right);

        if (grade < 0) {
            while (getAngle() == 0) {
            }
            while (getAngle() > grade) {
            }
        } else {
            while (getAngle() < grade) {
            }
        }

        setMotorsPower(0, 0, 0, 0);

        sleep(1000);
        resetAngle();
    }


    private void setMotorsPower(double lfm, double lrm, double rfm, double rrm) {
        lf.setPower(lfm);
        lr.setPower(lrm);
        rf.setPower(rfm);
        rr.setPower(rrm);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
