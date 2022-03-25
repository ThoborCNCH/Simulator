package org.firstinspires.ftc.teamcode.Matei;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.javafx.scene.traversal.Direction;

public class MotorProperties{

    private String name;
    private DcMotor motor;
    private String label;
    private DcMotor.Direction orientation;

    public MotorProperties() {
    }

    public DcMotor.Direction getOrientation() {
        return orientation;
    }

    public void setOrientation(DcMotor.Direction orientation) {
        this.orientation = orientation;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public void setMotor(DcMotor motor) {
        this.motor = motor;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }
}
