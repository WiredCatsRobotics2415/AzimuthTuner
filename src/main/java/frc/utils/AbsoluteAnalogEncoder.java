package frc.utils;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class AbsoluteAnalogEncoder extends AnalogInput {

    private double offset; //Declare offset variable
    private boolean reversed; //Declare reversed

    public AbsoluteAnalogEncoder(int port, double offset, boolean reversed) {
        super(port); 
        setOffset(offset);
        this.reversed = reversed;
    }

    public void setOffset(double offset) {
        this.offset = (offset / 360 * RobotController.getVoltage5V()) % RobotController.getVoltage5V();
        if (this.offset < 0)
            this.offset += RobotController.getVoltage5V();
    }

    @Override
    public double getVoltage() {
        double returnValue = super.getVoltage();
        if (this.reversed) returnValue *= -1;
        returnValue = (returnValue - this.offset) % RobotController.getVoltage5V(); 
        if (returnValue < 0)
            returnValue += RobotController.getVoltage5V();
        return returnValue;
    }
    
    public double getRotationPercent() {
        return (getVoltage() / RobotController.getVoltage5V());
    }

    public double getRotationDegrees() {
        return getRotationPercent() * 360.0;
    }

}