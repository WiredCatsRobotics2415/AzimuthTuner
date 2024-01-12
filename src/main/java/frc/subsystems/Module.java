package frc.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.utils.AbsoluteAnalogEncoder;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

public class Module extends SubsystemBase {
    private TalonFX motor;
    private AbsoluteAnalogEncoder encoder;

    private SysIdRoutine.Config cfg;

    private final SysIdRoutine routine;
    private double veloLast = 0;
    private double rate = 0;
    private double encoderLast = 0;
    private double totalRotations = 0;
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public Module(int motorCANId, int encoderPort) {
        motor = new TalonFX(motorCANId);
        encoder = new AbsoluteAnalogEncoder(encoderPort, 0, true);

        cfg = new SysIdRoutine.Config(Velocity.combine(Volts, Second).of(0.25), null, null);

        routine = new SysIdRoutine(
          cfg,
          new SysIdRoutine.Mechanism(
              // Tell SysId how to send the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                motor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("azi")
                    .voltage(
                        appliedVoltage.mut_replace(
                            motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(angle.mut_replace(Robot.isReal() ? //If simulated, apply random "ideal" values (ie. get a solid r for a linear best fit)
                      getEncoderWithRollOver() :
                      appliedVoltage.magnitude()*Math.random(), Rotations)) 
                    .angularVelocity(
                        velocity.mut_replace(rate, RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("module")
              this));
    }

    @Override
    public void periodic() {
      double read = getEncoderWithRollOver();
      if (Robot.isSimulation()) {
        //In this case, apply random "ideal" values (ie. get a solid r for a linear best fit)
        read = appliedVoltage.magnitude()*Math.random();
      }
      rate = (read-veloLast)/0.02;
      veloLast = read;
    }

    //Returns the angle of the encoder with rollover (ie. If it changed from 360 to 0, return 361)
    private double getEncoderWithRollOver() {
      double read = encoder.getRotationDegrees()/360;
      if (encoderLast > 0.5 && read < 0.5) { //Rollover past 360
        totalRotations++;
      }
      if (encoderLast < 0.5 && read > 0.5) { //Rollover past 0
       totalRotations--;
      }
      encoderLast = read;
      if (totalRotations < 0) read*=-1;
      return totalRotations+(read);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return routine.dynamic(direction);
    }
}
