package frc.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.AbsoluteAnalogEncoder;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
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
    private double lastMeasure;
    private double rate = 0;
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public Module(int motorCANId, int encoderPort) {
        motor = new TalonFX(motorCANId);
        encoder = new AbsoluteAnalogEncoder(encoderPort, 0, true);

        cfg = new SysIdRoutine.Config(Velocity.combine(Volts, Second).of(0.25), null, null);

        routine = new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
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
                    .angularPosition(angle.mut_replace(encoder.getRotationDegrees()/360, Rotations))
                    .angularVelocity(
                        velocity.mut_replace(rate, RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("module")
              this));
    }

    @Override
    public void periodic() {
      rate = ((encoder.getRotationDegrees()/360)-lastMeasure)/0.02;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return routine.dynamic(direction);
    }
}
