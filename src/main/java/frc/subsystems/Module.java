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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class Module extends SubsystemBase {
    private TalonFX motor;

    private SysIdRoutine.Config cfg;
    private final SysIdRoutine routine;

    private StatusSignal<Double> positionSignal;
    private StatusSignal<Double> velocitySignal;

    private VoltageOut sysIdControl = new VoltageOut(0);
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public Module(int motorCANId, int encoderPort) {
        motor = new TalonFX(motorCANId);

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        motor.optimizeBusUtilization();

        SignalLogger.start();

        // positionSignal = motor.getPosition();
        // velocitySignal = motor.getVelocity();

        cfg = new SysIdRoutine.Config(
          Velocity.combine(Volts, Second).of(0.5),
          Volts.of(5), 
          null,
          (state) -> SignalLogger.writeString("state", state.toString())
        );

        /*
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
                log.motor("azi")
                    .voltage(
                        appliedVoltage.mut_replace(
                            motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(angle.mut_replace(Robot.isReal() ? //If simulated, apply random "ideal" values (ie. get a solid r for a linear best fit)
                      positionSignal.getValue() :
                      appliedVoltage.magnitude()*Math.random(), Rotations)) 
                    .angularVelocity(
                        velocity.mut_replace(velocitySignal.getValue(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("module")
              this));
      */

        routine = new SysIdRoutine(cfg,
          new Mechanism(
            (Measure<Voltage> volts)-> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
            null,
            this
        ));
    }

    @Override
    public void periodic() {
      // positionSignal.refresh();
      // velocitySignal.refresh();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return routine.dynamic(direction);
    }
}
