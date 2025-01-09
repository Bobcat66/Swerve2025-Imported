package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim m_driveSim = new DCMotorSim(, 1, 0.01);
    private final DCMotorSim m_turnSim = new DCMotorSim(1, 12.8, 0.004);


    private double turnSetpoint = 0;
    private double driveSetpoint = 0;
    private double driveVoltage = 0;
    private double turnVoltage = 0;

    private final PIDController m_turnPIDController = new PIDController(2, 0, 0);
    private final PIDController m_drivePIDController = new PIDController(22, 0, 0);
    private final SimpleMotorFeedforward m_driveFFController = new SimpleMotorFeedforward(0, 2.78, 0);

    
    //Take in parameters to specify exact module
    public ModuleIOSim() {
        m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update the sim loop
        m_driveSim.update(0.02);
        m_turnSim.update(0.02);

        inputs.drivePositionMeters = m_driveSim


    }

    @Override
    public void setDriveVolts(double volts) {}

    @Override
    public void setTurnVolts(double volts) {}

    @Override
    public void setDriveVelocity(double velocityMetersPerSec, double FFVolts) {}

    @Override
    public void setTurnPosition(double positionRots, double FFVolts) {}
}
