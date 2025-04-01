package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class Module extends SubsystemBase {
  /** Creates a new Module. */

  public SparkFlex angleMotor;
  public SparkFlex driveMotor;

  public final RelativeEncoder integratedDriveEncoder;
  private final RelativeEncoder integratedAngleEncoder;

  private final int moduleNum; // ZERO INDEXED
  private final String modulePos;

  private SparkClosedLoopController angleController;
  private SparkClosedLoopController driveController;

  private CANcoder _CANCoder;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveBaseConstants.driveKS, 
      Constants.DriveBaseConstants.driveKV, Constants.DriveBaseConstants.driveKA);

  public Module(int module) {
    this.moduleNum = module;

    _CANCoder = new CANcoder(Constants.ModuleConstants.cancoderIDs[this.moduleNum], "canivore");

    /* Angle Motor Config */
    angleMotor = new SparkFlex(Constants.ModuleConstants.angleMotorIDs[this.moduleNum], MotorType.kBrushless);
    
    integratedAngleEncoder = angleMotor.getEncoder();
    
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkFlex(Constants.ModuleConstants.driveMotorIDs[this.moduleNum], MotorType.kBrushless);

    integratedDriveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDriveMotor();

    resetToAbsolute();
    // angleMotor.setVoltage(module);
    modulePos = Constants.ModuleConstants.modulePosition[this.moduleNum];
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if(isOpenLoop){
      double percentOutput = desiredState.speedMetersPerSecond / Constants.DriveBaseConstants.maxSpeed;
      driveMotor.set(percentOutput);    
    } else {        
        driveController.setReference(desiredState.speedMetersPerSecond, 
        ControlType.kVelocity, 
        ClosedLoopSlot.kSlot0, 
        feedforward.calculate(desiredState.speedMetersPerSecond));
    }  
  }

  public void setAngle(SwerveModuleState desiredState){
    // Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DriveBaseConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angleController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    // lastAngle = desiredState.angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private Rotation2d getAngleIntegrated(){
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(_CANCoder.getAbsolutePosition().getValueAsDouble() * 360);
  }

  public void resetToAbsolute(){
    double absolutePosition = getCanCoder().getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }
  
  private void configAngleMotor() {
    SparkFlexConfig aConfig = new SparkFlexConfig();
    
    aConfig.smartCurrentLimit(Constants.DriveBaseConstants.angleMaxCurrent);
    aConfig.inverted(Constants.DriveBaseConstants.invertAngleMotor);
    aConfig.idleMode(Constants.DriveBaseConstants.angleNeutralMode);   

    aConfig.closedLoop.positionWrappingEnabled(true);
    aConfig.closedLoop.positionWrappingInputRange(0, 360);

    aConfig.closedLoop.pid(Constants.DriveBaseConstants.angleKP, Constants.DriveBaseConstants.angleKI, Constants.DriveBaseConstants.angleKD);

    aConfig.closedLoop.velocityFF(Constants.DriveBaseConstants.angleKF);
    aConfig.voltageCompensation(Constants.DriveBaseConstants.voltageComp); 

    aConfig.encoder.positionConversionFactor(Constants.DriveBaseConstants.angleConversionFactor);

    angleMotor.configure(aConfig, ResetMode.kResetSafeParameters, null);
  }

  private void configDriveMotor() {
    SparkFlexConfig dConfig = new SparkFlexConfig();

    dConfig.smartCurrentLimit(Constants.DriveBaseConstants.driveMaxCurrent);
    dConfig.inverted(Constants.ModuleConstants.invertedDrive[moduleNum]);
    dConfig.idleMode(Constants.DriveBaseConstants.driveNeutralMode);

    dConfig.closedLoop.pid(Constants.DriveBaseConstants.driveKP, Constants.DriveBaseConstants.driveKI, Constants.DriveBaseConstants.driveKD);

    dConfig.closedLoop.velocityFF(Constants.DriveBaseConstants.driveKF);
    dConfig.voltageCompensation(Constants.DriveBaseConstants.voltageComp);

    dConfig.encoder.positionConversionFactor(Constants.DriveBaseConstants.driveConversionPositionFactor);
    dConfig.encoder.velocityConversionFactor(Constants.DriveBaseConstants.driveConversionVelocityFactor);    

    driveMotor.configure(dConfig, ResetMode.kResetSafeParameters, null);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(
        integratedDriveEncoder.getVelocity(), 
        getAngleIntegrated()
    ); 
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
        integratedDriveEncoder.getPosition(), 
        getAngleIntegrated()
    );
  }

  public double getDriveSpeed() {
    return integratedDriveEncoder.getVelocity();
  }

  public double getAnglePosition() {
    return getAngleIntegrated().getRadians();
  }

  public double getAnglePosDegrees() {
    return getAngleIntegrated().getDegrees();
  }

  public double getDriveTemperature() {
    return driveMotor.getMotorTemperature();
  }

  public double getAngleTemperature() {
    return angleMotor.getMotorTemperature();
  }

  public double getAngleOmega() {
    return (integratedDriveEncoder.getVelocity() * 2 * Math.PI) / 60;
  }

  @Override
  public void periodic() {
    resetToAbsolute();
    
    // Telemetry
    SmartDashboard.putNumber(modulePos + " Angle", OnboardModuleState.normalizeDegrees(getAnglePosDegrees()));
    SmartDashboard.putNumber(modulePos + " Speed", Math.abs(getDriveSpeed()));
    SmartDashboard.putNumber(modulePos + " Omega", getAngleOmega());
  }
}
