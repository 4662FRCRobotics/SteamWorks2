package org.usfirst.frc.team4662.robot.subsystems;

import org.usfirst.frc.team4662.robot.Robot;
import org.usfirst.frc.team4662.robot.RobotMap;
import org.usfirst.frc.team4662.robot.commands.ArcadeDrive;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import com.kauailabs.navx.frc.*;
/**
 *
 */
public class DriveSystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private AHRS navxGyro;
	private PIDController turnToAngle;
	private PIDController keepAngle;
	
	private CANTalon ControllerLeft1;
	private CANTalon ControllerLeft2;
	private CANTalon ControllerRight1;
	private CANTalon ControllerRight2;
	
	private Boolean m_bThrottleSwitch;
	
	private RobotDrive steamDrive; //makes the drive
	
	private double m_dThrottleDirection;
	private PIDController driveDistance;
	
	private double m_dWheelDiameter;
	private double m_dEncoderPulseCnt;
	
	private double m_dDriveP;
	private double m_dDriveI;
	private double m_dDriveD;
	
	private double m_dDistance;
	private double m_dRotations;
	
	private int m_iDriveError;
	
	private double m_dGyroP;
	private double m_dGyroI;
	private double m_dGyroD;

	private double m_dStraightP;
	private double m_dStraightI;
	private double m_dStraightD;
	
	private int m_iStraightError;
	
	private double m_dVoltageRampRate;
	private double m_dJoystickVolts;
	
	private double m_dAngle;
	
	private int m_iGyroError;
	
	private double lastLinearAccelX;
	private double lastLinearAccelY;
	
	private double kCollisionThreshold_DeltaG;
	
	private double currentJerkX;
	private double currentJerkY;
	
	private volatile double m_dSteeringHeading; 
	
	public DriveSystem(){
		
		ControllerLeft1 = new CANTalon(RobotMap.leftMotor1);
		ControllerLeft2 = new CANTalon(RobotMap.leftMotor2);
		ControllerLeft1.setInverted(false);
		ControllerRight1 = new CANTalon(RobotMap.rightMotor1);
		ControllerRight2 = new CANTalon(RobotMap.rightMotor2);
		ControllerRight1.setInverted(true);
		ControllerRight1.reverseSensor(true);
		
		ControllerLeft2.changeControlMode(CANTalon.TalonControlMode.Follower); //makes the controllers followers
		ControllerRight2.changeControlMode(CANTalon.TalonControlMode.Follower);
		
		ControllerLeft2.set(ControllerLeft1.getDeviceID()); //tells them who to follow
		ControllerRight2.set(ControllerRight1.getDeviceID());
		
		ControllerLeft1.configNominalOutputVoltage(+0f, -0f);
		ControllerRight1.configNominalOutputVoltage(+0f, -0f);
		
		ControllerLeft1.configPeakOutputVoltage(+4f, -4f);
		ControllerRight1.configPeakOutputVoltage(+4f, -4f);

		// new device and inverted needs validation 2/18 TRO
		ControllerRight1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		//ControllerRight1.reverseOutput(true);
		//ControllerRight1.reverseSensor(true);
		ControllerRight1.configEncoderCodesPerRev(1);
		
		ControllerLeft1.enable();
		ControllerRight1.enable();
		
		
		steamDrive = new RobotDrive(ControllerLeft1, ControllerRight1);
		steamDrive.setSafetyEnabled(false);
		
//		SmartDashboard.putString("DriveSytem", "ConstructorMethod");
	    
		driveDistance = new PIDController(0.7, 0.0, 0.0, new EncoderWrapper(), new DriveDistancePID());
		m_dThrottleDirection = 1;
		m_bThrottleSwitch = false;
		m_dVoltageRampRate = 3.0;
		m_dJoystickVolts = 8.0;
		
		m_dWheelDiameter = 4.0;
		m_dEncoderPulseCnt = 1024;
		m_dDriveP = 0.2;
		m_dDriveI = 0.0;
		m_dDriveD = 0.0;
		m_iDriveError = 20;
		
		m_dGyroP = 0.7;
		m_dGyroI = 0.0;
		m_dGyroD = 0.0;
		m_iGyroError = 2;
		
		m_dStraightP = 0.2;
		m_dStraightI = 0.0;
		m_dStraightD = 0.0;
		m_iStraightError = 1;
		turnToAngle = new PIDController(0.0, 0.0, 0.0, new getAngle(), new turnAngle());
		
		navxGyro = new AHRS(SPI.Port.kMXP);
		
		kCollisionThreshold_DeltaG = 0.5f;
		
		m_dSteeringHeading = 0;
		keepAngle = new PIDController(0.0, 0.0, 0.0, new getAngle(), new HeadingCapture());
	}
	
	public void ArcadeDrive(double stickX, double stickY){
		stickY = stickY * m_dThrottleDirection;
//		stickX = stickX * m_dThrottleDirection; tried this and it is backwards when reversed 
		steamDrive.arcadeDrive(stickX, stickY);		
		
		logDashboard(stickY, stickX);
		Robot.visionSystem.VisionNTDisplay();
	}
	
	public void ToggleEnds()  {
		ArcadeDrive(0,0);
		m_bThrottleSwitch = false;
		if (ControllerRight1.getSpeed() > -5 && ControllerRight1.getSpeed() < 5 ) {
			m_dThrottleDirection = m_dThrottleDirection * -1;
			m_bThrottleSwitch = true;
		}
	}
	
	public void GearForward()  {
		ArcadeDrive(0,0);
		m_bThrottleSwitch = false;
		if (ControllerRight1.getSpeed() > -5 && ControllerRight1.getSpeed() < 5 ) {
			m_dThrottleDirection = 1;
			m_bThrottleSwitch = true;
		}
	}
	
	
	public void ShooterForward()  {
		ArcadeDrive(0,0);
		m_bThrottleSwitch = false;
		if (ControllerRight1.getSpeed() > -5 && ControllerRight1.getSpeed() < 5 ) {
			m_dThrottleDirection = -1;
			m_bThrottleSwitch = true;
		}
	}
	
	public boolean isToggled()  {
		return m_bThrottleSwitch;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ArcadeDrive());
    }
    
    public void initJoystickDrive() {
    	ControllerRight1.enableLimitSwitch(false, false);
    	ControllerRight1.setVoltageRampRate(m_dVoltageRampRate);
    	ControllerRight1.configNominalOutputVoltage(-+0.0, -0.0);
    	ControllerRight1.configPeakOutputVoltage(m_dJoystickVolts, -m_dJoystickVolts);
    	ControllerRight1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	ControllerRight1.enable();
    	ControllerLeft1.enableLimitSwitch(false, false);
    	ControllerLeft1.setVoltageRampRate(m_dVoltageRampRate);
    	ControllerLeft1.configNominalOutputVoltage(-+0.0, -0.0);
    	ControllerLeft1.configPeakOutputVoltage(m_dJoystickVolts, -m_dJoystickVolts);
    	ControllerLeft1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	ControllerLeft1.enable();
    }
    
    public void initEncoder (double distance){
    	initEncoder(distance, 1.0);    	
    }
    
    public void initEncoder (double distance, double throttle){
    	driveDistance.reset();
    //	m_dDistance = distance;
    	double rotations = distance / (m_dWheelDiameter * Math.PI) * m_dEncoderPulseCnt;
    	driveDistance.setAbsoluteTolerance(m_iDriveError);
    	ControllerRight1.setPosition(0.0);
    	driveDistance.setInputRange(-Math.abs(rotations) * 1.5, Math.abs(rotations) * 1.5);
    	driveDistance.setOutputRange(-throttle, throttle);
    	driveDistance.setPID(m_dDriveP, m_dDriveI, m_dDriveD);
    	driveDistance.setSetpoint(rotations);
    	m_dRotations = rotations;
    	driveDistance.enable();
    	dashboardDisplay();
    }
    
    public void disableEncoder () {
    	driveDistance.disable();
    }
    
    public boolean encoderOnTarget() {
		return driveDistance.onTarget();
		//return driveDistance.onTarget() || isBumped();//
    }
    
    public void resetRightEncoder() {
    	ControllerRight1.setPosition(0.0);
    }
    
    public double getGyroAngle() {
    	double navxGyroAngle = navxGyro.getAngle();
    	return navxGyroAngle;
    }
    
    public void disableGyro() {
    	turnToAngle.disable();
    }
    
    public void turnToAngle(double angle) {
    	turnToAngle.reset();
    	navxGyro.zeroYaw();
    	turnToAngle.setInputRange(-180.0f, 180.0f);
    	turnToAngle.setOutputRange(-1.0, 1.0);
    	turnToAngle.setPID(m_dGyroP, m_dGyroI, m_dGyroD);
    	turnToAngle.setAbsoluteTolerance( m_iGyroError);
    	turnToAngle.setContinuous(true);
    	turnToAngle.setSetpoint(angle);
    	turnToAngle.enable();
    }
    
    public void stayTrue(double throttle){
 	   ArcadeDrive(m_dSteeringHeading, throttle);
    }
    
    public void keepHeading() {
    	keepAngle.reset();
    	navxGyro.zeroYaw();
    	keepAngle.setInputRange(-180.0f, 180.0f);
    	keepAngle.setOutputRange(-1.0, 1.0);
    	keepAngle.setPID(m_dStraightP, m_dStraightI, m_dStraightD);
    	keepAngle.setAbsoluteTolerance(m_iStraightError);
    	keepAngle.setContinuous(true);
    	keepAngle.setSetpoint(0.0);
    	keepAngle.enable();
    }
    
    public void disableKeepAngle() {
 	   keepAngle.disable();
    }
    
    public boolean gyroOnTarget() {
    	return turnToAngle.onTarget();
    }
    
    public void dashboardDisplay() {
    	SmartDashboard.putNumber("WheelDiameter", m_dWheelDiameter);
		SmartDashboard.putNumber("dDriveP", m_dDriveP);
		SmartDashboard.putNumber("dDriveI", m_dDriveI);
		SmartDashboard.putNumber("dDriveD", m_dDriveD);
		SmartDashboard.putNumber("DriveError", m_iDriveError);
		SmartDashboard.putNumber("Distance", m_dDistance);
		SmartDashboard.putNumber("Rotations", m_dRotations);
    }
    
    public double dashboardFetch() {
    	m_dWheelDiameter = SmartDashboard.getNumber("WheelDiameter", m_dWheelDiameter);
		m_dDriveP = SmartDashboard.getNumber("dDriveP", m_dDriveP);
		m_dDriveI = SmartDashboard.getNumber("dDriveI", m_dDriveI);
		m_dDriveD = SmartDashboard.getNumber("dDriveD", m_dDriveD);
		m_iDriveError = (int) SmartDashboard.getNumber("DriveError", m_iDriveError);
		m_dDistance = SmartDashboard.getNumber("Distance", m_dDistance);
		m_dRotations = SmartDashboard.getNumber("Rotations", m_dRotations);
		return m_dDistance;
    }
    
    public void dashboardGyroDisplay() {
		SmartDashboard.putNumber("dGyroP", m_dGyroP);
		SmartDashboard.putNumber("dGyroI", m_dGyroI);
		SmartDashboard.putNumber("dGyroD", m_dGyroD);
		SmartDashboard.putNumber("GyroError", m_iGyroError);
		SmartDashboard.putNumber("Angle", m_dAngle);
    }
  
    public double dashboardGyroFetch() {
		m_dGyroP = SmartDashboard.getNumber("dGyroP", m_dGyroP);
		m_dGyroI = SmartDashboard.getNumber("dGyroI", m_dGyroI);
		m_dGyroD = SmartDashboard.getNumber("dGyroD", m_dGyroD);
		m_iGyroError = (int) SmartDashboard.getNumber("GyroError", m_iGyroError);
		m_dAngle = SmartDashboard.getNumber("Angle", m_dAngle);
		return m_dAngle;
    }
    
    public boolean isBumped() {
    	boolean collisionDetected = false;
    	double currLinearAccelX = navxGyro.getWorldLinearAccelX();
    	double currentJerkX = currLinearAccelX - lastLinearAccelX;
    	lastLinearAccelX = currLinearAccelX;
    	double currLinearAccelY = navxGyro.getWorldLinearAccelY();
    	double currentJerkY = currLinearAccelY - lastLinearAccelY;
    	lastLinearAccelY = currLinearAccelY;
    	
    	if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG  ) ||
    	   ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
    		   collisionDetected = true;
    	   }
    	return collisionDetected;
    }
   
   
    
    public void logDashboard(double Y, double X){
    	
    	SmartDashboard.putNumber("DriverStickY", Y);
    	SmartDashboard.putNumber("DriverStickX", X);
    	//SmartDashboard.putNumber("Left1Temp", ControllerLeft1.getTemperature());
    	//SmartDashboard.putNumber("Left1Amps", ControllerLeft1.getOutputCurrent());
    	//SmartDashboard.putNumber("Left1VFactor", ControllerLeft1.getOutputVoltage()/ControllerLeft1.getBusVoltage());
    	//SmartDashboard.putNumber("LeftVBus", ControllerLeft1.getBusVoltage());
    	//SmartDashboard.putNumber("LeftVOut", ControllerLeft1.getOutputVoltage());
    	
    	
    	//SmartDashboard.putNumber("Right1Temp", ControllerRight1.getTemperature());
    	//SmartDashboard.putNumber("Right1Amps", ControllerRight1.getOutputCurrent());
    	SmartDashboard.putNumber("Right1Encoder", ControllerRight1.getSpeed());
    	SmartDashboard.putNumber("RightEncoderPos", ControllerRight1.getPosition());
    	SmartDashboard.putNumber("DriveYToggle", m_dThrottleDirection);
    	SmartDashboard.putNumber("GyroAngle", navxGyro.getAngle());
    	
    	//SmartDashboard.putNumber("lastLinearAccelX", navxGyro.getWorldLinearAccelX());
    	//SmartDashboard.putNumber("lastLinearAccelY", navxGyro.getWorldLinearAccelY());
    	
    	//SmartDashboard.putNumber("X Displacement", navxGyro.getDisplacementX());
    	//SmartDashboard.putNumber("Y Displacement", navxGyro.getDisplacementY());
    	//SmartDashboard.putNumber("Z Displacement", navxGyro.getDisplacementZ());
    	
    	SmartDashboard.putNumber("Jerk X Value", currentJerkX);
    	SmartDashboard.putNumber("Jerk Y Value", currentJerkY);
    	SmartDashboard.putBoolean("IsBumped", Robot.driveSystem.isBumped());
    	SmartDashboard.putNumber("RightEncoderPID", ControllerRight1.pidGet());
    }
   
    private class EncoderWrapper implements PIDSource{

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			// TODO Auto-generated method stub
			return ControllerRight1.getPosition();
		}
    	
    }
    
    private class DriveDistancePID implements PIDOutput{

		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			ArcadeDrive(0, output);
		}
    	
    }
   
    private class getAngle implements PIDSource {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			// TODO Auto-generated method stub
			return getGyroAngle();
		}			
    }
    
    private class turnAngle implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			ArcadeDrive(output, 0);
		}
    }
    
    private class HeadingCapture implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			m_dSteeringHeading = output;
		}
    	
    }
}

