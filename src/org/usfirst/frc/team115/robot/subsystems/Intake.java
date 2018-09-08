package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem 
{
	private DoubleSolenoid intake;
	public DigitalInput breakbeam;
	public double setPoint;
	public double rotations;
	
	public boolean enabled = false;
	
	public enum RotateState {
		ROTATEOUT,
		ROTATEIN,
		HOLD,
		DISABLED
	};
	
	public RotateState currState = RotateState.DISABLED;
	
	public Intake()  
	{

		Hardware.left = new TalonSRX(Constants.talonLeftIntake);	
		Hardware.right = new TalonSRX(Constants.talonRightIntake);  
		Hardware.rot = new TalonSRX(Constants.talonRotateIntake);     
		
		/* first choose the sensor */
		Hardware.rot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.rot.setSensorPhase(true);
		Hardware.rot.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		Hardware.rot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		Hardware.rot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* set the peak and nominal outputs */
		Hardware.rot.configNominalOutputForward(0, Constants.kTimeoutMs);
		Hardware.rot.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Hardware.rot.configPeakOutputForward(0.5, Constants.kTimeoutMs);
		Hardware.rot.configPeakOutputReverse(-0.5, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		Hardware.rot.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		Hardware.rot.config_kF(0, 0.2, Constants.kTimeoutMs);
		Hardware.rot.config_kP(0, 0.2, Constants.kTimeoutMs);
		Hardware.rot.config_kI(0, 0, Constants.kTimeoutMs);
		Hardware.rot.config_kD(0, 0, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		Hardware.rot.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		Hardware.rot.configMotionAcceleration(6000, Constants.kTimeoutMs);
		/* zero the sensor */
		Hardware.rot.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		new Notifier(new Runnable()
		{
			@Override
			public void run()
			{
				loop();
			}
		}).startPeriodic(0.005);
	}
	
	public void intakeCube() 
	{
		Hardware.left.set(ControlMode.PercentOutput, -0.65);
		Hardware.right.set(ControlMode.PercentOutput, 0.65);
	}
	
	public void outtakeCube() 
	{
		Hardware.left.set(ControlMode.PercentOutput, 0.65);
		Hardware.right.set(ControlMode.PercentOutput, -0.65);
	}
	
	public void rotateIn()
	{
		currState = RotateState.ROTATEIN;
	}
	
	public void rotateOut()
	{
		currState = RotateState.ROTATEOUT;
	}
	
	public void loop()
	{
		switch(currState)
		{
		
		case DISABLED:
			
			if(enabled)
			{
				currState = RotateState.HOLD;
			}
			
		case ROTATEOUT:
			
			
			if(Hardware.rot.getSelectedSensorPosition(0) > Constants.lowBufferIntakeOut)
			{
				stop();
				currState = RotateState.HOLD;
			}
			else
			{
				Hardware.rot.set(ControlMode.MotionMagic, setPoint);
			}
	
		case ROTATEIN:
			
			if(Hardware.rot.getSelectedSensorPosition(0) < Constants.lowBufferIntakeIn)
			{
				stop();
				currState = RotateState.HOLD;
			}
			else
			{
				Hardware.rot.set(ControlMode.MotionMagic, setPoint);
			}
			
		}
	}
	
	public void stop()  
	{
		Hardware.left.set(ControlMode.PercentOutput, 0);
		Hardware.right.set(ControlMode.PercentOutput, 0);
		Hardware.rot.set(ControlMode.PercentOutput, 0);
	}
	
	public void initDefaultCommand()  
	{
		
	}
}

