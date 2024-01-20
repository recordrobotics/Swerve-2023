package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavSensor extends SubsystemBase {
	AHRS _nav;


	public NavSensor() {

		//_nav = new AHRS(I2C.Port.kMXP);
		_nav = new AHRS(SerialPort.Port.kUSB1);

		_nav.reset();
		_nav.resetDisplacement();

		//_nav.enableBoardlevelYawReset​(true);
	}

	public double getPitch() {
		//System.out.println("pitch: " + _nav.getRoll());
		double pitch = _nav.getRoll();
		return Units.degreesToRadians(pitch);
	}

	public double getRoll() {
		double roll = _nav.getPitch();
		return Units.degreesToRadians(-1*roll);
	}

	public double getYaw() {
		double yaw = _nav.getYaw();
		return Units.degreesToRadians(-1*yaw);
	}

    public double getAngle() {
        double angle = _nav.getAngle();
        return Units.degreesToRadians(angle);
    }

	//None of the below are guarenteed to work (weird axis changes)

	public double getDisplacementX() {
		return _nav.getDisplacementX();
	}

	public double getDisplacementY() {
		return _nav.getDisplacementY();
	}

	public double getDisplacementZ() {
		return _nav.getDisplacementZ();
	}

	void resetAngle() {
		_nav.reset();
	}

	void resetDisplacement() {
		_nav.resetDisplacement();
	}

	void resetAll(){
		resetAngle();
		resetDisplacement();
	}
}