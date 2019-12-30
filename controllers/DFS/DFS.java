import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.Compass;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

class Point
{
	double x;
	double z;

	public Point(double x, double z)
	{
		this.x = x;
		this.z = z;
	}

	public double getLength()
	{
		return Math.sqrt(x * x + z * z);
	}

	public void normalize()
	{
		double len = this.getLength();
		this.x /= len;
		this.z /= len;
	}

	public static Point minus(Point p1, Point p2)
	{
		return new Point(p1.x - p2.x, p1.z - p2.z);
	}

}

public class DFS
{
	static double angle(Point p1, Point p2)
	{
		return modDouble(Math.atan2(p2.z, p2.x) - Math.atan2(p1.z, p1.x), 2.0 * Math.PI);
	}

	static double modDouble(double a, double m)
	{
		double div = Math.floor(a / m);
		double r = a - div * m;
		if(r < 0.0) r += m;
		return r;
	}

	final static double TOLERANCE = 0.05;
	final static double MAX_SPEED = Math.PI * 2;
	static ArrayList<Point> points = new ArrayList<>();
	



	public static void main(String[] args) throws IOException
	{

		File file_xzOut = new File(new File(new File(DFS.class.getResource("").getPath()).getParentFile().getParentFile(), "algo"), "xzOut.txt");
		Scanner scanner = new Scanner(file_xzOut);
		while (scanner.hasNext())
		{
			double x = scanner.nextDouble();
			double z = scanner.nextDouble();
			points.add(new Point(x, z));
		}
		scanner.close();

		// create the Robot instance.
		Robot robot = new Robot();

		// get the time step of the current world.
		int timeStep = (int) Math.round(robot.getBasicTimeStep());

		Motor leftMotor = robot.getMotor("left wheel motor");
		Motor rightMotor = robot.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		
		// initialize devices
		Keyboard keyboard = new Keyboard();
		keyboard.enable(timeStep);
		GPS gps = new GPS("gps");
		gps.enable(timeStep);
		
		Compass compass = new Compass("compass");
		compass.enable(timeStep);

		int curTargetPointInd = 0;

		while (robot.step(timeStep) != -1)
		{
			double leftSpeed = 0.0;
			double rightSpeed = 0.0;

			if(curTargetPointInd < points.size())
			{


				double[] gps_values = gps.getValues();
				double curX = gps_values[0];
				double curZ = gps_values[2];
				double targetX = points.get(curTargetPointInd).x;
				double targetZ = points.get(curTargetPointInd).z;
				Point moveVec = new Point(targetX - curX, targetZ - curZ);
				if(moveVec.getLength() < TOLERANCE)
				{
					++curTargetPointInd;
				}
				else
				{
					double[] compass_values = compass.getValues();
					double compassX = compass_values[0];
					double compassZ = compass_values[2];
					moveVec.normalize();
					Point front = new Point(-compassX, compassZ);
					double beta = angle(front, moveVec) - Math.PI;
					
					if(beta < -0.05)
					{
						leftSpeed = -MAX_SPEED;
						rightSpeed = MAX_SPEED;
					}
					else if(beta > 0.05)
					{
						leftSpeed = MAX_SPEED * beta / Math.PI;
						rightSpeed = -MAX_SPEED * beta / Math.PI;
					}
					else
					{
						leftSpeed = MAX_SPEED - Math.PI + beta;
						rightSpeed = MAX_SPEED - Math.PI - beta;
					}
				}

			}
			leftMotor.setVelocity(leftSpeed);
			rightMotor.setVelocity(rightSpeed);
		}

	}
}
