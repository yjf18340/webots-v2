import com.cyberbotics.webots.controller.Robot;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.Compass;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;
import java.util.Scanner;

import com.cyberbotics.webots.controller.DistanceSensor;

class Point
{
	double x;
	double z;

	public Point(double x, double z)
	{
		super();
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

public class Heuristic
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

	final static double DS_VALUE = 512.0;
	final static double DS_VALUE_2 = 1024.0;
	static final double DSIT_TOLERANCE = 0.08;
	final static double MAX_SPEED = Math.PI * 2;

	static ArrayList<Point> points = new ArrayList<>();
	static Random random = new Random();
	static boolean meetObstacle = false;
	static int bothCnt = 0;
	static int cntRand = 60;

	public static void main(String[] args) throws IOException
	{

		File file_xzOut = new File(
				new File(new File(Heuristic.class.getResource("").getPath()).getParentFile().getParentFile(),
						"algo"),
				"xzOut.txt");
		Scanner scanner = new Scanner(file_xzOut);
		while (scanner.hasNext())
		{
			double x = scanner.nextDouble();
			double z = scanner.nextDouble();
			points.add(new Point(x, z));
		}
		scanner.close();

		Robot robot = new Robot();

		int timeStep = (int) Math.round(robot.getBasicTimeStep());

		Motor leftMotor = robot.getMotor("left wheel motor");
		Motor rightMotor = robot.getMotor("right wheel motor");

		DistanceSensor[] ds = new DistanceSensor[8];
		for (int i = 0; i < 8; i++)
		{
			ds[i] = robot.getDistanceSensor("ds" + i);
			ds[i].enable(timeStep);
		}

		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);

		// initialize devices
		Keyboard keyboard = new Keyboard();
		keyboard.enable(timeStep);
		GPS gps = new GPS("gps");
		gps.enable(timeStep);
		Compass compass = new Compass("compass");
		compass.enable(timeStep);

		final Point SRC_POINT = points.get(0);
		final Point TAR_POINT = points.get(points.size() - 1);
		Point curTarget = new Point(TAR_POINT.x, TAR_POINT.z);

		while (robot.step(timeStep) != -1)
		{

			double leftSpeed = 0.0, rightSpeed = 0.0;
			{
				double[] gps_values = gps.getValues();
				double curX = gps_values[0];
				double curZ = gps_values[2];
				final Point targetVec = new Point(TAR_POINT.x - curX, TAR_POINT.z - curZ);
				if(targetVec.getLength() >= DSIT_TOLERANCE)
				{

					double[] dsValues = { 0, 0, 0, 0, 0, 0, 0, 0 };
					for (int i = 0; i < 8; i++)
					{
						dsValues[i] = ds[i].getValue();
					}

					// detect obstacles
					boolean left_obstacle = dsValues[7] > DS_VALUE || dsValues[6] > DS_VALUE_2;
					boolean right_obstacle = dsValues[0] > DS_VALUE || dsValues[1] > DS_VALUE_2;

					if(left_obstacle && right_obstacle || bothCnt > 0)
					{
						System.out.println("bC: " + bothCnt);
						// turn right
						leftSpeed += 0.2 * MAX_SPEED;
						rightSpeed -= 0.2 * MAX_SPEED;
						meetObstacle = true;
						++bothCnt;

						if(bothCnt >= cntRand)
						{
							bothCnt = 0;
							cntRand = 40 + random.nextInt(40);
						}
					}
					else if(left_obstacle)
					{
						System.out.println("left");
						// turn right
						leftSpeed += 0.2 * MAX_SPEED;
						rightSpeed -= 0.2 * MAX_SPEED;
						meetObstacle = true;

					}
					else if(right_obstacle)
					{
						System.out.println("right");
						// turn left
						leftSpeed -= 0.2 * MAX_SPEED;
						rightSpeed += 0.2 * MAX_SPEED;
						meetObstacle = true;
					}
					else
					{

						if(meetObstacle)
						{
							meetObstacle = false;
							double goTowardTar = random.nextDouble();
							double coeX = random.nextDouble();
							double coeZ = random.nextDouble();
							double newX = 0.0;
							double newZ = 0.0;
							if(goTowardTar <= 0.1)
							{
								newX = coeX * SRC_POINT.x + (1 - coeX) * TAR_POINT.x;
								newZ = coeZ * SRC_POINT.z + (1 - coeZ) * TAR_POINT.z;
							}
							else if(goTowardTar <= 0.35)
							{
								double augX = 2 * curX - TAR_POINT.x - 0.1;
								double augZ = 2 * curZ - TAR_POINT.z - 0.1;
								newX = coeX * augX + (1 - coeX) * TAR_POINT.x;
								newZ = coeZ * augZ + (1 - coeZ) * TAR_POINT.z;
							}
							else
							{
								newX = coeX * curX + (1 - coeX) * TAR_POINT.x;
								newZ = coeZ * curZ + (1 - coeZ) * TAR_POINT.z;
							}
							curTarget.x = newX;
							curTarget.z = newZ;

						}

						Point moveVec = new Point(curTarget.x - curX, curTarget.z - curZ);
						if(moveVec.getLength() < 0.05)
						{
							double coeX = random.nextDouble();
							double coeZ = random.nextDouble();
							double newX = coeX * curX + (1 - coeX) * TAR_POINT.x;
							double newZ = coeZ * curZ + (1 - coeZ) * TAR_POINT.z;
							curTarget.x = newX;
							curTarget.z = newZ;
							moveVec = new Point(curTarget.x - curX, curTarget.z - curZ);
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
				}
			}
			leftMotor.setVelocity(leftSpeed);
			rightMotor.setVelocity(rightSpeed);
		}

	}
}
