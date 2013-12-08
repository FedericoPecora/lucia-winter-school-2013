package se.oru.lucia.rfidlocalization;

public class Robot {

	private float X;
	private float Y;
	private float bearing; 
	private float radius;
	private float readRadius; 
	private String name = null;
	
	public Robot(float r, float rr, String n) {
		radius = r;
		readRadius = rr;
		name = n;
	}
		
	public String getName() {
		return name;
	}

	public void setX(float x) {
		X = x;
	}

	public float getX() {
		return X;
	}

	public void setY(float y) {
		Y = y;
	}

	public float getY() {
		return Y;
	}

	public void setBearing(float bearing) {
		this.bearing = bearing;
	}

	public float getBearing() {
		return bearing;
	}

	public float getRadius() {
		return radius;
	}

	public float getReadRadius() {
		return readRadius;
	}

}
