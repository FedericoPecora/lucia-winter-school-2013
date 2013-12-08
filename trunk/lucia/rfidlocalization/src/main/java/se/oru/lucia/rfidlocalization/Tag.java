package se.oru.lucia.rfidlocalization;

public class Tag implements Comparable {
	private float X;
	private float Y;
	private int readorder;
	private String id = null;
	private int gridX;
	private int gridY;
	private float influence;
	public Tag(float x, float y, int read, String id, int gX, int gY) {
		X = x;
		Y = y;
		readorder = read;
		this.id = id;
		gridX = gX;
		gridY = gY;
		influence = 8.0f;
	}
	public float getInfluence() { return influence;	}
	public float getX() { return X; }
	public float getY() { return Y; }
	public int getGridX() { return gridX; }
	public int getGridY() { return gridY; }
	public int getReadorder() { return readorder; }
	public String getID() { return id; }
	public void setID(String newID) { this.id = newID; }
	
	public String toString() {
		return (id + " (" + X + "," + Y + ") " + "[" + readorder + "]");
	}

	@Override
	public int compareTo(Object arg0) {
		if (arg0 instanceof Tag) {
			return (this.readorder - ((Tag)arg0).readorder); 
		}
		return 0;
	}
}
