import processing.serial.*;

Serial myPort;
float x_accel, y_accel, z_accel;
float x_accel_df, y_accel_df, z_accel_df;
float x_accel_kalman, y_accel_kalman, z_accel_kalman;
int lf = 10;
String inString;
String[] data;


void setup() {
  size(1250,800,P3D);
  noStroke();
  colorMode(RGB, 256);
  
  myPort = new Serial(this, "/dev/tty.usbmodem1422", 115200);
  myPort.clear();
  myPort.bufferUntil(lf);
}

void draw_rect_prism(int r, int g, int b) {
  scale(90);
  fill(r, g, b);
  //noFill();
  beginShape(QUADS);
 
  vertex(-1,  1.5,  0.25);
  vertex( 1,  1.5,  0.25);
  vertex( 1, -1.5,  0.25);
  vertex(-1, -1.5,  0.25);

  vertex( 1,  1.5,  0.25);
  vertex( 1,  1.5, -0.25);
  vertex( 1, -1.5, -0.25);
  vertex( 1, -1.5,  0.25);

  vertex( 1,  1.5, -0.25);
  vertex(-1,  1.5, -0.25);
  vertex(-1, -1.5, -0.25);
  vertex( 1, -1.5, -0.25);

  vertex(-1,  1.5, -0.25);
  vertex(-1,  1.5,  0.25);
  vertex(-1, -1.5,  0.25);
  vertex(-1, -1.5, -0.25);

  vertex(-1,  1.5, -0.25);
  vertex( 1,  1.5, -0.25);
  vertex( 1,  1.5,  0.25);
  vertex(-1,  1.5,  0.25);

  vertex(-1, -1.5, -0.25);
  vertex( 1, -1.5, -0.25);
  vertex( 1, -1.5,  0.25);
  vertex(-1, -1.5,  0.25);

  endShape(CLOSE);
}

void draw() {
  background(0);
  lights();
  
  pushMatrix();
  translate(width/4, height/1.5, -50);
  rotateX(-x_accel -(PI/2));
  rotateY(-y_accel);
  draw_rect_prism(249, 250, 50);
  popMatrix();
  
  pushMatrix();
  translate(width/2, height/1.5, -50);
  rotateX(-x_accel_df -(PI/2));
  rotateY(-y_accel_df);
  draw_rect_prism(255, 0, 0);
  popMatrix();
  
  pushMatrix();
  translate(width*3/4, height/1.5, -50);
  rotateX(-x_accel_kalman -(PI/2));
  rotateY(-y_accel_kalman);
  draw_rect_prism(0, 255, 0);
  popMatrix();
  
  String accStr = "(" + x_accel + ", " + y_accel + ")";
  String dfStr = "(" + x_accel_df + ", " + y_accel_df + ")";
  String kalmanStr = "(" + x_accel_kalman + ", " + y_accel_kalman + ")";
  
  fill(249, 250, 50);
  text("Raw Accel Data", (int) width/4.0 - 100, 25);
  text(accStr, (int) (width/4.0) - 100, 50);
  
  fill(255, 0, 0);
  text("Digital Filtered Accel Data (Avg of past 5 measurements)", (int) width/2.0 - 150, 25);
  text(dfStr, (int) (width/2.0) - 150, 50);
  
  fill(0, 255, 0);
  text("Kalman Filtered Data (Accel + Gyro)", (int) (width*3/4) - 40, 25);
  text(kalmanStr, (int) (width*3/4) - 40, 50);
  

}

void serialEvent(Serial p) {
  inString = p.readString();
  try {
    data = split(inString, " ");
    if (data[0].equals("Raw:") && data.length == 4){
      x_accel = float(data[1])/100;
      y_accel = float(data[2])/100;
    }
    else if (data[0].equals("Df:") && data.length == 4){
      x_accel_df = float(data[1])/100;
      y_accel_df = float(data[2])/100;
    }
    else if (data[0].equals("Kalman:") && data.length == 4){
      x_accel_kalman = float(data[1])/100;
      y_accel_kalman = float(data[2])/100;
    }
  }
  catch (Exception E){
    println("Error in Serial Data");
  }
} 
