// File: robot_controller.cpp
// Date: 
// Description: EN2532 Robot Design & Competition 
// Author: Robocop
// Modifications:

//openCV import
#include <opencv2/imgproc/imgproc.hpp>
//Webots include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
//CPP import
#include <iostream>
#include <fstream>
#include <string>
 
//constants
#define TIME_STEP 16
#define MAX_SPEED 8 
#define NORM_SPEED 5
#define Kp 1.45
#define Kd 0.3
#define HALF_WALL_GAP 200
#define SONAR_TRESH 123


//name spaces
using namespace webots;
using namespace std;
using namespace cv;

//Objects
Robot *robot;
Motor *motor_l;
Motor *motor_r;
Motor *servo_arm;
Motor *servo_rotate;
Motor *servo_l;
Motor *servo_r;
Motor *solenoid;
DistanceSensor *IR[6];
PositionSensor *encoder_l;
PositionSensor *encoder_r;
PositionSensor *encoder_arm;
PositionSensor *encoder_rotate;
PositionSensor *encoder_grip;
DistanceSensor *DS[3];  // left, front, right
Camera *cm;

//variables
char IR_names[6][4] = {"IR1", "IR2", "IR3", "IR4", "IR5", "IR6"};
char DS_names[3][5] = {"DS_L", "DS_F", "DS_R"};
double encoder_l_reading, encoder_r_reading, encoder_arm_reading, encoder_grip_reading, encoder_rotate_reading;
int previous_error = 0;
int pid_error;
double error_maze;
string colour;
double DS_readings[3] = {};
int width;
int height;
int IR_TRESH = 400;


bool pick_ob;
int rect_width;


//function protos
void delay(float time);
float PID_CV();
void line_follow(); 
void dash_line_follow();
void read_DS();
void go_ahead();
void revese();
void turn_right(short int degree=0);
void turn_left(short int degree=0);
void turn_back();
void stop();
void goto_next_square(float delay);
void wall_follow();
void arm(bool action);
void rotate(double angle);
void grip(bool action, double k);
void kick(double pos=0.035);
bool get_colour();
int getcon1(Mat imgdil, Mat img, string mode);
int getcon(Mat imgdil, Mat img, string mode);
void disp(const char *disp, Mat img);
void colorf(Mat img);
Mat color_extract(Mat img, string color);
int ob_det(string mode);
void pick_place();

int main(int argc, char **argv) {
  //Robot instance.
  robot = new Robot();
  //DC Motors
  motor_l = robot->getMotor("L_motor");
  motor_r = robot->getMotor("R_motor");  
  motor_l->setPosition(INFINITY);
  motor_r->setPosition(INFINITY);
  motor_l->setVelocity(0.0);
  motor_r->setVelocity(0.0);
  
  //Servo Motors
  servo_arm = robot->getMotor("servo_1");
  servo_l = robot->getMotor("servo_2");
  servo_r = robot->getMotor("servo_3");
  servo_rotate = robot->getMotor("servo_4");
  servo_arm->setPosition(INFINITY);
  servo_l->setPosition(INFINITY);
  servo_r->setPosition(INFINITY);
  servo_rotate->setPosition(INFINITY);
  servo_arm->setVelocity(0.0);
  servo_l->setVelocity(0.0);
  servo_r->setVelocity(0.0);
  servo_rotate->setVelocity(0.0);
  
  //Solenoid
  solenoid = robot->getMotor("solenoid");
  solenoid->setPosition(0);
  solenoid->setVelocity(10);
  
  //Encoders
  encoder_l = robot->getPositionSensor("L_encoder");
  encoder_l->enable(TIME_STEP);
  encoder_r = robot->getPositionSensor("R_encoder");
  encoder_r->enable(TIME_STEP);
  
  encoder_arm = robot->getPositionSensor("servo_pos_1");
  encoder_arm->enable(TIME_STEP);
  encoder_grip = robot->getPositionSensor("servo_pos_2");
  encoder_grip->enable(TIME_STEP);
  
  encoder_rotate = robot->getPositionSensor("servo_pos_4");
  encoder_rotate->enable(TIME_STEP);
  
  //IR sensors
  for (int i=0; i<6; i++){
    IR[i] = robot->getDistanceSensor(IR_names[i]);
    IR[i]->enable(TIME_STEP);
  } 
  
  //Ultrasonic sensors
  for (int i=0; i<3; i++){
    DS[i] = robot->getDistanceSensor(DS_names[i]);
    DS[i]->enable(TIME_STEP);
  } 

  // camera
  cm = robot->getCamera("camera");
  cm->enable(TIME_STEP); 
  width = cm->getWidth();
  height = cm->getHeight();
 
  while (robot->step(TIME_STEP) != -1){
    cout << "--------------Robocop--------------" << endl;
    if (get_colour()){
      // rotate(45);

      arm(1);
      line_follow();
      wall_follow();
      pick_place();
      dash_line_follow();
      cout << "[INFO] Process terminated" << endl;

      cout << " __   __   __   __   __   __   __" << endl;
      cout << "/  \\ /  \\ |  \\ /  \\ /  \\ /  \\ /  \\" << endl;
      cout << "|--| |  | |--< |  | |    |  | |--/" << endl;
      cout << "|  \\ \\__/ |__/ \\__/ \\__/ \\__/ |" << endl;

      break;

      // while (robot->step(TIME_STEP) != -1){;
      // }

    }
  }
  delete robot;
  return 0;
}

void delay(float time){
  float current_time = robot->getTime();
  while (robot->step(TIME_STEP) != -1){
    if (robot->getTime() > (current_time + time-0.032)){
      break;
    }
  }
  return;
}

float PID_CV(){
  int IR_values[6];
  float control_val;

  for (int j=0; j<6; j++){
    double IR_reading = IR[j]->getValue();
    // cout << IR_reading << "   "; 
    if (IR_reading > IR_TRESH){
      IR_values[j] = 0;
    }
    else {
      IR_values[j] = -(-5+2*j);
    }
  }
  // cout << endl;
  pid_error = 0;
  for (int i=0; i<6; i++){
    //cout << IR_values[i] << " " ;
    pid_error += IR_values[i];
  }
      
  if (pid_error%2 == 0){
    pid_error /= 2;
  }
  // cout << error << endl;
  control_val = Kp*pid_error + Kd*(pid_error-previous_error);
  previous_error = pid_error;
  return control_val;
}

void line_follow(){
  cout << "[INFO] line follow mode activated" << endl; 
  // CV negative turn left, otherwise turn right
  while (robot->step(TIME_STEP) != -1){
    float CV = PID_CV();
    if (CV<0) {
      motor_l->setVelocity(NORM_SPEED+CV);
      motor_r->setVelocity(NORM_SPEED);
    }
    else if (CV>0) {
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED-CV);
    }
    else {
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED);
    }
    read_DS();
    // Exit line following using distance sensors
  
    if ((DS_readings[0] < HALF_WALL_GAP) and (DS_readings[2] < HALF_WALL_GAP) ){
      cout << "[INFO] line follow -> walls detected. quit the line follow mode" << endl;
      break;
    }

    // (DS_readings[0] < HALF_WALL_GAP) and (DS_readings[2] < HALF_WALL_GAP)



  }
  return;
}

void dash_line_follow(){
  bool state = false;
  cout << "[INFO] line follow mode activated" << endl; 
  // CV negative turn left, otherwise turn right
  while (robot->step(TIME_STEP) != -1){
    float CV = PID_CV();
    if (CV<0) {
      motor_l->setVelocity(NORM_SPEED+CV);
      motor_r->setVelocity(NORM_SPEED);
    }
    else if (CV>0) {
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED-CV);
    }
    else {
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED);
    }
    
    if (state and ((IR[0]->getValue() < 400 ) and (IR[5]->getValue() < 400 ))){
      cout << "[INFO] Infront the goal post" << endl;
      stop();
      if (colour == "blue"){

        encoder_r_reading = encoder_r->getValue();
        while (robot->step(TIME_STEP) != -1){
          motor_r->setVelocity(NORM_SPEED);
          if (encoder_r->getValue() > (encoder_r_reading + 1.1)){
            stop();
            break;
          }
        }
      }
      else {
        encoder_l_reading = encoder_l->getValue();
        while (robot->step(TIME_STEP) != -1){
          motor_l->setVelocity(NORM_SPEED);
          if (encoder_l->getValue() > (encoder_l_reading + 1.1)){
            stop();
            break;
          }
        }
      }
      arm(0);
      kick(0.03);
      cout << "[INFO] ball kicked" << endl;
      break;
    }
// 682 = blue
    bool var = (((IR[0]->getValue() > 660 and IR[0]->getValue() < 750) or (IR[1]->getValue() > 660 and IR[1]->getValue() < 750)) and ( (IR[4]->getValue() < 400) or (IR[5]->getValue() < 400)));
    if (var){
      state = true;
      
      if (colour == "blue") {
      IR_TRESH = 750;
      motor_l->setVelocity(NORM_SPEED*1.9);
      motor_r->setVelocity(NORM_SPEED);
      cout << "[INFO] blue line selected" << endl;
      delay(0.8);
      
      }
      else {
        motor_r->setVelocity(NORM_SPEED*1.9);
        motor_l->setVelocity(NORM_SPEED);
        cout << "[INFO] red line selected" << endl;
        delay(0.5);
        
      }     
    }
  }
  return;
}

void read_DS(){
    for (int i=0; i<3; i++){
      DS_readings[i] = DS[i]->getValue();
    }
    // cout << "[INFO]" << "  LS : " << DS_readings[0] << " FS : " << DS_readings[1] << " RS : " << DS_readings[2] << endl;
    return ;
}

void go_ahead(){
  motor_l->setVelocity(NORM_SPEED);
  motor_r->setVelocity(NORM_SPEED);
  return;
}

void revese(){
  motor_l->setVelocity(-NORM_SPEED);
  motor_r->setVelocity(-NORM_SPEED);
  return;
}

void stop(){
  motor_l->setVelocity(0);
  motor_r->setVelocity(0);
  return;
}

void turn_right(short int degree){
  if (degree == 90){
    encoder_l_reading = encoder_l->getValue();
    while (robot->step(TIME_STEP) != -1){
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(-NORM_SPEED);
      if (encoder_l->getValue() > (encoder_l_reading + 3.27)){
        stop();
        break;
      }
    }
  }
  else{
    motor_l->setVelocity(NORM_SPEED);
    motor_r->setVelocity(0);
  }
  return;
}

void turn_left(short int degree){
  if (degree == 90){
    encoder_r_reading = encoder_r->getValue();
    while (robot->step(TIME_STEP) != -1){
      motor_r->setVelocity(NORM_SPEED);
      motor_l->setVelocity(-NORM_SPEED);
      if (encoder_r->getValue() > (encoder_r_reading + 3.27)){
        stop();
        break;
      }
    }
  } 
  else{
    motor_r->setVelocity(NORM_SPEED);
    motor_l->setVelocity(0);
  }
  return;
}

void turn_back(){
  encoder_l_reading = encoder_l->getValue();
  while (robot->step(TIME_STEP) != -1){
    motor_r->setVelocity(-NORM_SPEED);
    motor_l->setVelocity(NORM_SPEED);
    if (encoder_l->getValue() > (encoder_l_reading + 6.6)){
      stop();
      break;
    }
  }
  return;
}

void goto_next_square(float delay){
  go_ahead();
  encoder_l_reading = encoder_l->getValue();
  while (robot->step(TIME_STEP) != -1){
    double DS_readings[3] = {};
    for (int i=0; i<3; i++){
      DS_readings[i] = DS[i]->getValue();
    }
    if ((encoder_l->getValue() > (encoder_l_reading + delay)) or (DS_readings[1]<(HALF_WALL_GAP/2.25))){
      break;
    }
  }
  stop();
  return;
}


void wall_follow(){
  // left, front, right
  cout << "[INFO] wall follow mode activated" << endl;
  while (robot->step(TIME_STEP) != -1){
    // Exit maze using IR reading
    // cout << IR[2]->getValue() << endl;
    if (IR[2]->getValue() < 425){
      cout << "[INFO] wall follow -> successfully exit from the maze" << endl;
      break;
    }
    read_DS();
    // Maze navigation
    if (DS_readings[2] > HALF_WALL_GAP){
      delay(1.3);
      cout << "[INFO] wall follow -> Turn right" << endl;
      turn_right(90);
      go_ahead();
      delay(1);
    }
    else if (DS_readings[1] > HALF_WALL_GAP){
      delay(0.05);
      go_ahead();
    }
    else if (DS_readings[0] > HALF_WALL_GAP){
      delay(0.7);
      cout << "[INFO] wall follow -> Turn left" << endl;
      turn_left(90);
      go_ahead();
      delay(1);
    }
    else {
      cout << "[INFO] wall follow -> Turn back" << endl;
      turn_back(); 
    }
    // Maze eroor correction
    if (DS_readings[2] < SONAR_TRESH){
      error_maze = (SONAR_TRESH-DS_readings[2])/200;
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED+error_maze);
    }
    else if (DS_readings[0] < SONAR_TRESH){
      error_maze = (SONAR_TRESH-DS_readings[0])/200;
      motor_l->setVelocity(NORM_SPEED+error_maze);
      motor_r->setVelocity(NORM_SPEED);
    }
    else if ((HALF_WALL_GAP > DS_readings[0]) and (DS_readings[0] > SONAR_TRESH)){
      error_maze = (DS_readings[0]-SONAR_TRESH)/200;
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED+error_maze);
    }
    else if ((HALF_WALL_GAP > DS_readings[2]) and (DS_readings[2] > SONAR_TRESH)){
      error_maze = (DS_readings[2]-SONAR_TRESH)/200;
      motor_l->setVelocity(NORM_SPEED+error_maze);
      motor_r->setVelocity(NORM_SPEED);
    }
    else {
      error_maze = 0;
      motor_l->setVelocity(NORM_SPEED);
      motor_r->setVelocity(NORM_SPEED);
    }
    
  }
}

void arm(bool action){
  // 0 - down
  // 1 - up
  encoder_arm_reading = encoder_arm->getValue();
  if (action == 1){
    servo_arm->setVelocity(3.5);
    while (robot->step(TIME_STEP) != -1){
      if (encoder_arm->getValue() > (encoder_arm_reading + 1.3)){
        servo_arm->setVelocity(0);
        break;
      }
    }
  }
  else if (action == 0){
    servo_arm->setVelocity(-3.5);
    while (robot->step(TIME_STEP) != -1){
      if (encoder_arm->getValue() < (encoder_arm_reading - 1.3)){
        servo_arm->setVelocity(0);
        break;
      }
    }
  }
}

void rotate(double angle){
  if (angle > 0){
    encoder_rotate_reading = encoder_rotate->getValue();
    servo_rotate->setVelocity(3);
    while (robot->step(TIME_STEP) != -1){
      if (encoder_rotate->getValue() > (encoder_rotate_reading + angle/60)){
        break;
      }
    }
  }
  else if (angle < 0) {
    
    encoder_rotate_reading = encoder_rotate->getValue();
    servo_rotate->setVelocity(-3);
    while (robot->step(TIME_STEP) != -1){
      if (encoder_rotate->getValue() < (encoder_rotate_reading + angle/60)){
        break;
      }
    }
  }
  else return ;
  servo_rotate->setVelocity(0);
}

void grip(bool action, double k){
  // 1 - grip
  encoder_grip_reading = encoder_grip->getValue();
  if (action == 1){
    servo_l->setVelocity(5);
    servo_r->setVelocity(5);
    while (robot->step(TIME_STEP) != -1){
      if (encoder_grip->getValue() > (encoder_grip_reading + 1.55 + k)){
        servo_l->setVelocity(0);
        servo_r->setVelocity(0);
        break;
      }
    }
  }
  else if (action == 0){
    servo_l->setVelocity(-5);
    servo_r->setVelocity(-5);
    while (robot->step(TIME_STEP) != -1){
      if (encoder_grip->getValue() < (encoder_grip_reading - 1.55-k)){
        servo_l->setVelocity(0);
        servo_r->setVelocity(0);
        break;
      }
    }
  }
}

void kick(double pos){
  
  solenoid->setPosition(pos);
  delay(1);
  solenoid->setPosition(0);
  delay(1);
  return;
}

bool get_colour(){
  fstream file;
  file.open("colour.txt",ios::in);
  
  if (file.is_open()){
    getline(file, colour);
    file.close(); 
    if ((colour == "red") or (colour == "blue")){
    cout<<"[INFO] colour sets to "<<colour<<endl;
    return true;
    }
  }
  
  else {
    cout << "[Error] fails to set colour. Retrying...." << endl;
    delay(1);
    return false;
  }
  return false;
}

int getcon1(Mat imgdil, Mat img, string mode){   //take dilated image and original  image and draw contour on original image

  vector<vector<Point>> contours; 
  vector<Vec4i> hierarchy;
  findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  vector<vector<Point>> conPoly(contours.size());
  vector<Rect> boundRect(contours.size());

  for (int i = 0; i< (int)contours.size(); i++){
    float peri = arcLength(contours[i], true);
    approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);

    boundRect[i] = boundingRect(contours[i]);
    int con_size = conPoly[i].size();
    // float asratio = (float)boundRect[i].width/(float)boundRect[i].height;
    if (mode == "object"){
      rect_width = boundRect[i].width;
      float l = peri/(3.1416*rect_width);
      float m = 4*rect_width/peri;
      if(l < 1.1 && l > 0.9) cout << "circle" << endl;
      else if (m < 1.1 && m > 0.9) cout<< "sqaure" << endl;
      //else cout << "nothing" << endl;
      
      rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(250, 100, 250), 2);
      float p_coeff = 0.01;
      float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
      motor_r->setVelocity(5- error_ * p_coeff);
      motor_l->setVelocity(5+error_ * p_coeff);
    }
    else if(mode == "cyl"){     
      if (con_size > 12 && con_size < 15){

        rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(250, 100, 250), 2);

        float p_coeff = 0.005;
        float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
        motor_r->setVelocity(5- error_ * p_coeff);
        motor_l->setVelocity(5+error_ * p_coeff);

      }
    }
    else if(mode == "sq"){
      if (con_size > 6 && con_size < 10){
        rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(250, 100, 250), 2);

        float p_coeff = 0.005;
        float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
        motor_r->setVelocity(5- error_ * p_coeff);
        motor_l->setVelocity(5+error_ * p_coeff);
      }
    }
  }
  return 0;
}


int getcon(Mat imgdil, Mat img, string mode){   //take dilated image and original  image and draw contour on original image

  vector<vector<Point>> contours; 
  vector<Vec4i> hierarchy;
  findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  //drawContours(img, contours, -1, Scalar(100, 0, 255), 5);
  vector<vector<Point>> conPoly(contours.size());
  vector<Rect> boundRect(contours.size());

  for (int i = 0; i< (int)contours.size(); i++){
    //int area = contourArea(contours[i]);
    float peri = arcLength(contours[i], true);
    approxPolyDP(contours[i], conPoly[i], 0.002*peri, true);
    //drawContours(img, conPoly, i, Scalar(100, 0, 255), 2);
    boundRect[i] = boundingRect(contours[i]);
    //cout << conPoly[i].size() << endl;
    float asratio = (float)boundRect[i].width/(float)boundRect[i].height;
    if (mode == "object"){
      if (asratio > 0.9 && asratio < 1.1){
        float w = (float)boundRect[i].width, l = peri/(3.1416*w), m = 4*w/peri;
        if(l < 1.1 && l > 0.9) cout << "circle" << endl;
        else if (m < 1.1 && m > 0.9) cout<< "sqaure" << endl;
        //else cout << "nothing" << endl;
        
        rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(250, 100, 250), 2);

        float p_coeff = 0.01;
        float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
        motor_r->setVelocity(6 - error_ * p_coeff);
        motor_l->setVelocity(6 + error_ * p_coeff);
      }
      else cout << "nothing" << endl;
    }
    else if(mode == "cyl"){
      float w = (float)boundRect[i].width, l = peri/(3.1416*w);// m = 4*w/peri;
      // cout<< l << m << endl;
      if(l < 1.52 && l > 1.46) {
        rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(250, 100, 250), 2);
        float p_coeff = 0;
        float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
        motor_r->setVelocity(- error_ * p_coeff);
        motor_l->setVelocity(+error_ * p_coeff);
        //cout << "cylinder" << endl;
      }
    }
    else if(mode == "sq"){
      float w = (float)boundRect[i].width, m = 4*w/peri;// l = peri/(3.1416*w);
      // cout<< m << l << endl;
      if(m < 0.78 && m > 0.75){
        rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(250, 100, 250), 2);
        float p_coeff = 0.01;
        float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
        motor_r->setVelocity(4- error_ * p_coeff);
        motor_l->setVelocity(4+error_ * p_coeff);
        //cout << "square" << endl;
      }
    }
  }
  return 0;
}

void disp(const char *disp, Mat img){
  //get image BGR and display
  Display *display = robot->getDisplay(disp);

  cvtColor(img, img, COLOR_BGR2RGB);
  ImageRef *ir = display->imageNew(width, height, img.data, Display::RGB);
  display->imagePaste(ir, 0, 0, false); 
  display->imageDelete(ir);
}

void colorf(Mat img){
  cvtColor(img, img, COLOR_BGR2HSV);
  Scalar val = mean(img);
  cout << val << endl;
  if ((130 < val[0] && val[0] < 150) && val[1] > 100) cout << "purple" << endl;       //&& (100 < val[1] &&  val[1]< 250 ) && (20 <val[2] && val[2]< 250)
  else if((15 < val[0] && val[0] < 40) && val[1] > 100) cout << "yellow" << endl;
  else if((112 < val[0] && val[0] < 128) && val[1] > 100) cout << "blue" << endl;
  else if((165 < val[0] || val[0] < 8) && val[1] > 100) cout << "red" << endl;
  else if((85 < val[0] && val[0] < 95) && val[1] > 100) cout << "cyne" << endl;
  else if((val[2] < 50)) cout << "black" << endl;
  else if((val[1] < 50)) cout << "white" << endl;
  else cout << "no detection" << endl;
  //cout << val << endl;
}

Mat color_extract(Mat img,  string color){                    //input a BGR img and requared color, output the img of mask
  int hmin = 0 , smin = 0 , vmin = 0 ;
  int hmax = 255 , smax = 255 , vmax = 255;
  Mat imghsv, mask, mask2;
  cvtColor(img, imghsv, COLOR_BGR2HSV);
  if (color == "blue") {hmin=110, smin=200, vmin=70, hmax=128, smax=255,vmax =255;}
  else if (color == "black") {hmin=0, smin=0, vmin=0, hmax=180, smax=255,vmax =30;}
  else if (color == "yellow") {hmin=25, smin=100, vmin=70, hmax=35, smax=255,vmax =255;}
  else if (color == "object") {hmin=10, smin=100, vmin=20, hmax=28, smax=255,vmax =255;}
  else if (color == "white") {hmin=0, smin=0, vmin=125, hmax=180, smax=30,vmax =255;}
  else if (color == "red"){
    inRange(imghsv, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);
    inRange(imghsv, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
  return mask | mask2 ;
  }
  Scalar lower(hmin, smin, vmin);
  Scalar upper(hmax, smax, vmax);
  inRange(imghsv, lower, upper, mask);
  return mask;
}

void rectangle_width(string col){
  Mat img = Mat(Size(width, height), CV_8UC4, Scalar(255, 0, 0)); //scalar parameter not neccessary
  Mat imgpros, cany, imgdil, blur;
  const unsigned char *image = cm->getImage();

  if (image){ 
    img.data = (uchar *)image;  
    cvtColor(img, imgpros, COLOR_BGRA2BGR);
  
    Mat mask = color_extract(imgpros, col);
    disp("display1", mask);
    GaussianBlur(mask, blur, Size(9, 9),0, 0); 
    Canny(blur, cany, 20, 64);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(cany, imgdil, kernal); 
  
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  
    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i<(int)contours.size(); i++){
      boundRect[i] = boundingRect(contours[i]);
      rect_width = boundRect[i].width;
      // cout << rect_width << endl;

      float p_coeff = 0.005;
      float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
      motor_r->setVelocity(4- error_ * p_coeff);
      motor_l->setVelocity(4+error_ * p_coeff);

    }
  }
}

void focusing_keyhole(){
  Mat img = Mat(Size(width, height), CV_8UC4, Scalar(255, 0, 0)); //scalar parameter not neccessary
  Mat imgpros, cany, imgdil, blur;
  const unsigned char *image = cm->getImage();

  if (image){ 
    img.data = (uchar *)image;  
    cvtColor(img, imgpros, COLOR_BGRA2BGR);
  
    Mat mask = color_extract(imgpros, "white");
    disp("display1", mask);
    GaussianBlur(mask, blur, Size(9, 9),0, 0); 
    Canny(blur, cany, 20, 64);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(cany, imgdil, kernal); 
  
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  
    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i<(int)contours.size(); i++){

      float peri = arcLength(contours[i], true);
      approxPolyDP(contours[i], conPoly[i], 0.002*peri, true);

      if (conPoly[i].size() > 11){

      boundRect[i] = boundingRect(contours[i]);
      rect_width = boundRect[i].width;
      
      float p_coeff = 0.006;
      float error_ = (boundRect[i].tl().x+boundRect[i].br().x) / 2 - 256;
      motor_r->setVelocity(- error_ * p_coeff);
      motor_l->setVelocity(+error_ * p_coeff);

      }

    }
  }
}

void rectangle_width_new(string mode){
  Mat img = Mat(Size(width, height), CV_8UC4, Scalar(255, 0, 0)); //scalar parameter not neccessary
  Mat imgpros, cany, imgdil, blur;
  const unsigned char *image = cm->getImage();

  if (image){ 
    img.data = (uchar *)image;  
    cvtColor(img, imgpros, COLOR_BGRA2BGR);

    Mat mask = color_extract(imgpros, "white");

    GaussianBlur(mask, blur, Size(9, 9),0, 0); 
    Canny(blur, cany, 20, 64);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(cany, imgdil, kernal); 

    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i<(int)contours.size(); i++){
      float peri = arcLength(contours[i], true);
      approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
      int var1 = conPoly[i].size() ; 
      if (6 < var1 && var1 < 9 && mode == "sq"){

        boundRect[i] = boundingRect(contours[i]);
        rect_width = boundRect[i].width;
        // cout << rect_width << endl;
      }
      else if (12 < var1  && mode == "cyl"){
        boundRect[i] = boundingRect(contours[i]);
        rect_width = boundRect[i].width;
        // cout << rect_width << endl;
      }
    }
  }
}

void ping_pong(string col) {
  Mat img = Mat(Size(width, height), CV_8UC4, Scalar(255, 0, 0)); //scalar parameter not neccessary
  Mat imgpros, cany, imgdil, blur;
  const unsigned char *image = cm->getImage();

  if (image){ 
    img.data = (uchar *)image;  
    cvtColor(img, imgpros, COLOR_BGRA2BGR);

    Mat mask = color_extract(imgpros, col);
    disp("display1", mask);

    GaussianBlur(mask, blur, Size(9, 9),0, 0); 
    Canny(blur, cany, 20, 64);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(cany, imgdil, kernal); 

    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i<(int)contours.size(); i++){
      boundRect[i] = boundingRect(contours[i]);
      rect_width = boundRect[i].width;
      // cout << rect_width << endl;
    }        
  }
}

int detect_shape(string col){
  Mat img = Mat(Size(width, height), CV_8UC4, Scalar(255, 0, 0)); //scalar parameter not neccessary
  Mat imgpros, cany, imgdil, blur;
  const unsigned char *image = cm->getImage();

  if (image){ 
    img.data = (uchar *)image;  
    cvtColor(img, imgpros, COLOR_BGRA2BGR);

    Mat mask = color_extract(imgpros, col);

    GaussianBlur(mask, blur, Size(9, 9),0, 0); 
    Canny(blur, cany, 20, 64);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(cany, imgdil, kernal); 

    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    findContours(imgdil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i<(int)contours.size(); i++){
      float peri = arcLength(contours[i], true);
      approxPolyDP(contours[i], conPoly[i], 0.002*peri, true);

      int con_size = conPoly[i].size();
      // cout << con_size << endl;
      return con_size;
    }
  }
  return 0;
}




int ob_det(string mode){
  string col, con_mode;
  
  col = "object";
  con_mode = "object";

  if (mode == "object") col = "object", con_mode = "object";
  else if (mode == "cyl") col = "white", con_mode = "cyl";
  else if (mode == "sq") col = "white", con_mode = "sq";
  
  Mat img = Mat(Size(width, height), CV_8UC4, Scalar(255, 0, 0)); //scalar parameter not neccessary
  Mat imgpros, cany, imgdil, blur;
  
  while (robot->step(TIME_STEP) != -1) {
    const unsigned char *image = cm->getImage();
    if (image){
      img.data = (uchar *)image;  
      cvtColor(img, imgpros, COLOR_BGRA2BGR);
      // colorf(imgpros.clone());

      Mat mask = color_extract(imgpros, col);
      disp("display1", mask);

      GaussianBlur(mask, blur, Size(9, 9),0, 0); 
      Canny(blur, cany, 20, 64);
      Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
      dilate(cany, imgdil, kernal); 

      getcon1(imgdil, imgpros, con_mode);

      disp("display", imgpros);

      rect_width = 0;
      rectangle_width_new(mode);
      
      // cout << rect_width << endl;
      if ( rect_width > 300 && mode=="cyl"){
        stop();
        break;
      }
      if (rect_width > 300 &&  mode=="sq" ){
        stop();
        break;
      }
    }
  }    
  return 0;
}

void go_step(double dis){
  encoder_l_reading = encoder_l->getValue();
  while (robot->step(TIME_STEP) != -1){
    motor_l->setVelocity(NORM_SPEED);
    motor_r->setVelocity(NORM_SPEED);
    if (encoder_l->getValue() > (encoder_l_reading + dis)){
      stop();
      break;
    }
  }
}

void pick_place(){
  cout << "[INFO] pick and place mode activated" << endl;
  encoder_l_reading = encoder_l->getValue();
  while (robot->step(TIME_STEP) != -1){
    motor_l->setVelocity(NORM_SPEED);
    motor_r->setVelocity(NORM_SPEED);
    if (encoder_l->getValue() > (encoder_l_reading + 18.5)){
      stop();
      break;
    }
  }
  turn_left(90);
  revese();
  delay(0.75);
  go_ahead();
  
  double p_distance = DS[2]->getValue();
  double n_distance;
  while (robot->step(TIME_STEP) != -1){
    
    n_distance = DS[2]->getValue();
    if (abs(p_distance - n_distance) > 5) break;
    else p_distance = n_distance; 
  }
  // delay(0.35);
  go_step(3.2);
  turn_right(90);
  go_ahead();
  while (robot->step(TIME_STEP) != -1){
    rectangle_width("object");
    // int con_point = detect_shape("object");
    // cout << con_point << endl;
    if (rect_width > 420){
      break;
    }
  }
  stop();
  int con_point = detect_shape("object");
  // cout << con_point << endl;
  
  if (con_point < 15){                /////////////////////////// firstly detect cube ////////////////////////////////////////
    cout << "[INFO] cube detected" << endl;
    // pick the Cube  and turn back
    double theta = 0 ;
    if (con_point == 6 ) go_step(0.25);
    if (con_point == 12 ) theta = 45;
    if ( con_point == 13) theta = 35;
    if ( con_point == 7) {theta = 0;go_step(0.2);}

    rotate(theta);                         
    arm(0);
    delay(0.2);
    grip(1, 0.04); // k=0 for cube
    rotate(-theta);
    arm(1);
    delay(1);
    turn_back();
    go_ahead();
    // go till colour  boundry
    while (robot->step(TIME_STEP) != -1){
      if (IR[2]->getValue() < 425){
        break;
      } 
    }
    delay(2.5);
    turn_right(90);     
    go_ahead();
    // find the square hole
    ob_det("sq");
    // palce object and kick
    arm(0);
    delay(1);
    grip(0, 0.04);
    delay(1);
    kick();
    delay(1);
    arm(1);
    turn_right(90);
    // go till colour  boundry
    go_ahead();
    while (robot->step(TIME_STEP) != -1){
      if (IR[2]->getValue() > 475){
        break;
      } 
    }
    turn_right(90);
    go_ahead();
    // to pass the balls
    delay(3.5);
    
    // looking for cylinder object
    p_distance = DS[0]->getValue();
    n_distance = 0;
    while (robot->step(TIME_STEP) != -1){
      n_distance = DS[0]->getValue();
      if (abs(p_distance - n_distance) > 5) break;
      else p_distance = n_distance;
    }
    delay(0.5);
    turn_left(90);
    cout << "[INFO] cylinder detected" << endl;
    go_ahead();
    // go till object
    while (robot->step(TIME_STEP) != -1){
      rectangle_width("object");
      if (rect_width > 470){
        break;
      } 
    }
    go_step(0.2);
    stop();
    // grab the object
    arm(0);
    solenoid->setPosition(0.006);
    delay(1);
    grip(1, 0.13); // k=0.12 for cylinder
    delay(1);
    arm(1);
    delay(1);
    turn_back();
    go_ahead();
    // go till colour  boundry
    while (robot->step(TIME_STEP) != -1){
      if (IR[2]->getValue() < 425){
        break;
      }
    }
    delay(1.3);
    turn_right(90);
    go_ahead();
    // looking for circulular hole
    ob_det("cyl");
    // place the object
    arm(0);
    delay(1);
    kick(0.04);
    grip(0, 0.13);
    delay(1);
    arm(1);
  }
  else {                  /////////////////////////// firstly detect cylinder ////////////////////////////////////////
    cout << "[INFO] cylinder detected" << endl;

    go_step(0.2);
    arm(0);
    solenoid->setPosition(0.006);
    delay(1);
    grip(1, 0.13); // k=0.12 for cylinder
    delay(1);
    arm(1);
    delay(1);
    turn_back();
    go_ahead();
    // go till colour  boundry
    while (robot->step(TIME_STEP) != -1){
      if (IR[2]->getValue() < 425){
        break;
      }
    }
    delay(1.3);
    turn_right(90);
    go_ahead();
    // looking for circulular hole
    ob_det("cyl");
    // place the object
    go_step(0.1);
    arm(0);
    delay(1);
    kick(0.04);
    grip(0, 0.13);
    delay(1);
    arm(1);

    turn_right(90);
    // go till colour  boundry
    go_ahead();
    while (robot->step(TIME_STEP) != -1){
      if (IR[2]->getValue() > 475){
        break;
      } 
    }
    turn_right(90);
    go_ahead();
    // to pass the balls
    delay(3.5);
 
    // looking for cylinder object
    p_distance = DS[0]->getValue();
    n_distance = 0;
    while (robot->step(TIME_STEP) != -1){
        n_distance = DS[0]->getValue();
        if (abs(p_distance - n_distance) > 5) break;
        else p_distance = n_distance;  
    }
    delay(0.6);
    turn_left(90);
    cout << "[INFO] cube detected" << endl;
    go_ahead();
    // go till object
    while (robot->step(TIME_STEP) != -1){
      rectangle_width("object");
      if (rect_width > 420){
        break;
      } 
    }
    stop();
    int con_point = detect_shape("object");
    // cout << con_point << endl;
    double theta = 0 ;
    if (con_point == 6 ) go_step(0.35);
    if (con_point == 12 ) theta = 45;
    if ( con_point == 13) theta = 35;
    if ( con_point == 7) {theta = 0;go_step(0.2);}

    rotate(theta);                         
    arm(0);
    delay(0.2);
    grip(1, 0.08); // k=0 for cube
    rotate(-theta);
    arm(1);
    delay(1);
    turn_back();
    go_ahead();

    
    // go till colour  boundry
    while (robot->step(TIME_STEP) != -1){
      if (IR[2]->getValue() < 425){
        break;
      }
    }
    delay(2.5);
    turn_right(90);     
    go_ahead();
    // delay(2.5);
    // find the square hole
    ob_det("sq");
    // palce object and kick
    arm(0);
    delay(1);
    grip(0, 0.08);
    delay(1);
    kick();
    delay(1);
    arm(1);
  }

  turn_right(90);
  // go till colour  boundry
  go_ahead();
  while (robot->step(TIME_STEP) != -1){
    if (IR[2]->getValue() > 475){
      break;
    } 
  }
  turn_right(90);
  revese();
  delay(0.75);
  go_ahead();
  // looking for ping pong 
  p_distance = DS[0]->getValue();
  n_distance = 0;
  while (robot->step(TIME_STEP) != -1){
    n_distance = DS[0]->getValue();
    if ((p_distance - n_distance) > 5) break;
    else p_distance = n_distance;
  }
  delay(0.1);
  stop();

  turn_left(90);

//////////////////////////////////////ping pong ball detection///////////////////////////////////////////
  rect_width = 4152;  
  ping_pong(colour);

  if (rect_width == 4152){
    cout << "[INFO] wrong ball" << endl;
    turn_right(90);
    go_ahead();
    // looking for ping pong 
    p_distance = DS[0]->getValue();
    n_distance = 0;
    while (robot->step(TIME_STEP) != -1){
      n_distance = DS[0]->getValue();
      if ((p_distance - n_distance) > 5) break;
      else p_distance = n_distance;
    }
    turn_left(90);
  }
  cout << "[INFO] "<< colour << " ball found" << endl;
  go_ahead();
  // go till object
  while (robot->step(TIME_STEP) != -1){
    rectangle_width(colour);
    if (rect_width > 320){
      break;
    }
  }
  stop();
  solenoid->setPosition(0.01);
  arm(0);
  delay(1);
  grip(1, 0.29); // k=0.25 for ball
  arm(1);
  delay(1);
  turn_back();
  go_ahead();

  while (robot->step(TIME_STEP) != -1){
    if (IR[2]->getValue() < 425){
      break;
    }
  }

  stop();
  revese();
  delay(0.8);
  turn_right(90);
  go_ahead();

  while (robot->step(TIME_STEP) != -1){ 
    
    p_distance = DS[0]->getValue();
    if (IR[2]->getValue() < 425 || IR[4]->getValue() < 425){
    break;
    } 
    if (p_distance > 400){
      if (p_distance < 790){
        motor_l->setVelocity(NORM_SPEED*1.2);
        motor_r->setVelocity(NORM_SPEED);
      }
      else if (p_distance  > 790){
        motor_l->setVelocity(NORM_SPEED);
        motor_r->setVelocity(NORM_SPEED*1.2);
      }
      else {
        motor_l->setVelocity(NORM_SPEED);
        motor_r->setVelocity(NORM_SPEED);
      }
      
      // cout << p_distance << endl;
    }
  }
  cout << "[INFO] Exit from mosaic area" << endl;
  return;


}