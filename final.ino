#include <math.h>
#include <SCServo.h>

//Link Constant&s
const float length_AB = 3.0;   //AB
const float length_AE = 3.5;   //AE
const float length_ED = 10.5;  //ED
const float length_DC = 2.7;   //DC
const float length_CB = 9.8;   //CB
const float length_CF = 10.5;  //CF
const float one_degree = 11.37777777777778;

const float offset1 = -70;
const float offset2 = -140;

//Motor Constraints
SMS_STS st;
#define S_RXD 16
#define S_TXD 17
byte ID[8];
s16 Position[8];
u16 Speed[8];
byte ACC[8];
int pos1_off = 466;
int pos2_off = 279;
int pos3_off = 537;
int pos4_off = 300;
int pos5_off = 331;
int pos6_off = 192;
int pos7_off = 330;
int pos8_off = 264;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
 
  ID[0] = 1;
  ID[1] = 2;
  ID[2] = 3;
  ID[3] = 4;
  ID[4] = 5;
  ID[5] = 6;
  ID[6] = 7;
  ID[7] = 8;

  ACC[0] = 50;
  ACC[1] = 50;
  ACC[2] = 50;
  ACC[3] = 50;
  ACC[4] = 50;
  ACC[5] = 50;
  ACC[6] = 50;
  ACC[7] = 50;

  Speed[0] = 2000;
  Speed[1] = 2000;
  Speed[2] = 2000;
  Speed[3] = 2000;
  Speed[4] = 2000;
  Speed[5] = 2000;
  Speed[6] = 2000;
  Speed[7] = 2000;

  // go_to_zero();
  // delay(3000);
  // move_to_point(18, -5);
  // delay(500);
  // move_to_point(17, 1);
  // delay(500);
  // move_to_point(18, 2);
  // delay(500);


  // go_home();
  // delay(3000);


  go_to_zero();  // sit();
  delay(3000);
  juggle2();
  // delay(3000);
  // stand();
  // sit();
  // delay(3000);
  // stand();
  // sit();
  // delay(3000);
  // stand();

  Serial.println("Enter values between 0 and 4095.  Type 'done' to stop.");

  
  // juggle2();
  // for(int i =0; i<3; i++){
  //     juggle2();
  //     delay(500);

  // }
  // delay(500);
  // stand();
  // for(int i = 0; i<3; i++){
  //   sit();
  //   delay(500);
  //   stand();
  //   delay(500);
  // }
  //   delay(500);
  // stand();

}
void go_home() {
 Position[0] = 0;
 Position[1] = 0;
 Position[2] = 0;
 Position[3] = 0;
 Position[4] = 4096;
 Position[5] = 4096;
 Position[6] = 4096;
 Position[7] = 4096;
 
 st.SyncWritePosEx(ID ,8, Position, Speed, ACC);

}
void juggle2() {
  stand();
  delay(1500);

  Speed[0] = 3000;
  Speed[1] = 3000;
  Speed[2] = 3000;
  Speed[3] = 3000;
  Speed[4] = 3000;
  Speed[5] = 3000;
  Speed[6] = 3000;
  Speed[7] = 3000;

//   Position[0] = 500 + pos1_off;
//   Position[1] = 400 + pos2_off;
//   Position[2] = 500 + pos3_off;
//   Position[3] = 400 + pos4_off;
//   Position[4] = 4096 - 500 - pos5_off;
//   Position[5] = 4096 - 400 - pos6_off;
//   Position[6] = 4096 - 300 - pos7_off;
//   Position[7] = 4096 - 200 - pos8_off;
//   st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
//   delay(500);
//   Position[0] = 100 + pos1_off;
//   Position[1] = 0 + pos2_off;
//   Position[2] = 500 + pos3_off;
//   Position[3] = 400 + pos4_off;
//   Position[4] = 4096 - 500 - pos5_off;
//   Position[5] = 4096 - 400 - pos6_off;
//   Position[6] = 4096 - 300 - pos7_off;
//   Position[7] = 4096 - 200 - pos8_off;
//     st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
// delay(500);
// Position[0] = 500 + pos1_off;
//   Position[1] = 400 + pos2_off;
//   Position[2] = 500 + pos3_off;
//   Position[3] = 400 + pos4_off;
//   Position[4] = 4096 - 500 - pos5_off;
//   Position[5] = 4096 - 400 - pos6_off;
//   Position[6] = 4096 - 300 - pos7_off;
//   Position[7] = 4096 - 200 - pos8_off;
//     st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
// delay(500);
// stand();
 Position[0] = 500 + pos1_off;
  Position[1] = 400 + pos2_off;
  Position[2] = 500 + pos3_off;
  Position[3] = 400 + pos4_off;
  Position[4] = 4096 - 300 - pos5_off;
  Position[5] = 4096 - 200 - pos6_off;
  Position[6] = 4096 - 500 - pos7_off;
  Position[7] = 4096 - 400 - pos8_off;
  st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
  delay(500);
  Position[0] = 500 + pos1_off;
  Position[1] = 400 + pos2_off;
  Position[2] = 100 + pos3_off;
  Position[3] = 0 + pos4_off;
  Position[4] = 4096 - 300 - pos5_off;
  Position[5] = 4096 - 200 - pos6_off;
  Position[6] = 4096 - 500 - pos7_off;
  Position[7] = 4096 - 400 - pos8_off;
    st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
delay(500);
Position[0] = 500 + pos1_off;
  Position[1] = 400 + pos2_off;
  Position[2] = 500 + pos3_off;
  Position[3] = 400 + pos4_off;
  Position[4] = 4096 - 300 - pos5_off;
  Position[5] = 4096 - 200 - pos6_off;
  Position[6] = 4096 - 500 - pos7_off;
  Position[7] = 4096 - 400 - pos8_off;
    st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
delay(500);
stand();

  // delay(500);
  // Position[0] = 500;
  // Position[1] = 400;
  // Position[2] = 100;
  // Position[3] = 0;
  // Position[4] = 4096 - 500;
  // Position[5] = 4096 - 400;
  // Position[6] = 4096 - 500;
  // Position[7] = 4096 - 400;
  // st.SyncWritePosEx(ID ,8, Position, Speed, ACC);

  // delay(500);

  // stand();
  // delay(500);
  // Position[0] = 599;
  // Position[1] = 400;
  // Position[2] = 500;
  // Position[3] = 400;
  // Position[4] = 4096 - 100;
  // Position[5] = 4096 ;
  // Position[6] = 4096 - 500;
  // Position[7] = 4096 - 400;
  // st.SyncWritePosEx(ID ,8, Position, Speed, ACC);

  // delay(500);

  // stand();
  // delay(500);
  // Position[0] = 500;
  // Position[1] = 400;
  // Position[2] = 500;
  // Position[3] = 400;
  // Position[4] = 4096 - 500;
  // Position[5] = 4096 - 400;
  // Position[6] = 4096 - 100;
  // Position[7] = 4096 ;
  // st.SyncWritePosEx(ID ,8, Position, Speed, ACC);

  // delay(500);

  // stand();
}
void juggle() {
  // sit();
  // delay(500);
  stand();
  delay(3000);
  Speed[0] = 3000;
  Speed[1] = 3000;
  Speed[2] = 3000;
  Speed[3] = 3000;
  Speed[4] = 3000;
  Speed[5] = 3000;
  Speed[6] = 3000;
  Speed[7] = 3000;
  // sit();
  // Position[0] = 500;
  // Position[1] = 400;
  // Position[2] = 500;
  // Position[3] = 400;
  // Position[4] = 4096 - 500;
  // Position[5] = 4096 - 400;
  // Position[6] = 4096 - 500;
  // Position[7] = 4096 - 400;
  Position[0] = 300 + pos1_off;
  Position[1] = 0 + pos2_off;
  Position[2] = 500+ pos3_off;
  Position[3] = 400+ pos4_off;
  Position[4] = 4096 - 500 - pos5_off;
  Position[5] = 4096 - 400 - pos6_off;
  Position[6] = 4096 - 300 - pos7_off;
  Position[7] = 4096 - pos8_off;
  st.SyncWritePosEx(ID ,8, Position, Speed, ACC);

  delay(500);
  Position[0] = 500 + pos1_off;
  Position[1] = 400 + pos2_off;
  Position[2] = 300 + pos3_off;
  Position[3] = 0 + pos4_off;
  Position[4] = 4096 - 300 - pos5_off;
  Position[5] = 4096 - pos6_off;
  Position[6] = 4096 - 500 - pos7_off;
  Position[7] = 4096 - 400 - pos8_off;
  st.SyncWritePosEx(ID ,8, Position, Speed, ACC);

  delay(500);
  stand();

}
void move_to_point(float point_X, float point_Y) {
  volatile float angles_rads[2] = {0, 0};

  Serial.print("Point to reach:");
  Serial.print(point_X);
  Serial.print(",");
  Serial.println(point_Y);

  IK_solver(point_X, point_Y, angles_rads);
  float theta1_deg = 360 - abs(angles_rads[0] * (180/PI));
  float theta5_deg = abs(90 - (angles_rads[1] * (180/PI))); 
  Serial.print("final motor positions: ");
  Serial.print(theta1_deg*one_degree);
  Serial.print(",");
  Serial.println(theta5_deg*one_degree);
  
  int pos_mat[2];get_pos(pos_mat);
  int pos1 = pos_mat[0];
  int pos2 = pos_mat[1];
  Serial.print("IK Sols: ");
  Serial.print(theta1_deg);
  Serial.print(",");
  Serial.println(theta5_deg);
  
  Position[0] = (theta1_deg*one_degree) - offset1;
  Position[1] = (theta5_deg*one_degree) - offset2;
  Position[2] = (theta1_deg*one_degree) - offset1;
  Position[3] = (theta5_deg*one_degree) - offset2;
  Position[4] = 4096 - (theta1_deg*one_degree) - offset1;
  Position[5] = 4096 - (theta5_deg*one_degree) - offset2;
  Position[6] = 4096 - (theta1_deg*one_degree) - offset1;
  Position[7] = 4096 - (theta5_deg*one_degree) - offset2;
  if((0 < theta1_deg < 80) && (0 < theta5_deg < 70)){
    st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
    // Serial.print((theta5_deg*one_degree) - pos2);
    // Serial.print(",");
    // Serial.print((theta5_deg*one_degree));
    // Serial.print(":");
    // Serial.print((theta1_deg*one_degree) - pos1);
    // Serial.print(",");
    // Serial.println((theta1_deg*one_degree));
  }
  else{
    Serial.println("Joint Violation!!!");
  }

}
void get_pos(int* pos_matrix) {
  int pos1, pos2;
  pos1 = st.ReadPos(3);
  if (pos1 != -1) {
    Serial.print("Pos1:");
    Serial.println(pos1);
      pos_matrix[0] = pos1;

  } else {
    Serial.println("Error");
      pos_matrix[0] = 0;

  }
  pos2 = st.ReadPos(4);
  if (pos2 != -1) {
    Serial.print("Pos2:");
    Serial.println(pos2);
      pos_matrix[1] = pos2;

  } else {
    Serial.println("Error");
      pos_matrix[1] = 0;

  }
}
void sit() {
  Position[0] = 500  + pos1_off;
  Position[1] = 0  + pos2_off;
  Position[2] = 500  + pos3_off;
  Position[3] = 0  + pos4_off;
  Position[4] = 4096 - 500 - pos5_off;
  Position[5] = 4096 - pos6_off;
  Position[6] = 4096 - 500 - pos7_off;
  Position[7] = 4096 - pos8_off;

  st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
}
void go_to_zero() {
 Position[0] = 0 + pos1_off;
 Position[1] = 0 + pos2_off;
 Position[2] = 0 + pos3_off;
 Position[3] = 0 + pos4_off;
 Position[4] = 4096 - pos5_off;
 Position[5] = 4096 - pos6_off;
 Position[6] = 4096 - pos7_off;
 Position[7] = 4096 - pos8_off;
 
 st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
}
void stand() {
 
  Position[0] = 500+ pos1_off;
  Position[1] = 400+ pos2_off;
  Position[2] = 500+ pos3_off;
  Position[3] = 400+ pos4_off;
  Position[4] = 4096 - 500- pos5_off;
  Position[5] = 4096 - 400- pos6_off;
  Position[6] = 4096 - 500- pos7_off;
  Position[7] = 4096 - 400- pos8_off;

 st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
}
void IK_solver(float coord_X, float coord_Y, volatile float* angles_rads) {

  float constant_K1 = (pow(coord_X, 2) + pow(length_CB, 2) + pow(length_AB, 2) + pow(coord_Y, 2) - pow(length_CF, 2) - 2 * coord_X * length_AB) / (2 * length_CB);

  //Solving for Theta5 and Theta6
  float theta5 = 2 * atan2(coord_Y + sqrt(pow((coord_X - length_AB), 2) + pow(coord_Y, 2) - pow(constant_K1, 2)), (constant_K1 + (coord_X - length_AB)));
  float theta6 = atan2((coord_X - length_CB * cos(theta5) - length_AB), (length_CB * sin(theta5) - coord_Y));

  //Finding Coordinates of D
  float x2 = coord_X - (length_DC + length_CF) * sin(theta6);
  float y2 = coord_Y + (length_DC + length_CF) * cos(theta6);

  float k2 = (pow(x2, 2) + pow(y2, 2) + pow(length_AE, 2) - pow(length_ED, 2)) / (2 * length_AE);

  //Solving for Theta1 and Theta2
  float theta1 = 2 * atan2((y2 - sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - x2));
  float theta2 = atan2((y2 + length_AE * sin(theta1)), (x2 + length_AE * cos(theta1)));

  float x1 = -length_AE * cos(theta1);
  float y_1 = length_AE * sin(theta1);

  float x3 = length_CB * cos(theta5) + length_AB;
  float y3 = length_CB * sin(theta5);

  float x4 = length_AB;
  float y4 = 0;

  float x5 = 0;
  float y5 = 0;

  //Coordinates of all points
  // float _A[2] = { x5, y5 };
  // float B_B[2] = { x4, y4 };
  // float C_C[2] = { x1, y_1 };
  // float _D[2] = { x2, y2 };
  // float _E[2] = { x3, y3 };
  angles_rads[0] = theta1;
  angles_rads[1] = theta5;
  // Serial.print(theta1);
  // Serial.print(" : ");
  // Serial.println(theta5);
}

void FK_solver(float theta1, float theta5, float* coordinates) {
  // %Section formula Ratios
  float section_m = length_DC;
  float section_n = length_CF;

  // %Coordinates of Point C
  float point_cx = length_CB * cos(theta5) + length_AB;
  float point_cy = length_CB * sin(theta5);

  // %Coordinates of Point E
  float point_ex = -length_AE * cos(theta1);
  float point_ey = length_AE * sin(theta1);

  float constant_A = 2 * (point_cx - point_ex);
  float constant_B = 2 * (point_cy - point_ey);
  float constant_K = (length_ED * length_ED) - (length_DC * length_DC) + (point_cx * point_cx) - (point_ex * point_ex) + (point_cy * point_cy) - (point_ey * point_ey);
  float constant_D = point_cx + point_ex;
  float constant_E = point_ey + point_cy;
  float constant_Z = (length_ED * length_ED) + (length_DC * length_DC) - (point_cx * point_cx) - (point_ex * point_ex) - (point_cy * point_cy) - (point_ey * point_ey);
  float constant_A_tilda = (2 * (constant_B * constant_B) / (constant_A * constant_A)) + 2;
  float constant_B_tilda = (2 * constant_D * constant_B / constant_A) - (4 * constant_K * constant_B / (constant_A * constant_A)) - (2 * constant_E);
  float constant_C_tilda = -(constant_Z + (2 * constant_D * constant_K / constant_A) - (2 * (constant_K * constant_K) / (constant_A * constant_A)));
  float charct_eqn = (constant_B_tilda * constant_B_tilda) - 4 * constant_A_tilda * constant_C_tilda;
  float solution_y1 = (-constant_B_tilda + sqrt(charct_eqn)) / (2 * constant_A_tilda);
  float solution_y2 = (-constant_B_tilda - sqrt(charct_eqn)) / (2 * constant_A_tilda);
  float solution_x1 = (constant_K - constant_B * solution_y1) / constant_A;
  float solution_x2 = (constant_K - constant_B * solution_y2) / constant_A;

  //Solutions
  float X1 = ((point_cx * (section_m + section_n) - section_n * solution_x1) / section_m);
  float Y1 = ((point_cy * (section_m + section_n) - section_n * solution_y1) / section_m);
  float X2 = ((point_cx * (section_m + section_n) - section_n * solution_x2) / section_m);
  float Y2 = ((point_cy * (section_m + section_n) - section_n * solution_y2) / section_m);

  coordinates[0] = X1;
  coordinates[1] = Y1;
  // Serial.print(X1);
  // Serial.print(",");
  // Serial.print(Y1);
  // Serial.print(":");

  // Serial.print(X2);
  // Serial.print(",");
  // Serial.println(Y2);
}

void loop() {  
  // if (Serial.available() > 0) {
  //   String inputString = Serial.readStringUntil('\n');  // Read until newline
  //   inputString.trim();  // Remove any leading/trailing whitespace

  //   if (inputString.equalsIgnoreCase("done")) {
  //     Serial.println("Finished entering values.");
  //     while(true); // Stop the program
  //   } else {
  //     // Try to convert the input to an integer
  //     int inputValue = inputString.toInt();

  //     // Check if the conversion was successful and the value is within the valid range
  //     if (inputValue >= 0 && inputValue <= 4095) {
  //       Serial.print("Writing pos: ");
  //       Serial.println(inputValue);
  //       delay(100);
  //       Position[0] = 170;
  //       Position[1] = 200;
  //       Position[2] = 250;
  //       Position[3] = 240;
  //       Position[4] = 3920;
  //       Position[5] = 3920;
  //       Position[6] = 3890;
  //       Position[7] = 3850;

  //       st.SyncWritePosEx(ID ,8, Position, Speed, ACC);
  //       //Here you can place the remaining part of the code
  //     } else {
  //       Serial.println("Invalid input. Please enter a value between 0 and 4095 or type 'done'.");
  //     }
  //   }
  // }
}

//Working Code- Guru Meditation resolved!!
// #include <math.h>
// #include <SCServo.h>
// #include <stdio.h>
// #include <stdlib.h>

// #define M_PI 3.14159265358979323846
// #define NUM_LEGS 4
// #define NUM_POINTS 3 // Number of points in the trajectory

// const float l1 = 2.7;   // AB
// const float l2 = 2.5;   // AE
// const float l3 = 10;    // ED
// const float l4 = 2.7;   // DC
// const float l5 = 10;    // CB
// const float l6 = 10.3;  // CF
// const float one_degree = 11.37777777777778;

// SMS_STS st;
// #define S_RXD 16
// #define S_TXD 17
// byte ID[8];
// s16 pos[8];
// u16 Speed[8];
// byte ACC[8];


//  int count=0;  //for storing values properly in the array
// // float Position[NUM_LEGS][2];  // Array to store positions of all legs
// // float angles[NUM_LEGS][NUM_POINTS*4][2];        // Array to store servo angles

// float ***angles;
// float ***trajectories;


// void setup() {
//   Serial.begin(115200);
//   Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
//   st.pSerial = &Serial1;
//   delay(1000);
//   ID[0] = 1;
//   ID[1] = 2;
//   ID[2] = 3;
//   ID[3] = 4;
//   ID[4] = 5;
//   ID[5] = 6;
//   ID[6] = 7;
//   ID[7] = 8;
//   ACC[0] = 50;
//   ACC[1] = 50;
//   ACC[2] = 50;
//   ACC[3] = 50;
//   ACC[4] = 50;
//   ACC[5] = 50;
//   ACC[6] = 50;
//   ACC[7] = 50;
//   Speed[0] = 3000;
//   Speed[1] = 3000;
//   Speed[2] = 3000;
//   Speed[3] = 3000;
//   Speed[4] = 3000;
//   Speed[5] = 3000;
//   Speed[6] = 3000;
//   Speed[7] = 3000;
//   //go_to_zero();
//   delay(1000);


  
// }

// void loop() {

  

// count=0;  
//   angles = (float ***)malloc(NUM_LEGS * sizeof(float **));
//     trajectories = (float ***)malloc(NUM_LEGS * sizeof(float **));
//     for (int i = 0; i < NUM_LEGS; i++) {
//         angles[i] = (float **)malloc(NUM_POINTS * 4 * sizeof(float *));
//         trajectories[i] = (float **)malloc(NUM_POINTS * 4 * sizeof(float *));
//         for (int j = 0; j < NUM_POINTS * 4; j++) {
//             trajectories[i][j] = (float *)malloc(2 * sizeof(float));
//             angles[i][j] = (float *)malloc(2 * sizeof(float));
//         }
//     }

//     crawling_gait();

//     for (int i = 0; i < NUM_LEGS; i++) {
//         for (int j = 0; j < NUM_POINTS * 4; j++) {
//             free(trajectories[i][j]);
//         }
//         free(trajectories[i]);
//         free(angles[i]);
//     }
//     free(trajectories);
//     free(angles);

//   delay(1000);

  

// }


// void go_to_zero() {
//  pos[0] = 0;
//  pos[1] = 0;
//  pos[2] = 0;
//  pos[3] = 0;
//  pos[4] = 4096;
//  pos[5] = 4096;
//  pos[6] = 4096;
//  pos[7] = 4096;
//  st.SyncWritePosEx(ID ,8, pos, Speed, ACC);
// }


// // Function to generate circular trajectory points and corresponding servo angles for a leg
// //This function is checked- it moves from 3,-10 to 7,-10.
// void generate_circular_trajectory(int leg_id, float radius) {

// const float centerX = 4.0;  // X-coordinate of circle center
// const float centerY = -10.0; // Y-coordinate of circle center
// float new_centerX = 0.0;
// printf("leg_id:%d",leg_id);
// printf("\n");
// for (int i = 0; i < NUM_POINTS; i++) {
//     float angle =  (M_PI * (float)i / (NUM_POINTS - 1)); // Angles from -π to 0
//     //need to change centre of circle according to translation along the x axis

//      if(count==0){
//        new_centerX= trajectories[leg_id][count][0] - radius;
//          }
//     else{
//     new_centerX= trajectories[leg_id][count-1][0] - radius;
//     }              
    
//     float x = new_centerX - radius * cos(M_PI-angle);
//         float y = centerY + radius * sin(angle);
//          trajectories[leg_id][count+i][0] = x;
//          trajectories[leg_id][count+i][1] = y;
//         //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
//         IK_solver(x, y, angles[leg_id][count+i]);
        
// }
// printf("\n");

// }

// // Function to generate straight-line trajectory points and corresponding servo angles for a leg
// void generate_straight_line_trajectory(int leg_id, float start_x, float end_x, float y) {
//     float step = (end_x - start_x) / NUM_POINTS;
//     printf("leg_id:%d",leg_id);
//     printf("\n");
//     for (int i = 0; i < NUM_POINTS; i++) {
//         float x = start_x + i * step;
//         trajectories[leg_id][count+i][0] = x;
//         trajectories[leg_id][count+i][1] = y;
//         //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
//         IK_solver(x, y, angles[leg_id][count+i]);
//     }
//     printf("\n");
// }


// // Main function to implement crawling gait
// void crawling_gait() {
//     //initialize_positions();
//     float radius = 2.0;        // Example radius
//     //float trajectories[NUM_LEGS][NUM_POINTS*4][2];  // Array to store trajectories
//     trajectories[0][0][0]=6;
//     trajectories[1][0][0]=6;
//     trajectories[2][0][0]=6;
//     trajectories[3][0][0]=6;


//         for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {  
            
//             // printf("Leg id: %d\n", leg_id);
//            generate_circular_trajectory(leg_id, radius);
//            printf("4");
           
//             for (int other_leg_id = 0; other_leg_id < NUM_LEGS; other_leg_id++) {
//                 if (other_leg_id != leg_id) {
//                         if(count==0){
//                             generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count][0],trajectories[other_leg_id][count][0]+(6.0/3) , -10);
                
//                         }
//                         else{
                                
//                              generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count-1][0],trajectories[other_leg_id][count-1][0]+(6.0/3) , -10);
//                         }              
                
//                 }
//             }

//             count=count+NUM_POINTS;
//         }



//     for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {
//           pos[0] = angles[0][point_index][0]* one_degree;
//           pos[1] = angles[0][point_index][1] * one_degree;
//           // pos[2] = angles[1][point_index][0]* one_degree;
//           // pos[3] = angles[1][point_index][1] * one_degree;
//           // pos[4] = 4096 - angles[2][point_index][0]* one_degree;
//           // pos[5] = 4096 - angles[2][point_index][1] * one_degree;
//           // pos[6] = 4096 - angles[3][point_index][0] * one_degree;
//           // pos[7] = 4096 - angles[3][point_index][1] * one_degree;

//                     st.SyncWritePosEx(ID, 8 , pos, Speed, ACC);
//                     delay(500);


// }
// }

// void IK_solver(float coord_X, float coord_Y, float* angles_deg) {

//     float k1 = (pow(coord_X, 2) + pow(l5, 2) + pow(l1, 2) + pow(coord_Y, 2) - pow(l6, 2) + 2 * coord_Y * l1) / (2 * l5);

//     // Solve for Theta5 and Theta6
//     float theta5 = 2 * atan2(coord_X + sqrt(pow((coord_Y + l1), 2) + pow(coord_X, 2) - pow(k1, 2)), (k1 - coord_Y - l1));
//     float theta6 = atan2(-(coord_Y + l5 * cos(theta5) + l1), (-coord_X + l5 * sin(theta5)));

//     // Find coordinates of point D
//     float x2 = coord_X + (l4 + l6) * cos(theta6);
//     float y2 = coord_Y + (l4 + l6) * sin(theta6);

//     float k2 = (pow(x2, 2) + pow(y2, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l2);

//     // Solve for Theta1
//     float theta1 = 2 * atan2((x2 + sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - y2));

//     angles_deg[0] = 205 - theta1 * (180.0 / M_PI);
//     angles_deg[1] = 120 - theta5 * (180.0 / M_PI);

//     printf("x:%f y:%f theta1:%f theta5:%f \n",coord_X,coord_Y,theta1,theta5);
     
// }





















