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

//Motor Constraints
SMS_STS st;
#define S_RXD 16
#define S_TXD 17
byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];
int pos1_off = 466;
int pos2_off = 279;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
 
  ID[0] = 1;
  ID[1] = 2;


  ACC[0] = 100;
  ACC[1] = 100;

  Speed[0] = 3400;
  Speed[1] = 3400;

  go_home();
  delay(1000);
  go_to_zero();
  delay(2000);
  // for(int i = 100; i <= 300; i += 50){
  //   Position[0] = i + pos1_off;
  //   Position[1] = i + pos2_off;
  //   st.SyncWritePosEx(ID ,2, Position, Speed, ACC);
  //   delay(1000);
  //   int pos_mat[2];get_pos(pos_mat);    
  // }
  int count = 0;
  for(float i = -2; i<=5; i+=1){
      move_to_point(18, i);
      delay(500);
  }
  // delay(500);
  // move_to_point(16.5, 3.5);
  // delay(500);
  // move_to_point(18, 5);
  // delay(500);
  // move_to_point(22, 1.5);
  // delay(500);
}
void go_home() {
 Position[0] = 0;
 Position[1] = 0; 
 st.SyncWritePosEx(ID ,2, Position, Speed, ACC);

}

void semi_circle() {
}
void move_to_point(float point_X, float point_Y) {
  float angles_rads[2] = {0, 0};

  Serial.print("Point to reach:");
  Serial.print(point_X);
  Serial.print(",");
  Serial.println(point_Y);

  IK_solver(point_X, point_Y, angles_rads);
  float test[2];
  //FK_solver(0.8668, 0.5784, test);
  float theta1_deg = 360 - abs(angles_rads[0] * (180/PI));
  float theta5_deg = abs(90 - (angles_rads[1] * (180/PI))); 
  Serial.print("final motor positions irl: ");
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
  
  Position[0] = (theta1_deg*one_degree) + pos1_off;
  Position[1] = (theta5_deg*one_degree) + pos2_off;
  Serial.print("Motor pos: ");
  Serial.print(Position[0]);
  Serial.print(",");
  Serial.println(Position[1]);
  if((0 < theta1_deg < 80) && (0 < theta5_deg < 70)){
    st.SyncWritePosEx(ID ,2, Position, Speed, ACC);
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
  pos1 = st.ReadPos(1);
  if (pos1 != -1) {
    Serial.print("Pos1:");
    Serial.println(pos1);
      pos_matrix[0] = pos1;

  } else {
    Serial.println("Error");
      pos_matrix[0] = 0;

  }
  pos2 = st.ReadPos(2);
  if (pos2 != -1) {
    Serial.print("Pos2:");
    Serial.println(pos2);
      pos_matrix[1] = pos2;

  } else {
    Serial.println("Error");
      pos_matrix[1] = 0;

  }
}

void go_to_zero() {
 Position[0] = 0 + pos1_off;
 Position[1] = 0 + pos2_off;
 
 st.SyncWritePosEx(ID ,2, Position, Speed, ACC);
}

void IK_solver(float coord_X, float coord_Y, float* angles_rads) {
  
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
  Serial.print("FK Inputs - Theta1: ");
  Serial.print(theta1, 6);
  Serial.print(", Theta5: ");
  Serial.println(theta5, 6);
  // %Section formula Ratios
  float section_m = l4;
  float section_n = l6;

  // %Coordinates of Point C
  float point_cx = l5 * cos(theta5) + l1;
  float point_cy = l5 * sin(theta5);

  // %Coordinates of Point E
  float point_ex = -l2 * cos(theta1);
  float point_ey = l2 * sin(theta1);

  float constant_A = 2 * (point_cx - point_ex);
  float constant_B = 2 * (point_cy - point_ey);
  float constant_K = (l3 * l3) - (l4 * l4) + (point_cx * point_cx) - (point_ex * point_ex) + (point_cy * point_cy) - (point_ey * point_ey);
  float constant_D = point_cx + point_ex;
  float constant_E = point_ey + point_cy;
  float constant_Z = (l3 * l3) + (l4 * l4) - (point_cx * point_cx) - (point_ex * point_ex) - (point_cy * point_cy) - (point_ey * point_ey);
  float constant_A_tilda = (2 * (constant_B * constant_B) / (constant_A * constant_A)) + 2;
  if (fabs(constant_A_tilda) < 1e-6) {
    Serial.println("Division by zero risk: constant_A_tilda is too small");
    return;
}
  float constant_B_tilda = (2 * constant_D * constant_B / constant_A) - (4 * constant_K * constant_B / (constant_A * constant_A)) - (2 * constant_E);
  float constant_C_tilda = -(constant_Z + (2 * constant_D * constant_K / constant_A) - (2 * (constant_K * constant_K) / (constant_A * constant_A)));
  float charct_eqn = abs((constant_B_tilda * constant_B_tilda) - 4 * constant_A_tilda * constant_C_tilda);
  float solution_y1 = (-constant_B_tilda + sqrt(charct_eqn)) / (2 * constant_A_tilda);
  float solution_y2 = (-constant_B_tilda - sqrt(charct_eqn)) / (2 * constant_A_tilda);
  float solution_x1 = (constant_K - constant_B * solution_y1) / constant_A;
  float solution_x2 = (constant_K - constant_B * solution_y2) / constant_A;
  Serial.print("charct_eqn: ");
  Serial.println(charct_eqn);
  //Solutions

  float X1 = ((point_cx * (section_m + section_n) - section_n * solution_x1) / section_m);
  float Y1 = ((point_cy * (section_m + section_n) - section_n * solution_y1) / section_m);
  float X2 = ((point_cx * (section_m + section_n) - section_n * solution_x2) / section_m);
  float Y2 = ((point_cy * (section_m + section_n) - section_n * solution_y2) / section_m);

  coordinates[0] = X1;
  coordinates[1] = Y1;
  Serial.print("FK sol: ");
  Serial.print(X1);
  Serial.print(",");
  Serial.println(Y1);
   Serial.println("--------------------");
  Serial.print(X2);
  Serial.print(",");
  Serial.println(Y2);

}

//trajectories array can mostly be removed

//float angles[NUM_LEGS][NUM_POINTS*4][2];        // Array to store servo angles


float ***angles;
float ***trajectories;


void IK_solver(float coord_X, float coord_Y, float* angles_rads);


// Function to generate circular trajectory points and corresponding servo angles for a leg
//Moves from 3,-10 to 7,-10.
void generate_circular_trajectory(int leg_id, float radius) {

const float centerX = 4.0;  // X-coordinate of circle center
const float centerY = -12.0; // Y-coordinate of circle center
float new_centerX = 0.0;
printf("leg_id:%d",leg_id);
printf("\n");
for (int i = 0; i < NUM_POINTS; i++) {
    float angle =  (M_PI * (float)i / (NUM_POINTS - 1)); // Angles from -π to 0
    //need to change centre of circle according to translation along the x axis

     if(count==0){
       new_centerX= trajectories[leg_id][count][0] - radius;
         }
    else{
    new_centerX= trajectories[leg_id][count-1][0] - radius;
    }              
    
    float x = new_centerX - radius * cos(M_PI-angle);
        float y = centerY + radius * sin(angle);
         trajectories[leg_id][count+i][0] = x;
         trajectories[leg_id][count+i][1] = y;
        //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
        IK_solver(x, y, angles[leg_id][count+i]);
        
}
printf("\n");

}
// Function to generate straight-line trajectory points and corresponding servo angles for a leg
void generate_straight_line_trajectory(int leg_id, float start_x, float end_x, float y) {
    float step = (end_x - start_x) / NUM_POINTS;
    printf("leg_id:%d",leg_id);
    printf("\n");
    for (int i = 0; i < NUM_POINTS; i++) {
        float x = start_x + i * step;
        trajectories[leg_id][count+i][0] = x;
        trajectories[leg_id][count+i][1] = y;
        //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
        IK_solver(x, y, angles[leg_id][count+i]);
    }
    printf("\n");
}

// Main function to implement crawling gait
void crawling_gait() {
    //initialize_positions();

    float radius = 2.0;        // Example radius
    //float trajectories[NUM_LEGS][NUM_POINTS*4][2];  // Array to store trajectories
    trajectories[0][0][0]=6;
    trajectories[1][0][0]=6;
    trajectories[2][0][0]=6;
    trajectories[3][0][0]=6;



    for(int j= 0; j < 1; j++) { // Loop until stop condition
        for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {  
            
            // printf("Leg id: %d\n", leg_id);
           generate_circular_trajectory(leg_id, radius);
           printf("4");
           
            for (int other_leg_id = 0; other_leg_id < NUM_LEGS; other_leg_id++) {
                if (other_leg_id != leg_id) {
                        if(count==0){
                            generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count][0],trajectories[other_leg_id][count][0]+(6.0/3) , -12);
                
                        }
                        else{
                                
                             generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count-1][0],trajectories[other_leg_id][count-1][0]+(6.0/3) , -12);
                        }              
                
                }
            }

            count=count+NUM_POINTS;
        }

    }

}

void loop() {
  
}












