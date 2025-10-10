#include <math.h>
#include <SCServo.h>

#define M_PI 3.14159265358979323846
#define NUM_LEGS 4
#define NUM_POINTS 5  // Number of points in the trajectory

const float l1 = 2.7;   // AB
const float l2 = 2.5;   // AE
const float l3 = 10;    // ED
const float l4 = 2.7;   // DC
const float l5 = 10;    // CB
const float l6 = 10.3;  // CF
const float one_degree = 11.37777777777778;

SMS_STS st;
#define S_RXD 16
#define S_TXD 17
byte ID[8];
s16 pos[8];
u16 Speed[8];
byte ACC[8];

int count=0;  //for storing values properly in the array


//trajectories array can mostly be removed

float Position[NUM_LEGS][2];  // Array to store positions of all legs
float angles[NUM_LEGS][NUM_POINTS*4][2];        // Array to store servo angles


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
  Speed[0] = 3000;
  Speed[1] = 3000;
  Speed[2] = 3000;
  Speed[3] = 3000;
  Speed[4] = 3000;
  Speed[5] = 3000;
  Speed[6] = 3000;
  Speed[7] = 3000;
    go_to_zero();
    delay(3000);
}

void loop() {

   crawling_gait();
   delay(1000);
}


void go_to_zero() {
 pos[0] = 0;
 pos[1] = 0;
 pos[2] = 0;
 pos[3] = 0;
 pos[4] = 4096;
 pos[5] = 4096;
 pos[6] = 4096;
 pos[7] = 4096;
 st.SyncWritePosEx(ID ,8, pos, Speed, ACC);
}
//void IK_solver(float coord_X, float coord_Y, float* angles_rads);
//void move_to_point(float point_X, float point_Y);

// Define initial positions of the legs
// void initialize_positions() {
//     // Placeholder for actual initial positions
//     float initial_positions[NUM_LEGS][2] = {
//         {3, -10}, {3, -10}, {3, -10}, {3, -10}
//     };

//     for (int i = 0; i < NUM_LEGS; i++) {
//         move_to_point(initial_positions[i][0], initial_positions[i][1]);
//     }
// }

// Function to generate circular trajectory points and corresponding servo angles for a leg
//This function is checked- it moves from 3,-10 to 7,-10.
void generate_circular_trajectory(int leg_id, float radius, float angle_step, float trajectory[NUM_POINTS][2], float angles[NUM_POINTS][2]) {

  const float centerX = 4.0;  // X-coordinate of circle center
  const float centerY = -10.0; // Y-coordinate of circle center
  printf("leg_id:%d",leg_id);
  printf("\n");

  for (int i = 0; i < NUM_POINTS; i++) {
      float angle =  (M_PI * (float)i / (NUM_POINTS - 1)); // Angles from -π to 0
      //need to change centre of circle according to translation along the x axis
      float new_centerX= trajectory[count-1][0] - radius;
      float x = new_centerX - radius * cos(M_PI-angle);
          float y = centerY + radius * sin(angle);
          trajectory[count+i][0] = x;
          trajectory[count+i][1] = y;
          //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
          IK_solver(x, y, angles[count+i]);
          
  }
  printf("\n"); 
}

// Function to generate straight-line trajectory points and corresponding servo angles for a leg
void generate_straight_line_trajectory(int leg_id, float start_x, float end_x, float y, float trajectory[NUM_POINTS][2], float angles[NUM_POINTS][2]) {
    float step = (end_x - start_x) / NUM_POINTS;
    printf("leg_id:%d",leg_id);
    printf("\n");
    for (int i = 0; i < NUM_POINTS; i++) {
        float x = start_x + i * step;
        trajectory[count+i][0] = x;
        trajectory[count+i][1] = y;
        //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
        IK_solver(x, y, angles[count+i]);
    }
    printf("\n");
}

// Main function to implement crawling gait
void crawling_gait() {
    //initialize_positions();

    float radius = 2.0;        // Example radius
    float angle_step = 0.5;    // Example angle step
    float trajectories[NUM_LEGS][NUM_POINTS*4][2]={0};  // Array to store trajectories
    trajectories[0][-1][0]=6;
    trajectories[1][-1][0]=6;
    trajectories[2][-1][0]=6;
    trajectories[3][-1][0]=6;



    for(int j= 0; j < 1; j++) { // Loop until stop condition
        for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {  
            
            // printf("Leg id: %d\n", leg_id);  
           generate_circular_trajectory(leg_id, radius, angle_step, trajectories[leg_id], angles[leg_id]);
           
            for (int other_leg_id = 0; other_leg_id < NUM_LEGS; other_leg_id++) {
                if (other_leg_id != leg_id) {
                    generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count-1][0],trajectories[other_leg_id][count-1][0]+(5.0/3) , -10 , trajectories[other_leg_id], angles[other_leg_id]);
                    
                }
            }
            count=count+NUM_POINTS;
        }

    }


   // trajectories are stored correctly for all legs
    // for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
    //     Serial.print("Leg ID: %d\n");
    //     Serial.println(leg_id);
    //     for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {

    //         // printf("Point %d: ,x = %f, y = %f, theta1: %f, theta5:%f\n", point_index,trajectories[leg_id][point_index][0], trajectories[leg_id][point_index][1],angles[leg_id][point_index][0],angles[leg_id][point_index][1]);
    //     //     Serial.print("  Point, Theta1 , Theta5 = ");
    //     //     Serial.print(point_index);
    //     //     Serial.print(", ");
    //     //     Serial.print(angles[leg_id][point_index][0]);
    //     //     Serial.print(", ");
    //     //     Serial.println(angles[leg_id][point_index][1]);
    //     // }

    // }



    for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {
          pos[0] = angles[0][point_index][0]* one_degree;
          pos[1] = angles[0][point_index][1] * one_degree;
          pos[2] = angles[1][point_index][0]* one_degree;
          pos[3] = angles[1][point_index][1] * one_degree;
          pos[4] = 4096 - angles[2][point_index][0]* one_degree;
          pos[5] = 4096 - angles[2][point_index][1] * one_degree;
          pos[6] = 4096 - angles[3][point_index][0] * one_degree;
          pos[7] = 4096 - angles[3][point_index][1] * one_degree;

                    // for (int leg_id = 0; leg_id < NUM_LEGS*2; leg_id+=2){
                    //         pos[leg_id] = angles[leg_id][point_index][0]* one_degree;
                    //         pos[leg_id+1] = angles[leg_id][point_index][1] * one_degree;
                    // }
                    // for(int index = 0; index < 8; index++){
                    //   Serial.print("Vals ");
                    //   Serial.print(pos[index]);
                    //   Serial.print(", ");

                    // }
                  //  Serial.println();
                    st.SyncWritePosEx(ID, 8 , pos, Speed, ACC);
                    delay(500);


      //st.SyncWritePosEx(ID, 6 , pos, Speed, ACC);
    //st.SyncWritePosEx(ID, 8, pos, Speed, ACC);

}
}

void IK_solver(float coord_X, float coord_Y, float* angles_deg) {

    float k1 = (pow(coord_X, 2) + pow(l5, 2) + pow(l1, 2) + pow(coord_Y, 2) - pow(l6, 2) + 2 * coord_Y * l1) / (2 * l5);

    // Solve for Theta5 and Theta6
    float theta5 = 2 * atan2(coord_X + sqrt(pow((coord_Y + l1), 2) + pow(coord_X, 2) - pow(k1, 2)), (k1 - coord_Y - l1));
    float theta6 = atan2(-(coord_Y + l5 * cos(theta5) + l1), (-coord_X + l5 * sin(theta5)));

    // Find coordinates of point D
    float x2 = coord_X + (l4 + l6) * cos(theta6);
    float y2 = coord_Y + (l4 + l6) * sin(theta6);

    float k2 = (pow(x2, 2) + pow(y2, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l2);

    // Solve for Theta1
    float theta1 = 2 * atan2((x2 + sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - y2));

    angles_deg[0] = 205 - theta1 * (180.0 / M_PI);
    angles_deg[1] = 120 - theta5 * (180.0 / M_PI);

    printf("x:%f y:%f theta1:%f theta5:%f \n",coord_X,coord_Y,theta1,theta5);
     
}

// void move_to_point(float point_X, float point_Y) {
//     float angles_rads[2] = {0, 0};


//    // printf("Final motor positions: %f, %f\n", theta1_deg * one_degree, theta5_deg * one_degree);

//     Position[0][0] = theta1_deg * one_degree;
//     Position[0][1] = theta5_deg * one_degree;
//    // st.SyncWritePosEx(ID, 8, Position[0], Speed, ACC);
// }

// int main() {
//     crawling_gait();
    
//     return 0;
// }


// // #include <math.h>
// // #include <SCServo.h>

// // #define M_PI 3.14159265358979323846
// // #define NUM_LEGS 4
// // #define NUM_POINTS 5  // Number of points in the trajectory

// // const float l1 = 2.7;   // AB
// // const float l2 = 2.5;   // AE
// // const float l3 = 10;    // ED
// // const float l4 = 2.7;   // DC
// // const float l5 = 10;    // CB
// // const float l6 = 10.3;  // CF
// // const float one_degree = 11.37777777777778;

// // SMS_STS st;
// // #define S_RXD 18
// // #define S_TXD 19
// // byte ID[8];
// // s16 pos[8];
// // u16 Speed[8];
// // byte ACC[8];

// // int count=0;  //for storing values properly in the array


// // //trajectories array can mostly be removed

// // float Position[NUM_LEGS][2];  // Array to store positions of all legs
// // float angles[NUM_LEGS][NUM_POINTS*4][2];        // Array to store servo angles


// // void setup() {
// //   Serial.begin(115200);
// //   Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
// //   st.pSerial = &Serial1;
// //   delay(1000);
// //   ID[0] = 1;
// //   ID[1] = 2;
// //   ID[2] = 3;
// //   ID[3] = 4;
// //   ID[4] = 5;
// //   ID[5] = 6;
// //   ID[6] = 7;
// //   ID[7] = 8;
// //   ACC[0] = 50;
// //   ACC[1] = 50;
// //   ACC[2] = 50;
// //   ACC[3] = 50;
// //   ACC[4] = 50;
// //   ACC[5] = 50;
// //   ACC[6] = 50;
// //   ACC[7] = 50;
// //   Speed[0] = 2000;
// //   Speed[1] = 2000;
// //   Speed[2] = 2000;
// //   Speed[3] = 2000;
// //   Speed[4] = 2000;
// //   Speed[5] = 2000;
// //   Speed[6] = 2000;
// //   Speed[7] = 2000;

// //   for (int i = 0; i < NUM_LEGS; i++) {
// //     for (int j = 0; j < NUM_POINTS * 4; j++) {
// //         angles[i][j][0] = 0.0;
// //         angles[i][j][1] = 0.0;
// //     }
// //  }
// // // go_to_zero();
// // // delay(3000);

// // }

// // void loop() {
// //   crawling_gait();
// //   delay(500);
// // }


// //  void go_to_zero() {
// //  pos[0] = 0;
// //  pos[1] = 0;
// //  pos[2] = 0;
// //  pos[3] = 0;
// //  pos[4] = 4096;
// //  pos[5] = 4096;
// //  pos[6] = 4096;
// //  pos[7] = 4096;
// //  st.SyncWritePosEx(ID ,8, pos, Speed, ACC);
// // }
// // //void IK_solver(float coord_X, float coord_Y, float* angles_rads);
// // //void move_to_point(float point_X, float point_Y);

// // // Define initial positions of the legs
// // // void initialize_positions() {
// // //     // Placeholder for actual initial positions
// // //     float initial_positions[NUM_LEGS][2] = {
// // //         {3, -10}, {3, -10}, {3, -10}, {3, -10}
// // //     };

// // //     for (int i = 0; i < NUM_LEGS; i++) {
// // //         move_to_point(initial_positions[i][0], initial_positions[i][1]);
// // //     }
// // // }

// // // Function to generate circular trajectory points and corresponding servo angles for a leg
// // //This function is checked- it moves from 3,-10 to 7,-10.
// // void generate_circular_trajectory(int leg_id, float radius, float angle_step, float trajectory[NUM_POINTS][2], float angles[NUM_POINTS][2]) {

// // const float centerX = 8.0;  // X-coordinate of circle center
// // const float centerY = -10.0; // Y-coordinate of circle center

// // for (int i = 0; i < NUM_POINTS; i++) {
// //     float angle =  (M_PI * (float)i / (NUM_POINTS - 1)); // Angles from -π to 0
// //     //need to change centre of circle according to translation along the x axis
// //     float new_centerX= trajectory[count-1][0] + radius;
// //     float x = new_centerX - radius * cos(angle);
// //         float y = centerY + radius * sin(angle);
// //         trajectory[count+i][0] = x;
// //         trajectory[count+i][1] = y;
// //         //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
// //         IK_solver(x, y, angles[count+i]);
        
// // }


// // }

// // // Function to generate straight-line trajectory points and corresponding servo angles for a leg
// // void generate_straight_line_trajectory(int leg_id, float start_x, float end_x, float y, float trajectory[NUM_POINTS][2], float angles[NUM_POINTS][2]) {
// //     float step = (end_x - start_x) / NUM_POINTS;
// //     for (int i = 0; i < NUM_POINTS; i++) {
// //         float x = start_x + i * step;
// //         trajectory[count+i][0] = x;
// //         trajectory[count+i][1] = y;
// //         //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
// //         IK_solver(x, y, angles[count+i]);
// //     }
// // }

// // // Main function to implement crawling gait
// // void crawling_gait() {
// //     //initialize_positions();

// //     float radius = 2.0;        // Example radius
// //     float distance = 10.0;     // Example distance
// //     float angle_step = 0.1;    // Example angle step
// //     float trajectories[NUM_LEGS][NUM_POINTS*4][2];  // Array to store trajectories
    
// //     trajectories[0][-1][0]=6;
// //     trajectories[1][-1][0]=6;
// //     trajectories[2][-1][0]=6;
// //     trajectories[3][-1][0]=6;

// //     for(int j= 0; j < 1; j++) { // Loop until stop condition
// //         for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {  
            
// //             // printf("Leg id: %d\n", leg_id);
// //            generate_circular_trajectory(leg_id, radius, angle_step, trajectories[leg_id], angles[leg_id]);
// //            //Serial.println("4");
           
// //             for (int other_leg_id = 0; other_leg_id < NUM_LEGS; other_leg_id++) {
// //                 if (other_leg_id != leg_id) {
// //                     generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count-1][0],trajectories[other_leg_id][count-1][0]-(5.0/3) , -10 , trajectories[other_leg_id], angles[other_leg_id]);
// //                 }
// //             }
// //             count=count+NUM_POINTS;
// //         }

// //     }

// //     //trajectories are stored correctly for all legs
// //     for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
// //         // Serial.print("Leg ID: %d\n");
// //         // Serial.println(leg_id);
// //         for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {

// //             // printf("Point %d: ,x = %f, y = %f, theta1: %f, theta5:%f\n", point_index,trajectories[leg_id][point_index][0], trajectories[leg_id][point_index][1],angles[leg_id][point_index][0],angles[leg_id][point_index][1]);
// //             // Serial.print("  Point, Theta1 , Theta5 = ");
// //             // Serial.print(point_index);
// //             // Serial.print(", ");
// //             // Serial.print(angles[leg_id][point_index][0]);
// //             // Serial.print(", ");
// //             // Serial.println(angles[leg_id][point_index][1]);
// //         }

// //     }



// //     for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {
// //           pos[0] = angles[0][point_index][0]* one_degree;
// //           pos[1] = (90-angles[0][point_index][1]) * one_degree;
// //           pos[2] = angles[1][point_index][0]* one_degree;
// //           pos[3] = angles[1][point_index][1] * one_degree;
// //           pos[4] = 4096 - angles[2][point_index][0] * one_degree;
// //           pos[5] = 4096 - angles[2][point_index][1] * one_degree;
// //           pos[6] = 4096 - angles[3][point_index][0] * one_degree;
// //           pos[7] = 4096 - angles[3][point_index][1] * one_degree;
// //           // for(int index = 0; index < 8; index++){
// //           //   Serial.print(index);
// //           //   Serial.print(" : ");
// //           //   Serial.println(pos[index]);
// //           // }

                      
// //           Serial.println("--------------------------------------");
// //           st.SyncWritePosEx(ID, 8 , pos, Speed, ACC);
// //           delay(100);


// //       //st.SyncWritePosEx(ID, 6 , pos, Speed, ACC);
// //     //st.SyncWritePosEx(ID, 8, pos, Speed, ACC);

// // }
// // }

// // void IK_solver(float coord_X, float coord_Y, float* angles_deg) {

// //     float k1 = (pow(coord_X, 2) + pow(l5, 2) + pow(l1, 2) + pow(coord_Y, 2) - pow(l6, 2) + 2 * coord_Y * l1) / (2 * l5);

// //     // Solve for Theta5 and Theta6
// //     float theta5 = 2 * atan2(coord_X + sqrt(pow((coord_Y + l1), 2) + pow(coord_X, 2) - pow(k1, 2)), (k1 - coord_Y - l1));
// //     float theta6 = atan2(-(coord_Y + l5 * cos(theta5) + l1), (-coord_X + l5 * sin(theta5)));

// //     // Find coordinates of point D
// //     float x2 = coord_X + (l4 + l6) * cos(theta6);
// //     float y2 = coord_Y + (l4 + l6) * sin(theta6);

// //     float k2 = (pow(x2, 2) + pow(y2, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l2);

// //     // Solve for Theta1
// //     float theta1 = 2 * atan2((x2 + sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - y2));

// //     angles_deg[0] = 203 - theta1 * (180.0 / M_PI);
// //     angles_deg[1] = 180 - theta5 * (180.0 / M_PI);

// //     printf("x:%f y:%f theta1:%f theta5:%f \n",coord_X,coord_Y,theta1,theta5);
     
// // }

// // // void move_to_point(float point_X, float point_Y) {
// // //     float angles_rads[2] = {0, 0};


// // //    // printf("Final motor positions: %f, %f\n", theta1_deg * one_degree, theta5_deg * one_degree);

// // //     Position[0][0] = theta1_deg * one_degree;
// // //     Position[0][1] = theta5_deg * one_degree;
// // //    // st.SyncWritePosEx(ID, 8, Position[0], Speed, ACC);
// // // }

// // // int main() {
// // //     crawling_gait();
    
// // //     return 0;
// // // }














// // #include <math.h>
// // #include <SCServo.h>

// // #define M_PI 3.14159265358979323846
// // #define NUM_LEGS 4
// // #define NUM_POINTS 5  // Number of points in the trajectory

// // const float l1 = 2.7;   // AB
// // const float l2 = 2.5;   // AE
// // const float l3 = 10;    // ED
// // const float l4 = 2.7;   // DC
// // const float l5 = 10;    // CB
// // const float l6 = 10.3;  // CF
// // const float one_degree = 11.37777777777778;

// // SMS_STS st;
// // #define S_RXD 18
// // #define S_TXD 19
// // byte ID[8];
// // s16 pos[8];
// // u16 Speed[8];
// // byte ACC[8];

// // int count=0;  //for storing values properly in the array


// // //trajectories array can mostly be removed

// // float Position[NUM_LEGS][2];  // Array to store positions of all legs
// // float angles[NUM_LEGS][NUM_POINTS*4][2];        // Array to store servo angles


// // void setup() {
// //   Serial.begin(115200);
// //   Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
// //   st.pSerial = &Serial1;
// //   delay(1000);
// //   ID[0] = 1;
// //   ID[1] = 2;
// //   ID[2] = 3;
// //   ID[3] = 4;
// //   ID[4] = 5;
// //   ID[5] = 6;
// //   ID[6] = 7;
// //   ID[7] = 8;
// //   ACC[0] = 50;
// //   ACC[1] = 50;
// //   ACC[2] = 50;
// //   ACC[3] = 50;
// //   ACC[4] = 50;
// //   ACC[5] = 50;
// //   ACC[6] = 50;
// //   ACC[7] = 50;
// //   Speed[0] = 2000;
// //   Speed[1] = 2000;
// //   Speed[2] = 2000;
// //   Speed[3] = 2000;
// //   Speed[4] = 2000;
// //   Speed[5] = 2000;
// //   Speed[6] = 2000;
// //   Speed[7] = 2000;


// //   go_to_zero();
// // }

// // void loop() {

// //   // crawling_gait();
// //   // delay(1000);

// // }
// // //void IK_solver(float coord_X, float coord_Y, float* angles_rads);
// // //void move_to_point(float point_X, float point_Y);

// // // Define initial positions of the legs
// // // void initialize_positions() {
// // //     // Placeholder for actual initial positions
// // //     float initial_positions[NUM_LEGS][2] = {
// // //         {3, -10}, {3, -10}, {3, -10}, {3, -10}
// // //     };

// // //     for (int i = 0; i < NUM_LEGS; i++) {
// // //         move_to_point(initial_positions[i][0], initial_positions[i][1]);
// // //     }
// // // }


// // void go_to_zero() {
// //  pos[0] = 0;
// //  pos[1] = 0;
// //  pos[2] = 0;
// //  pos[3] = 0;
// //  pos[4] = 4096;
// //  pos[5] = 4096;
// //  pos[6] = 4096;
// //  pos[7] = 4096;
// //  st.SyncWritePosEx(ID ,8, pos, Speed, ACC);
// // }
// // // Function to generate circular trajectory points and corresponding servo angles for a leg
// // //This function is checked- it moves from 3,-10 to 7,-10.
// // void generate_circular_trajectory(int leg_id, float radius, float angle_step, float trajectory[NUM_POINTS][2], float angles[NUM_POINTS][2]) {

// // const float centerX = 8.0;  // X-coordinate of circle center
// // const float centerY = -10.0; // Y-coordinate of circle center

// // for (int i = 0; i < NUM_POINTS; i++) {
// //     float angle =  (M_PI * (float)i / (NUM_POINTS - 1)); // Angles from -π to 0
// //     //need to change centre of circle according to translation along the x axis
// //     float new_centerX= trajectory[count-1][0] + radius;
// //     float x = new_centerX - radius * cos(angle);
// //         float y = centerY + radius * sin(angle);
// //         trajectory[count+i][0] = x;
// //         trajectory[count+i][1] = y;
// //         //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
// //         IK_solver(x, y, angles[count+i]);
        
// // }


// // }

// // // Function to generate straight-line trajectory points and corresponding servo angles for a leg
// // void generate_straight_line_trajectory(int leg_id, float start_x, float end_x, float y, float trajectory[NUM_POINTS][2], float angles[NUM_POINTS][2]) {
// //     float step = (end_x - start_x) / NUM_POINTS;
// //     for (int i = 0; i < NUM_POINTS; i++) {
// //         float x = start_x + i * step;
// //         trajectory[count+i][0] = x;
// //         trajectory[count+i][1] = y;
// //         //printf("Leg id: %d x: %f y:%f \n", leg_id,x,y);
// //         IK_solver(x, y, angles[count+i]);
// //     }
// // }

// // // Main function to implement crawling gait
// // void crawling_gait() {
// //     //initialize_positions();

// //     float radius = 2.0;        // Example radius
// //     float distance = 10.0;     // Example distance
// //     float angle_step = 0.1;    // Example angle step
// //     float trajectories[NUM_LEGS][NUM_POINTS*4][2];  // Array to store trajectories
// //     trajectories[0][-1][0]=6;
// //     trajectories[1][-1][0]=6;
// //     trajectories[2][-1][0]=6;
// //     trajectories[3][-1][0]=6;



// //     for(int j= 0; j < 1; j++) { // Loop until stop condition
// //         for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {  
            
// //             // printf("Leg id: %d\n", leg_id);
// //            generate_circular_trajectory(leg_id, radius, angle_step, trajectories[leg_id], angles[leg_id]);
// //            Serial.println("4");
           
// //             for (int other_leg_id = 0; other_leg_id < NUM_LEGS; other_leg_id++) {
// //                 if (other_leg_id != leg_id) {
// //                     generate_straight_line_trajectory(other_leg_id, trajectories[other_leg_id][count-1][0],trajectories[other_leg_id][count-1][0]-(5.0/3) , -10 , trajectories[other_leg_id], angles[other_leg_id]);
// //                 }
// //             }
// //             count=count+NUM_POINTS;
// //         }

// //     }


// //     //trajectories are stored correctly for all legs
// //     for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
// //         Serial.print("Leg ID: %d\n");
// //         Serial.println(leg_id);
// //         for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {

// //             // printf("Point %d: ,x = %f, y = %f, theta1: %f, theta5:%f\n", point_index,trajectories[leg_id][point_index][0], trajectories[leg_id][point_index][1],angles[leg_id][point_index][0],angles[leg_id][point_index][1]);
// //             Serial.print("  Point, Theta1 , Theta5 = ");
// //             Serial.print(point_index);
// //             Serial.print(", ");
// //             Serial.print(angles[leg_id][point_index][0]);
// //             Serial.print(", ");
// //             Serial.println(angles[leg_id][point_index][1]);
// //         }

// //     }



// //     for (int point_index = 0; point_index < NUM_POINTS*4; point_index++) {
// //           // pos[0] = angles[0][point_index][0]* one_degree;
// //           // pos[1] = angles[0][point_index][1] * one_degree;
// //           // pos[2] = angles[1][point_index][0]* one_degree;
// //           // pos[3] = angles[1][point_index][1] * one_degree;
// //           // pos[4] = angles[2][point_index][0]* one_degree;
// //           // pos[5] = angles[2][point_index][1] * one_degree;
// //                     for (int leg_id = 0; leg_id < NUM_LEGS*2; leg_id+=2){
// //                             pos[leg_id] = angles[leg_id][point_index][0]* one_degree;
// //                             pos[leg_id+1] = angles[leg_id][point_index][1] * one_degree;
// //                     }
// //                     // for(int index = 0; index < 8; index++){
// //                     //   Serial.print("Vals ");
// //                     //   Serial.print(pos[index]);
// //                     //   Serial.print(", ");

// //                     // }
// //                     Serial.println();
// //                     st.SyncWritePosEx(ID, 8 , pos, Speed, ACC);
// //                     delay(500);


// //       //st.SyncWritePosEx(ID, 6 , pos, Speed, ACC);
// //     //st.SyncWritePosEx(ID, 8, pos, Speed, ACC);

// // }
// // }

// // void IK_solver(float coord_X, float coord_Y, float* angles_deg) {

// //     float k1 = (pow(coord_X, 2) + pow(l5, 2) + pow(l1, 2) + pow(coord_Y, 2) - pow(l6, 2) + 2 * coord_Y * l1) / (2 * l5);

// //     // Solve for Theta5 and Theta6
// //     float theta5 = 2 * atan2(coord_X + sqrt(pow((coord_Y + l1), 2) + pow(coord_X, 2) - pow(k1, 2)), (k1 - coord_Y - l1));
// //     float theta6 = atan2(-(coord_Y + l5 * cos(theta5) + l1), (-coord_X + l5 * sin(theta5)));

// //     // Find coordinates of point D
// //     float x2 = coord_X + (l4 + l6) * cos(theta6);
// //     float y2 = coord_Y + (l4 + l6) * sin(theta6);

// //     float k2 = (pow(x2, 2) + pow(y2, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l2);

// //     // Solve for Theta1
// //     float theta1 = 2 * atan2((x2 + sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - y2));

// //     angles_deg[0] = 203 - theta1 * (180.0 / M_PI);
// //     angles_deg[1] = 180 - theta5 * (180.0 / M_PI);

// //     printf("x:%f y:%f theta1:%f theta5:%f \n",coord_X,coord_Y,theta1,theta5);
     
// // }

// // // void move_to_point(float point_X, float point_Y) {
// // //     float angles_rads[2] = {0, 0};


// // //    // printf("Final motor positions: %f, %f\n", theta1_deg * one_degree, theta5_deg * one_degree);

// // //     Position[0][0] = theta1_deg * one_degree;
// // //     Position[0][1] = theta5_deg * one_degree;
// // //    // st.SyncWritePosEx(ID, 8, Position[0], Speed, ACC);
// // // }

// // // int main() {
// // //     crawling_gait();
    
// // //     return 0;
// // // }





// // #include <math.h>
// // #include <SCServo.h>

// // // Link Constants
// // const float l1 = 2.7;   // AB
// // const float l2 = 2.5;   // AE
// // const float l3 = 10;    // ED
// // const float l4 = 2.7;   // DC
// // const float l5 = 10;    // CB
// // const float l6 = 10.3;  // CF
// // const float one_degree = 11.37777777777778;

// // const float offset1 = -70;
// // const float offset2 = -140;

// // // Motor Constraints
// // SMS_STS st;
// // #define S_RXD 18
// // #define S_TXD 19
// // byte ID[2];
// // s16 Position[2];
// // u16 Speed[2];
// // byte ACC[2];

// // // Trajectory parameters
// // const float centerX = 5.0;  // X-coordinate of circle center
// // const float centerY = -10.0; // Y-coordinate of circle center
// // const float radius = 2.0;    // Circle radius
// // const int numPointsLine = 50;    // Points for the straight line
// // const int numPointsCircle = 20;  // Points for the circular path
// // const int trajectoryDelay = 50;  // Delay between trajectory points (ms)

// // void setup() {
// //   Serial.begin(115200);
// //   Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
// //   st.pSerial = &Serial1;
// //   delay(1000);

// //   ID[0] = 3;
// //   ID[1] = 4;
// //   ACC[0] = 50;
// //   ACC[1] = 50;
// //   Speed[0] = 2000;
// //   Speed[1] = 2000;
// // }

// // void loop() {
// //  // go_to_zero();
// //   const int totalPoints = numPointsLine + numPointsCircle;
// //   float X_traj[totalPoints];
// //   float Y_traj[totalPoints];

// //   // Generate straight-line trajectory (clockwise direction)
// //   const float X_start = centerX - radius; // Start point
// //   const float Y_start = centerY;         // Start point
// //   const float X_end = centerX + radius;  // End point

// //   for (int i = 0; i < numPointsLine; i++) {
// //     float t = (float)i / (numPointsLine - 1); // Normalized progress
// //     X_traj[i] = X_start + t * (X_end - X_start);
// //     Y_traj[i] = Y_start; // Constant Y-coordinate
// //   }

// //   // Generate circular trajectory (clockwise direction)
// //   for (int i = 0; i < numPointsCircle; i++) {
// //     float theta =  (PI * (float)i / (numPointsCircle - 1)); // Angles from -π to 0
// //     X_traj[numPointsLine + i] = centerX + radius * cos(theta);
// //     Y_traj[numPointsLine + i] = (centerY + radius * sin(theta));
// //   }

// //   // Move through the combined trajectory
// //   for (int i = 0; i < totalPoints; i++) {
// //     move_to_point(X_traj[i], Y_traj[i]); // Move the foot to the trajectory point

// //     // Delay for smooth motion
// //     delay(trajectoryDelay);
// //   }

// //   // Optional pause between cycles
// //   delay(100);
// // }

// // void go_to_zero() {
// //  Position[0] = 341;
// //  Position[1] = 1024;
// //  st.SyncWritePosEx(ID ,2, Position, Speed, ACC);
// // }

// // void move_to_point(float point_X, float point_Y) {
// //   volatile float angles_rads[2] = {0, 0};

// //   Serial.print("Point to reach: ");
// //   Serial.print(point_X);
// //   Serial.print(", ");
// //   Serial.println(point_Y);

// //   IK_solver(point_X, point_Y, angles_rads);
// //  float theta1_deg = 180 - angles_rads[0] * (180.0 / M_PI);
// //  float theta5_deg = 180-angles_rads[1] * (180.0 / M_PI);
// //   Serial.print("Final motor positions: ");
// //   Serial.print(theta1_deg * one_degree);
// //   Serial.print(", ");
// //   Serial.println(theta5_deg * one_degree);

// //   Position[0] = (theta1_deg * one_degree);
// //   Position[1] = (theta5_deg * one_degree);
// //   st.SyncWritePosEx(ID, 8, Position, Speed, ACC);
// // }

// // void IK_solver(float coord_X, float coord_Y, volatile float* angles_rads) {
// //   float k1 = (pow(coord_X, 2) + pow(l5, 2) + pow(l1, 2) + pow(coord_Y, 2) - pow(l6, 2) + 2 * coord_Y * l1) / (2 * l5);

// //   // Solve for Theta5 and Theta6
// //   float theta5 = 2 * atan2(coord_X + sqrt(pow((coord_Y + l1), 2) + pow(coord_X, 2) - pow(k1, 2)), (k1 - coord_Y - l1));
// //   float theta6 = atan2(-(coord_Y + l5 * cos(theta5) + l1), (-coord_X + l5 * sin(theta5)));

// //   // Find coordinates of point D
// //   float x2 = coord_X + (l4 + l6) * cos(theta6);
// //   float y2 = coord_Y + (l4 + l6) * sin(theta6);

// //   float k2 = (pow(x2, 2) + pow(y2, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l2);

// //   // Solve for Theta1
// //   float theta1 = 2 * atan2((x2 + sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - y2));

// //   angles_rads[0] = theta1;
// //   angles_rads[1] = theta5;
// // }
