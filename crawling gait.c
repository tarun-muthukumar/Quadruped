
// #include <math.h>
// #include <stdio.h>

// #define M_PI 3.14159265358979323846
// #define NUM_LEGS 4
// #define NUM_POINTS 5  // Number of points in the trajectory
//Circular trajectory Crawling Gait- Working Code
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define M_PI 3.14159265358979323846
#define NUM_LEGS 4
#define NUM_POINTS 3  // Number of points in the trajectory



const float l1 = 3.0;   // AB
const float l2 = 3.5;   // AE
const float l3 = 10.5;    // ED
const float l4 = 2.7;   // DC
const float l5 = 9.8;    // CB
const float l6 = 10.5;  // CF
const float one_degree = 11.37777777777778;

int count=0;  //for storing values properly in the array
//int leg_order[NUM_LEGS]={3,0,1,2};


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

     printf("x:%f y:%f theta1:%f theta5:%f \n",coord_X,coord_Y,angles_deg[0],angles_deg[1]);
}


int main() {
    angles = (float ***)malloc(NUM_LEGS * sizeof(float **));
    trajectories = (float ***)malloc(NUM_LEGS * sizeof(float **));
    for (int i = 0; i < NUM_LEGS; i++) {
        angles[i] = (float **)malloc(NUM_POINTS * 4 * sizeof(float *));
        trajectories[i] = (float **)malloc(NUM_POINTS * 4 * sizeof(float *));
        for (int j = 0; j < NUM_POINTS * 4; j++) {
            trajectories[i][j] = (float *)malloc(2 * sizeof(float));
            angles[i][j] = (float *)malloc(2 * sizeof(float));
        }
    }

    crawling_gait();

    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < NUM_POINTS * 4; j++) {
            free(trajectories[i][j]);
        }
        free(trajectories[i]);
        free(angles[i]);
    }
    free(trajectories);
    free(angles);
    return 0;
}
