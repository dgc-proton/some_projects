// Kinematics code written by Zayne, with some input from Dave so that compiles
// without warnings and the binary can be sucessfully called from the main Python script.

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

struct positions
{
    double x;
    double y;
};

// affector_position[0] and affector_position[1] are the x and y coordinate positions of the end effector on the air hockey table grid
struct positions affector_position;

// link_length_x are the lengths of the links from centre to centre, with 1 being the left most link and 4 being the right most link
double link_length_1, link_length_2, link_length_3, link_length_4;

// motor_1 and motor_2 are arrays which describe the position of the motors on the grid space
struct positions motor_1, motor_2;

// These are the left and right motor angles, respectively
double theta_1, theta_2;

double calc_distance(struct positions* point1, struct positions* point2)
{
    double length = sqrt(pow((point2->x - point1->x), 2) + pow((point2->y - point1->y) , 2));
    return length;
}

void position_end_affector(int in_Xcoord, int in_Ycoord)  // Function used to set the coordinates of the end affector
{

    affector_position.x = in_Xcoord;
    affector_position.y = in_Ycoord;
}

bool kinematics()
{
    double a1, a2, d1, d2, h1, h2, h1x, h1y, h2x, h2y;
    struct positions O3, O4, O5, O6;

    // d1 and d2 represent the magnitude of the vector from left motor and right motor to the end affector respectively
    d1 = calc_distance(&motor_1, &affector_position);
    d2 = calc_distance(&motor_2, &affector_position);


    if( (d1 > (link_length_1+link_length_2)) || (d2 > (link_length_3+link_length_4)) )
        {
            printf("\nError, arms too short\n");
            return false;
        }

    // Calculating the angle between links 1&2 and again between links 3&4 
    double left_joint_angle = acos((pow(link_length_1, 2) + pow(link_length_2, 2) - pow(d1, 2))/(2*link_length_1*link_length_2)) * 180/(3.14159265359);
    double right_joint_angle= acos((pow(link_length_3, 2) + pow(link_length_4, 2) - pow(d2, 2))/(2*link_length_3*link_length_4)) * 180/(3.14159265359);

    if(left_joint_angle<65||right_joint_angle<65)
    {
        printf("\nError, joint angles too tight\n");
        return false;
    }

    // a1 and a2 represent the magnitude of the adjacent side of the right angled triangle formed by link1 and link4 along the path of d1 and d2
    a1 = ((pow(link_length_1, 2) - pow(link_length_2, 2) + pow(d1, 2)))/(2*d1); 
    a2 = ((pow(link_length_4, 2) - pow(link_length_3, 2) + pow(d2, 2)))/(2*d2);


    // h1 and h2 represent the magnitude of the opposite side of the right angled triangle formed by link 1 and link4 and a1 and a2 respectively
    h1 = sqrt(pow(link_length_1, 2) - pow(a1, 2));
    h2 = sqrt(pow(link_length_4, 2) - pow(a2, 2));

    // h1 x and y are the projections made by the value of h1
    h1x = -h1 * (affector_position.y - motor_1.y)/d1;
    h1y = h1 * (affector_position.x - motor_1.x)/d1;

    // h2 x and y are the projections made by the value of h2
    h2x = -h2 * (affector_position.y - motor_2.y)/d2;
    h2y = h2 * (affector_position.x - motor_2.x)/d2;
        
    // O3 is the position at the end of vector a1
    O3.x = motor_1.x + a1*(affector_position.x - motor_1.x)/d1;  
    O3.y = motor_1.y + a1*(affector_position.y - motor_1.y)/d1;       

    // O5 is the position at the end of vector a2
    O5.x = motor_2.x + a2*(affector_position.x - motor_2.x)/d2;  
    O5.y = motor_2.y + a2*(affector_position.y - motor_2.y)/d2;   

    // Calculating the positions of the joints O4 (left hand arm) and O6 (right and arm) by adding the h1 vector to the O3 vector
    if(affector_position.y>(O3.y - h1y))
    {
        O4.x = O3.x + h1x;
        O4.y = O3.y + h1y;
    }else
    {
        O4.x = O3.x - h1x;
        O4.y = O3.y - h1y;
    }
    if(affector_position.y<(O5.y - h2y))
    {
        O6.x = O5.x + h2x;
        O6.y = O5.y + h2y;
    }else
    {
        O6.x = O5.x - h2x;
        O6.y = O5.y - h2y;  
    }

    double End_effector_angle = atan((pow(link_length_2, 2)+pow(link_length_3, 2)-pow(calc_distance(&O4, &O6),2))/(2*link_length_2*link_length_3));
        if(End_effector_angle<65.00)
        {
            O6.x = O5.x + h2x;
            O6.y = O5.y + h2y;
        }

    // theta_1 and theta_2 are the output angle orientations for link1 and link4 respectively
    // Adding 90 degrees allows the creation of an absolute step number
    theta_1 = atan((O4.y - motor_1.y)/(O4.x - motor_1.x)) * 180/(3.14159265359) + 90.00;
    theta_2 = atan((O6.y - motor_2.y)/(O6.x - motor_2.x)) * 180/(3.14159265359) + 90.00;

    if(O4.x<motor_1.x)
    {theta_1 = 180 + theta_1;}

    if(O6.x<motor_2.x)
    {theta_2 = 180 + theta_2;}

    // cout<<"Left motor angle: "<<theta_1<<"; Equals to (in step number): "<<theta_1/1.8<<" steps"<<endl<<"Right motor angle: "<<theta_2<<"; Equals to (in step number): "<<theta_2/1.8<<" steps"<<endl;
    // cout<<"Left joint angle: "<<left_joint_angle<<endl<<"Right joint angle: "<<right_joint_angle<<endl;


    return true;
}



//
////
//////
// How to use:
// Call the function set_up(). This takes as arguments four integer values which are the four link lengths
// It also sets the motor positions

// Next call the function get_steplists() which is what will return the list of steps

//////
////
//




void set_up(double link1, double link2, double link3, double link4, double motor1x, double motor1y, double motor2x, double motor2y)
{
    // Position of the motors on the grid in cm
    motor_1.x = motor1x;  // -183.00;
    motor_1.y = motor1y;  // -110.00;
    motor_2.x = motor2x;  // 183.00;
    motor_2.y = motor2y;  // -110.00;

    // Size of the link lengths
    link_length_1 = link1;
    link_length_2 = link2;
    link_length_3 = link3;
    link_length_4 = link4;
}  

// Returns true if route found, else false. Returns lists of motor steps and the actual size of each list using pointers.
bool get_steplists(int* left_list, int* right_list, int max_listsize, int* actual_listsize, int curr_lmotor_step, int curr_rmotor_Step, int desired_x_coord, int desired_y_coord)
{
    const double def_list_len = 20.0;
    int list_size = 0;
    double temp_left, temp_right;

    int start_step_1 = curr_lmotor_step;
    int start_step_2 = curr_rmotor_Step;

    position_end_affector(desired_x_coord, desired_y_coord);

    if(!kinematics())
    {  

        printf("\nmotor_1_x: %f\nmotor_1_y: %f\n",motor_1.x, motor_1.y);
        printf("\naffector_position_x: %f\naffector_position_y: %f\n",affector_position.x, affector_position.y);  
        return false;
    }
    else
    {
        // The end affector is set to the desired position input by x and y, and then the kinematics are run in order to get the next* angle configuration
        kinematics();
        
        // Below are the variables that tell the final step position
        int final_step_1 = (int)(theta_1)/1.8;
        int final_step_2 = (int)(theta_2)/1.8;

        for(int i=0; i<(int)def_list_len;i++)
        {
            list_size++;
            if(list_size > max_listsize) {
                printf("\nError get_steplists() has produced an angle list larger than the maximum allowed\n");
                return false;
            }
            if(i==0)                    // INITIAL ENTRY
            {
                temp_left = start_step_1;
                left_list[i] = start_step_1;
                temp_right = start_step_2;
                right_list[i] = start_step_2;
            }
            else if (i==(int)def_list_len-1) // FINAL ENTRY
            {
                left_list[i] = final_step_1;
                right_list[i] = final_step_2;
            }
            else                        // INTERMEDIARY ENTRIES
            {
                temp_left = temp_left + (final_step_1 - start_step_1)/def_list_len;
                left_list[i] = (int)round(temp_left);

                temp_right = temp_right + (final_step_2 - start_step_2)/def_list_len;
                right_list[i] = (int)round(temp_right);
            }
        }
        *actual_listsize = list_size;
        return true;
    }
}
