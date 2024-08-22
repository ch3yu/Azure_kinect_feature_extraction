#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <k4a.h>
#include <k4atypes.h>
#include <playback.h>
#include <k4abt.h>
#include <k4abttypes.h>

#define MAXCHAR 1000
#define NUM_PATICIPANTS 2
#define NUM_DATA 467


float joint_angle(k4a_float3_t end1, k4a_float3_t joint, k4a_float3_t end2) {
    //Cos(angle) = A*B / |A||B|
    k4a_float3_t A;
    A.xyz.x = end1.xyz.x - joint.xyz.x;
    A.xyz.y = end1.xyz.y - joint.xyz.y;
    A.xyz.z = end1.xyz.z - joint.xyz.z;
    k4a_float3_t B;
    B.xyz.x = end2.xyz.x - joint.xyz.x;
    B.xyz.y = end2.xyz.y - joint.xyz.y;
    B.xyz.z = end2.xyz.z - joint.xyz.z;

    float a_dot_b = (A.xyz.x * B.xyz.x) + (A.xyz.y * B.xyz.y) + (A.xyz.z * B.xyz.z);
    float mag_a = sqrt((A.xyz.x * A.xyz.x) + (A.xyz.y * A.xyz.y) + (A.xyz.z * A.xyz.z));
    float mag_b = sqrt((B.xyz.x * B.xyz.x) + (B.xyz.y * B.xyz.y) + (B.xyz.z * B.xyz.z));

    float angle = a_dot_b / (mag_a * mag_b);
    return angle;
}

// Determines how much all joints have moved.
float body_distance(k4abt_skeleton_t initial_skeleton, k4abt_skeleton_t skeleton) {

    float output_distance = 0;

    // Loop through all joints
    for(int joint=0; joint<K4ABT_JOINT_THUMB_RIGHT; joint++) {
        // Determine positions of joints with respect to naval joint.
        k4abt_joint_t initial_joint = initial_skeleton.joints[joint];
        k4abt_joint_t initial_joint_naval = initial_skeleton.joints[K4ABT_JOINT_SPINE_NAVEL];
        k4abt_joint_t curr_joint = skeleton.joints[joint];
        k4abt_joint_t curr_joint_naval = skeleton.joints[K4ABT_JOINT_SPINE_NAVEL];

        float x_initial = initial_joint.position.xyz.x - initial_joint_naval.position.xyz.x;
        float y_initial = initial_joint.position.xyz.y - initial_joint_naval.position.xyz.y;
        float z_initial = initial_joint.position.xyz.z - initial_joint_naval.position.xyz.z;

        float x_curr = curr_joint.position.xyz.x - curr_joint_naval.position.xyz.x;
        float y_curr = curr_joint.position.xyz.y - curr_joint_naval.position.xyz.y;
        float z_curr = curr_joint.position.xyz.z - curr_joint_naval.position.xyz.z;

        // Compare it against initial skeleton.
        float x_dist = x_curr - x_initial;
        float y_dist = x_curr - y_initial;
        float z_dist = z_curr - z_initial;

        float magnitude = sqrt((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist));

        output_distance += magnitude;
    }

    return output_distance;
    
}


//Calculates absolute distance (magnitude) between 2 coordinates
float abs_distance(k4a_float3_t A, k4a_float3_t B) {
  float x_dist = A.xyz.x - B.xyz.x;
  float y_dist = A.xyz.y - B.xyz.y;
  float z_dist = A.xyz.z - B.xyz.z;

  float magnitude = sqrt((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist));
  return magnitude;
}


int main(int argc, char* argv[]) {
    // Process input arguments from command line.
    // Input arguments are in the form as follow:
    //      <location> <num_actions> <participants> <action 1> <action 2> ... <action num_actions>
    char* location[1];
    location[0] = argv[1];
    int num_actions = atoi(argv[2]);
    int participants = atoi(argv[3]);

    // Check if input arguments are correct
    if((argc != num_actions+4) || (num_actions <= 0)) {
        printf("Expecting arguments --> <location> <num_actions> <participants> <action 1> <action 2> ... <action num_actions>\n");
        printf("num_vids and num_actions have to be greater than zero.\n");
        return 1;
    }

    // Create an array to store all action names.
    char* actions[num_actions];

    for(int i=0; i<num_actions; i++) {
        actions[i] = argv[i+4];
    }

    // Open the device for calibration.
    k4a_device_t device = NULL;
    if(k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED) {
        printf("Failed to open the device!\n");
        return 1;
    }

    // Configure the device.
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    // Obtain device calibration settings.
    k4a_calibration_t device_calib;
    if(k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution,     \
    &device_calib) != K4A_RESULT_SUCCEEDED) {                                                           \
        printf("Failed to get device calibration settings.\n");
        return 1;
    }

    // Close the device after getting device calibration.
    k4a_device_close(device);

    // Create a body tracker.
    k4abt_tracker_t bt_tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
    if(k4abt_tracker_create(&device_calib, tracker_config, &bt_tracker) != K4A_RESULT_SUCCEEDED) {
        printf("Failed to initialize a body tracker.\n");
        return 1;
    }

    // Create a playback handle.
    k4a_playback_t pb_handle = NULL;
    for(int current_action=1; current_action<=num_actions; current_action++) {
        // Specify the name of the video that is being processed in this loop.
        char video_name[7];
        sprintf(video_name, "%s%02d%02d", location[0], current_action, participants);
        printf("The video being processed now is: %s.mkv.\n", video_name);

        // Specify the path to the video.
        char video_path[60];
        sprintf(video_path, "..\\data\\%s.mkv", video_name);
        printf("The path to the current video is %s. \n", video_path);

        // Open the video.
        if(k4a_playback_open(video_path, &pb_handle) != K4A_RESULT_SUCCEEDED) {
            printf("Failed to open recording. Skip this video in the loop.\n");
            continue;
        }

        // Retrieve timestamps of the current file from timestamps.
        char timestamp_path[60];
        sprintf(timestamp_path, "..\\data\\timestamps_%02d.csv", participants);

        // Open the timestamp file.
        FILE* timestamp_file;
        timestamp_file = fopen(timestamp_path, "r");

        // Compare the name of the currently processed video against names in timestamp_file.
        // fgets() until the name in timestamp_file matches the currently processed video.
        char row[MAXCHAR];
        char* vid_name;  // vid_name is the name in the timestamp file.
        char* time_start;
        char* time_end;

        while(feof(timestamp_file) == 0) {
            fgets(row, MAXCHAR, timestamp_file);

            vid_name = strtok(row, ",");
            if(strcmp(vid_name, video_name) ==0) {
                time_start = strtok(NULL, ",");
                time_end = strtok(NULL, ",");
                break;
            }
        }
        fclose(timestamp_file);

        int action_cnt = 1;
        int frame_cnt = 1;

        float move_distance_1 = 0;
        float move_distance_2 = 0;

        k4abt_skeleton_t initial_skeleton;
        
        k4abt_skeleton_t prev_skeleton;
        float prev_angles[32];
        uint64_t prev_time;

        /*******************************************************************************
         * There are 419 data points for each frame, which are composed of the xyz
         * coordinates of joints (32*3*2), velocity of joints in 3 different directions
         * (32*3*2), angles of 13 joints (13*2), 5 distances between joints (5*2), 
         * and 1 distance between two participants. 
        *******************************************************************************/
        // Start retrieving captures and analyzing them frame by frame.
        k4a_capture_t capture = NULL;
        k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
        while(result == K4A_STREAM_RESULT_SUCCEEDED) {
            // Get the next capture.
            result = k4a_playback_get_next_capture(pb_handle, &capture);
            if(result == K4A_STREAM_RESULT_SUCCEEDED) {
                // Enqueue capture into tracker queue and release the capture once completed.
                k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(bt_tracker, capture, K4A_WAIT_INFINITE);
                if(queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED) {
                    printf("Failed to enqueue capture into tracker.\n");
                    k4a_capture_release(capture);
                    continue;
                }
                k4a_capture_release(capture);

                // Pop processed capture out of tracker's queue into a frame.
                k4abt_frame_t bframe = NULL;
                k4a_wait_result_t pop_result = k4abt_tracker_pop_result(bt_tracker, &bframe, K4A_WAIT_INFINITE);
                if(pop_result != K4A_WAIT_RESULT_SUCCEEDED) {
                    printf("Failed to pop a capture out of tracker's queue.\n");
                    break;
                }

                // Compare timestamps to check if it falls in between time span of interest.
                int time_start_int = atoi(time_start);
                int time_end_int = atoi(time_end);
                uint64_t time = k4abt_frame_get_device_timestamp_usec(bframe);

                k4abt_skeleton_t skeleton;

                FILE* joints_file;
                char csv_path[60];

                if((time >= time_start_int) && (time <= time_end_int)) {

                    printf("Currently processing video between %d and %d.\n", time_start_int, time_end_int);

                    // Open the csv file to write.
                    sprintf(csv_path, "..\\result\\%s_%d_%d.csv", video_name, time_start_int, time_end_int);
                    joints_file = fopen(csv_path, "a");

                    size_t num_bodies = k4abt_frame_get_num_bodies(bframe);
                    if(num_bodies < 2) {
                        
                        // Write all zeros to the csv file for this frame.
                        for(int i=0; i<NUM_DATA; i++) {
                            fprintf(joints_file, "%f,", 0);
                        }
                    }
                    else {
                        /*********************************************************
                        // Fetch two skeletons with best skeletons.
                        int best_skeleton = 0;
                        int second_skeleton = 0;
                        int best_skeleton_conf = 0;
                        int second_skeleton_conf = 0;

                        for(size_t i=0; i<num_bodies; i++) {
                            k4abt_skeleton_t skeleton;
                            k4abt_frame_get_body_skeleton(bframe, i, &skeleton);
                            int total_confidence = 0;

                            for(int j=0; j<K4ABT_JOINT_COUNT; j++) {
                                total_confidence = total_confidence + skeleton.joints[j].confidence_level;
                            }

                            if(total_confidence >= best_skeleton_conf) {
                                second_skeleton = best_skeleton;
                                second_skeleton_conf = best_skeleton_conf;
                                best_skeleton = i;
                                best_skeleton_conf = total_confidence;
                            }
                            else if((total_confidence>=second_skeleton_conf) && (total_confidence<best_skeleton_conf)) {
                                second_skeleton = i;
                                second_skeleton_conf = total_confidence;
                            }
                        }
                        ***********************************************************/
                    
                        // Write joint information to csv file.
                        k4abt_skeleton_t skeleton_first;
                        k4abt_skeleton_t skeleton_second;

                        for(int i=0; i<NUM_PATICIPANTS; i++) {
                            k4abt_frame_get_body_skeleton(bframe, i, &skeleton);

                            if(frame_cnt == 1) {
                                initial_skeleton = skeleton;
                            }

                            if(i == 0) {
                                move_distance_1 += body_distance(initial_skeleton, skeleton);
                            }
                            else if(i == 1) {
                                move_distance_2 += body_distance(initial_skeleton, skeleton);
                            }

                            // Joint xyz in Human Coordinate System (all joints are measured against Spine Navel).
                            for(int j=0; j<K4ABT_JOINT_COUNT; j++) {
                                k4abt_joint_t curr_joint = skeleton.joints[j];
                                k4abt_joint_t spine_naval = skeleton.joints[K4ABT_JOINT_SPINE_NAVEL];

                                float x = curr_joint.position.xyz.x - spine_naval.position.xyz.x;
                                float y = curr_joint.position.xyz.y - spine_naval.position.xyz.y;
                                float z = curr_joint.position.xyz.z - spine_naval.position.xyz.z;

                                fprintf(joints_file, "%f,%f,%f,", x, y, z);
                            }

                            // Joint Velocities
                            for(int j=0; j<K4ABT_JOINT_COUNT; j++) {
                                float v_x;
                                float v_y;
                                float v_z;

                                if(time == time_start_int) {
                                    v_x = 0;
                                    v_y = 0;
                                    v_z = 0;
                                }
                                else {
                                    k4abt_joint_t curr_joint = skeleton.joints[j];
                                    k4abt_joint_t prev_joint = prev_skeleton.joints[j];
                                    uint64_t delt_t = time - prev_time;

                                    v_x = (curr_joint.position.xyz.x - prev_joint.position.xyz.x) / delt_t;
                                    v_y = (curr_joint.position.xyz.y - prev_joint.position.xyz.y) / delt_t;
                                    v_z = (curr_joint.position.xyz.z - prev_joint.position.xyz.z) / delt_t;
                                }

                                fprintf(joints_file, "%f,%f,%f,", v_x, v_y, v_z);
                            }

                            for(int j=0; j<K4ABT_JOINT_COUNT; j++) {
                                k4abt_joint_t curr_joint = skeleton.joints[j];
                                float curr_angle = 0;
                                float angular_velocity = 0;
                                uint64_t delt_t = time - prev_time;

                                if(j == K4ABT_JOINT_NECK) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_HEAD];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_SPINE_NAVEL) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_PELVIS];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_ELBOW_RIGHT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_WRIST_RIGHT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_ELBOW_LEFT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_WRIST_LEFT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_WRIST_RIGHT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_HAND_RIGHT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_WRIST_LEFT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_HAND_LEFT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_SHOULDER_RIGHT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_SHOULDER_LEFT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT];
                                    curr_angle = joint_angle(end1.position, curr_joint.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_THUMB_RIGHT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_WRIST_RIGHT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT];
                                    curr_angle = joint_angle(curr_joint.position, end1.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j == K4ABT_JOINT_THUMB_LEFT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_WRIST_LEFT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT];
                                    curr_angle = joint_angle(curr_joint.position, end1.position, end2.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j ==K4ABT_JOINT_HIP_RIGHT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT];
                                    curr_angle = joint_angle(end1.position, end2.position, curr_joint.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                else if(j ==K4ABT_JOINT_HIP_LEFT) {
                                    k4abt_joint_t end1 = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT];
                                    k4abt_joint_t end2 = skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT];
                                    curr_angle = joint_angle(end1.position, end2.position, curr_joint.position);
                                    
                                    if(time != time_start_int) {
                                        angular_velocity = (curr_angle - prev_angles[j]) / delt_t;
                                    }

                                    fprintf(joints_file, "%f,%f,%hu,", curr_angle, angular_velocity, curr_joint.confidence_level);
                                }
                                prev_angles[j] = curr_angle;
                            }

                            // Acquire distances of one target.
                            k4a_float3_t hand_left = skeleton.joints[K4ABT_JOINT_HAND_LEFT].position;
                            k4a_float3_t hand_right = skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position;
                            k4a_float3_t head = skeleton.joints[K4ABT_JOINT_HEAD].position;
                            k4a_float3_t chest = skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position;

                            float dist1 = abs_distance(hand_left, chest);
                            float dist2 = abs_distance(hand_right, chest);
                            float dist3 = abs_distance(hand_left, head);
                            float dist4 = abs_distance(hand_right, head);
                            float dist5 = abs_distance(hand_left, hand_right);

                            fprintf(joints_file, "%f,%f,%f,%f,%f,", dist1, dist2, dist3, dist4, dist5);

                            // Get the distance between chest joints of two participants.
                            if(i == 0) {
                                k4abt_frame_get_body_skeleton(bframe, i, &skeleton_first);
                            }
                            else {
                                k4abt_frame_get_body_skeleton(bframe, i, &skeleton_second);
                                k4a_float3_t chest_first = skeleton_first.joints[K4ABT_JOINT_SPINE_NAVEL].position;
                                k4a_float3_t chest_second = skeleton_second.joints[K4ABT_JOINT_SPINE_NAVEL].position;
                                float dist6 = abs_distance(chest_first, chest_second);
                                fprintf(joints_file, "%f", dist6);
                            }
                        }
                    }

                    fprintf(joints_file, "\n");
                    fclose(joints_file);

                    frame_cnt += 1;
                }

                k4abt_frame_release(bframe);
                

                if(time >= time_end_int) {
                    // Determine the initiator.
                    char new_csv_path[60];
                    if(move_distance_1 >= move_distance_2) {
                        sprintf(new_csv_path, "..\\result\\%s_%d_%d_%d.csv", video_name, 1, time_start_int, time_end_int);
                    }
                    else {
                        sprintf(new_csv_path, "..\\result\\%s_%d_%d_%d.csv", video_name, 2, time_start_int, time_end_int);
                    }
                    // Change the name of the file to include information about the initiator
                    rename(csv_path, new_csv_path);

                    // Determine next time span.
                    time_start = strtok(NULL, ",");
                    time_end = strtok(NULL, ",");
                    action_cnt += 1;
                    frame_cnt = 1;

                    move_distance_1 = 0;
                    move_distance_2 = 0;

                    printf("\n");

                    bool break_condition_1 = (time_start == NULL) || (time_start == " ");
                    bool break_condition_2 = (time_end == NULL) || (time_end == " ");
                    if(break_condition_1 || break_condition_2) {
                        break;
                    }
                }
                prev_skeleton = skeleton;
                prev_time = time;
            }
            else if(result == K4A_STREAM_RESULT_EOF) {
                printf("End of file reached.\n");
                break;
            }
            else if(result == K4A_STREAM_RESULT_FAILED) {
                printf("Error getting frame number %d for video %s. Skipping over for now.\n");
                result = K4A_STREAM_RESULT_SUCCEEDED;
            }
        }

        k4a_playback_close(pb_handle);	
    }

    k4abt_tracker_shutdown(bt_tracker);
    k4abt_tracker_destroy(bt_tracker);
    
}