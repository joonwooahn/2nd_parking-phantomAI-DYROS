
#include "../inc/LocalPlannerThread.h"
#include "../inc/gear_ratio_table.data"  //add_
 
 
#define REPLANNING false // jw add
#define LOGGING false   // jw add

#define KMpH2MpS 0.277777
#define Mps2KMpH 3.6
#define REF_VEL 6.0*KMpH2MpS

#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))
#define SIGN(x) ((x >= 0) ? 1 : -1)
#define ONENORM(x, x_max) ((x < x_max) ? (x/x_max) : 1.0)
#define FLOAT2(x) (double)((int)(x*100))/100
 
#define _RAD2DEG 180 / M_PI
#define _DEG2RAD M_PI / 180

#define DEBUG_COUT false
#define MIN_DIST 3.0    //3.9*1.5    

#define TRUN_RADIUS 6.0
#define CURVATURE_MAX 1.0/TRUN_RADIUS
#define WHEEL_BASE  2.700 // 2.845 for grandeur
#define center2rear -1.096 // -1.115 for recent vestella lab test  //-1.3425 // 1.34 for grandeur /// add

#define MIN_VEL_INPUT 0.9
#define MAX_VEL_INPUT 1.7

// add ---------------------------------------------
#define vehicle2switchingPointDist 0.139  //0.12, 0.15
#define vehicle2goalPointDist 0.10  

#define LIMIT_DEGREE 300.0    //180.0 //180.0          // [deg] per dt
#define LIMIT_DEGREE_SLOW 210.0      // [deg] per dt
double limitDeg = LIMIT_DEGREE;     // [deg] per dt
#define CONSTANT_GEAR_RATIO false   // add

#define limitVel 2.0                // [km/h]   per dt
#define limitAcc 1.0  // [km/h^2]   per dt
#define WAITING_TIME_AT_SWITCHING_PT 0.0

// sample collision check parameter
#define SAFEREGION 1.2 // originally 1.1  Scenario2 --> 1.975/2.0 // orig: 1.975/2.0+0.1 == 1.05875
#define COLLISION_RADIUS 1.34  // CARLA: 1.3345 | 0.15 [cm] free_space: 1.32
#define POINTSAFEREGION1 0.22  //        0.255  |                       0.22
#define POINTSAFEREGION2 0.22  //        0.26   |                       0.22
#define   SIDESAFEREGION 0.21  //        0.20   |
#define CENTER_TO_REAR 1.4204513136643668

// LocalPlannerThread Constructor
LocalPlannerThread::LocalPlannerThread(int argc, char** argv)
    :init_argc(argc), init_argv(argv) {

    ros::init(init_argc, init_argv, "LocalPlannerThread");
    ros::NodeHandle nh_;

    Pub_MarkerLookaheadPtJW = nh_.advertise<visualization_msgs::Marker>("LookAheadPosJW", 1);
    Pub_MarkerCar = nh_.advertise<visualization_msgs::Marker>("markerVehicleTraj", 1);
    Pub_poseVehicle = nh_.advertise<geometry_msgs::PoseStamped>("poseVehicle", 1);
    Pub_CarPw = nh_.advertise<visualization_msgs::Marker>("Car_pw", 1);
    Pub_vehicleTraj = nh_.advertise<nav_msgs::Path>("vehicle_traj", 1);

    Pub_tentacle = nh_.advertise<geometry_msgs::PoseArray>("poseArrayTentacle", 1);
    Pub_chosenPath = nh_.advertise<geometry_msgs::PoseArray>("poseArraychosenPath", 1);
    Pub_chosenPathbyValue = nh_.advertise<geometry_msgs::PoseArray>("poseArraychosenPathbyValue", 1);

    Pub_ControlCmd = nh_.advertise<std_msgs::Float32MultiArray>("Control_Command", 1);
    // Pub_Replanning = nh_.advertise<std_msgs::Float32MultiArray>("replanning", 1);
    Pub_MarkerParkingPath = nh_.advertise<visualization_msgs::MarkerArray>("markerParkingPath", 10);
    Pub_temp_path = nh_.advertise<nav_msgs::Path>("temp_path_", 10);//for extension path

    // Pub_velocity = nh_.advertise<std_msgs::Float32MultiArray>("velocity_command", 1);
    Sub_LearningJW = nh_.subscribe("learningData", 10, &LocalPlannerThread::CallbackLearningData, this);
    Sub_localization = nh_.subscribe("/LocalizationData", 10, &LocalPlannerThread::CallbackLocalizationData, this);
    Sub_parkingPath = nh_.subscribe("parkingPath", 1, &LocalPlannerThread::CallbackParkingPath, this);// From doRRT node
    // Sub_driving_function = nh_.subscribe("/motion_planning_data", 10, &LocalPlannerThread::CallbackModelBasedDrivingData, this);

    MemberValInit();
    // cout << "LocalPlannerThreaded constructor" <<endl;
    // m_error_log.open("/home/dyros-vehicle/ms_ws/20210902/error_data.csv");  // MS: add for logging path-tracking test results
}
 
void LocalPlannerThread::MemberValInit() {
    MarkerInit();

    m_pathIdx = 0;
    m_carIdx = 0;
    m_pLIdx = 0;
    m_Steer_cmd = 0;
    m_acc = 0;
    m_brake = 0;
    m_I_brake = 0.0;
    m_I_accel = 0.0;
    m_orthogonal_val = 0.0;
    m_velocityGain = 1.0;
    m_targetVelocity = 1.0;
    m_controlVelocity = 0.0;
    m_maxSearchIdx = 0;
    // for PI controller (velocity)
    m_I_brake = 0;
    m_I_accel = 0;
    m_finishFlag = false;
    m_switchLen = 0;
    m_parkingPath.id = 0;
    m_pathFlag = false; 
    m_navi_info = 0;
    m_intersection_flag = 0;

    // MSK
    m_prev_steering = 0;
    m_prev_velocity = 0;
    m_diff_threshold = 150; //[deg]
    m_cumulative_error = 0;
    m_curvature = 0;
    m_counter = 0;
    m_using_tracking_strategy = false;
    m_cross_track_error = 0.0;
    m_cross_track_error_mean = 0.0;    
    count_cte = 1.0;
    m_carIdx_prev = 0;
    m_prev_curv = 0.0;
    m_car2switchPt = 0.0;
    m_dir_mode = 1.0;
    

    for (int i = 0 ; i < 99 ; i++) {
        m_LocalSplinePath[i].clear();
    }
    m_parkingPathArr.markers.clear();
    m_parkingPath.points.clear();
    m_parkingPath.colors.clear();

    m_CarPosLine.points.clear();
    m_ros_time = ros::Time::now();  // add
    m_ros_time2 = ros::Time::now();
    m_cum_distance = 0.0;
    
    m_CarPosLine.points.push_back(m_CarPos.pose.position);
    if (Pub_MarkerCar.getNumSubscribers() > 0)
        Pub_MarkerCar.publish(m_CarPosLine);

    m_switching_point_vector.clear();
    m_switching_point_idx.clear();
    m_waypt_seg.clear();
}

// Decstructor
LocalPlannerThread::~LocalPlannerThread() {

}

void LocalPlannerThread::CallbackLearningData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (!m_pathFlag)
        vehicleControl(msg->data.at(0), 0.0);
}

// void LocalPlannerThread::CallbackModelBasedDrivingData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//     if (!m_pathFlag) // while driving
//         vehicleControl(msg->data.at(0), 0.0);
// }

void LocalPlannerThread::CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // // cout << "LOCALIZATION CALLBACK!" << endl;
    m_car.x = m_car_x = msg->data.at(0);      // x
    m_car.y = m_car_y = msg->data.at(1);      // y
    m_car.th = msg->data.at(2);     // theta
    m_car.vel = msg->data.at(3)*KMpH2MpS;    // to [m/s]
    
    // Set a center to rear axis from COM
    m_car_center_x = m_car_x;
    m_car_center_y = m_car_y;
    // m_car.x = m_car_x + center2rear*cos(m_car.th);
    // m_car.y = m_car_y + center2rear*sin(m_car.th);

    // Car Red (Car)
    m_CarPos.header.stamp = ros::Time::now();
    m_CarPos.pose.position.x = m_car.x; //carX;//m_car.x;
    m_CarPos.pose.position.y = m_car.y; //carY;//m_car.y;

    // Create the vehicle's AXIS
    geometry_msgs::PoseStamped poseStamped;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    poseStamped.pose.position = m_CarPos.pose.position;
    poseStamped.pose.position.z = 0.5;
    poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(m_car.th);
    poseStamped.header = header;

    if (Pub_poseVehicle.getNumSubscribers() > 0)
        Pub_poseVehicle.publish(poseStamped);

    if (m_pathFlag && !m_finishFlag) {//PATH IS GENERATED, WHICH CONSISTS OF PATH SEGMENTS
        m_controlVelocity = 6*KMpH2MpS;
        Compute();
    }
    // else {
    //     vehicleControl(0.0, 0.0);   //Publish! add for parking. When parking is finished, the vehicle's steering becomes zero
    // }
    // vehicleTrajectory(1.5);    // visualization! based on the vehicle's kinematic model
}

std::pair<vector<double>, vector<double>> ComputeCurvature(vector<Vector3d> path, bool rs_path) {

    vector<double> cum_distance;
    vector<double> curvature;
    double distance_s = 0;
    
    std::ofstream myfile;
    // if (rs_path)
    //     myfile.open("/home/dyros-vehicle/catkin_ws/RStest.csv");
    // else
    //     myfile.open("/home/dyros-vehicle/catkin_ws/HCtest.csv");        

    for(int i = 1; i < path.size()-1 ; i++) {
        float fAreaOfTriangle = fabs((path[i-1][0] * (path[i][1] - path[i+1][1]) +\
                                        path[i][0] * (path[i+1][1] - path[i-1][1]) +\
                                        path[i+1][0] * (path[i-1][1] - path[i][1])) / 2);
        float fDist12 = sqrt((path[i][0] - path[i-1][0]) * (path[i][0] - path[i-1][0]) + (path[i][1] - path[i-1][1]) * (path[i][1] - path[i-1][1]));
        float fDist23 = sqrt((path[i][0] - path[i+1][0]) * (path[i][0] - path[i+1][0]) + (path[i][1] - path[i+1][1]) * (path[i][1] - path[i+1][1]));
        float fDist13 = sqrt((path[i+1][0] - path[i-1][0]) * (path[i+1][0] - path[i-1][0]) + (path[i+1][1] - path[i-1][1]) * (path[i+1][1] - path[i-1][1]));
        float fKappa = 4 * fAreaOfTriangle / (fDist12 * fDist23 * fDist13);
        
        //cross product
        double first_vec_x =path[i][0] - path[i-1][0];
        double first_vec_y =path[i][1] - path[i-1][1];
        double second_vec_x =path[i+1][0] - path[i][0];
        double second_vec_y =path[i+1][1] - path[i][1];
        double cross_product = first_vec_x*second_vec_y - second_vec_x*first_vec_y;
        int sign_ = (cross_product>=0) ? 1 : -1;
        int direc_ = path[i][2];
        if (!std::isnan(fKappa)) {
            distance_s += fDist12;
            if (fKappa > 0.25) fKappa = 0.0;
            else if (fKappa < -0.25) fKappa = 0;
            cum_distance.push_back(distance_s);
            curvature.push_back(direc_*sign_*fKappa);
            myfile << distance_s << "," << direc_*sign_*fKappa << endl;
        }
    }
    myfile.close();
    return std::make_pair(cum_distance, curvature);
}


void LocalPlannerThread::CallbackParkingPath(const nav_msgs::Path::ConstPtr& msg) {
    MemberValInit();

    if (msg->poses.size() != 0) {
        vector<Vector2d> nodeVec;
        vector<Vector3d> nodeVec_debug; // add
        for(int i = 0 ; i <= msg->poses.size()-1 ; i++) {
            nodeVec.push_back(Vector2d(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y));
            nodeVec_debug.push_back(Vector3d(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, tf::getYaw(msg->poses[i].pose.orientation)));
        }
        // cout << "Callback ParkingPath # Size (# of way-points) : " << nodeVec.size() << endl;

        // check forward/backward switching
        double vec_diff = 0.0;
        double vec_1, vec_2;  // add
        int pathIdx = 0;
        int count_waypt = 0;
        m_LocalSplinePath[pathIdx].push_back(Vector2d(nodeVec[0][0], nodeVec[0][1]));// Substitute the first value
        m_LocalSplinePath[pathIdx].push_back(Vector2d(nodeVec[1][0], nodeVec[1][1]));

        count_waypt += 2;
        
        // Each pathIdx is represented as the index of path segment which is distinguished by Gear Switching.
        for(int i = 2; i < nodeVec.size()-1 ; i++) {
            // vec_diff = abs(atan2((nodeVec[i-2][1] - nodeVec[i-1][1]), (nodeVec[i-2][0] - nodeVec[i-1][0])) 
            //             - atan2((nodeVec[i-1][1] - nodeVec[i][1]), (nodeVec[i-1][0] - nodeVec[i][0])));
            // if ((180-60)*3.14/180 < vec_diff && vec_diff < (360-60)*3.14/180) 
            //     pathIdx++;
            vec_1 = atan2(nodeVec[i][1] - nodeVec[i-1][1], nodeVec[i][0] - nodeVec[i-1][0]); // add
            vec_2 = atan2(nodeVec[i+1][1] - nodeVec[i][1], nodeVec[i+1][0] - nodeVec[i][0]);
            if (cos(vec_1) * cos(vec_2) + sin(vec_1) * sin(vec_2) < 0.0) { // dot product
                m_waypt_seg.push_back(count_waypt);
                count_waypt = 0;
                pathIdx++;    
            }
            m_LocalSplinePath[pathIdx].push_back(Vector2d(nodeVec[i][0], nodeVec[i][1]));
            count_waypt++;
        }
        m_waypt_seg.push_back(count_waypt);
        // cout << "----- Gear Switching Cnt: " << pathIdx << endl;

        // // Path segment extension 
        for(int i = 0; i < pathIdx; i++) {
            int path_segment_size = m_LocalSplinePath[i].size();
            // Store the original switching point
            m_switching_point_vector.push_back(Vector2d(m_LocalSplinePath[i][path_segment_size-1][0], m_LocalSplinePath[i][path_segment_size-1][1]));
            m_switching_point_idx.push_back(path_segment_size-1);
            double first_th = atan2(m_LocalSplinePath[i][path_segment_size-2][1] - m_LocalSplinePath[i][path_segment_size-3][1], 
                                     m_LocalSplinePath[i][path_segment_size-2][0] - m_LocalSplinePath[i][path_segment_size-3][0]);
            double second_th = atan2(m_LocalSplinePath[i][path_segment_size-1][1] - m_LocalSplinePath[i][path_segment_size-2][1], 
                                     m_LocalSplinePath[i][path_segment_size-1][0] - m_LocalSplinePath[i][path_segment_size-2][0]);
            double th_diff = second_th - first_th;
            if( th_diff > M_PI) th_diff = th_diff - 2*M_PI;
            else if( th_diff < -M_PI ) th_diff = 2*M_PI - abs(th_diff);
            double extend_distance = DISTANCE(m_LocalSplinePath[i][path_segment_size-2][0], m_LocalSplinePath[i][path_segment_size-2][1],
                                              m_LocalSplinePath[i][path_segment_size-1][0], m_LocalSplinePath[i][path_segment_size-1][1]);
            for(int j = 1 ; j <= 100; j++) {// extension
                m_LocalSplinePath[i].push_back(Vector2d(m_LocalSplinePath[i][path_segment_size-2+j][0] + extend_distance*cos(second_th + j*th_diff), 
                                                        m_LocalSplinePath[i][path_segment_size-2+j][1] + extend_distance*sin(second_th + j*th_diff)));                                                        
            }                                                        
        }
        
        // extended path rviz
        nav_msgs::Path temp_path_msg;
        std_msgs::Header m_header; 
        m_header.stamp = ros::Time::now();
        m_header.frame_id = "map";
        temp_path_msg.header = m_header;
        int path_total_size = 0;
        for(int i = 0; i < pathIdx+1; i++)
            path_total_size += m_LocalSplinePath[i].size();
        temp_path_msg.poses.resize(path_total_size);
        int temp_idx = 0;
        for(int i = 0; i <= pathIdx; i++)
            for (int j = 0 ; j < m_LocalSplinePath[i].size() ; j++) {
                temp_path_msg.poses[temp_idx].pose.position.x = m_LocalSplinePath[i][j][0];
                temp_path_msg.poses[temp_idx].pose.position.y = m_LocalSplinePath[i][j][1];
                temp_path_msg.poses[temp_idx].pose.position.z = 0.0; 
                temp_idx++;
            }
        Pub_temp_path.publish(temp_path_msg);


        // Which direction at first
        double second_waypt_vec = atan2(m_LocalSplinePath[0][5][1] - m_LocalSplinePath[0][3][1], m_LocalSplinePath[0][5][0] - m_LocalSplinePath[0][3][0]);
        if (-M_PI /2 <= second_waypt_vec && second_waypt_vec <= M_PI /2) {
            // cout << "forward first" << endl;
            m_dir_mode = 1;
        }
        else {
            // cout << "backward first" << endl;
            m_dir_mode = -1;
        }
        // m_dir_mode = -1;

        // Path Marker publish!
        // srand((unsigned int)time(0));
        // std_msgs::ColorRGBA c; 
        // // for (int i = 0 ; i < pathIdx+1+1+1 ; i++) {
        // //color set ===================================================================
        // double rgbarr[10][3] = {//@YANGWOO
        //                             {29.0/255.0 , 219.0/255.0 , 22.0/255.0}, //0 green
        //                             {1.0 , 187.0/255.0 , 0.0},               //1 mustard
        //                             {107.0/255.0 , 102.0/255.0 , 1.0},       //2 pastel_blue
        //                             {1.0 , 0.0 , 221.0/255.0},               //3 pink
        //                             {171.0/255.0 , 242.0/255.0 , 0.0},       //4 light_green
        //                             {103.0/255.0 , 153.0/255.0 , 1.0},       //6 light_pastel_blue
        //                             {1.0 , 0.0 , 127.0/255.0},               //5 rose_violet
        //                             {1.0 , 228.0/255.0 , 0.0},               //7 yellow
        //                             {0.0 , 216.0/255.0 , 1.0},               //8 skyblue
        //                             {1.0 , 94.0/255.0 , 0.0}                 //9 orange
        //                         };
        // // =============================================================================
        // }


        geometry_msgs::PoseStamped pp;
        geometry_msgs::Point p;

        // // publish markers with classified path segments
        // for (int i = 0 ; i < pathIdx+1 ; i++) {
        //     // double colorRand = (rand()%100)*0.01;
        //     m_parkingPath.id = i;
        //     for (int j = 0 ; j < m_LocalSplinePath[i].size() ; j++)  {
        //         p.x = m_LocalSplinePath[i][j][0];
        //         p.y = m_LocalSplinePath[i][j][1];
        //         p.z = 0.0;               
        //         c.r = rgbarr[i%10][0];  c.g = rgbarr[i%10][1];    c.b = rgbarr[i%10][2];
        //         c.a = 0.59;
        //         m_parkingPath.colors.push_back(c);
        //         m_parkingPath.points.push_back(p);
        //     }
        //     m_parkingPathArr.markers.push_back(m_parkingPath);// MarkerArray
        // }
        
        // if (Pub_MarkerParkingPath.getNumSubscribers() > 0)//Returns the number of subscribers that are currently connected to this Publisher.
        //     Pub_MarkerParkingPath.publish(m_parkingPathArr);

        m_switchLen = pathIdx;// + 1 + 1;

        // system("rosnode kill Driving_function_thread &"); // kill a driving function and parking starts!
        m_pathFlag = true;
        
        ros::Duration(0.5).sleep();
        // empty array pub
        geometry_msgs::PoseArray tentacle_path;  //add
        tentacle_path.header.stamp = ros::Time::now();
        tentacle_path.header.frame_id = "map";
        Pub_tentacle.publish(tentacle_path);
        //

        // PoseArray of chosen tentacle (by the decision process)
        geometry_msgs::PoseArray chosen_path_value, chosen_tentacle;
        chosen_path_value.header.stamp = ros::Time::now();
        chosen_path_value.header.frame_id = "map";
        Pub_chosenPathbyValue.publish(chosen_path_value);

        // PoseArray of chosen tentacle (by the decision process)
        chosen_tentacle.header.stamp = ros::Time::now();
        chosen_tentacle.header.frame_id = "map";
        Pub_chosenPath.publish(chosen_tentacle);
        //

    }
}

void LocalPlannerThread::Orthogonal(int value) {
    m_orthogonal_val = value * 0.5;
}

void LocalPlannerThread::Local2Global(double Lx, double Ly, double &Gx, double &Gy) {
    double tmpX = Lx;
    double tmpY = Ly;
    Gx = m_car.x + (tmpX * cos(m_car.th) - tmpY * sin(m_car.th));
    Gy = m_car.y + (tmpX * sin(m_car.th) + tmpY * cos(m_car.th));
}

void LocalPlannerThread::vehicleTrajectory(int time) {// Vehicle's future trajectory
    double dt = 0.1;
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.poses.resize((int)(time/dt));

    double yaw = 0.0, xL = 0.0, yL = 0.0;
    for (int i = 0 ; i < (int)(time/dt) ; i++) {  // a unit of the time is second.
        xL += dt*m_dir_mode*m_car.vel*cos(yaw);
        yL += dt*m_dir_mode*m_car.vel*sin(yaw);
        yaw += dt*dt*m_dir_mode*m_car.vel*atan2(-m_Steer_cmd*_DEG2RAD, 1.0);///2.85;

        Local2Global(xL, yL, msg.poses[i].pose.position.x, msg.poses[i].pose.position.y);
        msg.poses[i].pose.position.z = 0.0;
    }
    Pub_vehicleTraj.publish(msg);
    msg.poses.clear();
}

double LocalPlannerThread::CalculateClosestPt(int& pWIdx) {// Find the closest way-point from the vehicle
    double minDist = 99999, dist = 0;
    for(int i = 0; i < m_LocalSplinePath[m_pathIdx].size() - 1; i++) {
        dist = DISTANCE(m_LocalSplinePath[m_pathIdx][i][0], m_LocalSplinePath[m_pathIdx][i][1], m_car.x, m_car.y);
        if(dist < minDist) {
            minDist = dist;
            pWIdx = i;
        }
    }
    m_carIdx = pWIdx;
    return minDist;
}

// add_
double tire_to_steer_wheel(double tire_angle) {  // input must be [degree]
    // compute_gear_ratio
    int ind_i, ind_j;
    if (abs(tire_angle) > steer_angle_of_tire[TABLE_SIZE-1]) {// exceeds the last value
        return var_gear_ratio[TABLE_SIZE-1] * tire_angle;
    }

    for(int i = 1; i < TABLE_SIZE; i++)
        if (abs(tire_angle) < steer_angle_of_tire[i]) {
            ind_i = i-1;
            ind_j = i;
            break;
        }
    double m = (var_gear_ratio[ind_i] - var_gear_ratio[ind_j]) / (steer_angle_of_tire[ind_j] - steer_angle_of_tire[ind_i]);  // inclination
    double gear_ratio = m * (steer_angle_of_tire[ind_j] - abs(tire_angle)) + var_gear_ratio[ind_j];  // compute the gear_ratio
    // std::// cout << "gear_ratio: " << gear_ratio << std::endl;
    return gear_ratio * tire_angle;
}


double LocalPlannerThread::PurePursuit(double lookX, double lookY) {
    double ratio_s2w = 18.6;
    double limitSteerAngle = 455.0;

    double lfw = 0.3;
    double lrv = lfw;
    double L = 2.845;

    double steerAngle = 0.0;

    double lookAheadVec = atan2((lookY - m_car.y), (lookX - m_car.x));
    double eta = -(m_car.th - lookAheadVec);
    double Lw = DISTANCE(lookX, lookY, m_car.x, m_car.y);

    if (m_dir_mode == 1.0)// forward?
        steerAngle = (_RAD2DEG*ratio_s2w)*(-atan((L * sin(eta)) / ((Lw*1.5) / 2 + lfw*cos(eta))));
    else 
        steerAngle = 2.0*(_RAD2DEG*ratio_s2w)*(-atan((L * sin(eta)) / ((Lw) / 2 + lrv*cos(eta))));

    if(abs(steerAngle) > limitSteerAngle)
        steerAngle = SIGN(steerAngle) * limitSteerAngle;

    return steerAngle;
}


int LocalPlannerThread::CalculateLookAheadPt() {// Calculate the lookahead point index (way-point on that path)
    int pLIdx = 0;
    double LookAheadDist = MIN_DIST;//  m_car.vel*2.24;

    double dist = 0.0;
    for(int i = m_carIdx; i < m_LocalSplinePath[m_pathIdx].size(); i++) {
        dist += DISTANCE(m_LocalSplinePath[m_pathIdx][i][0], m_LocalSplinePath[m_pathIdx][i][1], m_LocalSplinePath[m_pathIdx][i+1][0], m_LocalSplinePath[m_pathIdx][i+1][1]);

        if( LookAheadDist <= dist) {
            m_pLIdx = pLIdx = i;
            break;
        }
        else
            m_pLIdx = pLIdx = m_LocalSplinePath[m_pathIdx].size() - 1;
    }
    return pLIdx;
}

void LocalPlannerThread::LookAheadOrthogonal(double& LookAheadPtX, double& LookAheadPtY){
    LookAheadPtX += m_orthogonal_val*cos(m_car.th + 90 *_DEG2RAD);
    LookAheadPtY += m_orthogonal_val*sin(m_car.th + 90 *_DEG2RAD);
}

pair<double, double> LocalPlannerThread::Kanayama(double velo_input) {
    double ClosestPtX, ClosestPtY;
    double RefPtX, RefPtY;
    ClosestPtX = m_LocalSplinePath[m_pathIdx][m_carIdx][0];
    ClosestPtY = m_LocalSplinePath[m_pathIdx][m_carIdx][1]; 
    
    // Calculate 'dy', 'dtheta' (it doesn't consider dx)
    double ctrl_dt = 0.05;  //control time interval
    double constant_velo = velo_input*KMpH2MpS;
    double dist = 0;
    int ref_idx = 0;
    double temp_x = ClosestPtX;
    double temp_y = ClosestPtY;
    for (int i=m_carIdx; i<m_LocalSplinePath[m_pathIdx].size(); i++) {
        if (ref_idx == 0) {
            dist += DISTANCE(temp_x, temp_y, m_LocalSplinePath[m_pathIdx][i][0], m_LocalSplinePath[m_pathIdx][i][1]);
            if (dist >= ctrl_dt*constant_velo) {
                ref_idx = i;
                RefPtX = m_LocalSplinePath[m_pathIdx][ref_idx][0];
                RefPtY = m_LocalSplinePath[m_pathIdx][ref_idx][1];
                dist = 0.0;
                break;
            }
            temp_x = m_LocalSplinePath[m_pathIdx][i][0];
            temp_y = m_LocalSplinePath[m_pathIdx][i][1];
        }
    }
    // cout << "eeeeeeeeeee " << m_pathIdx << endl;
    double closest_orientation, ref_orientation;
    closest_orientation = atan2(m_LocalSplinePath[m_pathIdx][m_carIdx+1][1] - ClosestPtY, 
                              m_LocalSplinePath[m_pathIdx][m_carIdx+1][0] - ClosestPtX);
    ref_orientation = atan2(m_LocalSplinePath[m_pathIdx][ref_idx+1][1] - RefPtY,
                               m_LocalSplinePath[m_pathIdx][ref_idx+1][0] - RefPtX);

    // cout << "eeeeeeeeeee 11111111111111" << m_pathIdx << endl;
// Lateral Feed-Forward
    double yaw_diff = ref_orientation - closest_orientation;
    if( yaw_diff > M_PI) yaw_diff = yaw_diff - 2*M_PI;
    else if( yaw_diff < -M_PI ) yaw_diff = 2*M_PI - abs(yaw_diff);
    yaw_diff = -m_dir_mode*yaw_diff;
    double FF_term = yaw_diff/ctrl_dt;

// Lateral Feed-Back
    double vehicle_heading = (m_dir_mode == 1.0) ? m_car.th : m_car.th - M_PI;
    double heading_err = vehicle_heading - closest_orientation;
    if( heading_err > M_PI) heading_err = heading_err - 2*M_PI;
    else if( heading_err < -M_PI ) heading_err = 2*M_PI - abs(heading_err);
    double y_e = -sin(m_car.th)*(RefPtX - m_car.x) + cos(m_car.th)*(RefPtY - m_car.y);    

    //ky/kth
    double K_y =  1.0;//2.5//
    double K_th = 1.0;//1.5//
    double FB_term = -constant_velo*K_y*y_e + m_dir_mode * constant_velo * K_th * sin(heading_err);


    // cout << "eeeeeeeeeee 22222222222222222" << m_pathIdx << endl;
// Lateral Control
    double ome_r = FF_term + FB_term;
    double ratio_s2w = 14.50;//13.5
    double limitSteerAngle = 460.0;
    double gain = 1.0;
    double steerAngle;
    if (CONSTANT_GEAR_RATIO)
        steerAngle = gain*(_RAD2DEG*ratio_s2w)*atan2(WHEEL_BASE*ome_r, constant_velo); // linear gear_ratio
    else
        steerAngle = tire_to_steer_wheel(atan2(WHEEL_BASE * ome_r, constant_velo) * _RAD2DEG); // add_for_variable_gear_ratio
    if(abs(steerAngle) > limitSteerAngle)
        steerAngle = SIGN(steerAngle) * limitSteerAngle;
    // // cout << "steer angle: " << steerAngle << endl;

// Longitudinal Control
    double velo_result;
    // add for computing the velocity proportional to 1 / curvature
    // velo_result = velo_input;// normal
    // velo_result = MIN_VEL_INPUT;
    // velo_result = (MIN_VEL_INPUT - MAX_VEL_INPUT) * TRUN_RADIUS *(abs(CalculateCurvature(m_carIdx)) - 1.0 / TRUN_RADIUS) + MIN_VEL_INPUT;
    

    // cout << "eeeeeeeeeee 33333333333333333" << m_pathIdx << endl;
    double K_v = 10.0;
    velo_result = velo_input*(1.0 - K_v*abs(CalculateCurvature(ref_idx) - CalculateCurvature(m_carIdx))*TRUN_RADIUS);  // inversely proportional to curvature rate
    velo_result = SlowAtSwitchingPoint(velo_result);
    velo_result = max(velo_result, MIN_VEL_INPUT);//velo_result = max(velo_result, MIN_VEL_INPUT);
    velo_result = min(velo_result, MAX_VEL_INPUT);

    return std::make_pair(steerAngle, velo_result);
}

void LocalPlannerThread::SwitchOrFinishChk() {
    // double dist = DISTANCE(m_LocalSplinePath[m_pathIdx][m_carIdx][0], m_LocalSplinePath[m_pathIdx][m_carIdx][1],
    //                        m_switching_point_vector[m_pathIdx][0], m_switching_point_vector[m_pathIdx][1]);
    // double car2finishDist = DISTANCE(m_car.x, m_car.y, m_LocalSplinePath[m_pathIdx].back()[0], m_LocalSplinePath[m_pathIdx].back()[1]);
    if (m_switchLen > 0)
    m_car2switchPt = (m_pathIdx < m_switchLen) ? DISTANCE(m_LocalSplinePath[m_pathIdx][m_carIdx][0], m_LocalSplinePath[m_pathIdx][m_carIdx][1], \
                                                m_switching_point_vector[m_pathIdx][0], m_switching_point_vector[m_pathIdx][1]) \
                                               : DISTANCE(m_car.x, m_car.y, m_LocalSplinePath[m_pathIdx].back()[0], m_LocalSplinePath[m_pathIdx].back()[1]);
    else
        m_car2switchPt = DISTANCE(m_car.x, m_car.y, m_LocalSplinePath[m_pathIdx].back()[0], m_LocalSplinePath[m_pathIdx].back()[1]);
        
    // add for parking
    // if (m_pathIdx < m_switchLen) 
    //     cout << std::setprecision(3) << "Left dist: " << m_car2switchPt << " [" << m_pathIdx << "/" << m_switchLen << "] " << endl;
    // else
    //     cout << std::setprecision(3) << "Left dist: " << m_car2switchPt << "[FINISH] " << endl;
    // double vehicle2switchingPointDist = 0.05;//0.13
    // double vehicle2goalPointDist = 0.1;
    
    if (m_pathIdx < m_switchLen) {
        if (m_car2switchPt <= vehicle2switchingPointDist) { 
            // cout << "----- Gear Switching !! " << m_pathIdx << endl;
            m_pathIdx++;
            m_carIdx = 1;
            m_dir_mode = -m_dir_mode;
            
            // if (REPLANNING) {
            //     MemberValInit();
                
            //     std_msgs::Float32MultiArray msg_;
            //     msg_.data.push_back(111);   // 0
            //     Pub_Replanning.publish(msg_);
            // }
            m_ros_time = ros::Time::now();
        }
    }
    else if (!m_finishFlag && m_car2switchPt <= vehicle2goalPointDist) {
        vehicleControl(0.0, 0.0);   //Publish!
        
        // double goalVec = atan2(m_LocalSplinePath[m_pathIdx][m_carIdx-1][1] - m_LocalSplinePath[m_pathIdx][m_carIdx][1], m_LocalSplinePath[m_pathIdx][m_carIdx-1][0] - m_LocalSplinePath[m_pathIdx][m_carIdx][0]);
        double goalVec = atan2(m_LocalSplinePath[m_pathIdx][m_carIdx-1][1] - m_LocalSplinePath[m_pathIdx].back()[1], m_LocalSplinePath[m_pathIdx][m_carIdx-1][0] - m_LocalSplinePath[m_pathIdx].back()[0]);
        double vec_diff = abs(goalVec - m_car.th);

        int last_path_size = m_LocalSplinePath[m_pathIdx].size();
        double car2finish_th = atan2(m_car.y - m_LocalSplinePath[m_pathIdx].back()[1], m_car.x - m_LocalSplinePath[m_pathIdx].back()[0]) - \
                                atan2(m_LocalSplinePath[m_pathIdx][last_path_size-2][1] - m_LocalSplinePath[m_pathIdx].back()[1],
                                      m_LocalSplinePath[m_pathIdx][last_path_size-2][0] - m_LocalSplinePath[m_pathIdx].back()[0]);
        // cout << "----- FINISH !!" << endl;
        cout <<"dist & vec_diff: " << m_car2switchPt << " & " << vec_diff*_RAD2DEG << endl;
        cout <<"dx & dy: "<< cos(car2finish_th)*m_car2switchPt << " & " << sin(car2finish_th)*m_car2switchPt << endl;
        double vecdiff = (fabs(vec_diff*_RAD2DEG)>30) ? fabs(fabs(vec_diff*_RAD2DEG)-360.0) : fabs(vec_diff*_RAD2DEG);
        m_error_log << "angle" << ", " << vec_diff*_RAD2DEG << ", " << "dx" << ", " << cos(car2finish_th)*m_car2switchPt 
                                                           << ", " << "dy" << ", " << sin(car2finish_th)*m_car2switchPt 
                                                           << ", " << "dummy1" << ", " << "dummy2" << endl;
        m_finishFlag = true;
    }
}

void LocalPlannerThread::Compute() {
    // Pure Pursuit related
    int pWIdx = 0, pLIdx = 0;
    double LookAheadPtX = 0.0, LookAheadPtY = 0.0;
    double steer, acc;
    
    bool KANAYAMA = true;// #MSK

    // Kanayama related
    if (m_LocalSplinePath[m_pathIdx].size() <= 10) {
        m_pathIdx ++; 
        m_carIdx = 1;
        if (m_pathIdx > 1){
            m_dir_mode = -m_dir_mode;
        }   
    }

    if (m_LocalSplinePath[m_pathIdx].size() > 1) {
        double current_veh_x = m_LocalSplinePath[m_pathIdx][m_carIdx][0];
        double current_veh_y = m_LocalSplinePath[m_pathIdx][m_carIdx][1];
        m_cross_track_error = CalculateClosestPt(pWIdx);
        // m_cross_track_error_mean = m_cross_track_error_mean + (1.0 / count_cte) * (m_cross_track_error - m_cross_track_error_mean);
        double curr_curv = CalculateCurvature(m_carIdx, m_prev_curv);
        
        pair<double, double> kanayama_control;
        if (KANAYAMA) {
            kanayama_control = Kanayama(MAX_VEL_INPUT);
        // cout << "-------------" << m_pathIdx << endl;
            SwitchOrFinishChk();
            vehicleControl(kanayama_control.first, m_dir_mode*kanayama_control.second);   //Publish! //[km/h]
        }
        else {  // Pure pursuit
            pLIdx = CalculateLookAheadPt();
            LookAheadPtX = m_LocalSplinePath[m_pathIdx][pLIdx][0]; LookAheadPtY = m_LocalSplinePath[m_pathIdx][pLIdx][1];            
            LookAheadOrthogonal(LookAheadPtX, LookAheadPtY);
            steer = m_Steer_cmd = PurePursuit(LookAheadPtX, LookAheadPtY);            
        }

        // add
        if ((ros::Time::now() - m_ros_time).toSec() < WAITING_TIME_AT_SWITCHING_PT) {
            limitDeg = LIMIT_DEGREE_SLOW;
            vehicleControl(Kanayama(MAX_VEL_INPUT).first, 0.0);
        }
        else {
            limitDeg = LIMIT_DEGREE;
        }
        m_ros_time2 = ros::Time::now();

        // add for logging -------------------------
        m_prev_curv = curr_curv;
        double cumd = 0.0;
        m_cum_distance += DISTANCE(m_LocalSplinePath[m_pathIdx][m_carIdx][0], m_LocalSplinePath[m_pathIdx][m_carIdx][1], current_veh_x, current_veh_y);
        for (int kk = 0; kk < m_pathIdx; kk++)
            cumd += m_waypt_seg[kk] * 0.01;
        cumd += m_carIdx * 0.01;

        if (!m_finishFlag)
            m_error_log << cumd << ", " << m_cross_track_error << ", " << curr_curv << ", " << m_dir_mode << 
                    ", " << kanayama_control.first << ", " << m_Steer_cmd << endl;
        // ----------------------------------------
    }
    // vehicleControl(steer, m_targetVelocity);//Publish!
    Publish_topic(LookAheadPtX, LookAheadPtY);
}

double three_pt_curvature(double x1, double y1, double x2, double y2, double x3, double y3) {
    double fAreaOfTriangle = fabs((x1 * (y2 - y3) +\
                                    x2 * (y3 - y1) +\
                                    x3 * (y1 - y2)) / 2);
    double fDist12 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double fDist23 = sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
    double fDist13 = sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    double fKappa = 4 * fAreaOfTriangle / (fDist12 * fDist23 * fDist13);

    //cross product
    double first_vec_x = x2 - x1;
    double first_vec_y = y2 - y1;
    double second_vec_x = x3 - x2;
    double second_vec_y = y3 - y2;
    double cross_product = first_vec_x * second_vec_y - second_vec_x * first_vec_y;
    int sign_ = (cross_product>=0) ? 1 : -1;

    if (isnan(fKappa) != 0)
        fKappa = 0.0;

    return sign_ * fKappa;    
}

double CalculateAcuteAngle(double vec1_x, double vec1_y, double vec2_x, double vec2_y){
    double inner_product = vec1_x * vec2_x + vec1_y * vec2_y;
    double vec1_length = sqrt(vec1_x * vec1_x + vec1_y * vec1_y);
    double vec2_length = sqrt(vec2_x * vec2_x + vec2_y * vec2_y);

    double cos_theta = inner_product/(vec1_length * vec2_length);
    double acute_angle = acos(cos_theta);

    return acute_angle;
}

double LocalPlannerThread::CalculateCurvature(int idx) {
    int path_size = m_LocalSplinePath[m_pathIdx].size();
    if (idx != 0 && idx <= path_size-2)
        return three_pt_curvature(m_LocalSplinePath[m_pathIdx][idx-1][0], m_LocalSplinePath[m_pathIdx][idx-1][1],
                                m_LocalSplinePath[m_pathIdx][idx][0], m_LocalSplinePath[m_pathIdx][idx][1],
                                m_LocalSplinePath[m_pathIdx][idx+1][0], m_LocalSplinePath[m_pathIdx][idx+1][1]);
    else
        return CURVATURE_MAX;
}

// MS add
double LocalPlannerThread::CalculateCurvature(int idx, double prev_curv) {
    int path_size = m_LocalSplinePath[m_pathIdx].size();
    if (idx != 0 && idx <= path_size-2)
        return three_pt_curvature(m_LocalSplinePath[m_pathIdx][idx-1][0], m_LocalSplinePath[m_pathIdx][idx-1][1],
                                m_LocalSplinePath[m_pathIdx][idx][0], m_LocalSplinePath[m_pathIdx][idx][1],
                                m_LocalSplinePath[m_pathIdx][idx+1][0], m_LocalSplinePath[m_pathIdx][idx+1][1]);
    else
        return prev_curv;
}

double LocalPlannerThread::SlowAtSwitchingPoint(double velo_input) {
    if (velo_input != 0.0) {
        double ClosestPtX = m_LocalSplinePath[m_pathIdx][m_carIdx][0];
        double ClosestPtY = m_LocalSplinePath[m_pathIdx][m_carIdx][1];
        
        double dist = DISTANCE(ClosestPtX, ClosestPtY, m_switching_point_vector[m_pathIdx][0], m_switching_point_vector[m_pathIdx][1]);
        if (m_pathIdx == m_switchLen)
            dist = DISTANCE(ClosestPtX, ClosestPtY, m_LocalSplinePath[m_pathIdx].back()[0], m_LocalSplinePath[m_pathIdx].back()[1]);

        double dist_tol = 2.5;
        if (dist < dist_tol) //[m]
            velo_input = (dist/dist_tol) * velo_input;//
    }
    return velo_input;    // km/h// 
}


void LocalPlannerThread::Publish_topic(double LookAheadPtX, double LookAheadPtY) {
    // Car Blue (p_w)
    m_CarPw.header.stamp = ros::Time::now();
    m_CarPw.pose.position.x = m_LocalSplinePath[m_pathIdx][m_carIdx][0];
    m_CarPw.pose.position.y = m_LocalSplinePath[m_pathIdx][m_carIdx][1];
    m_CarPw.pose.position.z = 0.5;
    if (Pub_CarPw.getNumSubscribers() > 0)
        Pub_CarPw.publish(m_CarPw);

    // Lines
   static int pub_cnt;
   pub_cnt++;
   if (pub_cnt == 7) {
       m_CarPosLine.points.push_back(m_CarPos.pose.position);
       pub_cnt = 0;
   }
   if (Pub_MarkerCar.getNumSubscribers() > 0)
       Pub_MarkerCar.publish(m_CarPosLine);
}


void LocalPlannerThread::MarkerInit() {
    m_parkingPath.header.frame_id =  m_CarPw.header.frame_id = m_CarPos.header.frame_id =  m_CarPosLine.header.frame_id = m_CarPosLine_center.header.frame_id = m_lookaheadPtJW.header.frame_id = m_lookaheadPtLineJW.header.frame_id = "map";
    m_parkingPath.ns = m_CarPw.ns = m_CarPos.ns = m_CarPosLine.ns = m_CarPosLine_center.ns = m_lookaheadPtJW.ns = m_lookaheadPtLineJW.ns = "RegionOfInterest";
    m_CarPw.id = m_CarPos.id = m_lookaheadPtJW.id = 0;
    m_CarPosLine.id = m_CarPosLine_center.id = m_lookaheadPtLineJW.id =  1;
    
    m_CarPos.type = visualization_msgs::Marker::ARROW;
    m_CarPw.type = m_lookaheadPtJW.type = visualization_msgs::Marker::SPHERE;
    m_parkingPath.type = m_CarPosLine.type = m_CarPosLine_center.type = m_lookaheadPtLineJW.type = visualization_msgs::Marker::LINE_STRIP;
    m_parkingPath.action = m_CarPw.action = m_CarPos.action = m_CarPosLine.action = m_CarPosLine_center.action = m_lookaheadPtJW.action = m_lookaheadPtLineJW.action = visualization_msgs::Marker::ADD;

    m_CarPos.scale.x = 4.5/2;
    m_CarPos.scale.y = 1.5;
    m_CarPos.scale.z = 0.1;
    m_CarPos.color.a = 0.7;
    m_CarPos.color.r = 1.0f;
    m_CarPos.color.g = 0.0f;
    m_CarPos.color.b = 0.0f;
    m_CarPos.pose.position.z = 0.5;

    m_CarPosLine.scale.x = 0.15;
    m_CarPosLine.pose.position.z = 0.3;
    m_CarPosLine.color.r = 1.0;
    m_CarPosLine.color.a = 0.9;

    m_CarPosLine_center.scale.x = 0.09;
    m_CarPosLine_center.pose.position.z = 0.3;
    m_CarPosLine_center.color.g = 1.0;
    m_CarPosLine_center.color.a = 0.9;
    
    m_lookaheadPtJW.scale.x = 0.99;
    m_lookaheadPtJW.scale.y = 0.99;
    m_lookaheadPtJW.scale.z = 0.99;
    m_lookaheadPtJW.color.a = 0.9;
    m_lookaheadPtJW.color.g = 1.0;
    m_lookaheadPtJW.pose.position.z = 0.5;

    m_lookaheadPtLineJW.scale.x = 0.09;
    m_lookaheadPtLineJW.pose.position.z = 0.05;
    m_lookaheadPtLineJW.color.r = 0.0;
    m_lookaheadPtLineJW.color.g = 1.0;
    m_lookaheadPtLineJW.color.a = 0.9;

    m_CarPw.scale.x = 1.1;
    m_CarPw.scale.y = 1.1;
    m_CarPw.scale.z = 1.1;
    m_CarPw.color.r = 0.0f;
    m_CarPw.color.g = 0.0f;
    m_CarPw.color.b = 1.0f;
    m_CarPw.color.a = 0.9;

    m_parkingPath.scale.x = 1.3;
}

vector<string> tokenize_getline(const string& data, const char delimiter = ' ') {
	vector<string> result;
	string token;
	stringstream ss(data);

	while (getline(ss, token, delimiter))
		result.push_back(token);
	return result;
}

double LocalPlannerThread::cur_rad(double x1, double y1,
                                    double x2, double y2,
                                    double x3, double y3) {
    double rad = 0.0, rad_max = 100;
    double d1 = (x2-x1)/(y2-y1);
    double d2 = (x3-x2)/(y3-y2);
    double c_x = ((y3-y1)+(x2+x3)*d2-(x1+x2)*d1)/(2*(d2-d1));
    double c_y = -d1*(c_x-((x1+x2)/2))+(y1+y2)/2;

    // rad = DISTANCE(x1, y1, c_x, c_y);
    rad = DISTANCE(x2, y2, c_x, c_y);

    // if (isnan(rad) != 0 || rad >= rad_max)
    if (isnan(rad) != 0)
        rad = rad_max;

    return 1.0/rad;
}

void LocalPlannerThread::vehicleControl(double steer, double velocity) {
    // add MSK: add TODO
    // =============== Physical limit ===============
    if (m_pathFlag) {
        double dt = (ros::Time::now() - m_ros_time2).toSec();
        if (limitDeg*dt < abs(steer - m_Steer_cmd)) {
            m_Steer_cmd += (double)(SIGN(steer - m_Steer_cmd)*limitDeg*dt);
        }
        else
            m_Steer_cmd = steer;
    }
    else 
        m_Steer_cmd = steer;
    
    // // cout << "ros wow: " << steer << endl;
    // if (limitVel*dt < abs(velocity - m_Velocity_cmd))
    //     m_Velocity_cmd += (double)(SIGN(velocity - m_Velocity_cmd)*limitVel*dt);
    // else
    //     m_Velocity_cmd = velocity;
    
    // add ,msg_.data.push_back(steer);  
    // std_msgs::Float32MultiArray msg_;
    // msg_.data.push_back(m_Steer_cmd); 
    // msg_.data.push_back(velocity);
    // Pub_ControlCmd.publish(msg_);

    // To IONIC_CAN_MASTER
    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(m_Steer_cmd); 
    msg_.data.push_back(velocity);
    msg_.data.push_back(m_dir_mode);
    msg_.data.push_back(1.0);// acc_gain
    msg_.data.push_back(m_cross_track_error);// cross-track error
    if (m_finishFlag) msg_.data.push_back(1.0);
    else              msg_.data.push_back(0.0);
    msg_.data.push_back(m_car2switchPt); // switchingDist
    msg_.data.push_back(m_switchLen - m_pathIdx); // leftSwitching
    Pub_ControlCmd.publish(msg_);
}

int main(int argc, char* argv[])
{
    LocalPlannerThread LocalPlannerThread(argc, argv); 
    ros::spin();

    return 0;
} 


