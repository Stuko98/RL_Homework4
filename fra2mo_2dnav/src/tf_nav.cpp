#include "../include/tf_nav.h"


///////////////////////////////////////////////////////////////
//                         HOMEWORK4.4                       // 
///////////////////////////////////////////////////////////////

// Global variables
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;



void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);                                                                                                       //l'elemento specificato riempie l'ultimo elemento vuoto dell'array
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}

///////////////////////////////////////////////////////////////

TF_NAV::TF_NAV() {                                                                                                                                   //costruttore senza argomenti

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );

    _pos_x_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/x_pos", 1 );                                                                             //HOMEWORK4
    _pos_y_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/y_pos", 1 );                                                                             //HOMEWORK4
    _pos_z_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/z_pos", 1 );                                                                             //HOMEWORK4
    _or_x_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/x_or", 1 );                                                                               //HOMEWORK4
    _or_y_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/y_or", 1 );                                                                               //HOMEWORK4
    _or_z_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/z_or", 1 );                                                                               //HOMEWORK4
    _or_w_pub = _nh.advertise<std_msgs::Float64>( "/fra2mo/w_or", 1 );                                                                               //HOMEWORK4

    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    _goal_pos << 0.0, 0.0, 0.0;
    _goal_or << 0.0, 0.0, 0.0, 1.0;
    _home_pos << -18.0, 2.0, 0.0;

    _aruco_position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/aruco_marker/pose", 1 );                                                      //HOMEWORK4.4 Sfizio mio. Alla fine si è rivelato necessario


///////////////////////////////////////////////////////////////
//                         HOMEWORK4                         // 
///////////////////////////////////////////////////////////////

    _goal_pos_2 << 0.0, 0.0, 0.0;
    _goal_or_2 << 0.0, 0.0, 0.0, 1.0;
    _goal_pos_3 << 0.0, 0.0, 0.0;
    _goal_or_3 << 0.0, 0.0, 0.0, 1.0;
    _goal_pos_4 << 0.0, 0.0, 0.0;
    _goal_or_4 << 0.0, 0.0, 0.0, 1.0;
    _goal_pos_5 << 0.0, 0.0, 0.0;
    _goal_or_5 << 0.0, 0.0, 0.0, 1.0;
    
    _goal_2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    _goal_3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    _goal_4 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    _goal_5 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    _goal_6 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                                    //HOMEWORK4.3
    _goal_7 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                                    //HOMEWORK4.3
    _goal_8 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                                    //HOMEWORK4.3
    _goal_9 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                                    //HOMEWORK4.3
    _goal_10 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                                   //HOMEWORK4.3
 
    _goal_aruco_mid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                            //HOMEWORK4.4
    _goal_aruco_approach << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                       //HOMEWORK4.4
    _d_cam_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                                //HOMEWORK4.4
    _goal_aruco_final << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;                                                                                          //HOMEWORK4.4





///////////////////////////////////////////////////////////////

}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}

void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);

    
    geometry_msgs::PoseStamped aruco_pose;

    aruco_pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    aruco_pose.pose.position.x = _goal_aruco_final[0];
    aruco_pose.pose.position.y = _goal_aruco_final[1];
    aruco_pose.pose.position.z = _goal_aruco_final[2];

    aruco_pose.pose.orientation.w = _goal_aruco_final[3];
    aruco_pose.pose.orientation.x = _goal_aruco_final[4];
    aruco_pose.pose.orientation.y = _goal_aruco_final[5];
    aruco_pose.pose.orientation.z = _goal_aruco_final[6];

    _aruco_position_pub.publish(aruco_pose);

///////////////////////////////////////////////////////////////
//                         HOMEWORK4                         // 
///////////////////////////////////////////////////////////////

    std_msgs::Float64 _pos_x_msg, _pos_y_msg, _pos_z_msg, _or_x_msg, _or_y_msg, _or_z_msg, _or_w_msg;

    _pos_x_msg.data = _cur_pos[0];
    _pos_y_msg.data = _cur_pos[1];
    _pos_z_msg.data = _cur_pos[2];
    _or_w_msg.data = _cur_or[0];
    _or_x_msg.data = _cur_or[1];
    _or_y_msg.data = _cur_or[2];
    _or_z_msg.data = _cur_or[3];

    _pos_x_pub.publish(_pos_x_msg);
    _pos_y_pub.publish(_pos_y_msg);
    _pos_z_pub.publish(_pos_z_msg);
    _or_x_pub.publish(_or_w_msg);
    _or_y_pub.publish(_or_x_msg);
    _or_z_pub.publish(_or_y_msg);
    _or_w_pub.publish(_or_z_msg);

///////////////////////////////////////////////////////////////

}

void TF_NAV::goal_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    tf::StampedTransform transform2, transform3, transform4, transform5;                                                                             //HOMEWORK4

    tf::StampedTransform transform6, transform7, transform8, transform9, transform10;                                                                //HOMEWORK4.3

    tf::StampedTransform transform_ar_mid, transform_ar_approach, transform_d_cam;                                                                   //HOMEWORK4.4

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform );

///////////////////////////////////////////////////////////////
//                         HOMEWORK4                         // 
///////////////////////////////////////////////////////////////

            listener.waitForTransform( "map", "goal2", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal2", ros::Time( 0 ), transform2 );
            listener.waitForTransform( "map", "goal3", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal3", ros::Time( 0 ), transform3 );
            listener.waitForTransform( "map", "goal4", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal4", ros::Time( 0 ), transform4 );
            listener.waitForTransform( "map", "goal5", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal5", ros::Time( 0 ), transform5 );
            
            listener.waitForTransform( "map", "goal6", ros::Time( 0 ), ros::Duration( 10.0 ) );                                                      //HOMEWORK4.3
            listener.lookupTransform( "map", "goal6", ros::Time( 0 ), transform6 );                                                                  //HOMEWORK4.3
            listener.waitForTransform( "map", "goal7", ros::Time( 0 ), ros::Duration( 10.0 ) );                                                      //HOMEWORK4.3
            listener.lookupTransform( "map", "goal7", ros::Time( 0 ), transform7 );                                                                  //HOMEWORK4.3
            listener.waitForTransform( "map", "goal8", ros::Time( 0 ), ros::Duration( 10.0 ) );                                                      //HOMEWORK4.3
            listener.lookupTransform( "map", "goal8", ros::Time( 0 ), transform8 );                                                                  //HOMEWORK4.3
            listener.waitForTransform( "map", "goal9", ros::Time( 0 ), ros::Duration( 10.0 ) );                                                      //HOMEWORK4.3
            listener.lookupTransform( "map", "goal9", ros::Time( 0 ), transform9 );                                                                  //HOMEWORK4.3
            listener.waitForTransform( "map", "goal10", ros::Time( 0 ), ros::Duration( 10.0 ) );                                                     //HOMEWORK4.3
            listener.lookupTransform( "map", "goal10", ros::Time( 0 ), transform10 );                                                                //HOMEWORK4.3
  
            listener.waitForTransform( "map", "goal_aruco_mid", ros::Time( 0 ), ros::Duration( 10.0 ) );                                             //HOMEWORK4.4
            listener.lookupTransform( "map", "goal_aruco_mid", ros::Time( 0 ), transform_ar_mid );                                                   //HOMEWORK4.4
            listener.waitForTransform( "map", "goal_aruco_approach", ros::Time( 0 ), ros::Duration( 10.0 ) );                                        //HOMEWORK4.4
            listener.lookupTransform( "map", "goal_aruco_approach", ros::Time( 0 ), transform_ar_approach );                                         //HOMEWORK4.4
            listener.waitForTransform( "map", "camera_depth_optical_frame", ros::Time( 0 ), ros::Duration( 10.0 ) );                                 //HOMEWORK4.4    //già esistente, quindi non devi crearlo con il tf_static_transform_publisher in "spawn_fra2mo_gazebo.launch"
            listener.lookupTransform( "map", "camera_depth_optical_frame", ros::Time( 0 ), transform_d_cam );                                        //HOMEWORK4.4



///////////////////////////////////////////////////////////////
            
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _goal_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        
///////////////////////////////////////////////////////////////
//                         HOMEWORK4                         // 
///////////////////////////////////////////////////////////////
        
        _goal_pos_2 << transform2.getOrigin().x(), transform2.getOrigin().y(), transform2.getOrigin().z();                                           //potevo anche risparmiarmi di inizializzare prima parzialmente ("_goal_pos_i" e "_goal_or_i") e poi per intero ("_goal_i"), facendo direttamente per intero. E' per evidenziare i passaggi durante la traccia: prima ho fatto come nell'esempio del prof, poi per necessità di homework ho fatto a modo mio (per intero)
        _goal_or_2 << transform2.getRotation().w(),  transform2.getRotation().x(), transform2.getRotation().y(), transform2.getRotation().z();
        
        _goal_pos_3 << transform3.getOrigin().x(), transform3.getOrigin().y(), transform3.getOrigin().z();
        _goal_or_3 << transform3.getRotation().w(),  transform3.getRotation().x(), transform3.getRotation().y(), transform3.getRotation().z();
        
        _goal_pos_4 << transform4.getOrigin().x(), transform4.getOrigin().y(), transform4.getOrigin().z();
        _goal_or_4 << transform4.getRotation().w(),  transform4.getRotation().x(), transform4.getRotation().y(), transform4.getRotation().z();
        
        _goal_pos_5 << transform5.getOrigin().x(), transform5.getOrigin().y(), transform5.getOrigin().z();
        _goal_or_5 << transform5.getRotation().w(),  transform5.getRotation().x(), transform5.getRotation().y(), transform5.getRotation().z();
        

        // debug: definizione parziale in "pos" e "or"
        //std::cout << "Goal2 (2 vettori): " << std::endl << "Position: " << std::endl << _goal_pos_2 << std::endl << "Orientation: " << std::endl << _goal_or_2 << std::endl;  
        
        // debug: inizializzazione totale a 0
        //std::cout << "Goal2 (1 vettore): " << std::endl << _goal_2 << std::endl;
        //std::cout << "Goal3 (1 vettore): " << std::endl << _goal_3 << std::endl;        
        //std::cout << "Goal4 (1 vettore): " << std::endl << _goal_4 << std::endl;
        //std::cout << "Goal5 (1 vettore): " << std::endl << _goal_5 << std::endl;


        _goal_2.segment<3>(0) = _goal_pos_2;                                                                                                         //riempiamo i primi tre elementi di _goal_2
        _goal_2.segment<4>(3) = _goal_or_2;                                                                                                          //riempiamo i rimanenti elementi di _goal_2

        _goal_3.segment<3>(0) = _goal_pos_3;
        _goal_3.segment<4>(3) = _goal_or_3;
        
        _goal_4.segment<3>(0) = _goal_pos_4;
        _goal_4.segment<4>(3) = _goal_or_4;
        
        _goal_5.segment<3>(0) = _goal_pos_5;
        _goal_5.segment<4>(3) = _goal_or_5;


        // debug: definizione totale
        //std::cout << "Goal2 (2 vettori): " << std::endl << "Position: " << std::endl << _goal_pos_2 << std::endl << "Orientation: " << std::endl << _goal_or_2 << std::endl;  
        
        //std::cout << "Goal2 (1 vettore): " << std::endl << _goal_2 << std::endl;
        //std::cout << "Goal3 (1 vettore): " << std::endl << _goal_3 << std::endl;        
        //std::cout << "Goal4 (1 vettore): " << std::endl << _goal_4 << std::endl;
        //std::cout << "Goal5 (1 vettore): " << std::endl << _goal_5 << std::endl;


        // HOMEWORK4.3
        _goal_6 << transform6.getOrigin().x(), transform6.getOrigin().y(), transform6.getOrigin().z(), transform6.getRotation().w(),  transform6.getRotation().x(), transform6.getRotation().y(), transform6.getRotation().z();                      //HOMEWORK4.3
        _goal_7 << transform7.getOrigin().x(), transform7.getOrigin().y(), transform7.getOrigin().z(), transform7.getRotation().w(),  transform7.getRotation().x(), transform7.getRotation().y(), transform7.getRotation().z();                      //HOMEWORK4.3
        _goal_8 << transform8.getOrigin().x(), transform8.getOrigin().y(), transform8.getOrigin().z(), transform8.getRotation().w(),  transform8.getRotation().x(), transform8.getRotation().y(), transform8.getRotation().z();                      //HOMEWORK4.3
        _goal_9 << transform9.getOrigin().x(), transform9.getOrigin().y(), transform9.getOrigin().z(), transform9.getRotation().w(),  transform9.getRotation().x(), transform9.getRotation().y(), transform9.getRotation().z();                      //HOMEWORK4.3
        _goal_10 << transform10.getOrigin().x(), transform10.getOrigin().y(), transform10.getOrigin().z(), transform10.getRotation().w(),  transform10.getRotation().x(), transform10.getRotation().y(), transform10.getRotation().z();              //HOMEWORK4.3

        // matrix for "for cicle"
        _goals_array.col(0) = _goal_2;                                                                                                               //i vettori-obiettivo costituiranno le COLONNE della matrice
        _goals_array.col(1) = _goal_3;
        _goals_array.col(2) = _goal_4;
        _goals_array.col(3) = _goal_5;

        _goals_array.col(4) = _goal_6;                                                                                                               //HOMEWORK4.3
        _goals_array.col(5) = _goal_7;                                                                                                               //HOMEWORK4.3
        _goals_array.col(6) = _goal_8;                                                                                                               //HOMEWORK4.3
        _goals_array.col(7) = _goal_9;                                                                                                               //HOMEWORK4.3
        _goals_array.col(8) = _goal_10;                                                                                                              //HOMEWORK4.3


        // debug: definizione matrice per ciclo for
        //std::cout << "_goals_array matrix: " << std::endl <<_goals_array << std::endl;
        //std::cout << "_goals_array size: " << _goals_array.size() << std::endl;


        // HOMEWORK4.4
        _goal_aruco_mid << transform_ar_mid.getOrigin().x(), transform_ar_mid.getOrigin().y(), transform_ar_mid.getOrigin().z(), transform_ar_mid.getRotation().w(),  transform_ar_mid.getRotation().x(), transform_ar_mid.getRotation().y(), transform_ar_mid.getRotation().z();                                             //HOMEWORK4.4
        _goal_aruco_approach << transform_ar_approach.getOrigin().x(), transform_ar_approach.getOrigin().y(), transform_ar_approach.getOrigin().z(), transform_ar_approach.getRotation().w(),  transform_ar_approach.getRotation().x(), transform_ar_approach.getRotation().y(), transform_ar_approach.getRotation().z();     //HOMEWORK4.4
        _d_cam_pose << transform_d_cam.getOrigin().x(), transform_d_cam.getOrigin().y(), transform_d_cam.getOrigin().z(), transform_d_cam.getRotation().w(),  transform_d_cam.getRotation().x(), transform_d_cam.getRotation().y(), transform_d_cam.getRotation().z();                                                        //HOMEWORK4.4


        // frames: "map" -> "camera" -> "aruco_marker"     ====     1 -> 2 -> 3
        Eigen::Matrix4d map_T_cam = Eigen::Matrix4d::Identity();                                                                                     //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.7
        Eigen::Matrix4d cam_T_object = Eigen::Matrix4d::Identity();                                                                                  //HOMEWORK4.4     
      //Eigen::Matrix4d map_T_object = Eigen::Matrix4d::Identity();                                                                                  //HOMEWORK4.4      inutile

        Eigen::Vector3d translation1_2(_d_cam_pose[0], _d_cam_pose[1], _d_cam_pose[2]);                                                              //HOMEWORK4.4
        Eigen::Quaterniond quaternion1_2 (_d_cam_pose[3], _d_cam_pose[4], _d_cam_pose[5], _d_cam_pose[6]); //w,x,y,z                                 //HOMEWORK4.4
        map_T_cam.block<3,3>(0,0) = quaternion1_2.toRotationMatrix();                                                                                //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.7
        map_T_cam.block<3,1>(0,3) = translation1_2;                                                                                                  //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.7

        Eigen::Vector3d translation2_3(aruco_pose[0], aruco_pose[1], aruco_pose[2]);                                                                 //HOMEWORK4.4
        Eigen::Quaterniond quaternion2_3(aruco_pose[6], aruco_pose[3], aruco_pose[4], aruco_pose[5]); //w,x,y,z ma aruco_pose li ha in ordine diverso//HOMEWORK4.4
        cam_T_object.block<3,3>(0,0) = quaternion2_3.toRotationMatrix();                                                                             //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.7
        cam_T_object.block<3,1>(0,3) = translation2_3;                                                                                               //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.7

        Eigen::Vector3d translation1_3 = translation1_2 + quaternion1_2.toRotationMatrix() * translation2_3;                                         //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.7
        Eigen::Quaterniond quaternion1_3 = quaternion1_2 * quaternion2_3;                                                                            //HOMEWORK4.4      "Robotics" di Siciliano -> cap 2 -> 2.3 , 2.6. Con "*", Eigen::Quaterniond ti permette di fare direttamente il prodotto di Hamilton!
        
      //map_T_object.block<3,3>(0,0) = quaternion1_3.toRotationMatrix();                                                                             //HOMEWORK4.4      inutile
      //map_T_object.block<3,1>(0,3) = translation1_3;                                                                                               //HOMEWORK4.4      inutile

        
        _goal_aruco_final << translation1_3[0], translation1_3[1], translation1_3[2], quaternion1_3.w(), quaternion1_3.x(), quaternion1_3.y(), quaternion1_3.z();//HOMEWORK4.4

///////////////////////////////////////////////////////////////      
 
        r.sleep();
    }
    
    
}
/*
void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() )
    {   
        std::cout<<"\nInsert 1 to send goal from TF "<<std::endl;
        std::cout<<"Insert 2 to send home position goal "<<std::endl;
        std::cout<<"Inser your choice"<<std::endl;
        std::cin>>cmd;

        if ( cmd == 1) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos[0];
            goal.target_pose.pose.position.y = _goal_pos[1];
            goal.target_pose.pose.position.z = _goal_pos[2];

            goal.target_pose.pose.orientation.w = _goal_or[0];
            goal.target_pose.pose.orientation.x = _goal_or[1];
            goal.target_pose.pose.orientation.y = _goal_or[2];
            goal.target_pose.pose.orientation.z = _goal_or[3];


            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 2 ) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }
         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();                                                                                                                                   //pausa per mantenere la frequenza desiderata
    }
    
}
*/

/*
///////////////////////////////////////////////////////////////
//                         HOMEWORK4                         // 
///////////////////////////////////////////////////////////////

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() )
    {   
        std::cout<<"\nSent a group of goals from TF "<<std::endl;
        MoveBaseClient ac("move_base", true);
        

        for(std::size_t i=0; i<_goals_array.cols(); i++){                                                                                            //scorro i vettori-obiettivo, quindi PER COLONNA

            //HOMEWORK4: DEBUG: controllo ciclo for
            //std::cout <<"Sent "<< i+1 <<"° goal:" << _goals_array.col(i) <<std::endl;

            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goals_array(0,i);
            goal.target_pose.pose.position.y = _goals_array(1,i);
            goal.target_pose.pose.position.z = _goals_array(2,i);

            goal.target_pose.pose.orientation.w = _goals_array(3,i);
            goal.target_pose.pose.orientation.x = _goals_array(4,i);
            goal.target_pose.pose.orientation.y = _goals_array(5,i);
            goal.target_pose.pose.orientation.z = _goals_array(6,i);
            std::cout<< "The actual goal is the "<< i+1 << "°: " << std::endl << goal << std::endl;

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
              ROS_INFO("The mobile robot arrived in the %ld ° TF goal", (i+1));
            else
              ROS_INFO("The base failed to reach the %ld ° TF goal", (i+1));
        }

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("The mobile robot arrived at the end!");
        else
          ROS_INFO("The base failed to reach the final goal!");

        r.sleep();   
    }
    
}
///////////////////////////////////////////////////////////////
*/


///////////////////////////////////////////////////////////////
//                         HOMEWORK4.4                       // 
///////////////////////////////////////////////////////////////

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    move_base_msgs::MoveBaseGoal goal;

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
          
    goal.target_pose.pose.position.x = _goal_aruco_mid[0];                                                                                           //mid aruco goal: serviva un goal intermedio, altrimenti non riusciva a localizzare l'approaching goal. Aumentando le ampiezze del costmap globale e di quello locale sicuramente lo avrebbe percepito (vedi ~/catkin_ws/src/fra2mo_2dnav/config/)
    goal.target_pose.pose.position.y = _goal_aruco_mid[1];
    goal.target_pose.pose.position.z = _goal_aruco_mid[2];
    goal.target_pose.pose.orientation.w = _goal_aruco_mid[3];
    goal.target_pose.pose.orientation.x = _goal_aruco_mid[4];
    goal.target_pose.pose.orientation.y = _goal_aruco_mid[5];
    goal.target_pose.pose.orientation.z = _goal_aruco_mid[6];

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The mobile robot arrived in the mid aruco goal");
    else
        ROS_INFO("The base failed to move for some reason");
            

    goal.target_pose.pose.position.x = _goal_aruco_approach[0];                                                                                      //approaching aruco goal
    goal.target_pose.pose.position.y = _goal_aruco_approach[1];
    goal.target_pose.pose.position.z = _goal_aruco_approach[2];

    goal.target_pose.pose.orientation.w = _goal_aruco_approach[3];
    goal.target_pose.pose.orientation.x = _goal_aruco_approach[4];
    goal.target_pose.pose.orientation.y = _goal_aruco_approach[5];
    goal.target_pose.pose.orientation.z = _goal_aruco_approach[6];


    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            
        ROS_INFO("The mobile robot arrived in the approaching aruco goal");
            
        if(aruco_pose_available) {                                                                                                                   //se raggiungo l'approaching aruco goal + inquadri l'aruco_marker...
                 
            goal.target_pose.pose.position.x = _goal_aruco_final[0] + 1;                                                                             // x = xm + 1
            goal.target_pose.pose.position.y = _goal_aruco_final[1];                                                                                 // y = ym
            goal.target_pose.pose.position.z = 0.0;                                                                                                  // il robot deve stare per terra, non alla stessa altezza dell'aruco marker

            goal.target_pose.pose.orientation.w = 0.0;                                                                                               //face to face: robot <-> marker
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 1.0;

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("YOU REACHED THE FINAL ARUCO GOAL!");
            else
                ROS_INFO("The base failed to move for some reason");
        }
    }
    else
        ROS_INFO("The base failed to move for some reason");
    
}



///////////////////////////////////////////////////////////////



void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}


///////////////////////////////////////////////////////////////
//                         HOMEWORK4.4                       // 
///////////////////////////////////////////////////////////////

void poseCallback(const geometry_msgs::PoseStamped& msg){                                                                                            //HOMEWORK4.4  creo il frame "aruco_marker", la cui posa è quella che viene stimata in ogni istante map -> marker (infatti contiene le info del topic "aruco_marker/pose", che era quello da me creato)  
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );                                                              //tf::Vector3 Class Reference: un oggetto di questa classe può essere inizializzato con parametri di tipo "const tfScalar &", che rende il codice più portabile perché sostituibile con "float" o "double"
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w );                              //tf::Quaternion Class Reference: stessa cosa

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_marker"));                                                    //indico il già esistente parent frame ("map") e il child frame che ci interessa creare ("aruco_marker")
}

//////////////////////////////////////////////////////////////

int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    
    ros::NodeHandle n;                                                                                                                               //HOMEWORK4.4 
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);                                                        //HOMEWORK4.4 mi sottoscrivo al topic che riceve dalla camera la stima della posa dell'aruco marker inquadrato. ATENZIONE: la posa dell'aruco marker, essendo stimata dalla camera, è rispetto al CAMERA FRAME. Più avanti quindi dovremo sicuramente riportare tutto rispetto allo Spatial/inertial/base frame
    
    ros::Subscriber aruco_marker_pose_sub = n.subscribe("aruco_marker/pose", 10, &poseCallback);                                                     //HOMEWORK4.4 mi sottoscrivo al topic che riceve la stima della posa dell'aruco marker RISPETTO AL MAP FRAME. Ti ricordo infatti che mi iscrivo al topic "aruco_marker/pose", che era quello da me creato!

    TF_NAV tfnav;                                                                                                                                    //oggetto di classe Tf_NAV
    tfnav.run();                                                                                                                                     //eseguo metodo "run()" associato a tale oggetto. Tale metodo contiene tutte le funzionalità sviluppate finora
    return 0;
}