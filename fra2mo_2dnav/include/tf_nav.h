#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float64.h>                                                                                       //HOMEWORK4

#include <tf/transform_broadcaster.h>                                                                               //HOMEWORK4

class TF_NAV {

    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        Eigen::Vector3d _goal_pos;
        Eigen::Vector4d _goal_or;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        
///////////////////////////////////////////////////////////////
//                         HOMEWORK4                         // 
///////////////////////////////////////////////////////////////

        ros::Publisher _pos_x_pub, _pos_y_pub, _pos_z_pub, _or_x_pub, _or_y_pub, _or_z_pub, _or_w_pub;

        Eigen::Vector3d _goal_pos_2,_goal_pos_3,_goal_pos_4,_goal_pos_5;                                            //HOMEWORK4.2 vettore per definizione parziale "pos"
        Eigen::Vector4d _goal_or_2,_goal_or_3,_goal_or_4,_goal_or_5;                                                //HOMEWORK4.2 vettore per definizione parziale "or"

        Eigen::Matrix<double,7,1> _goal_2, _goal_3, _goal_4, _goal_5;                                               //HOMEWORK4.2 vettore per definizione per intero
        
        //Eigen::Matrix<double,7,4> _goals_array;                                                                   //HOMEWORK4.2 matrice per ciclo for


        Eigen::Matrix<double,7,1> _goal_6,_goal_7,_goal_8,_goal_9,_goal_10;                                         //HOMEWORK4.3 restanti vettori per definizione per intero
        Eigen::Matrix<double,7,9> _goals_array;                                                                     //HOMEWORK4.3 matrice aggiornata per ciclo for


        Eigen::Matrix<double,7,1> _goal_aruco_mid,_goal_aruco_approach, _d_cam_pose, _goal_aruco_final;             //HOMEWORK4.4 restanti vettori per definizione per intero
        
        ros::Publisher _aruco_position_pub;                                                                         //HOMEWORK4.4 sfizio mio. Alla fine si è rivelato necessario
        




//////////////////////////////////////////////////////////////
};