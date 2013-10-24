#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <pid.h>
#include <dynamic_reconfigure/server.h>
#include <rate_control/RateControlConfig.h>

class AngularRateControl {
    protected:
        ros::Subscriber imu_sub_;
        ros::Subscriber cmd_sub_;
        ros::Publisher motor_pub_;
        ros::Publisher debug_pub_;
        tf::TransformListener listener_;
        ros::Timer pid_loop_;


        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        control_toolbox::Pid Pid_;
        geometry_msgs::Twist setpt_;
        double commandZ_;
        double kp_, ki_, kd_, imax_;
        double period_;
        ros::Time lastIMU_;


        dynamic_reconfigure::Server<rate_control::RateControlConfig> server_;
        dynamic_reconfigure::Server<rate_control::RateControlConfig>::CallbackType f_;
	
    protected: // ROS Callbacks
        void imu_callback(const sensor_msgs::Imu& msg) {
           if(lastIMU_ == ros::Time(0.0))
               lastIMU_ = ros::Time::now() - ros::Duration(0.01);
           // setpt_ and msg have opposite senses of Z, so we add
           double error = setpt_.angular.z + msg.angular_velocity.z;
           ros::Duration deltaT = msg.header.stamp - lastIMU_;
           lastIMU_ = msg.header.stamp;
           commandZ_ = Pid_.updatePid(error, deltaT);
        
        }
        
        void cmd_callback(const geometry_msgs::Twist& msg) {
            setpt_ = msg;
        }

        void do_pid(const ros::TimerEvent&)
        {
            geometry_msgs::Twist output;
            geometry_msgs::Point pt;
            double x, y, z;
            //Make sure we keep the rest of the output the same.
            output = setpt_;
            // Correct for the sign of the output
            output.angular.z = -commandZ_;
            Pid_.getCurrentPIDErrors(&x, &y, &z);
            pt.x = x;
            pt.y = y;
            pt.z = z;
            debug_pub_.publish(pt);
            motor_pub_.publish(output);            
        }

        void config_callback(rate_control::RateControlConfig &config, uint32_t level)
        {
            Pid_.setGains(config.kp, config.ki, config.kd, config.imax, -config.imax);
        }

    public:
        AngularRateControl() : nh_("~"),lastIMU_(0.0) {
            nh_.param("kp",kp_,1.0);
            nh_.param("ki",ki_,0.0);
            nh_.param("kd",kd_,0.0);
            nh_.param("imax",imax_,1E9);
            nh_.param("period",period_,0.05);

            Pid_.initPid(kp_,ki_,kd_,imax_,-imax_);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the velocity command and setup the motor publisher
            imu_sub_ = nh_.subscribe("imu",1,&AngularRateControl::imu_callback,this);
            cmd_sub_ = nh_.subscribe("command",1,&AngularRateControl::cmd_callback,this);
            motor_pub_ = nh_.advertise<geometry_msgs::Twist>("motors",1);
            debug_pub_ = nh_.advertise<geometry_msgs::Point>("debug",1);
            //Create a callback for the PID loop
            pid_loop_ = nh_.createTimer(ros::Duration(period_), &AngularRateControl::do_pid, this);

            f_ = boost::bind(&AngularRateControl::config_callback, this, _1, _2);
            server_.setCallback(f_);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"RateControl");
    AngularRateControl arc;

    ros::spin();
    return 0;
}


