#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <kingfisher_msgs/Sense.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <pid.h>
#include <dynamic_reconfigure/server.h>
#include <rate_control/RateControlConfig.h>
#include <iostream>

class AngularRateControl {
    protected:
        ros::Subscriber imu_sub_;
        ros::Subscriber cmd_sub_;
        ros::Subscriber mux_sub_;
        ros::Subscriber rc_sub_;
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
        double motor_zero_;
        double input_scale_;
        ros::Time lastIMU_;
        ros::Time lastCommand_;
        ros::Duration timeout_;

        std::string joy_control_;
        std::string auto_control_;
        bool manual_override_, rc_override_;

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
            lastCommand_ = ros::Time::now();
            setpt_.angular.z = setpt_.angular.z * input_scale_;
        }
        
        void mux_callback(const std_msgs::String::ConstPtr& msg) {
            if (msg->data == joy_control_) {
                manual_override_ = true;
            } else if (msg->data == auto_control_) {
                manual_override_ = false;
            } else {
                ROS_ERROR("Received unknown mux selection: '%s'",msg->data.c_str());
            }

        }
        
        void rc_callback(const kingfisher_msgs::Sense::ConstPtr& msg) {
            if (msg->rc & 0x02) {
                rc_override_ = true;
            } else {
                rc_override_ = false;
            }
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
            if(output.angular.z > 0)
                output.angular.z = output.angular.z + motor_zero_;
            else
                output.angular.z = output.angular.z - motor_zero_;
            Pid_.getCurrentPIDErrors(&x, &y, &z);
            pt.x = x;
            pt.y = y;
            pt.z = z;
            debug_pub_.publish(pt);
            if(!manual_override_ && !rc_override_ && 
               (lastCommand_ + timeout_) > ros::Time::now()){
                motor_pub_.publish(output);            
            } else {
                Pid_.reset();
            }
        }

        void config_callback(rate_control::RateControlConfig &config, uint32_t level)
        {
            Pid_.setGains(config.kp, config.ki, config.kd, config.imax, -config.imax);
            motor_zero_ = config.motor_zero;
            input_scale_ = config.input_scale;
        }

    public:
        AngularRateControl() : nh_("~"),lastIMU_(0.0),
        lastCommand_(0.0), timeout_(1.5),
        joy_control_("/teleop/twistCommand"), auto_control_("/mux/safeCommand"),
        manual_override_(false), rc_override_(true)
         {
            nh_.param("kp",kp_,1.0);
            nh_.param("ki",ki_,0.0);
            nh_.param("kd",kd_,0.0);
            nh_.param("imax",imax_,1E9);
            nh_.param("period",period_,0.05);
            nh_.param("motor_zero",motor_zero_,0.0);
            nh_.param("input_scale",input_scale_,1.0);

            Pid_.initPid(kp_,ki_,kd_,imax_,-imax_);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the velocity command and setup the motor publisher
            imu_sub_ = nh_.subscribe("imu",1,&AngularRateControl::imu_callback,this);
            cmd_sub_ = nh_.subscribe("command",1,&AngularRateControl::cmd_callback,this);
            mux_sub_ = nh_.subscribe("/mux/selected",1,&AngularRateControl::mux_callback,this);
            rc_sub_ = nh_.subscribe("/sense",1,&AngularRateControl::rc_callback,this);
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


