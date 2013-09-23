#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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
        tf::TransformListener listener_;
        ros::Timer pid_loop_;


        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        control_toolbox::Pid Pid_;
        geometry_msgs::Twist setpt_;
        double commandZ_;
        double kp_, ki_, kd_, imax_;
        double max_;
        int steps_;
        double period_;
        double increment_;
        ros::Time lastIMU_;


        dynamic_reconfigure::Server<rate_control::RateControlConfig> server_;
        dynamic_reconfigure::Server<rate_control::RateControlConfig>::CallbackType f_;
	
    protected: // ROS Callbacks
        void imu_callback(const geometry_msgs::TwistStamped& msg) {
           double error = msg.twist.angular.z - setpt_.angular.z;
           ros::Duration deltaT = msg.header.stamp - lastIMU_;
           lastIMU_ = msg.header.stamp;
           commandZ_ = Pid_.updatePid(error, deltaT);
        
        }
        
        void cmd_callback(const geometry_msgs::Twist& msg) {
            //setpt_ = msg;
        }

        void do_pid(const ros::TimerEvent&)
        {
            ROS_INFO("Starting, steps %d", steps_);
            if (steps_ > 0)
            {
                steps_ --;
                setpt_.angular.z += increment_;
                motor_pub_.publish(setpt_);
            }
            else
            {
                setpt_.angular.z = 0;
                motor_pub_.publish(setpt_);
            }
            //Make sure we keep the rest of the output the same.
        }

        void config_callback(rate_control::RateControlConfig &config, uint32_t level)
        {
            Pid_.setGains(config.kp, config.ki, config.kd, config.imax, -config.imax);
        }

    public:
        AngularRateControl() : nh_("~") {
            nh_.param("o_max",max_,1.0);
            nh_.param("steps",steps_,400);
            nh_.param("kd",kd_,0.0);
            nh_.param("imax",imax_,1E9);
            nh_.param("period",period_,0.05);

            increment_ = max_ / double(steps_);

            Pid_.initPid(kp_,ki_,kd_,imax_,-imax_);
            setpt_.linear.x = 0;
            setpt_.angular.z = 0;
            
            ROS_INFO("Starting, steps %d", steps_);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the velocity command and setup the motor publisher
            imu_sub_ = nh_.subscribe("imu",1,&AngularRateControl::imu_callback,this);
            cmd_sub_ = nh_.subscribe("command",1,&AngularRateControl::cmd_callback,this);
            motor_pub_ = nh_.advertise<geometry_msgs::Twist>("motors",1);
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

            ROS_INFO("Starting, steps %d", 21);
    ros::spin();
    return 0;
}


