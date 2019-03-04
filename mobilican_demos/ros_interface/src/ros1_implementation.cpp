#include "ros1_implementation.h"

Ros1Implementation::Ros1Implementation(ros::NodeHandle *nh)
        : nh_(nh)
{}

Ros1Implementation::~Ros1Implementation() {}

// If the pub name already exists, override it.
void Ros1Implementation::newPublisher(std::string pub_name)
{
        ros::Publisher pub = nh_->advertise<std_msgs::String>(pub_name, 1000);
        // ros::Publisher pub = nh_.advertise<msg_type>("chatter", 1000); TODO template.
        publisher_to_index_[pub_name] = publishers_vec_.size();
        publishers_vec_.push_back(pub); // TODO - SHOULD IT BE A PTR?
        ROS_INFO("publisher to 'chatter' is ready to publish.");
}


void Ros1Implementation::subCallback(const std_msgs::String::ConstPtr& msg) //TODO should not be here!
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void Ros1Implementation::newSubscriber(std::string topic_name)
{
        // TODO to send the fanction as parameter.
        ros::Subscriber sub = nh_->subscribe(topic_name, 1000, &Ros1Implementation::subCallback, this);
        // ros::Subscriber sub = nh_->subscribe(topic_name, 1000, &Vision::subCallback, this);

        subscribers_vec_.push_back(sub);
        ROS_INFO("subscriber is ready.");
}

void Ros1Implementation::publishToTopic(std::string topic_name)
{
        int pub_index = -1;
        std::map<std::string, int>::iterator result = publisher_to_index_.find(topic_name);
        if (result == publisher_to_index_.end())
        {
                newPublisher(topic_name);
                pub_index = publisher_to_index_[topic_name];
        }
        else {
          pub_index = result->second;
        }

        int count = 0;
        // while (ros::ok())
        // {
                std_msgs::String msg; // TODO

                std::stringstream ss;
                ss << "hello world " << count;
                msg.data = ss.str();

                ROS_INFO("%s", msg.data.c_str());
                publishers_vec_[pub_index].publish(msg);

                ros::spinOnce();

                // loop_rate.sleep();
                // ++count;
        // }
}
