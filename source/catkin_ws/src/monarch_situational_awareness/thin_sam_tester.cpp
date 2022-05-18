#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/this_node.h>
#include <ros/exceptions.h>

#include <monarch_msgs/BatteriesVoltage.h>
#include <monarch_msgs/PersonLocalizationTrackingData.h>
#include <monarch_msgs/PersonLocalizationTrackingDataArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>

#include "sam_helpers/writer.h"
#include "sam_helpers/reader.h"

#include <ros/message_event.h>

using namespace ros;
using namespace std;

void peopleLocalizationTrackerCB(const monarch_msgs::PersonLocalizationTrackingDataArray& msg)
{
    ROS_INFO_STREAM("People_Localization_Tracker:: " << msg);
}


void callback(const ros::MessageEvent<monarch_msgs::PersonLocalizationTrackingDataArray>& event)
{
    /* Message event
     *
     * callerid: name of node sending data (publisher)
     * topic: name of the topic the subscriber is connecting to
     * service: name of service the client is calling
     * md5sum: md5sum of the message type
     * type: message type
     * message_definition: full text of message definition (output of gendeps --cat)
     * error: human-readable error message if the connection is not successful
     * persistent: sent from a service client to a service. If '1', keep connection open for multiple requests.
     * tcp_nodelay: sent from subscriber to publisher. If '1', publisher will set TCP_NODELAY on socket if possible
     * latching: publisher is in latching mode (i.e. sends the last value published to new subscribers)
     */


    const ros::M_string& header = event.getConnectionHeader();
    
    string callerid = header.at("callerid");
    string topic = header.at("topic");

    boost::shared_ptr<monarch_msgs::PersonLocalizationTrackingDataArray> msg = event.getMessage();
    ROS_INFO_STREAM("People_Localization_Tracker:: " << (*msg));
}


void batteriesVoltageCB(const monarch_msgs::BatteriesVoltage& msg)
{
    ROS_INFO_STREAM("BatteriesVoltage:: " << msg);
}

void leaderAssistCB(const std_msgs::UInt32& msg)
{
    ROS_INFO_STREAM("LeaderAssist:: " << msg);
}

void tfPoseCB(const geometry_msgs::Pose& msg)
{
    ROS_INFO_STREAM("tfPose:: " << msg);
}

int main(int argc, char** argv)
{
    monarch_msgs::PersonLocalizationTrackingDataArray pltda;
    monarch_msgs::BatteriesVoltage bv;
    std_msgs::UInt32 la;
    geometry_msgs::Pose tfp;

    ros::init(argc,argv,"thin_sam_tester");
    ros::NodeHandle n;

    // Create Reader Service
    ros::service::waitForService("create_reader");    
    monarch_situational_awareness::CreateReader cr;
    ros::Duration d(5.0);
    while(!cr.response.success)
    {
       cr.request.properties.slot_name = "Leader_Assist";
       service::call("create_reader",cr);
       d.sleep();
    }
    ros::Subscriber sub = n.subscribe(cr.response.topic_name,1,leaderAssistCB);


    // Create Writer Service    
    ros::service::waitForService("create_writer");
    monarch_situational_awareness::CreateWriter wr;
    wr.request.properties.slot_name = "BatteriesVoltage";
    wr.request.properties.agent_name = "mbot01";
    if (!ros::service::call("create_writer",wr)) {
        ROS_ERROR("Could not create a SAM writer");
        return 0;
    }
    ros::Publisher pub = n.advertise<monarch_msgs::BatteriesVoltage>(wr.response.topic_name, 1);


    // SAM Helpers
    SAMWriter<monarch_msgs::PersonLocalizationTrackingDataArray> *sam_writer_pltda = new SAMWriter<monarch_msgs::PersonLocalizationTrackingDataArray>("People_Localization_Tracker");
    SAMWriter<std_msgs::UInt32> *sam_writer_la = new SAMWriter<std_msgs::UInt32>("Leader_Assist");
    SAMWriter<geometry_msgs::Pose> *sam_writer_tfp = new SAMWriter<geometry_msgs::Pose>("tfPose","mbot10");

    SAMReader sub_pltda("People_Localization_Tracker", "", &callback);
    SAMReader sub_bv("BatteriesVoltage", "mbot01", &batteriesVoltageCB);
    SAMReader sub_tfp("tfPose", "mbot10", &tfPoseCB);

    sam_writer_pltda->publish(pltda);
    sam_writer_la->publish(la);
    sam_writer_tfp->publish(tfp);
    pub.publish(bv);


    // Remove Reader Service
    /*ros::service::waitForService("remove_reader");    
    monarch_situational_awareness::RemoveReader rr;
    while(!rr.response.success)
    {
       rr.request.slot_name = "Leader_Assist";
       rr.request.reader_name = "std_msgs/UInt32";
       service::call("remove_reader",rr);
       //sub_pltda.remove();
       d.sleep();
    }*/

    //sub_bv.remove();
    //sub_tfp.remove();
    //sub.shutdown();

    ros::spin();
}
