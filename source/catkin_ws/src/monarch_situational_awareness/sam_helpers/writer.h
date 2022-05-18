#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/this_node.h>
#include <ros/exceptions.h>

#include <monarch_situational_awareness/CreateWriter.h>
#include <monarch_situational_awareness/RemoveWriter.h>

using namespace ros;
using namespace std;
using namespace monarch_situational_awareness;

template <class T> class SAMWriter
{    
  public:
    SAMWriter (const string& slot_name, 
               const string& agent_name="",
               const int q_size=1):
    nh_(),
    pub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createWriter(slot_name, agent_name);
        pub_ = nh_.advertise<T> (topic_name,q_size);
    }
    
    //Overriding the ROS publisher methods
    void publish(const boost::shared_ptr<T>& message) const
    {
        pub_.publish(message);
    }
    void publish(const T& message) const
    {
        pub_.publish(message);
    }

	void publish_burst(const T& message) const
    {
        int i;
		for(i=0;i<10;i++)
        {
			pub_.publish(message);
			Duration(0.1).sleep();
		}
			
    }

    void remove()
    {
        pub_.shutdown();
    }

  private:
    string createWriter(const string& slot_name, const string& agent_name="")
    {
        string topicname;
        string slotname = slot_name;
        string agentname = agent_name;

        if (slotname.find("]") != std::string::npos)
        {
            agentname = slot_name.substr(1,slotname.find("]")-1);
            slotname = slotname.substr(slotname.find("]")+1);  
            slotname.erase(std::remove(slotname.begin(), slotname.end(),' '),slotname.end());
        }

        if (agentname != "")
        {
            topicname = "/sar/" + agentname + "/" + slotname;
        }
        else
        {
            topicname = "/sar/" + agentname + slotname;
        }

        return topicname;
    }

    NodeHandle nh_;
    Publisher pub_;
    const string slotname;
    const string agentname;
};
