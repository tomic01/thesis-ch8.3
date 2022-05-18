#include <boost/function.hpp>

#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/this_node.h>
#include <ros/exceptions.h>

#include <monarch_situational_awareness/CreateReader.h>
#include <monarch_situational_awareness/RemoveReader.h>

using namespace ros;
using namespace std;
using namespace monarch_situational_awareness;

class SAMReader
{    
  public:
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(M), 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(M) const, 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(const boost::shared_ptr< M const > &), 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(const boost::shared_ptr< M const > &) const, 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(M), 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(M) const, 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(const boost::shared_ptr< M const > &), 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               void(T::*fp)(const boost::shared_ptr< M const > &) const, 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M>
    SAMReader (const string& slot_name,
               void(*fp)(M),
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M>(topic_name,q_size,fp);
    }
    
    template <class M>
    SAMReader (const string& slot_name,
               void(*fp)(const boost::shared_ptr< M const > &),
               const int q_size=1) :
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M>(topic_name,q_size,fp);
    }
    
    template <class M>
    SAMReader (const string& slot_name,
               const boost::function< void (const boost::shared_ptr< M const >&)> &callback,
               const VoidConstPtr &tracked_object=VoidConstPtr(),
               const int q_size=1) :
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M>(topic_name,q_size,callback,tracked_object);
    }
    
    template <class M, class C>
    SAMReader (const string& slot_name,
               const boost::function< void(C) > &callback,
               const VoidConstPtr &tracked_object=VoidConstPtr(),
               const int q_size=1) :
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname("")
    {
        string topic_name = createReader(slot_name);
        sub_ = nh_.subscribe<M,C>(topic_name,q_size,callback,tracked_object);
    }
    
    //--- with agent_name
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(M), 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(M) const, 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(const boost::shared_ptr< M const > &), 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(const boost::shared_ptr< M const > &) const, 
               T *obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(M), 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(M) const, 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(const boost::shared_ptr< M const > &), 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M , class T >
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(T::*fp)(const boost::shared_ptr< M const > &) const, 
               const boost::shared_ptr< T > &obj,
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,T>(topic_name,q_size,fp,obj);
    }
    
    template <class M>
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(*fp)(M),
               const int q_size=1):
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M>(topic_name,q_size,fp);
    }
    
    template <class M>
    SAMReader (const string& slot_name,
               const string& agent_name,
               void(*fp)(const boost::shared_ptr< M const > &),
               const int q_size=1) :
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M>(topic_name,q_size,fp);
    }
    
    template <class M>
    SAMReader (const string& slot_name,
               const string& agent_name,
               const boost::function< void (const boost::shared_ptr< M const >&)> &callback,
               const VoidConstPtr &tracked_object=VoidConstPtr(),
               const int q_size=1) :
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M>(topic_name,q_size,callback,tracked_object);
    }
    
    template <class M, class C>
    SAMReader (const string& slot_name,
               const string& agent_name,
               const boost::function< void(C) > &callback,
               const VoidConstPtr &tracked_object=VoidConstPtr(),
               const int q_size=1) :
    nh_(),
    sub_ (),
    slotname(slot_name),
    agentname(agent_name)
    {
        string topic_name = createReader(slot_name,
                                         agent_name);
        sub_ = nh_.subscribe<M,C>(topic_name,q_size,callback,tracked_object);
    }

    void remove()
    {
        sub_.shutdown();
    }

  private:
    string createReader(const string& slot_name, const string& agent_name="")
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
    Subscriber sub_;
    string slotname;
    string agentname;
};
