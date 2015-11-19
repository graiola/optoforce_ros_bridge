#include "QtCore/qcoreapplication.h"
#include "omd/opto.h"

#include <iostream>
#include <iomanip>
#include <QThread>

////////// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>

////////// Toolbox
#include <toolbox/toolbox.h>

////////// Eigen
#include <eigen3/Eigen/Core>

using namespace tool_box;
using namespace Eigen;
using namespace ros;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    VectorXd user_force(3);
    user_force.fill(0.0);
    std::vector<VectorXd> forces;

    OptoPorts ports;
    OPort* list=ports.listPorts(true);

    std::cout<<"\nAvailable ports:"<<std::endl;
    int i=0;

    for (i=0;i<ports.getLastSize();i++)
        std::cout<<"   "<<i+1<<". "<<list[i].name<<" "<<list[i].deviceName<<std::endl;

    std::cout<<"\nTotal:"<<ports.getLastSize()<<"\n"<<std::endl;

    if (i==0)
    {
        std::cout<<"No sensor found"<<std::endl;
        return -1;
    }
    
    //int input;
    //if (ports.getLastSize()==1)
	//input=1;
    //else
	//std::cin>>input;
	
    // Initialize the ros node
    RosNode ros_node("optoforce_ros_bridge_node");

    // Initialize ros rt publishers and classic publishers
    RealTimePublishers<RealTimePublisherWrench> rt_publishers;
    boost::shared_ptr<RealTimePublisherWrench> tmp_ptr = boost::make_shared<RealTimePublisherWrench>(ros_node.GetNode(),"user_force","T0");
    rt_publishers.AddPublisher(tmp_ptr,&user_force);

    ros::NodeHandle nh = ros_node.GetNode();
    ros::Publisher contact_force_pub = nh.advertise<std_msgs::Bool>("contact_force", 1000);

    std_msgs::Bool contact_force;
    contact_force.data = false;

    for(int i=0; i<ports.getLastSize();i++)
        forces.push_back(VectorXd(3));
    
    for(int i=0; i<ports.getLastSize();i++)
    {
        std::string topic_name = "force_" + std::to_string(i+1);
        tmp_ptr = boost::make_shared<RealTimePublisherWrench>(ros_node.GetNode(),topic_name,"T0");
        rt_publishers.AddPublisher(tmp_ptr,&forces[i]);
    } 
	
    std::vector<OptoDAQ> daqs(ports.getLastSize());
    
    for(int i=0;i<daqs.size();i++)
    {
      if(!daqs[i].open(list[i]))
      {
        std::cout<<"Can not open port: "<<i+1<<std::endl;
        return a.exec();
      }
      daqs[i].zeroAll();
    }
    
    //for (int i=0;i<ports.getLastSize();i++)
     // if(!daq.open(list[i]))
     // {
	//std::cout<<"Can not open port: "<<i+1<<std::endl;
	//return a.exec();
      //}
    
    
    // Set some config
    /*SensorConfig c;
    
    //void set(sensor_state st, sensor_speed sp, sensor_filter ft, sensor_mode rf);
    
    c.setSpeed(0);
    c.setFilter(3);
    c.setMode(1);
    
    if(daq.sendConfig(c))
    {*/
      while (true)
      {
	
	  for(int j=0;j<daqs.size();j++)
	  {
	    OptoPackage* pa=0;

	    int size = daqs[j].readAll(pa);
	    
	    for (int i=0;i<size;i++)
	    {
	      
	      /*std::cout	<<" x: "<<std::setw(6)<<pa[i].x
			<<" y: "<<std::setw(6)<<pa[i].y
			<<" z: "<<std::setw(6)<<pa[i].z
			<<" s1: "<<std::setw(6)<<pa[i].s1
			<<" s2: "<<std::setw(6)<<pa[i].s2
			<<" s3: "<<std::setw(6)<<pa[i].s3
			<<" s4: "<<std::setw(6)<<pa[i].s4
			<<" TEMPO: "<<std::setw(6)<<pa[i].temp<<"\n";*/
			
			/*forces[j](0) = pa[i].x;
			forces[j](1) = pa[i].y;
			forces[j](2) = pa[i].z;*/
			
			if(j == 0)
			{
			  forces[j](0) = pa[i].y;
			  forces[j](1) = pa[i].z;
			  forces[j](2) = -pa[i].x;
			}
			else if(j == 1)
			{
			  forces[j](0) = pa[i].y;
			  forces[j](1) = -pa[i].z;
			  forces[j](2) = pa[i].x;
			}
			
            if (forces[j].norm() < 50.0 && forces[j].norm() > -50.0)
                contact_force.data = false;
            else
                contact_force.data = true;

            contact_force_pub.publish(contact_force);

			rt_publishers.PublishAll();  
	    }	  
	  }
	  if (daqs.size() == 2)
	  {
	    user_force(0) = forces[0](0) - forces[1](0);
	    user_force(1) = forces[0](1) - forces[1](1);
	    user_force(2) = forces[0](2) - forces[1](2);
	  }
	  
      }
      
      for(int j=0;j<daqs.size();j++)
        daqs[j].close();
	    
    return a.exec();
	    
}
