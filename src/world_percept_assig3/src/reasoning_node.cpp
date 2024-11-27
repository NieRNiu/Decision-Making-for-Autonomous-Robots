#include <ros/ros.h>
#include <iostream>
#include <string>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig3/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);
    };

    ~Reasoner(){

    };

private:    


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(world_percept_assig3::UpdateObjectList::Request &req,
                             world_percept_assig3::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;
        
        //TODO: A2.T03: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful.

        object=req.object_name;
        res.confirmation = false;

        getClass(object);
        if (assertKnowledge(object)) {
            ROS_INFO_STREAM("Successfully asserted knowledge for object: " << object);
            res.confirmation = true;
        } else {
            ROS_ERROR_STREAM("Failed to assert knowledge for object: " << object);
            res.confirmation = false;
        }


        return res.confirmation;
    }


    void getClass(std::string className)
    {
        
        // A2.T03:Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" 
        std:string query= "instance_utils:get_class(ssy236Ontology:'" + className + "')";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        bool res = false;
        for (auto &it : bdgs) 
        {
            res = true;
            ROS_INFO_STREAM("A new class was created in the ontology");
            break;
        }

        //bool res = false;
        //for (auto &it : bdgs) 
        //{
        //    if (res == true)
        //    {

        //        ROS_INFO_STREAM("A new class was created in the ontology");
        //    }
        //}

       
    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;

        // A2.T03: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils"
        std:string query= "instance_utils:create_instance_from_class(ssy236Ontology:'" + className + "', 'instance_id', Instance)";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            for (auto val : *it)
            {
                //A2.T03: Retrive the value from Prolog
                //instanceName = val.second;
                //instanceName = "object";
                //std::ostringstream oss;
                //oss << val; // PrologValue typically supports stream output for conversion
                instanceName = val.first;
                ROS_WARN_STREAM("new instance in knowledge base: "<<instanceName);
            }
            
        }

        bdgs.finish();
        
        return true;
    }

}; //class Reasoner

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "reasoning_node");

  ros::NodeHandle nh;   
  
  Reasoner myReasoner(nh);

  ros::spin();

  
  return 0;
}