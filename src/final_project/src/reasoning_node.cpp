#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig4/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_; // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;
    std::ofstream file;  
    
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);

        this->m_query_flag_save=false;
    };

    ~Reasoner(){

    };

   //TODO A04.T02: This function should open a file if the path of the file is found, otherwise it should print out that the file or directory were not found (1 pt)
    void setOutQueriesFile(string QueryfileName)
    {
    
        file = std::ofstream(QueryfileName, ios::out | ios::app);
        
        if (!file.is_open()) {                 
            ROS_ERROR_STREAM("File not found: " << QueryfileName);
        } else {
            
            this->m_query_flag_save = true; 
        }

        this->m_query_flag_save = true;
    }   

private:    


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(world_percept_assig4::UpdateObjectList::Request &req,
                             world_percept_assig4::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;
        
        //TODO: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful.

        object=req.object_name;
        res.confirmation = false;

        getClass(object);
        res.confirmation = assertKnowledge(object);

        return res.confirmation;
    }


    bool getClass(std::string className)
    {
        
        // TODO: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils"
        std:string query= "get_class('" + className + "')";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);
            
          if(m_query_flag_save){
            file << query << endl;
        }

        bool res = false;
        for (auto &it : bdgs) 
        {
            res = true;
            ROS_INFO_STREAM("A new class was created in the ontology");
            break;
        }
        return res;
       
    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;

        // TODO: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" 
        std::string query = "create_instance_from_class('"     + className +     "', '"       + std::to_string(ID_++)+ "', Instance)";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);
          if(m_query_flag_save){
            file << query << endl;
        }

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            for (auto val : *it)
            {
                //TODO: Retrive the value from Prolog
                instanceName = className + std::to_string(ID_-1);
                ROS_WARN_STREAM("new instance in knowledge base: "<<instanceName);
                break;
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

   //+ Information about the path for the file that will save the queries
  std::string saveFilePath;
  saveFilePath = argv[1]; // This means that the node expects a path value as input. This means that we need to run this node as follows: rosrun world_percept

 
   bool saveQueries_flag; //variable that will receive the value from the yaml file

   //TODO A04.T02: Retrieve the variable from the yaml file and save it in the new variable "saveQueries_flag" (0.25 pts) 
   std::string savedQueryFile;
   nh.getParam("read_prolog_queries/save_flag", saveQueries_flag);

   if(saveQueries_flag)
    { //If the flag is true, then we will configure the file to save the asserted queries
         //TODO A04.T02: Include the code to load the rosparam from a yaml file (0.75 pt)
         // First define a new string variable "savedQueryFile"
         // Then load the yaml parameter in the new variable
        nh.getParam("read_prolog_queries/saved_query_file", savedQueryFile);
        //This node now needs a path as input when we run it. This path is addedd to the obtain variable from the yaml file, as follows
        savedQueryFile= saveFilePath+savedQueryFile;
        ROS_INFO_STREAM("query_file: "<< savedQueryFile);

        //Now we call a new function which will create and open the new file
        myReasoner.setOutQueriesFile(savedQueryFile);
     
    }

  ros::spin();

  
  return 0;
}