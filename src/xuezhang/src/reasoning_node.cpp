#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig3/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;     
    std::ofstream file;                       // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;
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

   //TODO A03.T02: This function should open a file if the path of the file is found, otherwise it should print out that the file or directory were not found (0.4 pts)
    // void setOutQueriesFile(string QueryfileName)
    // {

    //     this->m_query_flag_save=true; //This means that I want to save the queries in a file
    // }


// void setOutQueriesFile(string QueryfileName)
// {
//     std::ofstream SavedOpenFile;
//     SavedOpenFile.open(QueryfileName);
//     if (!SavedOpenFile.is_open()) {                 
//         ROS_ERROR_STREAM("File not found: " << QueryfileName);
//     } else {
//         SavedOpenFile.close(); 
//     }
//     this->m_query_flag_save = true; 
// }
void setOutQueriesFile(string QueryfileName)
{
    // 使用 std::ofstream 替代 std::fstream 并以追加模式打开文件
    file = std::ofstream(QueryfileName, ios::out | ios::app);
    
    // 检查文件是否成功打开
    if (!file.is_open()) {                 
        ROS_ERROR_STREAM("File not found: " << QueryfileName);
    } else {
        // 如果文件成功打开，设置标志以启动查询的保存
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
    bool srv_assert_callback(world_percept_assig3::UpdateObjectList::Request &req,
                             world_percept_assig3::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;
        
        //done: A2.T03: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful (1.5 pts).

        object=req.object_name;
        res.confirmation = false;
        
        getClass(object);
        res.confirmation = assertKnowledge(object);


        return res.confirmation;
    }


    bool getClass(std::string className)
    {
        
        //done A2.T03:Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" -> 1.5 pts
        //std:string query= "query";
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

        // A2.T03: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" -> 1 pt
        //std:string query= "query";
        // std:string query= "create_instance_from_class('"    + className +    "', '"      + '" + std::to_string(ID_++) + "' +     "', Instance)";
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
                //A2.T03: Retrive the value from Prolog -> 1 pt
                instanceName = className + std::to_string(ID_-1);
                //instanceName = "object";
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

   //TODO A03.T02: Retrieve the variable from the yaml file and save it in the new variable "saveQueries_flag" (0.2 pts) 
   std::string savedQueryFile;
   nh.getParam("read_prolog_queries/save_flag", saveQueries_flag);
   if(saveQueries_flag)
    { //If the flag is true, then I will configure the file to save the asserted queries
         //TODO A03.T02: Include the code to load the rosparam from a yaml file (0.4 pts)
         // First define a new string variable "savedQueryFile"
         // Then load the yaml parameter in the new variable

         // "parameter_name_for_file" should be the name of the parameter defined in the yaml file
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