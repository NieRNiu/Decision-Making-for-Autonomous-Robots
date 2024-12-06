#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig4/loadKnowledge.h>

using namespace std;

class Knowledge
{
private:

    PrologClient pl_;
    int ID_;

    std::string srv_load_knowledge_name_;
    ros::ServiceServer load_knowledge_srv_;  
    bool m_query_flag_save; 
    std::string queryFilePath_; // File path to load queries

public:

    Knowledge(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_load_knowledge_name_ = "load_knowledge";
        load_knowledge_srv_ = nh.advertiseService(srv_load_knowledge_name_, &Knowledge::callback_load_knowledge, this);

        this->m_query_flag_save=false;
    };

    ~Knowledge(){

    };

    void setQueryFile(std::string fileName_Q)
    {

        std::ofstream outfile;

        // Attempt to open the file
        outfile.open(fileName_Q, std::ios::out);

        if (outfile.is_open())
        {
            this->m_query_flag_save = true; 
            queryFilePath_ = fileName_Q; // Store the query file path
            outfile.close();
        }
        else
        {
            // File could not be opened
            this->m_query_flag_save = false;
            ROS_ERROR_STREAM("File not found and exit the function.");
        }

    }

    void loadQueries()
    {
        if (!m_query_flag_save)
        {
            ROS_ERROR_STREAM("Query file not set. Cannot load queries.");
            return;
        }

        std::ifstream infile(queryFilePath_);

        if (!infile.is_open())
        {
            ROS_ERROR_STREAM("Failed to open the query file: " << queryFilePath_);
            return;
        }

        std::string line;
        while (std::getline(infile, line))
        {
            if (line.empty())
                continue; // Skip empty lines

            pl_.query(line); // Send the query line to PrologClient
            ROS_INFO_STREAM("Successfully asserted query: " << line);
        }

        infile.close();

    }

private:

bool callback_load_knowledge(world_percept_assig4::loadKnowledge::Request &req, 
                             world_percept_assig4::loadKnowledge::Response &res) 
{

    if(req.start == 1)
    {
        loadQueries();
        res.confirm = true;
    } 
    else 
    {
        res.confirm = false;
    }

    return res.confirm;

}

}; //class Knowledge

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "knowledge_node");

  ros::NodeHandle nh;   
  Knowledge myKnowledge(nh);

   //+ Information about the path for the file that will save the queries
  std::string saveFilePath;
  saveFilePath = argv[1]; // This means that the node expects a path value as input. This means that we need to run this node as follows: rosrun world_percept

 
   bool saveQueries_flag; //variable that will receive the value from the yaml file

   std::string savedQueryFile;
   nh.getParam("read_prolog_queries/save_flag", saveQueries_flag);

   if(saveQueries_flag)
    { 
        nh.getParam("read_prolog_queries/saved_query_file", savedQueryFile);
        //This node now needs a path as input when we run it. This path is addedd to the obtain variable from the yaml file, as follows
        savedQueryFile= saveFilePath+savedQueryFile;
        ROS_INFO_STREAM("query_file: "<< savedQueryFile);

        //Now we call a new function which will create and open the new file
        myKnowledge.setQueryFile(savedQueryFile);
    }

  ros::spin();
  
  return 0;
}