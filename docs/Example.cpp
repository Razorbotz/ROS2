#include <iostream>
using namespace std;

/** @file
 * @brief File that shows documentation for Doxygen
 * 
 * This is the detailed description for the file.  This should be 
 * informative for the reader and include all pertinent information.
 * After the description of the file is written, it should be followed
 * by a list of the topics that the node subscribes to.  
 * The topics that the node subscribes to are as follows:
 * \li \b subscribed_topic_1
 * \li \b subscribed_topic_1
 * \li \b subscribed_topic_3
 * 
 * To read more about the related node
 * \see related_node.cpp
 * 
 * The topics that are being published are as follows:
 * \li \b published_topic_1
 * \li \b published_topic_2
 * \li \b published_topic_3
 * 
 * To read more about the nodes that subscribe to this one
 * \see related_node.cpp
 * 
 * */

/** @brief Short description of the function
 * 
 * This is the detailed description for the function.  This should include
 * a description of the function, as well as any parameters, return values,
 * and the related files or functions.  If the parameters or return values
 * are a message type defined by a file, link the file as well.
 * @param argc
 * @param argv
 * @param axisState \see AxisState.msg
 * @return void
 * */
void main(int argc, char** argv, axisState AxisState){
    std::cout << "Hello World" << std::endl;
}