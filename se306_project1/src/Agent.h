#include <nav_msgs/Odometry.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/AssistantMsg.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include "CheckPointGraph.hpp"

/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
*	Contains common navigation functionality and properties.
*/
class Agent
{
	public:
		Agent(){
			linear_x = 0;
			angular_z = 0;
			px = 0;
			py = 0;
			currentAngle = 0;
			isFacingCorrectly = false;
			shortestPathIndex = 0;
			checkpointAngle = 0;
			isClockwise = true;
			robot_id = 0;
			isMoving = false;
		}

		void StageOdom_callback(nav_msgs::Odometry msg);

	protected:
		std::string originName;

		void setPath(std::string start, std::string end);

		// Checkpoint graph object
		CheckPointGraph g; /*!< The graph of all the checkpoints in the system */

		//velocity of the robot
		double linear_x; /*!< Linear velocity of the robot */
		double angular_z; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */
		double currentAngle; /*!< angle of the robot*/

		//current checkpoint
		std::pair<double, double> currentCheckpoint; /*!< The agent's current (or  previous, if they are moving) checkpoint */

		//shortestPath
		std::vector <std::pair<double,double> > shortestPath;  /*!< A path of checkpoint co-ordinates that the agent is expected to move along */
		int shortestPathIndex; /*!< Pointer to the current vertex in the path */
		bool isFacingCorrectly; /*!< True if the agent is facing the correct direction (and can therefore begin moving) */

		//moving status of the robot
		bool isMoving; /*!< True if the agent is moving */

		int robot_id; /*!< Robot's ID */

		double checkpointAngle; /*!< The angle to the checkpoint*/
		bool isClockwise; /*!< True if the agent should turn clockwise to face its current goal */

		void turn();
		void moveForward(std::pair<double,double> nextCheckpoint);
		double calculateGoalAngle(std::pair<double,double> goalCheckpoint);
		void move(std::string goal);
		bool isTurnClockwise();
		void stopMoving();
	
};
