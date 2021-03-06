#include "Visitor.h"

/**
*	@brief Class for Friend nodes.
*/
class Friend2 : public Visitor
{
	private:

	protected:
		void delegate(se306_project1::ResidentMsg msg);
		void doTimedVisit(const ros::TimerEvent&);

	public:

		Friend2() : Visitor(){
			currentCheckpoint = std::make_pair(-20, -48);
			px = -20;
			py= -48;
		}
		int run(int argc, char *argv[]);
		void friendsDoneCallback(const ros::TimerEvent&);
};
