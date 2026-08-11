#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/angles.h>

#include <fstream>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <math.h> 

using namespace argos;

class CTunnelling : public CLoopFunctions {

public:

	/**
	 * Class constructor
	 */
	CTunnelling();

	/**
	 * Class destructor
	 */
	virtual ~CTunnelling();

	/**
	 * Initializes the experiment.
	 * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
	 * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
	 */
	virtual void Init(TConfigurationNode& t_tree);
	virtual void Init();

	CVector3 GetRandomPosition();
	void MoveRobots();
	void InitializeArena();

	/**
	 * Resets the experiment to the state it was right after Init() was called.
	 * It is executed every time you press the 'reset' button in the GUI.
	 */
	virtual void Reset();

	/**
	 * Undoes whatever Init() did.
	 * It is executed once when ARGoS has finished the experiment.
	 */
	virtual void Destroy();

	/**
	 * Performs actions right before a simulation step is executed.
	 */
	virtual void PreStep();

	/**
	 * Performs actions right after a simulation step is executed.
	 */
	virtual void PostStep();

	virtual void PostExperiment();

	/**
	 * Returns the color of the floor at the specified point on.
	 * @param c_position_on_plane The position at which you want to get the color.
	 * @see CColor
	 */
	virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

	bool IsOnColor(const CVector2& c_position_on_plane, const CVector2& center);

	void FilterObjects();
	
private:

	/**
	 * The path of the output file.
	 */
	std::string m_strOutFile;

	/**
	 * The stream associated to the output file.
	 */
	std::ofstream m_cOutFile;

	/**
	 * Number of robots in black area.
	 */
	UInt64 m_unNumRobots;

	/**
	 * Number of items collected in tagret area
	 */
	UInt32 m_unNbrItems;

	/**
	 * Time step counter
	 */
	UInt32 m_unTimeStep;

	/**
	 * Score = sum(numRobots - numObstacles) over time 
	 */
	int32_t m_unScore;

	/**
	* This vector contains a list of positions of objects in the construction area
	*/
	std::vector<CVector3> m_vecConstructionObjectsInArea;

	/**
	 * Random number generator
	 */
	CRandom::CRNG* m_pcRNG;
};
