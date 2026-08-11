#include "tunnelling.h"

/****************************************/
/****************************************/
static const Real HEIGHT = 2;
static const Real WIDTH = 8;

static const CVector2 cTarget = CVector2(0,-2);
static const CVector2 cNest = CVector2(0,2);

/****************************************/
/****************************************/

CTunnelling::CTunnelling()
{
	m_unNumRobots = 0;
	m_unScore = 0;
	m_unTimeStep = 0;
}

/****************************************/
/****************************************/

CTunnelling::~CTunnelling() {}

/****************************************/
/****************************************/

void CTunnelling::Init(TConfigurationNode& t_tree)
{
	GetNodeAttribute(t_tree, "output", m_strOutFile);
	Init();
}

/****************************************/
/****************************************/

void CTunnelling::Init()
{
	/* Open the file for text writing */
	m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
	if (m_cOutFile.fail())
	{
		THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}

	m_pcRNG = CRandom::CreateRNG("argos");

	m_unNbrItems = 0;
	m_unNumRobots = 0;
	m_unScore = 0;

	/* Initialise food data for the robots */
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	//m_unNumRobots = m_cFootbots.size();
	//for (size_t i = 0; i < m_unNumRobots; ++i)
	m_unNumRobots = 0;

	// std::cout << "m_unNumRobots: " << m_unNumRobots << std::endl;

	/* Write the header of the output file */
	m_cOutFile << "#Clock\tRobots\tObstacles\tScore" << std::endl;
}

/****************************************/
/****************************************/

void CTunnelling::Reset()
{
	/* Close the output file */
	m_cOutFile.close();
	if (m_cOutFile.fail())
	{
		THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}

	Init();

	/* Reseting the variables. */
	m_unNbrItems = 0;
	m_unNumRobots = 0;
	m_unTimeStep = 0;
	m_unScore = 0;

	/* Erasing content of file. Writing new header. */
	m_cOutFile << "#Clock\tRobots\tObstacles\tScore" << std::endl;
}

/****************************************/
/****************************************/

void CTunnelling::Destroy()
{
	/* Close the output file */
	m_cOutFile.close();
	if (m_cOutFile.fail())
	{
		THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}
}

/****************************************/
/****************************************/

void CTunnelling::PreStep() {}

/****************************************/
/****************************************/

void CTunnelling::PostStep()
{
	/* Get the position of the foot-bot on the ground as a CVector2 */
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	m_unNumRobots = 0;
	for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
	{
		/* Get handle to foot-bot entity and controller */
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CVector2 cPos;
		cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
		if(IsOnColor(cPos, cTarget)){
			m_unNumRobots = m_unNumRobots + 1;
		}
			
	}

	FilterObjects();
	m_unNbrItems = m_vecConstructionObjectsInArea.size();

	m_unScore = m_unNumRobots - m_unNbrItems;
	
	/* Increase the time step counter */
	m_unTimeStep += 1;
	
	/* Writting data to output file. */
	m_cOutFile << m_unTimeStep << "\t" << m_unNumRobots << "\t" << m_unNbrItems << "\t" << m_unScore << std::endl;

	/* Output in simulator */
	LOGERR << "Score = " << m_unScore << std::endl;
}

/****************************************/
/****************************************/

void CTunnelling::PostExperiment()
{   
	/* Get the position of the foot-bot on the ground as a CVector2 */
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	m_unNumRobots = 0;
	for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
	{
		/* Get handle to foot-bot entity and controller */
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CVector2 cPos;
		cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
		if(IsOnColor(cPos, cTarget)){
			m_unNumRobots = m_unNumRobots + 1;
		}
			
	}

	FilterObjects();
	m_unNbrItems = m_vecConstructionObjectsInArea.size();

	m_unScore = m_unNumRobots - m_unNbrItems;
	
	/* Increase the time step counter */
	m_unTimeStep += 1;
	
	/* Writting data to output file. */
	m_cOutFile << m_unTimeStep << "\t" << m_unNumRobots << "\t" << m_unNbrItems << "\t" << m_unScore << std::endl;
	
	/* Output in simulator */
	LOGERR << "Score = " << m_unScore << std::endl;

}

/****************************************/
/****************************************/

argos::CColor CTunnelling::GetFloorColor(const argos::CVector2& c_position_on_plane) 
{
	/* target area is black */
	//CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
	
	if (IsOnColor(c_position_on_plane, cTarget))
	{
		return CColor::BLACK;
	}
	
	/* Nest area is white */
	if (IsOnColor(c_position_on_plane, cNest))
	{
		return CColor::WHITE;
	}
	
	else {
		return CColor::GRAY50;
 	}
}

/****************************************/
/****************************************/

bool CTunnelling::IsOnColor(const CVector2& c_position_on_plane, const CVector2& center)
{
	return (center.GetX() - WIDTH/2 < c_position_on_plane.GetX() - center.GetX() && c_position_on_plane.GetX() - center.GetX() < center.GetX() + WIDTH/2 
	 && center.GetY() - HEIGHT/2 < c_position_on_plane.GetY() - center.GetY() && c_position_on_plane.GetY() - center.GetY() < center.GetY() + HEIGHT/2);
}

/****************************************/
/****************************************/

void CTunnelling::FilterObjects() {
   /* Clear list of positions of objects in construction area */
   m_vecConstructionObjectsInArea.clear();

   /* Get the list of cylinders from the ARGoS space */
   CSpace::TMapPerType& tCylinderMap = GetSpace().GetEntitiesByType("cylinder");
   /* Go through the list and collect data */
   CCylinderEntity* pcCylinder;
   for(CSpace::TMapPerType::iterator it = tCylinderMap.begin();
	   it != tCylinderMap.end();
	   ++it) {
	  /* Get a reference to the object */	 
	  pcCylinder = any_cast<CCylinderEntity*>(it->second);
	  //CEmbodiedEntity& cBody = any_cast<CCylinderEntity*>(it->second)->GetEmbodiedEntity();
	  /* Check if object is in target area */
	  CVector2 cPos;
	  cPos.Set(pcCylinder->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pcCylinder->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
	  if(IsOnColor(cPos, cTarget)) {
		 /* Yes, it is */
		 /* Add it to the list */
		 m_vecConstructionObjectsInArea.push_back(pcCylinder->GetEmbodiedEntity().GetOriginAnchor().Position);
	  }
   }
}

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CTunnelling, "tunnelling");
