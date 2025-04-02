#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    /* ROBOT */
    CEpuck* m_pcEpuck;
   
	/* SENSORS / ACTUATORS */
   	CWheelsActuator* m_acWheels;
	CEpuckProximitySensor* m_seProx;
   	CGroundSensor* m_seGround;
   	CGroundMemorySensor* m_seGroundMemory;
	CRealBlueLightSensor* m_seBlueLight;
	CRealRedLightSensor* m_seRedLight;
	CBlueBatterySensor* m_seBlueBattery;  
  

   	/* Global Variables */
   	double 		m_fLeftSpeed;
   	double 		m_fRightSpeed;
   	double**	m_fActivationTable;
   	int 		m_nWriteToFile;
   	double 		m_fTime;
	double 		newBattery;

   	/* Functions */
   	void ExecuteBehaviors ( void );
   	void Coordinator ( void );

   	void ObstacleAvoidance ( unsigned int un_priority );
	void Rescue ( unsigned int un_priority );
	void LoadWater ( unsigned int un_priority );
	void FightFire ( unsigned int un_priority );
   	void Navigate ( unsigned int un_priority );
};

#endif
