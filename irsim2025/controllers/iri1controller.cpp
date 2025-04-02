/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "bluebatterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

/******************** Behaviors **************/
#define BEHAVIORS	5

#define AVOID_PRIORITY 		0
#define RESCUE_PRIORITY 	1
#define RELOAD_PRIORITY		2
#define FIRE_PRIORITY		3
#define NAVIGATE_PRIORITY   4

/******************** Constants **************/
/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.6
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.499999
/* Threshold to fight fires */
#define FIGHT_FIRE_THRESHOLD 0.1
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9 //REVISAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAR

#define SPEED 500

CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);

	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}

}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();
	
	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	/* Leer Sensores de Bateria Azul */
	double* battery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
	printf("Bateria: %f",battery[0]);

	if (m_nWriteToFile ) 
	{
	/* INIT: WRITE TO FILES */
	/* Write robot position and orientation */
		FILE* filePosition = fopen("outputFiles/robotPosition", "a");
		fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
		fclose(filePosition);

		/* Write robot wheels speed */
		FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
		fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileWheels);
		/* END WRITE TO FILES */
	}

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLUE);
	
	ObstacleAvoidance ( AVOID_PRIORITY );
	Rescue ( RESCUE_PRIORITY );
	LoadWater ( RELOAD_PRIORITY );
	FightFire ( FIRE_PRIORITY );
	Navigate ( NAVIGATE_PRIORITY );
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator ( void )
{
	int nBehavior;
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
			break;
		}
	}

	m_fLeftSpeed = m_fActivationTable[nBehavior][0];
	m_fRightSpeed = m_fActivationTable[nBehavior][1];
	
  printf("%d %2.4f %2.4f \n", nBehavior, m_fLeftSpeed, m_fRightSpeed);
	
  if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);


		double fCLinear = 1.0;
		double fCAngular = 1.0;
		double fC1 = SPEED / M_PI;
		
		/* Calc Linear Speed */
		double fVLinear = SPEED * fCLinear * ( cos ( fRepelent / 2) );

		/*Calc Angular Speed */
		double fVAngular = fRepelent;

		m_fActivationTable[un_priority][0] = fVLinear - fC1 * fVAngular;
		m_fActivationTable[un_priority][1] = fVLinear + fC1 * fVAngular;
		m_fActivationTable[un_priority][2] = 1.0;
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate ( unsigned int un_priority )
{
	// /* Leer Sensores de Luz */
	// double* light = m_seBlueLight->GetSensorReading(m_pcEpuck);
	// double fTotalLight = 0.0;

	// for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	// {
	// 	fTotalLight += light[i];
	// }
	
	// if ( fTotalLight >= NAVIGATE_LIGHT_THRESHOLD )
	// {
	// 	m_fActivationTable[un_priority][0] = SPEED/4;
	// 	m_fActivationTable[un_priority][1] = SPEED/4;
	// }
	// else
	// {
	// 	m_fActivationTable[un_priority][0] = SPEED;
	// 	m_fActivationTable[un_priority][1] = SPEED;
	// }

	m_fActivationTable[un_priority][0] = SPEED;
	m_fActivationTable[un_priority][1] = SPEED;
	
	m_fActivationTable[un_priority][2] = 1.0;

	// if (m_nWriteToFile ) 
	// {
	// 	/* INIT: WRITE TO FILES */
	// 	/* Write level of competence ouputs */
	// 	FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
	// 	fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f %2.4f\n", m_fTime, fTotalLight, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
	// 	fclose(fileOutput);
	// 	/* END WRITE TO FILES */
	// }
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::LoadWater( unsigned int un_priority )
{
	/* Leer Sensores de Bateria Azul */
	double* battery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
		
	/* Leer Sensores de Luz Azul */
	double* light = m_seBlueLight->GetSensorReading(m_pcEpuck);

	if ( battery[0] < BATTERY_THRESHOLD )
	{
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_RED);
		
		/* If not pointing to the light */
		if ( ( light[0] * light[7] == 0.0 ) )
		{
			m_fActivationTable[un_priority][2] = 1.0;

			double lightLeft 	= light[0] + light[1] + light[2] + light[3];
			double lightRight = light[4] + light[5] + light[6] + light[7];

			if ( lightLeft > lightRight )
			{
				m_fActivationTable[un_priority][0] = -SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
			else
			{
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = -SPEED;
			}
		}
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Rescue ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* light = m_seBlueLight->GetSensorReading(m_pcEpuck);
	
	/* Si recoge a una persona */
	if ( groundMemory[0]  == 1.0 )
	{
		/* Set Leds to YELLOW */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_YELLOW);
		
		/* Go oposite to the light */
		if ( ( light[3] * light[4] == 0.0 ) )
		{
			m_fActivationTable[un_priority][2] = 1.0;

			double lightLeft 	= light[0] + light[1] + light[2] + light[3];
			double lightRight = light[4] + light[5] + light[6] + light[7];

			if ( lightLeft > lightRight )
			{
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = -SPEED;
			}
			else
			{
				m_fActivationTable[un_priority][0] = -SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
		}
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/forageOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, groundMemory[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::FightFire ( unsigned int un_priority )
{
	/* Leer Sensores de Luz Roja */
	 double* light = m_seRedLight->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Bateria Azul */
	double* battery = m_seBlueBattery->GetSensorReading(m_pcEpuck);

	double fTotalLight = 0.0;
	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	{
		fTotalLight += light[i];
	}

	/* Si se acerca a un fuego y hay suficiente agua */
	if ((fTotalLight >= FIGHT_FIRE_THRESHOLD) && (battery[0] >= BATTERY_THRESHOLD)){

		m_fActivationTable[un_priority][2] = 1.0;

		/* Apaga el fuego */
		m_seRedLight->SwitchNearestLight(0);

		/* Gasta medio tanque de agua*/
		newBattery = battery[0] - 0.5;
		m_seBlueBattery->SetBatteryLevel(newBattery);
	}

	// if (m_nWriteToFile){
	// 	/* INIT WRITE TO FILE */
	// 	FILE* fileOutput = fopen("outputFiles/fightFireOutput", "a");
	// 	if (fileOutput != NULL){
	// 		fprintf(fileOutput, "%2.4f %2.4f ", m_fTime, newBattery);
	// 		for (int i = 0; i < m_seRedLight->GetNumberOfInputs(); i++){
	// 			fprintf(fileOutput, "%2.4f ", light[i]);
	// 		}
	// 		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
	// 		fclose(fileOutput); 
	// 	}
	// 	/* END WRITE TO FILE */
	// }

}