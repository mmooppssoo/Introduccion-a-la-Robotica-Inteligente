#include "irifitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::CIriFitnessFunction(const char* pch_name, 
                                                                 CSimulator* pc_simulator, 
                                                                 unsigned int un_collisions_allowed_per_epuck)
    :
    CFitnessFunction(pch_name, pc_simulator)
{

	/* Check number of robots */
	m_pcSimulator = pc_simulator;
	TEpuckVector* pvecEpucks=m_pcSimulator->GetEpucks();
	
	if ( pvecEpucks->size() == 0 )
	{
		printf("No Robot, so fitness function can not be computed.\n Exiting...\n");
		fflush(stdout);
		exit(0);
	}
	else if  (pvecEpucks->size()>1)
	{
		printf("More than 1 robot, and fitness is not prepared for it.\n Exiting...\n");
	}
    
	m_pcEpuck=(*pvecEpucks)[0];

	m_unNumberOfSteps = 0;
	m_fComputedFitness = 0.0;
	
}

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::~CIriFitnessFunction(){
}
/******************************************************************************/
/******************************************************************************/

double CIriFitnessFunction::GetFitness()
{    

	/* If you need to check the collisions of the robot, here are the total number of 
	 * collisions done by the robot in the simulations */
	int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

	/* Get the fitness divided by the number of steps */
	double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(coll,10.0)/10.0)));

	/* If fitness less than 0, put it to 0 */
	if ( fit < 0.0 ) fit = 0.0;

	/* If fitness more than 1, put it to 1 */
	if ( fit > 1.0 ) fit = 1.0;

	return fit;   

}

/******************************************************************************/
/******************************************************************************/
void CIriFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
{
	/* See Evolutionary Robotics Book */
	/* This is the function to be implemented */
	/* f = V * ( 1 - sqrt(Delta(v)) ) * (1 - i)
	 * V relates to the maximum speed
	 * Delta(v) relates to the movement on the same direction
	 * i relates to the maximum sensor value
	 */

	/* Get actual SPEED of the left and right wheel */
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	m_pcEpuck->GetWheelSpeed(&leftSpeed,&rightSpeed);
	leftSpeed = 0.5 + ( leftSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed = 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));

	/* Eval same direction partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));
	
	/* Eval SENSORS */

	/* Where the Max PROXIMITY sensor will be stored*/
	double maxProxSensorEval 		= 0.0;
	/* Where the Max LIGHT sensor will be stored*/
	double maxLightSensorEval 	= 0.0;
	/* Where the Max BLUE LIGHT sensor will be stored*/
	double maxBlueLightSensorEval 	= 0.0;
	/* Where the Max RED LIGHT sensor will be stored*/
	double maxRedLightSensorEval 	= 0.0;
	/* Where the Max CONTACT sensor will be stored*/
	double maxContactSensorEval = 0.0;

	/* Where the GROUND MEMORY will be stored */
	double* groundMemory;
	/* Where the GROUND will be stored */
	double* ground;
	/* whre the BATTERY will be sotored */
	double *battery;
	/* whre the BLUE BATTERY will be sotored */
	double *blueBattery;
	/* whre the RED BATTERY will be sotored */
	double *redBattery;
	/* whre the PROXIMITY will be sotored */
	double *prox;
	/* whre the ENCODER will be sotored */
	double *enc;

	double blueLightS0=0;
	double blueLightS7=0;
	double lightS0=0;
	double lightS7=0;

	/* Auxiluar variables */
	unsigned int unThisSensorsNumberOfInputs; 
	double* pfThisSensorInputs; 
	
	/* Go in all the sensors */
	TSensorVector vecSensors = m_pcEpuck->GetSensors();
	for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
	{
		/* Check type of sensor */
		switch ( (*i)->GetType() )
		{
			/* If sensor is PROXIMITY */
			case SENSOR_PROXIMITY:
				/* Get the number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					m_fProx[j] = pfThisSensorInputs[j];
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxProxSensorEval )
					{	
						/* Store maximum value */
						maxProxSensorEval = pfThisSensorInputs[j];
					}
				}
				break;

			/* If sensor is GROUND_MEMORY */
			case SENSOR_GROUND_MEMORY:
				/* Get the actual value */
				groundMemory = (*i)->GetComputedSensorReadings();
				break;
	
			/* If sensor is GROUND */
			case SENSOR_GROUND:
				/* Get actual values */
				ground = (*i)->GetComputedSensorReadings();
				break;	
			/* If sensor is LIGHT */
			case SENSOR_REAL_LIGHT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxLightSensorEval )
					{	
						/* Store maximum value */
						maxLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						lightS0 = pfThisSensorInputs[j];
					else if (j==7)
						lightS7 = pfThisSensorInputs[j];
				}
				break;
			case SENSOR_REAL_BLUE_LIGHT:
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					if ( pfThisSensorInputs[j] > maxBlueLightSensorEval )
					{	
						maxBlueLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						blueLightS0 = pfThisSensorInputs[j];
					else if (j==7)
						blueLightS7 = pfThisSensorInputs[j];
				}
				break;
			
			case SENSOR_REAL_RED_LIGHT:
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					m_fRed[j] = pfThisSensorInputs[j];
					if ( pfThisSensorInputs[j] > maxRedLightSensorEval )
					{	
						maxRedLightSensorEval = pfThisSensorInputs[j];
					}
				}
				break;

			/* If sensor is BATTERY */
			case SENSOR_BATTERY:
         battery = (*i)->GetComputedSensorReadings();
				 break;
			
			case SENSOR_BLUE_BATTERY:
				blueBattery = (*i)->GetComputedSensorReadings();
				break;
			
			case SENSOR_RED_BATTERY:
				redBattery = (*i)->GetComputedSensorReadings();
				break;
			
			/* If sensor is CONTACT */
			case SENSOR_CONTACT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxContactSensorEval )
					{
						/* Store maximum value */
						maxContactSensorEval = pfThisSensorInputs[j];
					}
				}
				break;
			/* If sensor is ENCODER */
			case SENSOR_ENCODER:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					m_fEncoder[j] = pfThisSensorInputs[j];
				}
				break;
		}
	}
	
	
	/* FROM HERE YOU NEED TO CREATE YOU FITNESS */	
	/* ------------------ parametros ------------------ */
	double fWheelsDistance = CEpuck::WHEELS_DISTANCE;
	const double CELL = 0.05;    // rejilla 5 cm

	/* ---------- reset al inicio del episodio ---------- */
	if(n_simulation_step == 0){
		m_Visited.clear();
		m_fOrientation = 0.0;
    	m_vPosition.x = 0.0;
  		m_vPosition.y = 0.0;
	}
	
	/* Obtener desplazamiento y cambio de orientacion */
	double dl = m_fEncoder[0];  
	double dr = m_fEncoder[1];  
	double dc = (dl + dr) / 2.0;
	double dtheta = (dr - dl) / fWheelsDistance; 

  /* Actualizacion de la posicion y orientacion con encoders */
  	m_fOrientation += dtheta;
  	if (m_fOrientation > 2 * M_PI) m_fOrientation -= 2 * M_PI;
  	if (m_fOrientation < 0) m_fOrientation += 2 * M_PI;
  	m_vPosition.x += dc * cos(m_fOrientation);
  	m_vPosition.y += dc * sin(m_fOrientation);

	/* ---------- actualiza rejilla ---------- */
	int ix = (int) floor( (m_vPosition.x+1.5)/CELL );
	int iy = (int) floor( (1.5-m_vPosition.y)/CELL );
	long long key = ((long long)ix<<20) | iy;
	int v = ++m_Visited[key];          // visitas reales

	// printf("Encoder: (%f, %f) \n", m_fEncoder[0], m_fEncoder[1]);
	/* 2.   progreso hacia la meta */
	double Y = maxLightSensorEval;

	/* 3.   velocidad recta  */
	double Fwd =  maxSpeedEval * sameDirectionEval; // 0–1

	/* 4.   wallFactor dependiente de Y */
	double rightWall = m_fProx[2];
	double wallGauss = exp( -pow(rightWall - 0.70,2)/(2*0.08*0.08) );
	double wallFactor = (Y > 0 ? 0.5 + 0.5*wallGauss : 0.5);

	/* 5.   penalizaciones */
	double P = std::max(m_fProx[0], m_fProx[7]);
	double R = *std::max_element(m_fRed, m_fRed+8);
	double wallSafe = 1.0 - P;
	double redSafe = 1.0 - R;

	/* 6.   fitness base */
	double fitness = (0.7*Y + 0.2*Fwd + 0.1*wallFactor) * wallSafe * redSafe;

	// /* 7.   castigo por revisitar celdas */
	const int MAX_VISITS = 2;    // toleramos 2 pasos en la misma celda
	const double P_REVISIT = 0.1;    // penaliza al 10 % si se pasa
	if(v > MAX_VISITS) fitness *= P_REVISIT;

	/* 8.   decaimiento temporal (fuerza a progresar) */
	fitness *= 0.999;

	/* 9.   eventos terminales */
	bool goal   = (Y > 0.95);
	bool fallen = (groundMemory && groundMemory[0] > 0.5);
	if(goal) {fitness = 1.0; m_bGoalReached = true;}
	if(fallen) fitness = 0.0;

	
	/* TO HERE YOU NEED TO CREATE YOU FITNESS */	

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
}

/******************************************************************************/
/******************************************************************************/