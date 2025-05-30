#ifndef BLUEBATTERYSENSOR_H_
#define BLUEBATTERYSENSOR_H_

/******************************************************************************/
/******************************************************************************/


/* This is the class of the ground sensor */
/* In the real robot there is 3 ground sensors but here we will use just one, that will correspond to the 
 * central one */
class CBlueBatterySensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_BLUE_BATTERY_SENSORS 1

/******************************************************************************/
/******************************************************************************/

class CBlueBatterySensor : public CSensor
{
public:
    CBlueBatterySensor(const char* pch_name, double f_range, double f_charge_coef, double f_discharge_coef);
    ~CBlueBatterySensor();
	
	//Method Set Battery Level
	void SetBatteryLevel(double fLevel);


    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator);
	 	
		//Get Reading
		double* GetSensorReading( CEpuck *pc_epuck);
		double GetBatteryLevel ( void );
		void 	Reset();
    
protected:
	double m_fBatteryLevel;
	double m_fRange;
	double m_fChargeCoef;
	double m_fDischargeCoef;
};

/******************************************************************************/
/******************************************************************************/

#endif
