#ifndef IRIFITNESSFUNCTION_H_
#define IRIFITNESSFUNCTION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <utility>          // std::pair
using namespace std;

class CIriFitnessFunction;

#include "fitnessfunction.h"
#include "simulator.h"
#include "encodersensor.h"

/******************************************************************************/
/******************************************************************************/

class CIriFitnessFunction : public CFitnessFunction
{
public:
    CIriFitnessFunction(const char* pch_name, CSimulator* pc_simulator,
                                    unsigned int un_collisions_allowed_per_epuck);
		~CIriFitnessFunction();
    virtual double GetFitness();
		virtual void SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval);

protected:
		unsigned int m_unNumberOfSteps;
		double 			m_fComputedFitness;
		bool m_bGoalReached;
		CEpuck* m_pcEpuck;
		double m_fProx[8];
		double m_fRed[8];
		double m_fEncoder[2];
		/* --- odometría --- */
		double           m_fOrientation;      // orientación acumulada
		dVector2         m_vPosition;     // posición acumulada
	
		/* --- mapa de visitas --- */
		std::unordered_map<long long,int> m_Visited;  // key = (ix<<20)|iy

};

/******************************************************************************/
/******************************************************************************/

#endif
