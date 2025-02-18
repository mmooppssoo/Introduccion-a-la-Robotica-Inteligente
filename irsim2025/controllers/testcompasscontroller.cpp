
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testcompasscontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestCompassController::CTestCompassController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor(SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor(SENSOR_COMPASS);

  /* Init variables to use for odometry calculation */
  m_fOrientation = 0.0;
  m_vPosition.x = 0.0;
  m_vPosition.y = 0.0;

  maxErrorA.x = 0.0;
  maxErrorA.y = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CTestCompassController::~CTestCompassController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestCompassController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	/* Read Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Read Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);
  /* Get distance between wheels */
  double fWheelsDistance = CEpuck::WHEELS_DISTANCE;

  /* STUDENTS MUST IMPLEMENT THE CODE HERE - START 
   * IMPORTANT!!: The code must exclusively depend of "encoder" and "compass" variable */
    
  /* Definir parametros */
  const double METRO = 1.0; 
  const double GIRO_90_RADIANES = M_PI / 2.0; 
  const double TOLERANCIA_DISTANCIA_VEL_500 = 0.00644;
  const double TOLERANCIA_DISTANCIA_VEL_10 = 0.000128; 
  const double TOLERANCIA_ANGULO_VEL_100 = 0.0486;
  const double TOLERANCIA_ANGULO_VEL_1 = 0.000486;  

  static int estado = 0; 
  static double distancia_recorrida = 0.0;
  static double angulo_recorrido = 0.0;
  static double sentido_giro = 1;
  static bool primera_lectura = true; 

  /* Obtener desplazamiento y cambio de orientacion */
  double dl = encoder[0];  
  double dr = encoder[1];  
  double dc = (dl + dr) / 2.0;
  double dtheta = (dr - dl) / fWheelsDistance; 

  /* Primera lectura: usar valores reales para ubicar la posicion y orientacion iniciales */
  if (primera_lectura) {
    m_fOrientation = compass[0];
    m_vPosition.x = m_pcEpuck->GetPosition().x;
    m_vPosition.y = m_pcEpuck->GetPosition().y;
    primera_lectura = false;
  } else {
    /* Actualizacion de la posicion y orientacion con encoders */
    m_fOrientation += dtheta;
    if (m_fOrientation > 2 * M_PI) m_fOrientation -= 2 * M_PI;
    if (m_fOrientation < 0) m_fOrientation += 2 * M_PI;
    m_vPosition.x += dc * cos(m_fOrientation);
    m_vPosition.y += dc * sin(m_fOrientation);
  }

  printf("Posicion: (%f, %f) - Orientacion: %f\n", m_vPosition.x, m_vPosition.y, m_fOrientation);

  /* Estado 0: Determinar orientacion inicial y sentido de giro */
  if (estado == 0) { 
    double objetivo_orientacion;
    if (m_vPosition.x + METRO < 1.35) {
      objetivo_orientacion = 0; 
      sentido_giro = (m_vPosition.y + METRO > 1.35) ? -1 : 1;
    } else {
      objetivo_orientacion = M_PI; 
      sentido_giro = (m_vPosition.y + METRO > 1.35) ? 1 : -1;
    }

    double diferencia = objetivo_orientacion - m_fOrientation;
    if (diferencia > M_PI) diferencia -= 2 * M_PI;
    if (diferencia < -M_PI) diferencia += 2 * M_PI;

    if (fabs(diferencia) >= 2 * TOLERANCIA_ANGULO_VEL_1) { 
      int velocidad_giro = (fabs(diferencia) > TOLERANCIA_ANGULO_VEL_100) ? 100 : 1;
      m_acWheels->SetSpeed(diferencia > 0 ? -velocidad_giro : velocidad_giro, diferencia > 0 ? velocidad_giro : -velocidad_giro);
    } else {
      if (sentido_giro == 1) printf("Sentido giro: antihorario\n");
      else printf("Sentido giro: horario\n");
      estado = 1;
    }

  /* Estado 1: Avanzar 1 metro en linea recta */ 
  } else if (estado == 1) { 
    distancia_recorrida += dc;
    int velocidad = (distancia_recorrida >= METRO - TOLERANCIA_DISTANCIA_VEL_500) ? 10 : 500;
    m_acWheels->SetSpeed(velocidad, velocidad);

    if (distancia_recorrida >= METRO - TOLERANCIA_DISTANCIA_VEL_10) {
      distancia_recorrida = 0.0;
      estado = 2;
    }
   
  /* Estado 2: Girar 90 grados */
  } else if (estado == 2) { 
    angulo_recorrido += dtheta; 
    int velocidad_giro = (fabs(angulo_recorrido) >= GIRO_90_RADIANES - TOLERANCIA_ANGULO_VEL_100) ? 1 : 100;
    m_acWheels->SetSpeed(sentido_giro * -velocidad_giro, sentido_giro * velocidad_giro);
    
    if (fabs(angulo_recorrido) >= GIRO_90_RADIANES - TOLERANCIA_ANGULO_VEL_1) {
      angulo_recorrido = 0.0;
      estado = 1;
    }
  }

 
  /* STUDENTS MUST IMPLEMENT THE CODE HERE - END*/
  
  /* DEBUG */ 
  //printf("REAL: %2f,%2f,%2f  -- ODOM: %2f,%2f,%2f -- ENC: %2f,%2f \n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y, compass[0], m_vPosition.x,m_vPosition.y,m_fOrientation,encoder[0], encoder[1]);
  
  //if ( fabs((m_pcEpuck->GetPosition()).x - m_vPosition.x ) > maxErrorA.x )
    //maxErrorA.x = fabs((m_pcEpuck->GetPosition()).x - m_vPosition.x );

  //if ( fabs((m_pcEpuck->GetPosition()).y - m_vPosition.y ) > maxErrorA.y )
    //maxErrorA.y = fabs((m_pcEpuck->GetPosition()).y - m_vPosition.y );

  //printf("ERROR: %2f,%2f,%2f\n", maxErrorA.x, maxErrorA.y, compass[0] - m_fOrientation);
  /* DEBUG */ 
 



}

/******************************************************************************/
/******************************************************************************/

