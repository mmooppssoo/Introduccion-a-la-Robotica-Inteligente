
	
	/* FROM HERE YOU NEED TO CREATE YOU FITNESS */	

	/* 1.   calculo de posicion del robot con encoders */
	/* Reset al comienzo de cada episodio */
	if(n_simulation_step == 0){
		m_Visited.clear();      // vacia el mapa de celdas
		bestY = 0.0;          // reinicia gradiente
		m_fOrient = 0.0;
		m_fX = 0.0;
		m_fY = 0.0;
	}

	/* Obtener desplazamiento y cambio de orientacion */
	double dl = m_fEncoder[0];  
	double dr = m_fEncoder[1];  
	double dc = (dl + dr) / 2.0;
	double dtheta = (dr - dl) / 0.053; 

	/* Actualizacion de la posicion y orientacion con encoders */
	m_fOrient += dtheta;
	if (m_fOrient> 2 * M_PI) m_fOrient -= 2 * M_PI;
	if (m_fOrient < 0) m_fOrient += 2 * M_PI;
	m_fX += dc * cos(m_fOrient);
	m_fY += dc * sin(m_fOrient);

	/* Calculo de visitas */
	const double CELL = 0.05;        // rejilla de 5 cm
	int ix = (int) floor( (m_fX + 1.5) / CELL );   // 3×3 arena centrada en 0
	int iy = (int) floor( (1.5 - m_fY) / CELL );
	long long key = ((long long)ix << 20) | iy;    // empaqueta dos int en 64 bit
	int v = ++m_Visited[key];                      // incrementa visitas

	/* 2.   progreso hacia la meta */
	double Y = maxLightSensorEval;
	double deltaY = (Y > bestY + 1e-3) ? (Y - bestY) : 0.0;
	if(deltaY > 0) bestY = Y;

	/* 3.   velocidad recta hacia delante */
	double Fwd = maxSpeedEval * sameDirectionEval;                   // 0-1                  

	/* 4.   wallFactor dependiente de δY */
	double rightWall = m_fProx[2];
	double wallGauss = exp( -pow(rightWall - 0.70,2)/(2*0.08*0.08) );
	double wallFactor = (deltaY > 0 ? 0.5 + 0.5*wallGauss : 0.5);

	/* 5.   penalizaciones */
	double P = std::max(m_fProx[0], m_fProx[7]);
	double R = *std::max_element(m_fRed, m_fRed+8);
	double wallSafe = 1.0 - P;
	double redSafe  = 1.0 - R;

	/* 6.   fitness base */
	double fitness = (0.7*deltaY + 0.2*Fwd + 0.1*wallFactor) * wallSafe * redSafe;

	/* 7.   castigo por revisitar celdas */
	const int MAX_VISITS = 2;    // toleramos 2 pasos en la misma celda
	const double P_REVISIT = 0.1;    // penaliza al 10 % si se pasa
	if(v > MAX_VISITS) fitness *= P_REVISIT;

	/* 8.   decaimiento temporal (fuerza a progresar) */
	fitness *= 0.999;

	/* 9.   eventos terminales */
	bool goal = (Y > 0.95);
	bool fallen = (groundMemory[0] > 0.5);
	if(goal) fitness = 1.0;
	if(fallen) fitness = 0.0;

	m_bGoalReached |= goal;
	
	/* TO HERE YOU NEED TO CREATE YOU FITNESS */	

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
}

/******************************************************************************/
/******************************************************************************/
