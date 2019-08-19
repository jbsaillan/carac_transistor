#ifndef DRIVER_CARAC_C
#define DRIVER_CARAC_C




/* 
	-----------------------------------------------------------------------------
		On choisit ici les différents paramètres des différentes carac à faire
	-----------------------------------------------------------------------------
*/

//Vds à imposer pour les impulsions en mV
uint16_t vds_impulsion[3] = {50, 23, 233};
//Durees des impulsions en µs
uint16_t temps_impulsion[3] = {10, 14, 15};

//Temps fixe pour attendre la polarisation de la diode structurelle
uint16_t temps_temp = 5;

//Vds à imposer pour la phase de caractérisation en mV
uint16_t vds_carac[3] = {50, 23, 3500};
//Temps d'établissement des valeurs pour la phase de caractérisation
uint16_t temps_carac[3] = {17, 11, 15};



#endif
