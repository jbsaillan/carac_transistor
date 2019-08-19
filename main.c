/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;




/* USER CODE BEGIN PV */

/* 
	-----------------------------------------------------------------------------
		On choisit ici les différents paramètres des différentes carac à faire
	-----------------------------------------------------------------------------
*/

//Le nombre de points à évaluer
uint16_t const nmbr_tests = 3;

//Vds à imposer pour les impulsions en mV
uint16_t vds_impulsion[nmbr_tests] = {50, 23, 233};
//Durees des impulsions en µs
uint16_t temps_impulsion[nmbr_tests] = {10, 14, 15};

//Temps fixe pour attendre la polarisation de la diode structurelle
uint16_t temps_temp = 5;

//Vds à imposer pour la phase de caractérisation en mV
uint16_t vds_carac[nmbr_tests] = {50, 23, 3500};
//Temps d'établissement des valeurs pour la phase de caractérisation
uint16_t temps_carac[nmbr_tests] = {17, 11, 15};

/* 
	-----------------------------------------------------------------------------
		On stocke ici les différents résultats
	-----------------------------------------------------------------------------
*/
uint16_t res_mesure_temp[nmbr_tests] = {0,0,0};
uint16_t res_mesure_carac[nmbr_tests] = {0,0,0};


/* 
	-----------------------------------------------------------------------------
		On initialise ici différents paramètres utiles
	-----------------------------------------------------------------------------
*/
//Le numéro du test en cours (représente l'indice pour accéder aux différents valeurs dans les tableaux ci-dessous
uint16_t id_test = 0;
//Le nombre de µs ecoulees depuis le début de la carac
uint16_t temps_ecoule = 0;


/* USER CODE END PV */






/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);



/* USER CODE BEGIN PFP */
static void IntToChar(uint32_t number, uint8_t * buffer, uint8_t len);
static void init_test(uint16_t id);
static void phase_impulsion(void);
static void phase_mesure_temp(void);
static void mesure_temp(void);
static void phase_carac(void);
static void mesure_carac(void);
static void finir_test(void);
static void envoyer_resultats(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
	
	

  /* USER CODE BEGIN 2 */
	
	//On lance la PWM de synchronisation
	//HAL_TIM_PWM_Init(&htim1);
	
	//Une fois que les paramètres sont choisi, on lance l'initialisation des mesures
	init_test(id_test);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_CYCLE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|cmd_mesure_Pin|cmd_blocage_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, img_acq_Pin|img_carac_Pin|img_mesure_Pin|img_impulsion_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin cmd_mesure_Pin cmd_blocage_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|cmd_mesure_Pin|cmd_blocage_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : img_acq_Pin img_carac_Pin img_mesure_Pin img_impulsion_Pin */
  GPIO_InitStruct.Pin = img_acq_Pin|img_carac_Pin|img_mesure_Pin|img_impulsion_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}






/* USER CODE BEGIN 4 */


/**
	* @fn IntToChar
	* @brief Converts an integer to a string.
	*
	* @param number The integer to convert
	* @param buffer The buffer where to store the characters 
	* @len the lenght of the buffer
*/
static void IntToChar(uint32_t number, uint8_t * buffer, uint8_t len) {
	uint32_t a;
	for(int i=1; i < 100000; i*=10) {
		a = (number / i)%10;
		len = len - 1;
		if(len >= 0) buffer[len] = '0' + a;
	}
}



/**
 * @fn init_test
 * @brief Initialize the circuit in order to be ready for the next test.
 * 
 * @note At the end, we directly begin the test with pahse_impulsion()
 *
 * @param id The id of the next test (will be used to pick the command in the dfirrent arrays)
*/
static void init_test(uint16_t id) {
	//On arrête le décompte du temps
	HAL_TIM_Base_Stop_IT(&htim2);
	//On reset la valeur du timer pour bien être précis
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	//On re-initialise le compteur virtuel de temps :
	temps_ecoule = 0;
	
	//On actualise l'id du test que l'on souhaite effectuer
	if(id < nmbr_tests) {
		id_test = id;
		//Ici on activera/désactivera l'ensemble des différentes commandes pour les mesures de précision suivant les valeurs de carac
		//---------
		phase_impulsion();
	}
	
	
}

/**
 * @fn phase_impulsion
 * @brief Launch the current pulse.
 * 
 * @note It is launched after the initialization. At the end we launch the timer. The pulse will be over when the time is ellapsed.
*/
static void phase_impulsion(void) {
	// On désactive la commande du blocage du transistor du haut, il sera passant !
	HAL_GPIO_WritePin(cmd_blocage_GPIO_Port, cmd_blocage_Pin, GPIO_PIN_RESET);
	
	//On applique le pnt de fonctionnement
	//Normalement, on sort les valeurs de 3 DACs, ici on simule avec un GPIO
	HAL_GPIO_WritePin(img_impulsion_GPIO_Port, img_impulsion_Pin, GPIO_PIN_SET);
	
	//On lance le décompte du temps à partir de maintenant
	HAL_TIM_Base_Start_IT(&htim2);
}





/**
 * @fn phase_mesure_temp
 * @brief Launch the temperature measure.
 * 
 * @note It is launched when phase_impulsion is over. At the end, we wait for the voltage to be settled.
*/
static void phase_mesure_temp(void) {
	//On arrête le décompte du temps
	HAL_TIM_Base_Stop_IT(&htim2);
	//On reset la valeur du timer pour bien être précis
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	
	
	// On active la commande du blocage du transistor du haut, il sera bloqué !
	HAL_GPIO_WritePin(cmd_blocage_GPIO_Port, cmd_blocage_Pin, GPIO_PIN_SET);
	//On désactive le point de fonctionnement (normalement on met les DACs à 0)
	HAL_GPIO_WritePin(img_impulsion_GPIO_Port, img_impulsion_Pin, GPIO_PIN_RESET);	
	
	
	//On active la source de courant qui polarisera la diode
	HAL_GPIO_WritePin(cmd_mesure_GPIO_Port, cmd_mesure_Pin, GPIO_PIN_SET);
	//On active la visualisation de la mesure de température
	HAL_GPIO_WritePin(img_mesure_GPIO_Port, img_mesure_Pin, GPIO_PIN_SET);
	
	//On relance la mesure du temps quand tout est terminé
	HAL_TIM_Base_Start_IT(&htim2);
}




/**
 * @fn mesure_temp
 * @brief Measure the temperature with an ADC
 * 
 * @note It is launched when the voltage is settled. We activate the ADC
*/
static void mesure_temp(void) {
	//On arrête le décompte du temps
	HAL_TIM_Base_Stop_IT(&htim2);
	//On reset la valeur du timer pour bien être précis
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	
	//On lance directement la conversion
	HAL_ADC_Start_IT(&hadc1);	
}





/**
 * @fn phase_carac
 * @brief Launch the characterisation
 * 
 * @note It is launched when the temperature measure is over. We the wait for the working point to be settled.
*/
static void phase_carac(void) {
	//On désactive la source de courant
	HAL_GPIO_WritePin(cmd_mesure_GPIO_Port, cmd_mesure_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(img_mesure_GPIO_Port, img_mesure_Pin, GPIO_PIN_RESET);
	
	//On désactive la commande de blocage du transistor du haut : il sera passant
	HAL_GPIO_WritePin(cmd_blocage_GPIO_Port, cmd_blocage_Pin, GPIO_PIN_RESET);
	
	//On active le pnt de fonctionnement (normalement, on donne un consigne aux DACs)
	HAL_GPIO_WritePin(img_carac_GPIO_Port, img_carac_Pin, GPIO_PIN_SET);
	
	//On relance la mesure du temps quand tout est terminé
	HAL_TIM_Base_Start_IT(&htim2);
}



/**
 * @fn mesure_carac
 * @brief Measure all the data of the working point
 * 
 * @note It is launched when the all the voltage is settled. We activate all the ADCs
*/
static void mesure_carac(void) {
	//On arrête le décompte du temps
	HAL_TIM_Base_Stop_IT(&htim2);
	//On reset la valeur du timer pour bien être précis
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	
	//On lance directement la conversion (normalement, on lance plusieurs ADC pour Vds, Vgs et Ids)
	HAL_ADC_Start_IT(&hadc2);	
}



/**
 * @fn finir_test
 * @brief Ends the current test
 * 
 * @note If the we've done the last test, it sends all the results to the computer. Otherwise, we keep going.
*/
static void finir_test(void) {
	//Si on a fini tous nos tests, on envoie les résultats
	if(id_test == nmbr_tests - 1) {
		envoyer_resultats();
	//Si il reste des tests...
	} else if (id_test < (nmbr_tests - 1)) {
		//On attend éventuellement le refroidissement avec un timer
		//-------------------------
		//Ou en enchaine avec le prochain tests
		init_test(id_test + 1);
	}
}



/**
 * @fn envoyer_resultat
 * @brief Sends the results to the computer
*/
static void envoyer_resultats(void) {
	const uint16_t taille_nombre = 5;
	uint8_t nombre_ecrit[taille_nombre] = "00000";
	
	char message_initial[] = "Les tests on été terminés. Les résultats vont être envoyés \n ID_test | Temperature | Vds | Carac | \n";
	char separateur[] = "\n";
	
	HAL_UART_Transmit(&huart2, (uint8_t *)message_initial, sizeof(message_initial), 0xFFFF);
	
	for(int i=0; i<nmbr_tests; i++) {
		//ID
		IntToChar(i, nombre_ecrit, taille_nombre);
		HAL_UART_Transmit(&huart2, (uint8_t *)nombre_ecrit, sizeof(nombre_ecrit), 0xFFFF);
		
		HAL_UART_Transmit(&huart2, (uint8_t *)" , ", 3, 0xFFFF);
		
		//Temperature
		IntToChar(res_mesure_temp[i], nombre_ecrit, taille_nombre);
		HAL_UART_Transmit(&huart2, (uint8_t *)nombre_ecrit, sizeof(nombre_ecrit), 0xFFFF);
		
		HAL_UART_Transmit(&huart2, (uint8_t *)" , ", 3, 0xFFFF);
		
		//Vds consigne
		IntToChar(vds_carac[i], nombre_ecrit, taille_nombre);
		HAL_UART_Transmit(&huart2, (uint8_t *)nombre_ecrit, sizeof(nombre_ecrit), 0xFFFF);
		
		HAL_UART_Transmit(&huart2, (uint8_t *)" , ", 3, 0xFFFF);
		
		//Mesure carac
		IntToChar(res_mesure_carac[i], nombre_ecrit, taille_nombre);
		HAL_UART_Transmit(&huart2, (uint8_t *)nombre_ecrit, sizeof(nombre_ecrit), 0xFFFF);
			
		HAL_UART_Transmit(&huart2, (uint8_t *)separateur, sizeof(separateur), 0xFFFF);
	}
}


/**
 * @fn HAL_TIM_PeriodElapsedCallback
 * @brief We overload the function that handle timer's overload
 * 
 * @param htim The time that launched the IT
 *
 * @note It is used to count the number of ellapsed µseconds. We then launch the corrects functions..
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
	//On vérifie que l'interruption veinne bien du TIMER2
	if(htim->Instance == htim2.Instance) {
		
		//On incrémente le nombre de µs écoulees
		temps_ecoule++;
		
		if(temps_ecoule == temps_impulsion[id_test]) {
			//Quand l'impulsion est terminée
			phase_mesure_temp();
			
		} else if(temps_ecoule == temps_impulsion[id_test] + temps_temp) {
			//Quand le tension de la diode est établie
			mesure_temp();
			
		} else if(temps_ecoule == temps_impulsion[id_test] + temps_temp + temps_carac[id_test]) {
			//Quand le pnt de fonctionnement est établi
			mesure_carac();
		}
		
	}
}




/**
 * @fn HAL_ADC_ConvCpltCallback
 * @brief We overload the function that ADC conersions
 * 
 * @param hadc The adc that launched the IT
 *
 * @note It is to launch the function following the different emasures
*/
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc) {
	
	if(hadc->Instance == hadc1.Instance) {
		//On récupère la veleur mesurée par l'ADC
		res_mesure_temp[id_test] = HAL_ADC_GetValue(&hadc1);
		
		//char msg1[] = "Mesure temperatur"; 
		//HAL_UART_Transmit(&huart2, (uint8_t *)msg1, sizeof(msg1), 0xFFFF);
		
		
		//On lance la phase de caractérisation
		phase_carac();
		
	} else if (hadc->Instance == hadc2.Instance) {
		//On récupère la valeur mesurée par l'ADC
		res_mesure_carac[id_test] = HAL_ADC_GetValue(&hadc2);
		
		//char msg1[] = "Mesure pnt carac"; 
		//HAL_UART_Transmit(&huart2, (uint8_t *)msg1, sizeof(msg1), 0xFFFF);
		
		//On remet à 0 le point de fonctionnement de la carac (normalement on donnera 0 en consigne aux DACs)
		HAL_GPIO_WritePin(img_carac_GPIO_Port, img_carac_Pin, GPIO_PIN_RESET);

		//On a terminé, on pourra pk pas passer au prochain point de test
		finir_test();
	}
	
	
}








/* USER CODE END 4 */







/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
