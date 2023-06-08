/*****************************************************
 * FileName:        PROYECTO_IO.c
 * Dependencies:	PROYECTO.h, string.h, stdlib.c
 * Processor:       ESP32
 * Board:           ESP32
 * Program version: ECLIPSE IDE
 * Company:         TecNM /IT Chihuahua
 * Description:     Funciones de control de HW.
 * Authors:         Kevin Galaviz, Wendy Marquez, Mario Marcial y Raul Rodriguez.
 * Nota:
 *  Created on: Junio 2023
 **************************************************/
/************************************************************************************************
 * * Copyright (C) 2023 by Kevin Galaviz - TecNM /IT Chihuahua
 *
 * Se permite la redistribucion, modificacion o uso de este software en formato fuente o binario
 * siempre que los archivos mantengan estos derechos de autor.
 * Los usuarios pueden modificar esto y usarlo para aprender sobre el campo de software embebido.
 * Alfredo Chacon y el TecNM /IT Chihuahua no son responsables del mal uso de este material.
 *************************************************************************************************/

/*---------------------------------------------------------------
          DEPENDENCIAS
---------------------------------------------------------------*/
#include "PROYECTO.h"
/*---------------------------------------------------------------
          VARIABLES Y DEFINICION ES PARA EL PROYECTO
---------------------------------------------------------------*/
#define HIGH 1 // Alto
#define LOW  0 // Bajo
static uint8_t s_led_state = 0;
// Elementos del proyecto
#define S_IN  4 		// Sensores S_IN entrada y S_OUT de salida para el conteo de las personas que entran y salen.
#define S_OUT 15
//ADC2 Canales
#define S_TEMCOR  ADC_CHANNEL_4	    // Sensor de temperatura corporal TEMCOR para el acceso. (gpio13)
#define S_TEMPAMB ADC_CHANNEL_5	    // Sensor de temperatura ambiental TEMPAMB.			     (gpio12)
#define DOOR       16   // Actuador o cerradura eléctrica para permitir acceso DOOR.
#define FAN        5	// Abanicos FAN.
#define LED_ON_OFF 17   // LED indica el estado del sistema.
#define LED_ROJO   18     // LED alarma.
#define LED_AZUL   19     // LED alarma.
#define BTN_ON_OFF 21	// Boton para el encendido y apagado del sistema.
#define BTN_MODO   22   // Boton para el modo.
#define BTN_COOL   23	// Boton cool/heat.

volatile bool sistemaEstado=0;	// Estado del sistema.
bool DoorEstado=0;	    		// Estado de la puerta.
bool MODO=0;	    		    // Estado cool/heat
bool Cool_Heat=0;	    		// MODO 0 = AUTO; MODO 1 = ON

float TEMCOR=0;					// Variable de temperatura corporal.
float TEMPAMB=0;				// Variable de temperatura ambiental.

uint32_t cuenta=0;		// Cuenta de las personas dentro.
#define espacioMax  5   // Limite de personas para el establecimiento.
#define tempMaxCorp 20  // Variable para el set point de la temperatura corporal.
#define set_point   20  // Variable para el set point de la temperatura ambiental.
/*---------------------------------------------------------------
          UART CONFIGURACIÓN
---------------------------------------------------------------*/
#define ECHO_TEST_TXD 1
#define ECHO_TEST_RXD 3
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      0
#define ECHO_UART_BAUD_RATE     115200

#define BUF_SIZE (1024)
char state[BUF_SIZE*2];      // Cadena a imprimir.
/*---------------------------------------------------------------
          ADC CONFIGURACIÓN
---------------------------------------------------------------*/
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11 // ATENUACIÓN
adc_oneshot_unit_handle_t adc2_handle;
/* Función calibracion del ADC. */
static bool ADC_CALIBRATION_INIT(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
/*****************************************************************************
 * Function: ALARMA
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Parpadeo de las luces de la alrma.
 * Input: None.
 * Output: None.
 *
 *****************************************************************************/
void ALARMA(void){
	for(int i=0; i<10; i++){
		GPIO_SET_LEVEL(LED_AZUL,s_led_state);
		s_led_state = ~s_led_state;
		GPIO_SET_LEVEL(LED_ROJO,s_led_state);
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
	// Apagamos las luces.
	GPIO_SET_LEVEL(LED_AZUL,0);
	GPIO_SET_LEVEL(LED_ROJO,0);
}
/*****************************************************************************
 * Function: GPIO_INIT_PROYECT
 * Preconditions: None.
 * Overview: Inicialización de todos los componentes del proyecto
 * Input: None.
 * Output: None.
 *
 *****************************************************************************/
void GPIO_INIT_PROYECT(void)
{
	ADC_INIT();				// Inicializamos el ADC.
	UART_INIT();			// Inicializamos el UART.
	/* Configuración de los actuadores.*/
	GPIO_CONFIG(FAN, GPIO_OUT);
	GPIO_SET_LEVEL(FAN,HIGH); // Apaga el abanico.
	GPIO_CONFIG(DOOR, GPIO_OUT);
	GPIO_SET_LEVEL(DOOR,LOW); // Puerta cerrada
	/* Configuración LED encendido.*/
	GPIO_CONFIG(LED_ON_OFF, GPIO_OUT);
	GPIO_SET_LEVEL(LED_ON_OFF, LOW);
	/* Secuencia de luces. */
	GPIO_CONFIG(LED_ROJO, GPIO_OUT);
	GPIO_SET_LEVEL(LED_ROJO, LOW);
	GPIO_CONFIG(LED_AZUL, GPIO_OUT);
	GPIO_SET_LEVEL(LED_AZUL, LOW);

	/* Configuración de los botones.*/
	GPIO_CONFIG(BTN_ON_OFF, GPIO_IN);
	GPIO_CONFIG(BTN_COOL, GPIO_IN);
	GPIO_CONFIG(BTN_MODO, GPIO_IN);
	/* Configuración de los sensores.*/
	GPIO_CONFIG(S_IN, GPIO_IN);
	GPIO_CONFIG(S_OUT, GPIO_IN);
}
/*****************************************************************************
 * Function: ADC_CALIBRATION_INIT
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para calibrar el ADC.
 * Input: EL ADC que se quiere, factor de atenuacion y estructura de calibración.
 * Output: bool.
 *
 *****************************************************************************/
static bool ADC_CALIBRATION_INIT(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    const static char *TAG = "EXAMPLE";

    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
/*****************************************************************************
 * Function: ADC_INIT
 * Preconditions: Ninguna.
 * Overview: Función para inicializar el ADC.
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void ADC_INIT(void)
{
	/*---------------------------------------------------------------
			ADC2
	---------------------------------------------------------------*/
    //-------------ADC2 Config CHANELS---------------//
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = EXAMPLE_ADC_ATTEN,
	};
	//-------------ADC2 Init---------------//
	adc_oneshot_unit_init_cfg_t init_config2 = {
		.unit_id = ADC_UNIT_2,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));
    //-------------ADC2 Calibration Init---------------//
	adc_cali_handle_t adc2_cali_handle = NULL;
	ADC_CALIBRATION_INIT(ADC_UNIT_2, EXAMPLE_ADC_ATTEN, &adc2_cali_handle);
	//-------------ADC2 Config CHANELS**********************************************************---------------//
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, S_TEMCOR, &config));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, S_TEMPAMB, &config));
}
/*****************************************************************************
 * Function: UART_INIT
 * Preconditions: Ninguna.
 * Overview: Función para inicializar el puerto UART.
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void UART_INIT(void)
{
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = ECHO_UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	int intr_alloc_flags = 0;

	ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}
/*****************************************************************************
 * Function: ADC_READ
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para leer los ADC.
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void ADC_READ(void)
{
	int adc_val;

	adc_oneshot_read(adc2_handle, S_TEMCOR, &adc_val);
	TEMCOR = (float)adc_val*330/4095;

	adc_oneshot_read(adc2_handle, S_TEMPAMB, &adc_val);
	TEMPAMB = (float)adc_val*330/4095;
}
/*****************************************************************************
 * Function: Encendido
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para el inicio
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void Encendido(void)
{
    vTaskDelay(10 / portTICK_PERIOD_MS);

	while(!sistemaEstado);
}
/*****************************************************************************
 * Function: checarEntradas
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para detectar el estado de los botones y sensores.
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void checarEntradas(void *arg)
{
    for(;;){

    	if(GPIO_GET_LEVEL(BTN_ON_OFF))
    	{
			while(GPIO_GET_LEVEL(BTN_ON_OFF)); // Antirebote
    		if(sistemaEstado){
    			sistemaEstado=false;	// Actualizamos el estado del sistema.
    			DoorEstado=false;	// Actualizamos el estado la puerta.
    			GPIO_SET_LEVEL(DOOR,0); // Cierra la puerta
    			mostrarMensaje();
    		}
    		else{
    			sistemaEstado=true;
    			mostrarMensaje();
    		}
    		GPIO_SET_LEVEL(LED_ON_OFF, sistemaEstado);
    		// Apagamos las luces.
			GPIO_SET_LEVEL(LED_AZUL,LOW);
			GPIO_SET_LEVEL(LED_ROJO,LOW);
			GPIO_SET_LEVEL(FAN,HIGH); // Apaga el abanico.
			GPIO_SET_LEVEL(DOOR,LOW); // Puerta cerrada
			cuenta=0; // Reinicio de la cuenta.
    	}
    	if(GPIO_GET_LEVEL(S_OUT))
		{
			while(GPIO_GET_LEVEL(S_OUT)); // Antirebote
    		if(sistemaEstado){
    			if(cuenta<=0) cuenta=0; // Para no tener numeros negativos.
    			else cuenta--;
				memset(state, '\0', 2*1024); // Limpiar state
				sprintf(state,"Cuenta: %ld \r\n",cuenta);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,124);
    		}
		}
    	if(GPIO_GET_LEVEL(BTN_MODO))
		{
			while(GPIO_GET_LEVEL(BTN_MODO)); // Antirebote
    		if(sistemaEstado){
				// Cambia de modo.
				if(MODO) MODO = 0;
				else     MODO = 1;

				memset(state, '\0', 2*1024); // Limpiar state
				sprintf(state,"MODO: %s; Clima: %s; TEMPAMB: %f \r\n",
						MODO == 0? "AUTO":"ON",
						Cool_Heat == 0? "COOL":"HEAT",
						TEMPAMB );
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,256);
    		}
		}
		if(GPIO_GET_LEVEL(BTN_COOL))
		{
			while(GPIO_GET_LEVEL(BTN_COOL)); //Antirebote
			if(sistemaEstado){
				// Cambia de configuracion cool/heat.
				if(Cool_Heat) Cool_Heat=0;
				else          Cool_Heat = 1;

				memset(state, '\0', 2*1024); // Limpiar state
				sprintf(state,"MODO: %s; Clima: %s; TEMPAMB: %f \r\n",
						MODO == 0? "AUTO":"ON",
						Cool_Heat == 0? "COOL":"HEAT",
						TEMPAMB	);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,256);
			}
		}
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
/*****************************************************************************
 * Function: mostrarMensaje
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para mostrar el estado del sistema..
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void mostrarMensaje(void)
{
	memset(state, '\0', 2*1024); // Limpiar state
	sprintf(state,"Sistema: %s \r\n",
			sistemaEstado == true? "On":"OFF");
	uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,30);
	GPIO_SET_LEVEL(LED_ON_OFF, sistemaEstado);

	memset(state, '\0', 2*1024); // Limpiar state
	sprintf(state,"DOOR: %s \r\n",
			DoorEstado == true? "open":"closed");
	uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,30);
}
/*****************************************************************************
 * Function: checarSensorEntrada
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para checar el estado del sensor S_IN.
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void checarSensorEntrada(void)
{
	if(GPIO_GET_LEVEL(S_IN))	// Sensor de entrada.
	{
		while(GPIO_GET_LEVEL(S_IN)); // Antirebote
		memset(state, '\0', 2*1024); // Limpiar state

		if(cuenta < espacioMax)
		{
			if(TEMCOR < tempMaxCorp)
			{
				cuenta++; // Actualiza la cuenta.
				sprintf(state,"DOOR: open. Cuenta: %ld \r\n",cuenta);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,100);
				DoorEstado=true; // Actualiza el estado de la puerta.
				GPIO_SET_LEVEL(DOOR,DoorEstado); // Abre la puerta

				vTaskDelay(5000 / portTICK_PERIOD_MS);  // Delay 5 segundos.
				DoorEstado=false; // Actualiza el estado de la puerta.
				GPIO_SET_LEVEL(DOOR,DoorEstado); // Cierra la puerta
				sprintf(state,"DOOR: close. Cuenta: %ld \r\n",cuenta);
			}
			else
			{
				sprintf(state,"Temp_out_of_range. TEMPCOR: %.2f \r\n",TEMCOR);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,100);
				ALARMA();  // Alarma activada.
			}
		}
		else
		{
			sprintf(state,"We are Full, wait. Cuenta: %ld \r\n",cuenta);
			uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)state,100);
		}
	}
    vTaskDelay(50 / portTICK_PERIOD_MS);
}
/*****************************************************************************
 * Function: controlDeTemperatura
 * Preconditions: GPIO_INIT_PROYECT().
 * Overview: Función para hacer el control del abanico.
 * Input: Ninguna.
 * Output: Ninguna.
 *
 *****************************************************************************/
void controlDeTemperatura(void)
{
	/* FAN. */
	GPIO_SET_LEVEL(FAN,1); // Apaga el abanico.

	if(MODO == 0 && Cool_Heat == 0 && TEMPAMB > set_point)       // a. En Modo AUTO y cool, si TEMPAMB está por encima del set point entonces FAN: On, de lo contrario Off
	{
		GPIO_SET_LEVEL(FAN,0); // Enciende el abanico.
	}
	else if(MODO == 0 && Cool_Heat == 1 && TEMPAMB < set_point)  // b. En Modo AUTO y heat, si TEMPAMB está por abajo del set point entonces FAN: On de lo contrario Off
	{
		GPIO_SET_LEVEL(FAN,0); // Enciende el abanico.
	}
	else if(MODO == 1 && Cool_Heat == 0)                         // c. Modo On y cool, entonces FAN: On, de lo contrario Off
	{
		GPIO_SET_LEVEL(FAN,0); // Enciende el abanico.
	}
    vTaskDelay(50 / portTICK_PERIOD_MS);
}


