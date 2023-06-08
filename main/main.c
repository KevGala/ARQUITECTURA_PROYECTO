/*****************************************************
 * FileName:        main.c
 * Dependencies:	freertos/FreeRTOS.h, freertos/task.h, GPIO2023.h, PROYECTO.h y esp_task_wdt.h
 * Processor:       ESP32
 * Board:           ESP32
 * Program version: ECLIPSE IDE
 * Company:         TecNM /IT Chihuahua
 * Description:     Archivo principal de aplicación del proyecto.
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
/************************************************
  HEADER FILES
 ************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GPIO2023.h"
#include "PROYECTO.h"

#include <esp_task_wdt.h>

//#define controlTemperatura	// Si se eliminan comentarios se desactiva el control de temperatura.

void app_main(void)
{
	esp_task_wdt_deinit(); // Desactiva watchdotimer.

	GPIO_INIT_PROYECT();	// Inicializamos entras y salidad del proyecto.

	xTaskCreate(checarEntradas, "check_in", 2048, NULL, 10, NULL);  // Crea el hilo. Hilo para el control de algunas salidad.

	mostrarMensaje();	// Mostramos sistema apagado y puerta cerrada.

	while(1){
		Encendido();			// Para el control de encendido.

		ADC_READ();				// Lectura de los ADC. (para los sensores de temperatua).
		checarSensorEntrada();  // Checamos si hay alguien que quiera entrar.

		#ifndef controlTemperatura
		controlDeTemperatura(); // Hacemos el contro de la temperatura.
		#endif

		vTaskDelay(300 / portTICK_PERIOD_MS); // Delay de 300 milisegundos.
	}
}










