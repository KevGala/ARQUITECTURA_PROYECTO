/*****************************************************
 * FileName:        PROYECTO.c
 * Dependencies:	GPIO2023.h, uart.h, esp_log.h, adc_oneshot.h, adc_cali_scheme.h, string.h
 * Processor:       ESP32
 * Board:           ESP32
 * Program version: ECLIPSE IDE
 * Company:         TecNM /IT Chihuahua
 * Description:     Incluye librerías y define funciones.
 * Authors:         Kevin Galaviz, Wendy Marquez, Mario Marcial y Raul Rodirguez
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

#ifndef MAIN_PROYECTO_H_
#define MAIN_PROYECTO_H_
#pragma once

/*---------------------------------------------------------------
          DEPENDENCIAS
---------------------------------------------------------------*/
#include "GPIO2023.h"
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
/*---------------------------------------------------------------
    PROTOTIPO DE FUNCIONES
---------------------------------------------------------------*/

/* Función de alarma. */
extern void ALARMA(void);
/* Función que inicializa del proyecto. */
extern void GPIO_INIT_PROYECT(void);
/* Función inicialización del ADC. */
extern void ADC_INIT(void);
/* Función inicialización del UART. */
extern void UART_INIT(void);
/* Función leer el ADC. */
extern void ADC_READ(void);
/* Función el inicio. */
extern void Encendido(void);
/* Funición para mostrar mensaje del estado sistema. */
extern void mostrarMensaje(void);
/* Funición de un hilo para leer los botones y sensores. */
extern void checarEntradas(void *arg);
/* Funición para checar el estado del sensor S_IN. */
extern void checarSensorEntrada(void);
/* Funición para hacer el control del abanico. */
extern void controlDeTemperatura(void);


#endif /* MAIN_PROYECTO_H_ */
