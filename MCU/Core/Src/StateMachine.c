/*
 * StateMachine.c
 *
 *  Created on: Jun 29, 2025
 *      Author: Enzo Perrier
 */

#include "main.h"
#include "cmsis_os.h"

#include "StateMachine.h"
#include "uart.h"

#include <string.h>
#include <stdio.h>

#define MAX_PER_LENGTH 8


uint8_t state = 0;
char* per_value;

void StateMachineTask(void){
	static uint8_t action_done = 0;

	    //--------------------------- TRANSITIONS
	    switch (state) {
	        case 0:
	        case 2:
	        case 5:
	        case 6:
	        case 7:
	            if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin)) {
	                state++;
	                action_done = 0;
	            } else if (!HAL_GPIO_ReadPin(BP3_GPIO_Port, BP3_Pin) && state > 0) {
	                state--;
	                action_done = 0;
	            }
	            break;
	        case 1:
	            if (message_complete3) {
	                message_complete3 = 0;
	                if (strlen((char*)rx_buffer3) == MAX_PER_LENGTH) {
	                    strncpy(per_value, (char*)rx_buffer3, MAX_PER_LENGTH);
	                    per_value[MAX_PER_LENGTH+1] = '\0';

	                    char cmd[20];
	                    sprintf(cmd, "PER=%s\r", per_value);
	                    send_UART1(cmd);
	                    send_UART3("PER envoyé. Attente confirmation…\n");
	                } else {
	                    send_UART3("Format invalide. Le PER doit faire 8 digits, recommencez…\n");
	                }
	            }

	            if (message_complete1) {
	                message_complete1 = 0;

	                char expected[20];
	                sprintf(expected, "PER=%s", per_value);

	                if (strncmp((char*)rx_buffer1, expected, strlen(expected)) == 0) {
	                    send_UART3("PER VALIDE --> Étape suivante\n");
	                    state++;
	                    action_done = 0;
	                } else {
	                    send_UART3("Valeur différente. Entrez à nouveau le PER :\n");
	                }

	                rx_buffer1[0] = '\0'; // vide le buffer après réponse
	            }
	            break;

	        case 3:
	            if (message_complete1) {
	                message_complete1 = 0;
	                TrameDataSTS data = {0};
	                //parse_data_STS(rx_buffer1, &data);
	                if ((data.acc >= 8.5 && data.acc <= 10) && (data.bat >= 11.5 && data.bat <= 13) && (data.dips[0] == 1 && data.dips[1] == 1 && data.dips[2] == 1 && data.dips[3] == 1 && data.dips[4] == 1 && data.dips[5] == 1 && data.dips[6] == 1 && data.dips[7] == 1)) {
	                    send_UART3("STS OK --> Etape suivante\n");
	                    HAL_Delay(500);
	                    state++;
	                }
	            }
	            break;
	        case 4:
	            if (message_complete1) {
	                message_complete1 = 0;
	                TrameDataSTS data = {0};
	                //parse_data_STS(rx_buffer1, &data);
	                if (data.inps[0] == 1 && data.inps[1] == 1 && data.inps[2] == 1) {
	                    send_UART3("Entrees OK --> Etape suivante\n");
	                    osDelay(500);
	                    state++;
	                    action_done = 0;
	                } else {
	                    send_UART3("Erreur défaut entrée:");
	                    for (int i = 0; i < 3; i++) {
	                        char msg[50];
	                        sprintf(msg, "Entree num %d : %d\n", i, data.inps[i]); // On affiche l'état des entrées
	                        send_UART3(msg);
	                    }
	                }
	            }
	            break;
	        case 8:
	            if (HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) == GPIO_PIN_RESET) {
	                state = 0;
	                action_done = 0;
	            }
	            break;
	    }

	    //--------------------------- ACTIONS
	    switch (state) {
	        case 0:
	        	HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_RESET);
	            if (!action_done) {
	            	send_UART3("ETAPE 0\n");
	            	osDelay(10);
	                send_UART3("Appuyer sur le bouton pour commencer\n");
	                action_done = 1;
	            }
	            break;
	        case 1:
	        if (!action_done) {
	        send_UART3("Entrez le PER (juste la valeur sur 8 digits)\n");
	        if (message_complete3) {
	        message_complete3 = 0;
	        if (strlen((char *)rx_buffer3) == MAX_PER_LENGTH) {
	        strcpy(per_value, (char *)rx_buffer3);
	        char per_command[20];
	        sprintf(per_command, "PER=%s\r", per_value);
	        send_UART1(per_command);
	        osDelay(500);
	        } else {
	        send_UART3("Format invalide. Le PER est sur 8 digits, recommencez...\n");
	        }
	        }
	        action_done = 1;
	        }
	        break;

	        case 2:
	            if (!action_done) {
	            	send_UART3("ETAPE 2\n");
	                send_UART3("Mettez tous les DIPs sur ON, une fois fait appuyez sur le bouton\n");
	                action_done = 1;
	            }
	            break;
	        case 3:
	            if (!action_done) {
	            	send_UART3("ETAPE 3\n");
	                send_UART3("Test STS en cours...\n");
	                send_UART1("STS\n");
	                //send_UART3(rx_buffer1);
	                action_done = 1;
	            }
	            break;
	        case 4:
	            if (!action_done) {
	            	send_UART3("ETAPE 4\n");
	                send_UART3("Test entrees en cours...\n");
	                // Activation de toutes les entrées
	                HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
	                HAL_Delay(300);
	                send_UART1("STS\r");
	                action_done = 1;
	            }
	            break;
	        case 5:
	            send_UART3("Test du décompteur...\n Veuillez valider en appuyant sur le BP si toutes les leds s'allument correctement et dans le bon ordre sur le décompteur");
	            send_UART1("TST=1\r");
	            break;
	        case 6:
	            send_UART1("TST=0\r");
	            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
	            send_UART3("Test des ampoules ...\n Vérifiez que les ampoules s'éteignent et se rallument et que le défaut sur l'écran LCD de la carte corresponde bien à la bonne ampoule");
	            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
	            HAL_Delay(1500);
	            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
	            HAL_Delay(1500);
	            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
	            HAL_Delay(1500);
	            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
	            break;
	        case 7:
	            send_UART3("Test de l'infrarouge...\n Veuillez valider en appuyant sur le BP si la télécommande fonctionne en émission et réception");
	            break;
	        case 8:
	            HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_SET);
	            send_UART3("Test de l'accu...\n Veuillez vérifier que vous avez bien le message suppression batterie qui s'affiche à l'écran, si le cas validez");
	            break;
	        default:
	            break;
	    }

}
