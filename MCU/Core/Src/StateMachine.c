/*
 * StateMachine.c
 *
 *  Created on: Jun 29, 2025
 *      Author: Enzo Perrier
 */
//---------------------------------------------------------------- Include
#include "main.h"
#include "cmsis_os.h"

#include "StateMachine.h"
#include "uart.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
//---------------------------------------------------------------- Define

#define MAX_PER_LENGTH 8

//---------------------------------------------------------------- Variables

uint8_t state = 0;
char per_value[MAX_PER_LENGTH + 1];

//---------------------------------------------------------------- typedef

typedef struct
{ // Structure pour gérer la trame STS
    char ver[32];
    char crc[32];
    char lan[16];
    float acc;
    float bat;
    float cel_val;
    char cel_mode;
    char lum;
    bool dips[8]; // true = ON
    bool inps[3]; // true = ON
} TrameDataSTS;

//---------------------------------------------------------------- Prototypes fonctions
void parse_data_STS(char *buffer, TrameDataSTS *data);

void StateMachineTask(void)
{
    static bool action_done = 0;
    static bool bp_pressed = 0, bp3_pressed = 0, bp4_pressed = 0;



    //--------------------------- Vérification de l'appui sur BP3 pour revenir à l'étape précédente
    if (!HAL_GPIO_ReadPin(BP3_GPIO_Port, BP3_Pin) && state > 0 && !bp3_pressed)
        {
            // Si BP3 est pressé et qu'on n'est pas déjà revenu à l'étape précédente
            state--;  // On revient à l'étape précédente
            action_done = 0;
            bp3_pressed = 1;
            osDelay(250);
        }
    else
    {
    	bp3_pressed = 0;
    }
    //--------------------------- Vérification de l'appui sur BP4 pour reset (étape 0)
    if (!HAL_GPIO_ReadPin(BP4_GPIO_Port, BP4_Pin) && !bp4_pressed)
            {
                // Si BP3 est pressé et qu'on n'est pas déjà revenu à l'étape précédente
                state = 0;  // On revient à l'étape 0
                action_done = 0;
                bp4_pressed = 1;
                osDelay(250);
            }
    else
    {
    	bp4_pressed =  0;
    }


    //--------------------------- TRANSITIONS
    switch (state)
    {
    case 0:
    case 7:
    case 8:
    case 11:
    case 12:
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
        	if(state < 12){
        		state++;
        	}
        	else{
        		state = 0;
        	}

            action_done = 0;
        }
        break;
    case 1:
        Check_UART1_Timeout(); // Permer de vérifier si on a fini de recevoir le message et retoure message_complete1

        if (message_complete1)
        {
            message_complete1 = 0;

            char expected[20];
            sprintf(expected, "PER = %s", per_value);

            if (strstr((char *)rx_buffer1, expected) != NULL)
            {
                send_UART3("PER VALIDE --> Etape suivante\r\n");
                // memset(rx_buffer1, 0, RX_BUFFER1_SIZE ); // On reset notre buffer par sécurité
                state++;
                action_done = 0;
            }
            else
            {
                send_UART3("Valeur differente. Entrez a nouveau le PER :\r\n");
            }
        }
        break;

    case 2: // Check STS
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            bool acc_ok = false;
            bool bat_ok = false;

            send_UART3((char *)rx_buffer1); // On a besoin de traiter la trame dans le logiciel de test

            // Vérification ACC
            if (data.acc >= 8.0 && data.acc <= 10.0)
            {
                //send_UART3("Accu OK\n"); // Utile pour test via terminal
                acc_ok = true;
            }
            else
            {
                send_UART3("Accu NOK : Valeur hors plage !!\r\n");
            }

            // Vérification BAT
            if (data.bat >= 11.0 && data.bat <= 14.0)
            {
                //send_UART3("Tension batterie OK\r\n"); // Utile pour test via
                bat_ok = true;
            }
            else
            {
                send_UART3("Tension batterie NOK : Valeur hors plage !!\r\n");
            }

            // Transition si tout est bon
            if (acc_ok &&  bat_ok)
            {
                send_UART3("STS OK --> Etape suivante\r\n");
                osDelay(500);
                action_done = 0;
                state++;
            }
            else
            {
                send_UART3("STS invalide. Corrigez les erreurs ci-dessus et recommencez le  test\r\n");
            }
        }
        break;

    case 3: // DIPS OFF
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            bool dips_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Vérification DIP switches
            for (int i = 0; i < 8; i++)
            {
                if (data.dips[i])
                {
                    dips_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: DIP %d a ON !\r\n", i + 1); // On affiche l'état des DIPs en défaut
                    send_UART3(msg);
                }
            }

            if (dips_ok)
            {
                send_UART3("DIP a OFF --> OK\r\n");
                osDelay(500);
                state++;
                bp_pressed = 0;
                action_done = 0;
            }
            else
            {
                send_UART3("DIP NOK : Tous les DIPs ne sont pas a OFF, mettez les a OFF et rappuyez sur le bouton\r\n");
                bp_pressed = 0;
            }
        }
        break;

    case 4: // DIPS ON
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            bool dips_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Vérification DIP switches
            for (int i = 0; i < 8; i++)
            {
                if (!data.dips[i])
                {
                    dips_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: DIP %d a OFF !\r\n", i + 1); // On affiche l'état des DIPs en défaut
                    send_UART3(msg);
                }
            }

            if (dips_ok)
            {
                send_UART3("DIP a ON --> OK\r\n");
                osDelay(500);
                state++;
                action_done = 0;
                bp_pressed = 0;
            }
            else
            {
                send_UART3("DIP NOK : Tous les DIPs ne sont pas a ON, mettez les a ON et rappuyez sur le bouton\r\n");
                bp_pressed = 0;
            }
        }
        break;

    case 5: // Entrées à OFF
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;
            bool inps_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Verification entrées
            for (int i = 0; i < 3; i++)
            {
                if (data.inps[i])
                {
                    inps_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: Entree %d a ON!\r\n", i + 1); // On affiche l'état des entrées
                    send_UART3(msg);
                }
            }

            if (inps_ok)
            {
                send_UART3("Entrees a OFF --> OK\r\n");
                osDelay(500);
                state++;
                action_done = 0;
            }
        }
        break;

    case 6: // Test entrées à ON
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            bool inps_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Verification entrées
            for (int i = 0; i < 3; i++)
            {
                if (!data.inps[i])
                {
                    inps_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: Entree %d a OFF!\r\n", i + 1); // On affiche l'état des entrées
                    send_UART3(msg);
                }
            }

            if (inps_ok)
            {
                send_UART3("Entrees a ON --> OK\r\n");
                osDelay(500);
                state++;
                action_done = 0;
            }
        }
        break;

    case 9: // Cellule JOUR
    	if(!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin)){
    	Check_UART1_Timeout();
    	        if (message_complete1)
    	        {
    	            message_complete1 = 0;

    	            TrameDataSTS data = {0};
    	            parse_data_STS((char *)rx_buffer1, &data);

    	            if(data.cel_mode == 'J'){
    	            	send_UART3("Cellule JOUR --> OK\r\n");
    	            	osDelay(500);
    	            	state++;
    	            	action_done = 0;
    	            }else{
    	            	send_UART3("Défaut cellule en NUIT, veuillez réessayer\r\n");
    	            	action_done = 0;
    	            }
    	        }
    	}
    	break;

    case 10: //Cellule NUIT
    	if(!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin)){
        	Check_UART1_Timeout();
        	        if (message_complete1)
        	        {
        	            message_complete1 = 0;

        	            TrameDataSTS data = {0};
        	            parse_data_STS((char *)rx_buffer1, &data);

        	            if(data.cel_mode == 'N'){
        	            	send_UART3("Cellule NUIT --> OK\r\n");
        	                osDelay(500);
        	            	state++;
        	            	action_done = 0;
        	            }else{
        	            	send_UART3("Défaut cellule en JOUR, veuillez réessayer\r\n");
        	            	action_done = 0;
        	            }
        	        }
    	}
      break;
    }



    //--------------------------- ACTIONS
    switch (state)
    {
    case 0:
    	if (!action_done)
    	{
    		//On met bien les I/0 par défaut
    	HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);


            send_UART3("---- ETAPE 0 ----\r\n");
            osDelay(10);
            send_UART3("Appuyer sur le bouton pour commencer\r\n");
            action_done = 1;
        }
        break;
    case 1:
        if (!action_done)
        {
        	send_UART3("---- ETAPE 1 ----\n");
            send_UART3("Entrez le PER (juste la valeur sur 8 digits)\r\n");
            action_done = 1;
        }

        if (message_complete3)
        {
            message_complete3 = 0;
            // Nettoyer la chaîne des caractères \r et \n
            char cleaned_buffer[20];
            int i = 0;
            // Copier les caractères sauf \r et \n
            for (int j = 0; j < strlen((char *)rx_buffer3); j++)
            {
                if (rx_buffer3[j] != '\r' && rx_buffer3[j] != '\n')
                {
                    cleaned_buffer[i++] = rx_buffer3[j];
                }
            }
            cleaned_buffer[i] = '\0'; // Terminer la chaîne propre

            // Vérifier la longueur après nettoyage
            if (strlen(cleaned_buffer) == MAX_PER_LENGTH)
            {
                strncpy(per_value, cleaned_buffer, MAX_PER_LENGTH);

                char cmd[30];
                sprintf(cmd, "PER=%s\r", per_value);
                send_UART1(cmd);
                osDelay(10);
                send_UART1(cmd);
                send_UART3("PER envoye. Attente confirmation…\r\n");
                osDelay(1000);
                send_UART1("PER\r");
                osDelay(50);
            }
            else
            {
                send_UART3("Format invalide. Le PER doit faire 8 digits, recommencez…\r\n");
            }
        }
        break;

    case 2:
        if (!action_done)
        {
        	send_UART3("---- ETAPE 2 ----\r\n");
            send_UART3("Test STS en cours ...\r\n");
            osDelay(500);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;
    case 3:
        if (!action_done)
        {
        	send_UART3("---- ETAPE 3 ----\r\n");
            send_UART3("Mettez les DIPs a OFF et appuyez sur le bouton\r\n");
            osDelay(500);
            action_done = 1;
        }
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) && !bp_pressed)
        {
            bp_pressed = 1;
            send_UART1("STS\r");
        }

        break;

    case 4:
        if (!action_done)
        {
        	send_UART3("---- ETAPE 4 ----\r\n");
            send_UART3("Mettez les DIPs a ON, appuyez sur le BP reset et appuyez sur le bouton\r\n");
            osDelay(500);
            action_done = 1;
        }
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) && !bp_pressed)
        {
            bp_pressed = 1;
            send_UART1("STS\r");
        }

        break;
    case 5: // Entrées à OFF
        if (!action_done)
        {
        	send_UART3("---- ETAPE 5 ----\r\n");
        	osDelay(250);
            send_UART3("Test entrees à OFF en cours...\r\n");
            // Activation de toutes les entrées
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
            osDelay(1000);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;

    case 6: // Entrées à ON
        if (!action_done)
        {
        	send_UART3("---- ETAPE 6 ----\r\n");
            send_UART3("Test entrees à ON en cours...\r\n");
            osDelay(1000);
            // Activation de toutes les entrées
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
            osDelay(500);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;
    case 7:
    	if (!action_done)
    	        {
    		        // On réinitialise les entrées précédemment sur ON
    		    	HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
    		    	HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
    		    	HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
    		    	send_UART3("---- ETAPE 7 ----\n");
    		        send_UART3("Test du décompteur...\n Veuillez valider en appuyant sur le BP si toutes les leds s'allument correctement et dans le bon ordre sur le décompteur\n\r");
    		        send_UART1("TST=1\r");
    		        action_done = 1;
    	        }
    	        break;
    case 8: // Défauts ampoules
    	if (!action_done)
    	{
    		send_UART1("TST=0\r"); // On arrête le test décompteur
    		send_UART3("---- ETAPE 8 ----\r\n");
    		send_UART3("Test des ampoules ...\n\r Verifiez que les ampoules s'éteignent et se rallument et que le défaut sur l'écran LCD de la carte corresponde bien a la bonne optique\n\rEnsuite appuyez sur le bouton pour valider\r\n");
    		osDelay(2500);
    		HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
    		osDelay(2500);
    		HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
    		osDelay(2500);
    		HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
    		osDelay(2500);
    		HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
    		action_done = 1;
    	}
    	break;
    case 9: // Test cellule Jour
    	// on remet les relais à l'état initial
    	if(!action_done){
    	osDelay(1000);
    	HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
    	send_UART3("---- ETAPE 9 ----\r\n");
    	send_UART3("Test cellule JOUR, veuillez exposer la cellule a la lumière\r\nappuyez sur le bouton pour tester\r\n");
    	send_UART1("STS\r");
    	action_done = 1;
    	}
    	 break;

    case 10: //Test cellule Nuit
    	if(!action_done){
    	osDelay(1000);
    	send_UART3("---- ETAPE 10 ----\r\n");
    	send_UART3("Test cellule NUIT, veuillez cacher la cellule et reset la cart\r\nappuyez sur le bouton pour tester\n\r");
    	send_UART1("STS\r");
    	action_done=1;
    	}
    	break;


    case 11:// Test IR
    	if(!action_done){
    	osDelay(1000);
    	send_UART3("---- ETAPE 11 ----\r\n");
    	send_UART3("Test de l'infrarouge...\n Veuillez valider en appuyant sur le BP si la telecommande fonctionne en emission et reception");
    	action_done = 1;
    	}
        break;
    case 12:
    	if(!action_done){
    	osDelay(1000);
    	send_UART3("---- ETAPE 12 ----\r\n");
        HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_SET);
        send_UART3("Test de l'accu...\n Veuillez verifier que vous avez bien le message suppression batterie qui s'affiche à l'écran, si le cas validez");
    	action_done=1;
    	}
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------- Fonctions outils
void parse_data_STS(char *buffer, TrameDataSTS *data)

{

    char *line = strtok(buffer, "\r\n");



    bool cel_val_parsed = false;

    bool cel_mode_parsed = false;



    while (line != NULL)

    {

        if (strncmp(line, "VER =", 5) == 0)

        {

            sscanf(line, "VER = %31[^\r\n]", data->ver);

        }

        else if (strncmp(line, "CRC =", 5) == 0)

        {

            sscanf(line, "CRC = %63[^\r\n]", data->crc);

        }

        else if (strncmp(line, "LAN =", 5) == 0)

        {

            sscanf(line, "LAN = %15[^\r\n]", data->lan);

        }

        else if (strncmp(line, "ACC =", 5) == 0)

        {

            sscanf(line, "ACC = %f", &data->acc);

        }

        else if (strncmp(line, "BAT =", 5) == 0)

        {

            sscanf(line, "BAT = %f", &data->bat);

        }

        else if (strncmp(line, "CEL =", 5) == 0)

        {

            // Essaye de parser float si pas encore fait

            if (!cel_val_parsed && sscanf(line, "CEL = %f", &data->cel_val) == 1)

            {

                cel_val_parsed = true;

            }

            // Sinon essaye de parser mode char, indépendamment

            else if (!cel_mode_parsed && sscanf(line, "CEL = %c", &data->cel_mode) == 1)

            {

                cel_mode_parsed = true;

            }

        }

        else if (strncmp(line, "LUM =", 5) == 0)

        {

            sscanf(line, "LUM = %c", &data->lum);

        }

        else if (strncmp(line, "DIP =", 5) == 0)

        {

            char *p = line + 5;

            int num;

            char state[4];



            while (sscanf(p, " %d:%3s", &num, state) == 2)

            {

                if (num >= 1 && num <= 8)

                    data->dips[num - 1] = (strcmp(state, "ON") == 0);



                char *next = strchr(p, ' ');

                if (!next) break;

                p = next + 1;

            }

        }

        else if (strncmp(line, "INP =", 5) == 0)

        {

            char *p = line + 5;

            int num;

            char state[4];



            while (sscanf(p, " %d:%3s", &num, state) == 2)

            {

                if (num >= 1 && num <= 3)

                    data->inps[num - 1] = (strcmp(state, "ON") == 0);



                char *next = strchr(p, ' ');

                if (!next) break;

                p = next + 1;

            }

        }



        line = strtok(NULL, "\r\n");

    }

}
