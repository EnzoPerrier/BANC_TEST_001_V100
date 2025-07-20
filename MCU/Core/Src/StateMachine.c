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
    static bool bp_pressed = 0;

    //--------------------------- TRANSITIONS
    switch (state)
    {
    case 0:
    case 7:
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
            state++;
            action_done = 0;
        }
        else if (!HAL_GPIO_ReadPin(BP3_GPIO_Port, BP3_Pin) && state > 0)
        {
            state--;
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
                send_UART3("PER VALIDE --> Étape suivante\n");
                // memset(rx_buffer1, 0, RX_BUFFER1_SIZE ); // On reset notre buffer par sécurité
                state++;
                action_done = 0;
            }
            else
            {
                send_UART3("Valeur différente. Entrez à nouveau le PER :\n");
            }
        }
        break;

    case 2:
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            bool acc_ok = false;
            bool bat_ok = false;

            // Vérification ACC
            if (data.acc >= 8.5 && data.acc <= 10.0)
            {
                send_UART3("Accu OK\n");
                acc_ok = true;
            }
            else
            {
                send_UART3("Accu NOK : Valeur hors plage !!\n");
            }

            // Vérification BAT
            if (data.bat >= 11.5 && data.bat <= 13.0)
            {
                send_UART3("Tension batterie OK\n");
                bat_ok = true;
            }
            else
            {
                send_UART3("Tension batterie NOK : Valeur hors plage !!\n");
            }

            // Transition si tout est bon
            if (/*acc_ok && */ bat_ok)
            {
                send_UART3("STS OK --> Etape suivante\n");
                osDelay(500);
                action_done = 0;
                state++;
            }
            else
            {
                send_UART3("STS invalide. Corrigez les erreurs ci-dessus.\n");
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
                    sprintf(msg, "ERROR: DIP %d à ON !\n", i + 1); // On affiche l'état des DIPs en défaut
                    send_UART3(msg);
                }
            }

            if (dips_ok)
            {
                send_UART3("DIP switches OK\n");
                osDelay(500);
                state++;
                bp_pressed = 0;
                action_done = 0;
            }
            else
            {
                send_UART3("DIP switches NOK : Tous les DIPs ne sont pas à OFF, mettez les à OFF et rappuyez sur le bouton\n");
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
                    sprintf(msg, "ERROR: DIP %d à OFF !\n", i + 1); // On affiche l'état des DIPs en défaut
                    send_UART3(msg);
                }
            }

            if (dips_ok)
            {
                send_UART3("DIP switches OK\n");
                osDelay(500);
                state++;
                action_done = 0;
                bp_pressed = 0;
            }
            else
            {
                send_UART3("DIP switches NOK : Tous les DIPs ne sont pas à ON, mettez les à ON et rappuyez sur le bouton\n");
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
                    sprintf(msg, "ERROR: Entrée %d à ON!", i + 1); // On affiche l'état des entrées
                    send_UART3(msg);
                }
            }

            if (inps_ok)
            {
                send_UART3("Entrées à OFF --> OK\n");
                osDelay(500);
                state++;
                action_done = 0;
            }
        }
        break;

    case 6:
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;
            bool inps_ok = true;
            send_UART3((char *)rx_buffer1);
            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Verification entrées
            for (int i = 0; i < 3; i++)
            {
                if (!data.inps[i])
                {
                    inps_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: Entrée %d à OFF!", i + 1); // On affiche l'état des entrées
                    send_UART3(msg);
                }
            }

            if (inps_ok)
            {
                send_UART3("Entrées à ON --> OK\n");
                osDelay(500);
                state++;
                action_done = 0;
            }
        }
        break;

    case 8:
        if (HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) == GPIO_PIN_RESET)
        {
            state = 0;
            action_done = 0;
        }
        break;
    }

    //--------------------------- ACTIONS
    switch (state)
    {
    case 0:
        HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_RESET);
        if (!action_done)
        {
            send_UART3("ETAPE 0\n");
            osDelay(10);
            send_UART3("Appuyer sur le bouton pour commencer\n");
            action_done = 1;
        }
        break;
    case 1:
        if (!action_done)
        {
            send_UART3("Entrez le PER (juste la valeur sur 8 digits)\n");
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
                send_UART3("PER envoyé. Attente confirmation…\n");
                osDelay(1000);
                send_UART1("PER\r");
                osDelay(50);
            }
            else
            {
                send_UART3("Format invalide. Le PER doit faire 8 digits, recommencez…\n");
            }
        }
        break;

    case 2:
        if (!action_done)
        {
            send_UART3("ETAPE 2\n");
            send_UART3("Test STS en cours ...\n");
            osDelay(500);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;
    case 3:
        if (!action_done)
        {
            send_UART3("Mettez les DIPs à OFF et appuyez sur le bouton\n");
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
            send_UART3("Mettez les DIPs à ON et appuyez sur le bouton\n");
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
            send_UART3("ETAPE 4\n");
            send_UART3("Test entrees à OFF en cours...\n");
            // Activation de toutes les entrées
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
            osDelay(1000);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;

    case 6: // Entrées à OFF
        if (!action_done)
        {
            send_UART3("Test entrees à ON en cours...\n");
            // Activation de toutes les entrées
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
            osDelay(1000);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;
    case 7:
        send_UART3("Test du décompteur...\n Veuillez valider en appuyant sur le BP si toutes les leds s'allument correctement et dans le bon ordre sur le décompteur");
        send_UART1("TST=1\r");
        break;
    case 8:
        send_UART1("TST=0\r");
        HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
        send_UART3("Test des ampoules ...\n Vérifiez que les ampoules s'éteignent et se rallument et que le défaut sur l'écran LCD de la carte corresponde bien à la bonne ampoule");
        HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
        osDelay(2000);
        HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
        osDelay(2000);
        HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
        osDelay(2000);
        HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
        break;
    case 9:
        send_UART3("Test de l'infrarouge...\n Veuillez valider en appuyant sur le BP si la télécommande fonctionne en émission et réception");
        break;
    case 10:
        HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_SET);
        send_UART3("Test de l'accu...\n Veuillez vérifier que vous avez bien le message suppression batterie qui s'affiche à l'écran, si le cas validez");
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------- Fonctions outils
void parse_data_STS(char *buffer, TrameDataSTS *data)
{
    char *line = strtok(buffer, "\r\n");
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
        else if (strncmp(line, "CEL =", 5) == 0 && strchr(line, 'v'))
        {
            sscanf(line, "CEL = %f", &data->cel_val);
        }
        else if (strncmp(line, "CEL =", 5) == 0 && !strchr(line, 'v'))
        {
            sscanf(line, "CEL = %c", &data->cel_mode);
        }
        else if (strncmp(line, "LUM =", 5) == 0)
        {
            sscanf(line, "LUM = %c", &data->lum);
        }
        else if (strncmp(line, "DIP =", 5) == 0)
        {
            int index = 0;
            char *token = strtok(line + 5, " ");
            while (token != NULL && index < 8)
            {
                int num;
                char state[4];
                if (sscanf(token, "%d:%3s", &num, state) == 2)
                {
                    if (num >= 1 && num <= 8)
                    {
                        data->dips[num - 1] = (strcmp(state, "ON") == 0);
                    }
                }
                index++;
                token = strtok(NULL, " ");
            }
        }
        else if (strncmp(line, "INP =", 5) == 0)
        {
            int index = 0;
            char *token = strtok(line + 5, " ");
            while (token != NULL && index < 3)
            {
                int num;
                char state[4];
                if (sscanf(token, "%d:%3s", &num, state) == 2)
                {
                    if (num >= 1 && num <= 3)
                    {
                        data->inps[num - 1] = (strcmp(state, "ON") == 0);
                    }
                }
                index++;
                token = strtok(NULL, " ");
            }
        }

        line = strtok(NULL, "\r\n");
    }
}
