/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Corps principal du programme
  ******************************************************************************
  * Cette notice s'applique à toutes les parties de ce fichier
  * qui ne sont pas entre les paires de commentaires USER CODE BEGIN et
  * USER CODE END. Les autres parties de ce fichier, qu'elles soient
  * insérées par l'utilisateur ou par des outils de développement de logiciels,
  * sont la propriété de leurs détenteurs respectifs des droits d'auteur.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution et utilisation sous forme de code source et binaire, avec ou sans modification,
  * sont autorisées sous les conditions suivantes :
  *   1. Les redistributions de code source doivent conserver la mention de droit d'auteur ci-dessus,
  *      cette liste de conditions et la clause de non-responsabilité suivante.
  *   2. Les redistributions sous forme binaire doivent reproduire la mention de droit d'auteur ci-dessus,
  *      cette liste de conditions et la clause de non-responsabilité suivante dans la documentation
  *      et/ou les autres matériaux fournis avec la distribution.
  *   3. Ni le nom de STMicroelectronics ni les noms de ses contributeurs
  *      ne peuvent être utilisés pour approuver ou promouvoir des produits dérivés de ce logiciel
  *      sans autorisation écrite préalable spécifique.
  *
  * CE LOGICIEL EST FOURNI PAR LES TITULAIRES DES DROITS D'AUTEUR ET LES CONTRIBUTEURS "EN L'ÉTAT"
  * ET TOUTES LES GARANTIES EXPRESSES OU IMPLICITES, Y COMPRIS, MAIS SANS S'Y LIMITER, LES
  * GARANTIES IMPLICITES DE QUALITÉ MARCHANDE ET D'ADÉQUATION À UN USAGE PARTICULIER SONT
  * DÉCLINÉES. EN AUCUN CAS, LE TITULAIRE DU DROIT D'AUTEUR OU LES CONTRIBUTEURS NE SAURAIENT ÊTRE TENUS RESPONSABLES
  * DES DOMMAGES DIRECTS, INDIRECTS, ACCESSOIRES, SPÉCIAUX, EXEMPLAIRES OU CONSÉCUTIFS
  * (Y COMPRIS, MAIS SANS S'Y LIMITER, L'ACQUISITION DE BIENS OU DE SERVICES DE SUBSTITUTION ; LA PERTE D'UTILISATION, DE DONNÉES OU DE PROFITS ; OU L'INTERRUPTION D'ACTIVITÉ) TOUTEFOIS CAUSÉS ET SUR TOUTE THÉORIE DE RESPONSABILITÉ, QU'ELLE SOIT CONTRACTUELLE, STRICTE OU DÉLICTUELLE
  * (Y COMPRIS LA NÉGLIGENCE OU AUTRE) DÉCOULANT DE TOUTE UTILISATION DE CE LOGICIEL, MÊME SI
  * CONSEILLÉ DE LA POSSIBILITÉ DE TELS DOMMAGES.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <string.h>
#include <stdint.h>

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Variables privées ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint8_t supported_commands[] = {
                               BL_GET_VER,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_P_STATUS};

/* Définir les alias CAN */
#define CAN_TX   &hcan1
#define CAN_RX   &hcan1
/* USER CODE BEGIN PV */
/* Variables privées ---------------------------------------------------------*/

/* USER CODE END PV */

/* Prototypes de fonctions privées -------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_CAN1_Init(void);
void bootloader_can_transmit(uint8_t *pData, uint32_t len);
static void printmsg_can(char *format,...);


char somedata[] = "Hello from Bootloader via CAN\r\n";

#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];

/* USER CODE END 0 */

/**
 * @brief Fonction de test du Flash, définissant des protections en lecture et écriture.
 *        Cette fonction active la protection en lecture/écriture sur les secteurs spécifiés
 *        de la mémoire Flash du microcontrôleur.
 */
void flash_testing(void)
{
    uint8_t protection_mode = 2;  // Mode de protection (1 = Écriture seule, 2 = Lecture/Écriture)
    uint8_t sector_details = 0x80;  // Masque des secteurs à protéger (exemple : secteur 7)

    // Adresse du registre de contrôle des options de Flash (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

    // 1. Déverrouillage des options de configuration de la Flash
    HAL_FLASH_OB_Unlock();

    // 2. Attendre que l'interface Flash ne soit plus occupée (bit BSY)
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

    // 3. Configurer la protection en fonction du mode sélectionné
    if (protection_mode == 1)  // Protection en écriture seule
    {
        // Désactiver la protection en lecture
        *pOPTCR &= ~(1 << 31);
        // Configurer les secteurs à protéger en écriture
        *pOPTCR &= ~(0xff << 16);  // Effacer les anciens bits de protection
        *pOPTCR |= (sector_details << 16);  // Appliquer la protection d'écriture
    }
    else if (protection_mode == 2)  // Protection en lecture/écriture
    {
        // Activer la protection en lecture et en écriture
        *pOPTCR |= (1 << 31);  // Activer le bit de protection lecture/écriture
        *pOPTCR &= ~(0xff << 16);  // Effacer les anciens bits de protection
        *pOPTCR |= (sector_details << 16);  // Appliquer la protection lecture/écriture
    }

    // 4. Démarrer l'opération en définissant le bit OPTSTRT (bit 1)
    *pOPTCR |= (1 << 1);

    // 5. Attendre que l'interface Flash ne soit plus occupée
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

    // 6. Verrouiller les options de configuration de la Flash après l'opération
    HAL_FLASH_OB_Lock();
}


/**
 * @brief Fonction principale du programme.
 */
int main(void)
{
  /* Réinitialisation de tous les périphériques, initialisation de l'interface Flash et du Systick */
  HAL_Init();

  /* Configurer l'horloge du système */
  SystemClock_Config();

  /* Initialiser tous les périphériques configurés */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_CAN1_Init();  // Initialiser le CAN au lieu de l'UART

  /*
   * Vérifier l'état du bouton utilisateur pour déterminer s'il faut rester en mode bootloader ou
   * sauter à l'application utilisateur :
   * - Si le bouton est pressé, le bootloader reste actif et attend des commandes via CAN.
   * - Sinon, le bootloader transfère le contrôle à l'application utilisateur stockée dans la mémoire Flash.
   */
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
  {
      printmsg_can("BL_DEBUG_MSG:Le bouton est pressé... passage en mode BL\n\r");
      bootloader_can_read_data();  // Rester en mode bootloader et attendre les commandes via CAN.
  }
  else
  {
      printmsg_can("BL_DEBUG_MSG:Le bouton n'est pas pressé... exécution de l'application utilisateur\n");
      bootloader_jump_to_user_app();  // Transférer le contrôle à l'application utilisateur.
  }
}


/**
 * @brief Fonction pour lire les commandes envoyées par l'hôte via l'interface CAN.
 * Cette fonction attend continuellement les commandes entrantes, les traite et
 * appelle la fonction de gestion de commande correspondante en fonction du code de commande.
 */
void bootloader_can_read_data(void)
{
    CAN_RxHeaderTypeDef rxHeader;  // Structure pour stocker l'en-tête des messages CAN reçus
    uint8_t rcv_len = 0;           // Variable pour stocker la longueur du paquet de commande reçu
    uint8_t can_rx_buffer[8];      // Tampon de réception pour les données CAN

    while (1)
    {
        // Réinitialiser le tampon de réception pour effacer les anciennes données
        memset(bl_rx_buffer, 0, 200);

        // Attendre un message CAN et lire les données
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, can_rx_buffer) == HAL_OK)
        {
            // La première donnée contient la longueur du paquet de commande
            rcv_len = can_rx_buffer[0];

            // Copier les données reçues dans le tampon de réception principal
            memcpy(bl_rx_buffer, can_rx_buffer, rcv_len);

            // Décoder la commande envoyée par l'hôte, qui se trouve à l'index 1 du tampon
            switch (bl_rx_buffer[1])
            {
                case BL_GET_VER:
                    bootloader_handle_getver_cmd(bl_rx_buffer);  // Gérer la commande 'GET Version'
                    break;
                case BL_GET_HELP:
                    bootloader_handle_gethelp_cmd(bl_rx_buffer);  // Gérer la commande 'GET Help'
                    break;
                case BL_GET_CID:
                    bootloader_handle_getcid_cmd(bl_rx_buffer);  // Gérer la commande 'GET Chip ID'
                    break;
                case BL_GET_RDP_STATUS:
                    bootloader_handle_getrdp_cmd(bl_rx_buffer);  // Gérer la commande 'GET Read Protection Status'
                    break;
                case BL_GO_TO_ADDR:
                    bootloader_handle_go_cmd(bl_rx_buffer);  // Gérer la commande 'Go to Address'
                    break;
                case BL_FLASH_ERASE:
                    bootloader_handle_flash_erase_cmd(bl_rx_buffer);  // Gérer la commande 'Flash Erase'
                    break;
                case BL_MEM_WRITE:
                    bootloader_handle_mem_write_cmd(bl_rx_buffer);  // Gérer la commande 'Memory Write'
                    break;
                case BL_EN_RW_PROTECT:
                    bootloader_handle_en_rw_protect(bl_rx_buffer);  // Gérer la commande 'Enable Read/Write Protect'
                    break;
                case BL_MEM_READ:
                    bootloader_handle_mem_read(bl_rx_buffer);  // Gérer la commande 'Memory Read'
                    break;
                case BL_READ_SECTOR_P_STATUS:
                    bootloader_handle_read_sector_protection_status(bl_rx_buffer);  // Gérer la commande 'Read Sector Protection Status'
                    break;
                case BL_OTP_READ:
                    bootloader_handle_read_otp(bl_rx_buffer);  // Gérer la commande 'Read OTP Memory'
                    break;
                case BL_DIS_R_W_PROTECT:
                    bootloader_handle_dis_rw_protect(bl_rx_buffer);  // Gérer la commande 'Disable Read/Write Protection'
                    break;
                default:
                    printmsg_can("BL_DEBUG_MSG:Code de commande invalide reçu de l'hôte\n");  // Imprimer un message d'erreur pour une commande invalide
                    break;
            }
        }
    }
}


/**
 * @brief Fonction pour sauter à l'application utilisateur stockée dans la mémoire flash.
 * Cette fonction suppose que l'application utilisateur est stockée à une adresse spécifique,
 * définie par `FLASH_SECTOR2_BASE_ADDRESS`. Elle lit le MSP (Main Stack Pointer) et
 * l'adresse du Reset Handler à partir de cet emplacement et transfère le contrôle à l'application utilisateur.
 */
void bootloader_jump_to_user_app(void)
{
    /* 1. Ce pointeur est utilisé pour stocker l'adresse du Reset Handler de l'application utilisateur.
     * Le reset handler est la première fonction exécutée lors du démarrage de l'application.
    */
    void (*app_reset_handler)(void);

    // 2. Message de débogage : Ce message est envoyé via CAN pour indiquer que l'on est entré dans la fonction 'bootloader_jump_to_user_app'
    printmsg_can("BL_DEBUG_MSG:bootloader_jump_to_user_app\n");

    /* 3. Lecture de la valeur du Main Stack Pointer (MSP):
     * Cette ligne lit la valeur du MSP stockée à la première adresse du secteur de mémoire où l'application utilisateur est enregistrée.
     * Le MSP indique où la pile principale doit commencer.
    */
    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;

    // 4. Message de débogage pour la valeur du MSP
    printmsg_can("BL_DEBUG_MSG:MSP value : %#x\n", msp_value);

    // 5. Configuration du MSP : on utilise une fonction CMSIS (__set_MSP) pour définir
    // la valeur du MSP.
    __set_MSP(msp_value);

    // 6. Lecture de l'adresse du Reset Handler, située après l'adresse du MSP (offset +4 de l'adresse de base)
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    // 7. Initialisation du pointeur de fonction
    app_reset_handler = (void*) resethandler_address;

    // 8. Message de débogage pour l'adresse du reset handler
    printmsg_can("BL_DEBUG_MSG: app reset handler addr : %#x\n", app_reset_handler);

    // 9. Cette fonction fait un saut vers l'application utilisateur, une fois cette ligne exécutée, le bootloader transfère le contrôle à l'application utilisateur
    app_reset_handler();
}

/**
 * @brief Imprime des messages de débogage formatés via l'interface CAN.
 * Cette fonction est utilisée pour imprimer des messages de débogage via l'interface CAN,
 * ce qui permet de surveiller les activités du bootloader.
 *
 * @param format: La chaîne de format pour le message à imprimer.
 */
void printmsg_can(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
    char str[80];  // Tampon pour stocker la chaîne de message formatée

    // Extraire la liste des arguments en utilisant les macros VA (Variable Argument)
    va_list args;
    va_start(args, format);

    // Formater la chaîne en fonction du format et des arguments fournis
    vsprintf(str, format, args);

    // Transmettre la chaîne formatée via l'interface CAN (simulation via HAL_CAN_AddTxMessage)
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t can_data[8];  // Un message CAN ne peut transporter que 8 octets, donc nous envoyons le message en plusieurs morceaux

    // Configurer l'en-tête du message CAN
    txHeader.StdId = 0x65D;  // Identifiant standard pour les messages de débogage
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 8;  // Taille des données à envoyer

    for (int i = 0; i < strlen(str); i += 8)
    {
        // Copier 8 octets de la chaîne dans le tampon can_data
        strncpy((char *)can_data, &str[i], 8);

        // Envoyer le message via l'interface CAN
        HAL_CAN_AddTxMessage(&hcan1, &txHeader, can_data, &txMailbox);

        // Attendre la fin de la transmission
        while (HAL_CAN_IsTxMessagePending(&hcan1, txMailbox));
    }

    va_end(args);
#endif
}


/**
 * @brief Configure l'horloge du système pour le microcontrôleur.
 *
 * Cette fonction configure l'oscillateur interne à haute vitesse (HSI) et les paramètres de la PLL
 * pour générer une source d'horloge stable pour le CPU et les périphériques.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;    // Structure pour la configuration des oscillateurs
    RCC_ClkInitTypeDef RCC_ClkInitStruct;    // Structure pour la configuration de l'horloge

    /** Activer l'horloge de contrôle de l'alimentation pour la régulation de la tension */
    __HAL_RCC_PWR_CLK_ENABLE();

    /** Configurer la tension de sortie du régulateur principal */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initialiser les horloges du CPU, de l'AHB et de l'APB en utilisant l'oscillateur HSI */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Type d'oscillateur HSI
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Activer l'oscillateur HSI
    RCC_OscInitStruct.HSICalibrationValue = 16;                // Valeur de calibration HSI
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Activer la PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // Utiliser HSI comme source PLL
    RCC_OscInitStruct.PLL.PLLM = 16;                           // Diviseur PLLM
    RCC_OscInitStruct.PLL.PLLN = 336;                          // Multiplicateur PLLN
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;                // Diviseur PLLP pour l'horloge principale
    RCC_OscInitStruct.PLL.PLLQ = 2;                            // Diviseur PLLQ pour l'horloge USB
    RCC_OscInitStruct.PLL.PLLR = 2;                            // Diviseur PLLR
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);                    // Gestion d'erreur lors de l'initialisation
    }

    /** Initialiser les horloges du CPU, de l'AHB et de l'APB */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Utiliser la sortie de la PLL comme horloge système
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // Diviseur de l'horloge AHB
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // Diviseur de l'horloge APB1
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // Diviseur de l'horloge APB2

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);                    // Gestion d'erreur de configuration
    }

    /** Configurer le temps d'interruption du Systick */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);          // Configurer le SysTick pour générer des interruptions toutes les 1 ms

    /** Configurer la source d'horloge du Systick */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);       // Utiliser HCLK comme source d'horloge SysTick

    /* Définir la priorité de l'interruption SysTick à 0 (priorité la plus élevée) */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
 * @brief  Initialise le périphérique CRC.
 *
 * Cette fonction configure et initialise le périphérique CRC (Cyclic Redundancy Check),
 * qui est utilisé pour calculer les valeurs CRC afin de vérifier l'intégrité des données dans le bootloader.
 */
static void MX_CRC_Init(void)
{
    hcrc.Instance = CRC;               // Sélectionner l'instance du périphérique CRC
    if (HAL_CRC_Init(&hcrc) != HAL_OK)  // Initialiser le périphérique CRC
    {
        _Error_Handler(__FILE__, __LINE__);  // Gestion d'erreur lors de l'initialisation
    }
}

/**
 * @brief  Initialise le périphérique CAN1 pour la communication CAN en mode loopback.
 *
 * Cette fonction configure les paramètres pour l'interface CAN1 en mode loopback,
 * y compris le débit en bauds, les modes et filtres de réception.
 */
static void MX_CAN1_Init(void)
{
    CAN_HandleTypeDef hcan1;

    hcan1.Instance = CAN1;                                // Sélectionner l'instance du périphérique CAN1
    hcan1.Init.Prescaler = 16;                            // Configurer le prescaler pour la vitesse de communication
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;                  // Configurer le CAN en mode loopback
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;               // Largeur de saut de synchronisation
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;                   // Configuration du segment de temps 1
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;                    // Configuration du segment de temps 2
    hcan1.Init.TimeTriggeredMode = DISABLE;               // Mode déclenché par le temps désactivé
    hcan1.Init.AutoBusOff = DISABLE;                      // Désactiver la gestion automatique de l'arrêt du bus
    hcan1.Init.AutoWakeUp = DISABLE;                      // Désactiver le réveil automatique
    hcan1.Init.AutoRetransmission = ENABLE;               // Activer la retransmission automatique
    hcan1.Init.ReceiveFifoLocked = DISABLE;               // Déverrouiller le FIFO de réception
    hcan1.Init.TransmitFifoPriority = DISABLE;            // Désactiver la priorité du FIFO de transmission

    if (HAL_CAN_Init(&hcan1) != HAL_OK)                   // Initialiser le CAN
    {
        _Error_Handler(__FILE__, __LINE__);               // Gestion d'erreur lors de l'initialisation
    }

    /* Configuration des filtres CAN */
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                         // Utiliser le premier filtre
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;     // Mode de filtrage par masque d'identifiant
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;    // Taille du filtre 32 bits
    sFilterConfig.FilterIdHigh = 0x0000;                  // Identifiant haut
    sFilterConfig.FilterIdLow = 0x0000;                   // Identifiant bas
    sFilterConfig.FilterMaskIdHigh = 0x0000;              // Masque haut
    sFilterConfig.FilterMaskIdLow = 0x0000;               // Masque bas
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;// Assignation au FIFO 0
    sFilterConfig.FilterActivation = ENABLE;              // Activer le filtre
    sFilterConfig.SlaveStartFilterBank = 14;              // Configurer le filtre de départ esclave

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);               // Gestion d'erreur lors de la configuration du filtre
    }
}



/**
 * @brief Configure les broches GPIO comme Analog, Input, Output, EVENT_OUT, ou EXTI.
 *
 * Cette fonction configure les broches GPIO pour différentes fonctions comme entrée numérique, sortie,
 * fonctions analogiques, sources d'interruptions externes (EXTI), ou autres configurations personnalisées.
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;  // Structure pour la configuration GPIO

    /* Activer les horloges pour les ports GPIO C, H, A, et B */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configurer les broches CAN TX et RX (PA11, PA12 par exemple) */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;         // Configurer les broches PA11 et PA12 pour CAN RX et TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                  // Configurer en mode alternate function push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;                      // Pas de pull-up ni pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       // Vitesse très élevée
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;               // Fonction alternative CAN1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                  // Initialiser les broches GPIOA pour CAN1

    /* Configurer le niveau de sortie GPIO pour LD2 (LED embarquée) */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  // Définir la broche LED2 à un état bas

    /* Configurer la broche GPIO : B1_Pin (bouton utilisateur) */
    GPIO_InitStruct.Pin = B1_Pin;                // Sélectionner la broche B1
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Configurer comme interruption externe sur front descendant
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // Pas de résistances pull-up ou pull-down
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);  // Initialiser la configuration de la broche

    /* Configurer la broche GPIO : LD2_Pin (LED) */
    GPIO_InitStruct.Pin = LD2_Pin;               // Sélectionner la broche LD2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Configurer comme sortie push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // Pas de résistances pull-up ou pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Vitesse de sortie basse
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);  // Initialiser la configuration de la broche
}

/**
 * @brief Fonction exécutée en cas d'erreur.
 *
 * Cette fonction est utilisée pour signaler des erreurs ou des comportements inattendus.
 * Lorsqu'elle est appelée, elle entre dans une boucle infinie, arrêtant l'exécution du code.
 * Les utilisateurs peuvent personnaliser cette fonction pour gérer les erreurs plus gracieusement.
 *
 * @param file: Nom du fichier source où l'erreur s'est produite.
 * @param line: Numéro de ligne dans le fichier source où l'erreur s'est produite.
 */
void _Error_Handler(char * file, int line)
{
  while(1)
  {
      // Boucle infinie pour indiquer l'état d'erreur
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  // Clignoter la LED pour indiquer une erreur
      HAL_Delay(500);                              // Délai de 500 ms
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Signale le nom du fichier source et le numéro de ligne
 *        où l'erreur assert_param s'est produite.
 *
 * Cette fonction est utilisée par la macro `assert_param` pour signaler des erreurs lorsque
 * une valeur de paramètre est invalide. Elle imprime le fichier source et le numéro de ligne
 * où l'erreur a été détectée.
 *
 * @param file: Pointeur vers le nom du fichier source.
 * @param line: Numéro de ligne où l'erreur assert_param a été déclenchée.
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  // L'utilisateur peut implémenter ici un rapport d'erreur personnalisé
  // par exemple l'impression d'un message d'erreur sur le terminal série ou un log
  printf("Erreur dans le fichier %s à la ligne %d\n", file, line);
}

#endif


/************** Implémentation des fonctions de gestion des commandes Bootloader *********/

/**
 * @brief Gère la commande 'Get Version' (BL_GET_VER) envoyée par l'hôte via CAN.
 *
 * Cette fonction est utilisée pour répondre à l'hôte avec la version actuelle du bootloader.
 * Elle lit le paquet de commande, vérifie son intégrité en utilisant le CRC,
 * et envoie la version du bootloader si le test CRC est réussi. Sinon, elle répond avec un NACK.
 *
 * @param bl_rx_buffer: Pointeur vers le tampon de commande reçu contenant le paquet de commande.
 */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;  // Variable pour stocker la version du bootloader

    // 1) Imprimer un message de débogage indiquant l'entrée dans la fonction.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_getver_cmd\n");

    // Calculer la longueur totale du paquet de commande reçu.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoyé par l'hôte à la fin du paquet de commande.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier l'intégrité du paquet reçu en calculant le CRC local
    // et en le comparant avec celui reçu. Si bootloader_verify_crc() retourne 0, le CRC est valide.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Si le CRC est correct, imprimer un message indiquant la réussite de la vérification du checksum.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN.
        bootloader_send_ack(bl_rx_buffer[0], 1);

        // Récupérer la version actuelle du bootloader en utilisant la fonction get_bootloader_version().
        bl_version = get_bootloader_version(); // Récupérer la version du bootloader.

        // Imprimer la version du bootloader dans le terminal de débogage.
        printmsg_can("BL_DEBUG_MSG:BL_VER : %d %#x\n", bl_version, bl_version);

        // Envoyer la version du bootloader à l'hôte via l'interface CAN.
        bootloader_can_write_data(&bl_version, 1);  // Envoyer la réponse à l'hôte via CAN.
    }
    else
    {
        // Si le CRC est incorrect, imprimer un message indiquant un échec de la vérification du checksum.
      printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accusé de réception négatif (NACK) à l'hôte via CAN.
        bootloader_send_nack();
    }
}

/**
 * @brief Gère la commande 'Get Help' (BL_GET_HELP) envoyée par l'hôte via CAN.
 *
 * Cette commande informe l'hôte de toutes les commandes supportées par le bootloader.
 * Lorsqu'elle est appelée, cette fonction renvoie la liste de toutes les commandes supportées à l'hôte.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu de l'hôte.
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
    // Imprimer un message de débogage indiquant que la commande 'Get Help' est en cours de traitement.
  printmsg_can("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoyé par l'hôte.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier la validité du CRC32 pour garantir que les données n'ont pas été altérées.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Si le CRC est valide, envoyer un message de succès pour la vérification du checksum.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) avec le nombre d'octets que le bootloader enverra en réponse via CAN.
        bootloader_send_ack(pBuffer[0], sizeof(supported_commands));

        // Envoyer la liste des commandes supportées à l'hôte via l'interface CAN.
        bootloader_can_write_data(supported_commands, sizeof(supported_commands));
    }
    else
    {
        // Si le CRC est incorrect, envoyer un message d'erreur de vérification.
      printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accusé de réception négatif (NACK) via CAN.
        bootloader_send_nack();
    }
}


/**
 * @brief Gère la commande 'Get Chip ID' (BL_GET_CID) envoyée par l'hôte via CAN.
 *
 * Cette commande est utilisée pour lire l'identifiant unique de la puce du microcontrôleur.
 * L'ID de la puce permet à l'hôte de déterminer le numéro de pièce ST et la révision du silicium du périphérique.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu de l'hôte.
 */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
    uint16_t bl_cid_num = 0;  // Variable pour stocker l'ID unique de la puce du microcontrôleur.

    // Imprimer un message de débogage indiquant que la commande est en cours de traitement.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoyé par l'hôte.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier la validité du CRC32.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Si le CRC est valide, imprimer un message indiquant la réussite de la vérification du checksum.
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) avec la longueur de la réponse (2 octets pour l'ID de la puce) via CAN.
        bootloader_send_ack(pBuffer[0], 2);

        // Récupérer l'ID de la puce du microcontrôleur.
        bl_cid_num = get_mcu_chip_id();
        printmsg_can("BL_DEBUG_MSG:MCU id : %d %#x !!\n", bl_cid_num, bl_cid_num);

        // Envoyer l'ID de la puce (2 octets) à l'hôte via CAN.
        bootloader_can_write_data((uint8_t *)&bl_cid_num, 2);
    }
    else
    {
        // Si le CRC n'est pas valide, imprimer un message indiquant un échec de vérification.
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accusé de réception négatif (NACK) à l'hôte via CAN.
        bootloader_send_nack();
    }
}

/**
 * @brief Gère la commande 'Get Read Protection Status' (BL_GET_RDP_STATUS) envoyée par l'hôte via CAN.
 *
 * Cette fonction récupère le niveau de protection en lecture (RDP) de la mémoire Flash du microcontrôleur
 * et envoie cette information au système hôte via l'interface CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu de l'hôte.
 */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    // Variable pour stocker le niveau de protection en lecture (RDP) de la mémoire Flash.
    uint8_t rdp_level = 0x00;

    // Imprimer un message de débogage indiquant l'entrée dans la fonction de gestion de la commande RDP.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoyé par l'hôte.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour s'assurer de l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Imprimer un message de débogage indiquant que le CRC est valide.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) avec la longueur de la réponse (1 octet pour le niveau RDP) via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Récupérer le niveau actuel de protection en lecture de la mémoire Flash.
        rdp_level = get_flash_rdp_level();

        // Imprimer le niveau de protection RDP dans le terminal de débogage.
        printmsg_can("BL_DEBUG_MSG:RDP level: %d %#x\n", rdp_level, rdp_level);

        // Envoyer le niveau de protection RDP à l'hôte via CAN.
        bootloader_can_write_data(&rdp_level, 1);
    }
    else
    {
        // Si le CRC est incorrect, imprimer un message d'erreur.
      printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accusé de réception négatif (NACK) à l'hôte via CAN.
        bootloader_send_nack();
    }
}

/**
 * @brief Gère la commande 'Go to Address' (BL_GO_TO_ADDR) envoyée par l'hôte via CAN.
 *
 * Cette commande est utilisée pour sauter à une adresse spécifiée par l'hôte.
 * La fonction vérifie l'intégrité du paquet de commande via le CRC, valide l'adresse,
 * et si tout est correct, elle saute à l'adresse spécifiée.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu de l'hôte.
 */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    // Variable pour stocker l'adresse vers laquelle le bootloader doit sauter.
    uint32_t go_address = 0;

    // Variables pour indiquer si l'adresse est valide ou non.
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;

    // Imprimer un message de débogage indiquant l'entrée dans la fonction de gestion de la commande GO.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_go_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoyé par l'hôte.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour s'assurer de l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Imprimer un message indiquant que le CRC est valide.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN pour confirmer la réception correcte de la commande.
        bootloader_send_ack(pBuffer[0], 1);

        // Extraire l'adresse vers laquelle l'hôte souhaite que le bootloader saute.
        go_address = *((uint32_t *)&pBuffer[2]);
        printmsg_can("BL_DEBUG_MSG:GO addr: %#x\n", go_address);

        // Vérifier si l'adresse extraite est valide pour l'exécution.
        if (verify_address(go_address) == ADDR_VALID)
        {
            // Indiquer à l'hôte que l'adresse est valide via CAN.
            bootloader_can_write_data(&addr_valid, 1);

            // Ajouter 1 à l'adresse pour définir le bit Thumb (T bit) à 1, nécessaire pour les processeurs ARM Cortex M.
            go_address += 1;

            // Déclarer un pointeur de fonction pour sauter à l'adresse spécifiée.
            void (*lets_jump)(void) = (void *)go_address;

            // Imprimer un message indiquant que le bootloader saute à l'adresse spécifiée.
            printmsg_can("BL_DEBUG_MSG: jumping to go address! \n");

            // Sauter à l'adresse spécifiée et exécuter le code à cet emplacement.
            lets_jump();
        }
        else
        {
            // Si l'adresse n'est pas valide, imprimer un message d'erreur.
            printmsg_can("BL_DEBUG_MSG:GO addr invalid ! \n");

            // Indiquer à l'hôte que l'adresse est invalide via CAN.
            bootloader_can_write_data(&addr_invalid, 1);
        }
    }
    else
    {
        // Si le CRC est incorrect, imprimer un message d'erreur.
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accusé de réception négatif (NACK) via CAN à l'hôte.
        bootloader_send_nack();
    }
}


/**
 * @brief Gère la commande 'Flash Erase' (BL_FLASH_ERASE) envoyée par l'hôte via CAN.
 *
 * Cette fonction traite la commande 'Flash Erase' reçue de l'hôte.
 * Elle vérifie l'intégrité du paquet de commande via le CRC, extrait les détails de l'effacement,
 * effectue l'opération d'effacement de la mémoire Flash, et envoie le statut à l'hôte via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande.
 */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;  // Variable pour capturer le statut de l'effacement.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoyée par l'hôte.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour garantir l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Extraire les informations d'effacement : secteur initial et nombre de secteurs.
        printmsg_can("BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d\n", pBuffer[2], pBuffer[3]);

        // Allumer une LED pendant l'opération d'effacement pour indiquer que l'opération est en cours.
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
        erase_status = execute_flash_erase(pBuffer[2], pBuffer[3]);  // Appeler la fonction d'effacement de la Flash.
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);  // Éteindre la LED après l'effacement.

        // Afficher le statut de l'effacement.
        printmsg_can("BL_DEBUG_MSG: flash erase status: %#x\n", erase_status);

        // Envoyer le statut de l'effacement à l'hôte via CAN.
        bootloader_can_write_data(&erase_status, 1);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");
        // Si le CRC est invalide, envoyer un NACK à l'hôte via CAN.
        bootloader_send_nack();
    }
}



/**
 * @brief Gère la commande 'Memory Write' (BL_MEM_WRITE) envoyée par l'hôte via CAN.
 *
 * Cette fonction traite la commande 'Memory Write' reçue de l'hôte.
 * Elle vérifie le CRC du paquet de commande, vérifie si l'adresse mémoire est valide,
 * effectue l'opération d'écriture, et envoie le statut de l'écriture à l'hôte via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu.
 */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
    // Initialisation des variables de statut.
    // uint8_t addr_valid = ADDR_VALID;  // Indicateur si l'adresse est valide.
    uint8_t write_status = 0x00;      // Statut de l'opération d'écriture.
    uint8_t chksum = 0;
    uint8_t len = 0;      // Variables pour la longueur et la vérification du checksum.

    // Lire la longueur du paquet de commande.
    len = pBuffer[0];

    // Longueur de la charge utile (les données à écrire).
    uint8_t payload_len = pBuffer[6];

    // Extraire l'adresse mémoire où l'écriture doit commencer.
    uint32_t mem_address = *((uint32_t *) (&pBuffer[2]));

    // Extraire le checksum envoyé par l'hôte pour vérifier l'intégrité des données.
    if (chksum != pBuffer[len]) {
        // Gestion de l'erreur de checksum
        printmsg_can("Erreur de checksum\n");
    }


    // Imprimer un message de débogage indiquant l'exécution de la commande.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoyé par l'hôte pour vérification.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour garantir l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Imprimer l'adresse mémoire cible pour l'opération d'écriture.
        printmsg_can("BL_DEBUG_MSG: mem write address : %#x\n", mem_address);

        // Vérifier si l'adresse mémoire cible est valide.
        if (verify_address(mem_address) == ADDR_VALID)
        {
            printmsg_can("BL_DEBUG_MSG: valid mem write address\n");

            // Allumer la LED pour indiquer que l'opération d'écriture est en cours.
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            // Effectuer l'opération d'écriture en mémoire.
            write_status = execute_mem_write(&pBuffer[7], mem_address, payload_len);

            // Éteindre la LED pour indiquer la fin de l'opération d'écriture.
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

            // Envoyer le statut de l'écriture à l'hôte (succès ou échec) via CAN.
            bootloader_can_write_data(&write_status, 1);
        }
        else
        {
            printmsg_can("BL_DEBUG_MSG: invalid mem write address\n");
            write_status = ADDR_INVALID;

            // Envoyer le statut d'adresse invalide à l'hôte via CAN.
            bootloader_can_write_data(&write_status, 1);
        }
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un NACK si la vérification du CRC échoue via CAN.
        bootloader_send_nack();
    }
}


/**
 * @brief Gère la commande 'Enable/Disable Read/Write Protection' (BL_EN_RW_PROTECT) envoyée par l'hôte via CAN.
 *
 * Cette fonction traite la commande pour activer ou désactiver la protection en lecture/écriture sur les secteurs de la Flash.
 * Elle vérifie le CRC du paquet de commande reçu, applique la protection souhaitée,
 * et envoie le statut à l'hôte via le bus CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu.
 */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;  // Variable pour stocker le statut de protection.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\n");  // Message de débogage indiquant le traitement de la commande.

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoyée par l'hôte.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour garantir l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");  // Le CRC est valide, poursuivre le traitement.

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Activer ou désactiver la protection selon les paramètres de la commande.
        status = configure_flash_sector_rw_protection(pBuffer[2], pBuffer[3], 0);

        printmsg_can("BL_DEBUG_MSG: flash erase status: %#x\n", status);

        // Envoyer le statut de protection à l'hôte via CAN.
        bootloader_can_write_data(&status, 1);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");  // Le CRC est invalide, envoyer un NACK via CAN.
        bootloader_send_nack();
    }
}


/**
 * @brief Gère la commande 'Disable Read/Write Protection' (BL_DIS_RW_PROTECT) envoyée par l'hôte via CAN.
 *
 * Cette fonction traite la commande pour désactiver la protection en lecture/écriture sur les secteurs de la Flash.
 * Elle vérifie le CRC du paquet de commande reçu, désactive la protection,
 * et envoie le statut à l'hôte via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu.
 */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoyée par l'hôte.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour garantir l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Désactiver la protection en lecture/écriture.
        status = configure_flash_sector_rw_protection(0, 0, 1);

        printmsg_can("BL_DEBUG_MSG: flash erase status: %#x\n", status);

        // Envoyer le statut de désactivation de la protection à l'hôte via CAN.
        bootloader_can_write_data(&status, 1);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accusé de réception négatif (NACK) via CAN si la vérification du CRC échoue.
        bootloader_send_nack();
    }
}


/**
 * @brief Gère la commande 'Memory Read' (BL_MEM_READ) envoyée par l'hôte via CAN.
 *
 * Cette fonction est un espace réservé pour l'implémentation de la fonctionnalité de lecture mémoire
 * afin de lire une adresse mémoire spécifiée et renvoyer les données à l'hôte via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu.
 */
void bootloader_handle_mem_read(uint8_t *pBuffer)
{
    // À implémenter dans le futur pour la lecture du contenu de la mémoire via CAN.
}

/**
 * @brief Gère la commande 'Read Sector Protection Status' (BL_READ_SECTOR_P_STATUS) envoyée par l'hôte via CAN.
 *
 * Cette fonction lit l'état de la protection en lecture/écriture des secteurs.
 * Elle vérifie le CRC du paquet de commande reçu, récupère l'état de protection,
 * et l'envoie à l'hôte via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu.
 */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
    uint16_t status;  // Variable pour stocker l'état de protection.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoyée par l'hôte.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Vérifier le CRC pour garantir l'intégrité des données.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accusé de réception (ACK) à l'hôte via CAN.
        bootloader_send_ack(pBuffer[0], 2);

        // Lire l'état de protection depuis les Bytes d'Option (OB).
        status = read_OB_rw_protection_status();

        printmsg_can("BL_DEBUG_MSG: nWRP status: %#x\n", status);

        // Envoyer l'état de protection en lecture/écriture à l'hôte via CAN.
        bootloader_can_write_data((uint8_t *)&status, 2);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un NACK à l'hôte via CAN en cas d'échec de la vérification du CRC.
        bootloader_send_nack();
    }
}


/**
 * @brief Gère la commande 'Read OTP Memory' (BL_OTP_READ) envoyée par l'hôte via CAN.
 *
 * Cette fonction est un espace réservé pour l'implémentation de la fonctionnalité de lecture de la mémoire OTP (One-Time Programmable).
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande reçu via CAN.
 */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{
    // À implémenter dans le futur pour la lecture du contenu de la mémoire OTP via CAN.
}

/**
 * @brief Envoie un message d'accusé de réception (ACK) avec une longueur de données de suivi via CAN.
 *
 * Cette fonction est utilisée pour indiquer que la commande envoyée par l'hôte a été correctement
 * reçue et traitée par le bootloader via CAN.
 *
 * @param command_code: Le code de la commande pour laquelle l'ACK est envoyé.
 * @param follow_len: Longueur des données de suivi à envoyer.
 */
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
    // On envoie ACK + Length (2 octets)
    uint8_t ack_buf[2];

    ack_buf[0] = BL_ACK;       // Code ACK
    ack_buf[1] = follow_len;   // Longueur des données de suivi

    // Envoyer le message ACK via CAN
    bootloader_can_transmit(ack_buf, 2);  // Envoyer via l'interface CAN
}

/**
 * @brief Envoie un message de Non Accusé de Réception (NACK) via CAN.
 *
 * Cette fonction est utilisée pour indiquer que le CRC de la commande envoyée par l'hôte est incorrect
 * ou que la commande n'a pas pu être traitée.
 */
void bootloader_send_nack(void)
{
    // On envoie 1 octet NACK
    uint8_t nack = BL_NACK;

    // Envoyer le message NACK via CAN
    bootloader_can_transmit(&nack, 1);  // Envoyer via l'interface CAN
}

/**
 * @brief Vérifie le CRC (Cyclic Redundancy Check) d'un tampon de données reçu via CAN.
 *
 * Cette fonction prend en entrée un pointeur vers les données (`pData`), la longueur des données (`len`),
 * et la valeur CRC envoyée par l'hôte (`crc_host`).
 * Si le CRC calculé correspond au CRC de l'hôte, elle renvoie `VERIFY_CRC_SUCCESS`.
 * Sinon, elle renvoie `VERIFY_CRC_FAIL`.
 *
 * @param pData: Pointeur vers le tampon de données à vérifier.
 * @param len: Longueur des données à traiter.
 * @param crc_host: Valeur CRC reçue de l'hôte.
 * @retval VERIFY_CRC_SUCCESS si le CRC est correct, VERIFY_CRC_FAIL sinon.
 */
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue = 0xff;  // Valeur initiale du CRC

    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t i_data = pData[i];

        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);  // Calculer et accumuler le CRC
    }

    __HAL_CRC_DR_RESET(&hcrc);  // Réinitialiser l'unité de calcul du CRC

    return (uwCRCValue == crc_host) ? VERIFY_CRC_SUCCESS : VERIFY_CRC_FAIL;
}


/**
 * @brief Envoie des données au terminal série via UART.
 * Cette fonction utilise l'API HAL_UART_Transmit pour envoyer des données via l'UART spécifiée.
 *
 * @param pBuffer: Pointeur vers le tampon contenant les données à transmettre.
 * @param len: Longueur des données à transmettre (en octets).
 */
/**
 * @brief Envoie des données via CAN.
 * Cette fonction utilise l'API HAL_CAN_AddTxMessage pour envoyer des données via l'interface CAN spécifiée.
 *
 * @param pBuffer: Pointeur vers le tampon contenant les données à transmettre.
 * @param len: Longueur des données à transmettre (en octets).
 */
void bootloader_can_write_data(uint8_t *pBuffer, uint32_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    // Configurer l'en-tête du message CAN
    txHeader.DLC = len;          // Longueur du message (Data Length Code)
    txHeader.StdId = 0x65D;      // Identifiant standard (modifiable selon votre système)
    txHeader.IDE = CAN_ID_STD;   // Identifiant standard
    txHeader.RTR = CAN_RTR_DATA; // Type de trame (trame de données)
    txHeader.TransmitGlobalTime = DISABLE;  // Désactiver la temporisation globale

    // Envoyer les données via CAN
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, pBuffer, &txMailbox) != HAL_OK)
    {
        // Gestion des erreurs de transmission
        printmsg_can("BL_DEBUG_MSG: CAN transmission failed\n");
    }
}

void bootloader_can_transmit(uint8_t *pData, uint32_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t can_data[8];

    // Configuration de l'en-tête CAN
    txHeader.DLC = len;          // Longueur des données (0-8 octets)
    txHeader.StdId = 0x65D;      // Identifiant CAN (modifiable selon votre système)
    txHeader.IDE = CAN_ID_STD;   // Format standard d'identifiant
    txHeader.RTR = CAN_RTR_DATA; // Trame de données
    txHeader.TransmitGlobalTime = DISABLE;

    // Découpage des données en blocs de 8 octets (taille max d'une trame CAN)
    for (uint32_t i = 0; i < len; i += 8)
    {
        // Copier jusqu'à 8 octets dans can_data
        memcpy(can_data, &pData[i], (len - i > 8) ? 8 : (len - i));

        // Ajouter le message CAN à la file de transmission
        if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, can_data, &txMailbox) != HAL_OK)
        {
            printmsg_can("BL_DEBUG_MSG: CAN transmission failed\n");
            break;
        }

        // Attendre que le message soit envoyé
        while (HAL_CAN_IsTxMessagePending(&hcan1, txMailbox));
    }
}


/**
 * @brief Renvoie la version du bootloader.
 * Cette fonction renvoie simplement la valeur définie dans la macro `BL_VERSION`.
 *
 * @retval La version du bootloader sous forme d'entier 8 bits.
 */
uint8_t get_bootloader_version(void)
{
    // Retourner la version du bootloader définie par la macro `BL_VERSION`.
    return (uint8_t)BL_VERSION;
}

/**
 * @brief Lit l'identifiant unique de la puce du microcontrôleur.
 * L'identifiant unique du microcontrôleur est stocké dans le registre `IDCODE` du composant `DBGMCU`.
 * Les 12 premiers bits de ce registre contiennent l'ID du périphérique, qui identifie le numéro de série du MCU.
 *
 * @retval uint16_t: L'identifiant unique du microcontrôleur.
 */
uint16_t get_mcu_chip_id(void)
{
    /*
     * Les MCU STM32F4xx intègrent un code d'identification (ID code).
     * Ce code est contenu dans le registre `DBGMCU->IDCODE`.
     * Les bits [11:0] de ce registre identifient l'ID du périphérique.
     * Le registre `DBGMCU->IDCODE` est mappé à l'adresse 0xE0042000.
     */
    uint16_t cid;

    // Lire le registre `IDCODE` et masquer les 12 bits inférieurs pour obtenir l'ID du périphérique.
    cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;

    // Retourner l'identifiant unique.
    return cid;
}


/**
 * @brief Lit le niveau de protection en lecture (RDP) de la mémoire Flash.
 * Le niveau de protection en lecture est utilisé pour protéger le contenu de la mémoire Flash contre
 * les opérations de lecture, d'écriture et d'effacement.
 *
 * @retval Un `uint8_t` représentant le niveau actuel de protection RDP :
 *         - 0xAA : Niveau 0 (Pas de protection)
 *         - 0xBB : Niveau 1 (Protection activée)
 *         - 0xCC : Niveau 2 (Protection maximale, irréversible)
 */
uint8_t get_flash_rdp_level(void)
{
    // Variable pour stocker le niveau RDP lu.
    uint8_t rdp_status = 0;

    // Directive de compilation pour sélectionner la méthode d'implémentation basée sur la préférence.
#if 0
    // Méthode 1 : Utiliser la bibliothèque HAL pour lire les octets d'option.

    // Structure HAL pour contenir la configuration des octets d'option.
    FLASH_OBProgramInitTypeDef ob_handle;

    // Récupérer la configuration actuelle des octets d'option, y compris le niveau RDP.
    HAL_FLASHEx_OBGetConfig(&ob_handle);

    // Le niveau RDP est stocké dans le champ `RDPLevel` de la structure `ob_handle`.
    rdp_status = (uint8_t)ob_handle.RDPLevel;

#else
    // Méthode 2 : Lire directement depuis l'adresse mémoire où les octets d'option sont stockés.

    // Adresse de départ des octets d'option dans le STM32F446xx.
    // Les octets d'option sont stockés à l'adresse 0x1FFFC000 dans la mémoire Flash.
    volatile uint32_t *pOB_addr = (uint32_t*)0x1FFFC000;

    // Lire les bits 8 à 15 pour obtenir le niveau RDP.
    // Effectuer un décalage de 8 bits vers la droite pour isoler le deuxième ensemble de 8 bits dans le registre 32 bits.
    rdp_status = (uint8_t)(*pOB_addr >> 8);

#endif

    // Retourner le niveau RDP lu.
    return rdp_status;
}

/**
 * @brief Vérifie l'adresse envoyée par l'hôte.
 * Retourne ADDR_VALID si l'adresse est valide pour l'exécution de code.
 * Sinon, retourne ADDR_INVALID.
 *
 * @param go_address: Adresse à vérifier.
 * @retval ADDR_VALID si l'adresse est valide, ADDR_INVALID sinon.
 */
uint8_t verify_address(uint32_t go_address)
{
    // Vérifier si l'adresse appartient à la mémoire SRAM1
    if (go_address >= SRAM1_BASE && go_address <= SRAM1_END)
    {
        return ADDR_VALID;
    }
    // Vérifier si l'adresse appartient à la mémoire SRAM2
    else if (go_address >= SRAM2_BASE && go_address <= SRAM2_END)
    {
        return ADDR_VALID;
    }
    // Vérifier si l'adresse appartient à la mémoire Flash (mémoire utilisateur)
    else if (go_address >= FLASH_BASE && go_address <= FLASH_END)
    {
        return ADDR_VALID;
    }
    // Vérifier si l'adresse appartient à la mémoire SRAM de sauvegarde
    else if (go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
    {
        return ADDR_VALID;
    }
    else
    {
        return ADDR_INVALID; // L'adresse n'est pas dans une plage valide
    }
}

/**
 * @brief Efface les secteurs de mémoire Flash.
 *
 * Nous avons un total de 8 secteurs (de 0 à 7) dans ce microcontrôleur.
 * Le nombre de secteurs doit être compris entre 0 et 7.
 * Si le secteur = 0xFF, cela indique qu'un effacement de masse (effacement complet) est demandé.
 *
 * @param sector_number: Le premier secteur à effacer.
 * @param number_of_sector: Nombre de secteurs à effacer.
 * @retval Le statut de l'opération (OK, erreur, etc.).
 */
uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector)
{
    FLASH_EraseInitTypeDef flashErase_handle; // Structure pour gérer le processus d'effacement
    uint32_t sectorError; // Variable pour capturer les erreurs de secteur
    HAL_StatusTypeDef status; // Statut de retour de l'API HAL

    // Si le nombre de secteurs dépasse 8, retourner une erreur de secteur non valide
    if (number_of_sector > 8)
        return INVALID_SECTOR;

    // Assurez-vous que le secteur est soit 0xFF (effacement de masse) soit compris entre 0 et 7
    if ((sector_number == 0xFF) || (sector_number <= 7))
    {
        // Si c'est un effacement de masse
        if (sector_number == 0xFF)
        {
            flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE; // Indique un effacement complet de la mémoire
        }
        // Si c'est un effacement de secteurs spécifiques
        else
        {
            // Calculer les secteurs restants à partir du secteur de départ
            uint8_t remaining_sector = 8 - sector_number;
            if (number_of_sector > remaining_sector)
            {
                number_of_sector = remaining_sector;
            }
            flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS; // Indique un effacement secteur par secteur
            flashErase_handle.Sector = sector_number; // Premier secteur à effacer
            flashErase_handle.NbSectors = number_of_sector; // Nombre de secteurs à effacer
        }
        flashErase_handle.Banks = FLASH_BANK_1; // Effacement sur la banque Flash 1

        // Déverrouiller la mémoire Flash pour les accès en écriture
        HAL_FLASH_Unlock();
        flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Plage de tension utilisée par le microcontrôleur
        // Appeler l'API HAL pour effectuer l'opération d'effacement
        status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
        // Verrouiller la mémoire Flash après l'effacement
        HAL_FLASH_Lock();

        return status; // Retourner le statut de l'opération (OK, erreur, etc.)
    }

    return INVALID_SECTOR; // Si le numéro de secteur est invalide, retourner une erreur
}

/**
 * @brief Écrit le contenu de pBuffer à l'adresse "mem_address" octet par octet.
 *
 * Remarque 1 : Actuellement, cette fonction ne prend en charge que l'écriture en Flash.
 * Remarque 2 : Cette fonction ne vérifie pas si "mem_address" est une adresse valide dans la plage Flash.
 *
 * @param pBuffer: Tampon contenant les données à écrire.
 * @param mem_address: Adresse mémoire où l'écriture doit commencer.
 * @param len: Longueur des données à écrire.
 * @retval Statut de l'opération (HAL_OK ou erreur).
 */
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status = HAL_OK;

    // Déverrouiller la mémoire Flash pour les opérations d'écriture
    HAL_FLASH_Unlock();

    // Écrire chaque octet dans la mémoire
    for (uint32_t i = 0; i < len; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_address + i, pBuffer[i]);
    }

    // Verrouiller la mémoire Flash après l'opération
    HAL_FLASH_Lock();

    return status;
}




/**
 * @brief Modifie les octets d'option utilisateur pour configurer la protection en lecture/écriture.
 * Pour modifier la valeur des octets d'option utilisateur, suivez la séquence ci-dessous :
 * 1. Vérifiez qu'aucune opération de mémoire Flash n'est en cours en vérifiant le bit BSY dans le registre FLASH_SR.
 * 2. Écrivez la valeur d'option souhaitée dans le registre FLASH_OPTCR.
 * 3. Définissez le bit de démarrage d'option (OPTSTRT) dans le registre FLASH_OPTCR.
 * 4. Attendez que le bit BSY soit désactivé.
 *
 * Cette fonction configure la protection en lecture/écriture au niveau des secteurs en fonction du mode fourni :
 * - Si `disable` est défini, il désactive la protection en lecture/écriture sur tous les secteurs.
 * - Si `protection_mode` est 1, il active la protection en écriture uniquement pour les secteurs spécifiés.
 * - Si `protection_mode` est 2, il active la protection en lecture/écriture pour les secteurs spécifiés.
 *
 * @param sector_details : Masque de bits pour les secteurs à protéger.
 * @param protection_mode : Le type de protection (1 : Protection en écriture, 2 : Protection en lecture/écriture).
 * @param disable : Si défini, désactive la protection.
 *
 * @return 0 en cas de succès.
 */
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
    // Pointeur vers le registre de contrôle des options de la mémoire Flash (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

    // Désactiver la protection en lecture/écriture si l'argument `disable` est défini
    if (disable)
    {
        HAL_FLASH_OB_Unlock();  // Déverrouiller les octets d'option Flash

        // Attendre jusqu'à ce que la mémoire Flash ne soit plus occupée
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

        // Désactiver la protection sur tous les secteurs
        *pOPTCR &= ~(1 << 31);  // Désactiver le bit SPRMOD (protection en lecture/écriture)
        *pOPTCR |= (0xFF << 16);  // Désactiver la protection sur tous les secteurs

        *pOPTCR |= (1 << 1);  // Démarrer la programmation des options

        // Attendre jusqu'à ce que la mémoire Flash ne soit plus occupée
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

        HAL_FLASH_OB_Lock();  // Verrouiller les octets d'option Flash
        return 0;
    }

    // Configurer la protection en fonction du mode sélectionné
    if (protection_mode == (uint8_t) 1)  // Mode 1 : Protection en écriture uniquement
    {
        HAL_FLASH_OB_Unlock();  // Déverrouiller les octets d'option Flash

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occupée

        *pOPTCR &= ~(1 << 31);  // Désactiver la protection en lecture (bit 31 à 0)
        *pOPTCR &= ~(sector_details << 16);  // Activer la protection en écriture pour les secteurs spécifiés

        *pOPTCR |= (1 << 1);  // Démarrer la programmation des options

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occupée

        HAL_FLASH_OB_Lock();  // Verrouiller les octets d'option Flash
    }
    else if (protection_mode == (uint8_t) 2)  // Mode 2 : Protection en lecture/écriture
    {
        HAL_FLASH_OB_Unlock();  // Déverrouiller les octets d'option Flash

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occupée

        *pOPTCR |= (1 << 31);  // Activer la protection en lecture (bit 31 à 1)
        *pOPTCR &= ~(0xff << 16);  // Effacer tous les bits de protection
        *pOPTCR |= (sector_details << 16);  // Activer la protection en lecture/écriture pour les secteurs spécifiés

        *pOPTCR |= (1 << 1);  // Démarrer la programmation des options

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occupée

        HAL_FLASH_OB_Lock();  // Verrouiller les octets d'option Flash
    }

    return 0;
}

/**
 * @brief Lit l'état de la protection en lecture/écriture des octets d'option Flash.
 *
 * Cette fonction lit la configuration des octets d'option (OB) pour déterminer l'état actuel de la protection
 * en lecture/écriture. L'état de la protection est stocké dans le champ `WRPSector` de la structure OB.
 *
 * @return Une valeur de 16 bits représentant les secteurs avec une protection en lecture/écriture active.
 */
uint16_t read_OB_rw_protection_status(void)
{
    // Cette structure est fournie par le pilote Flash de ST pour contenir le contenu des OB (Option Bytes).
    FLASH_OBProgramInitTypeDef OBInit;

    // Tout d'abord, déverrouiller l'accès mémoire des OB (Option Bytes)
    HAL_FLASH_OB_Unlock();
    // Récupérer les détails de la configuration des OB
    HAL_FLASHEx_OBGetConfig(&OBInit);
    // Verrouiller l'accès mémoire des OB
    HAL_FLASH_Lock();

    // Nous nous intéressons uniquement à l'état de protection en lecture/écriture des secteurs.
    return (uint16_t)OBInit.WRPSector;
}
