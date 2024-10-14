/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Corps principal du programme
  ******************************************************************************
  * Cette notice s'applique � toutes les parties de ce fichier
  * qui ne sont pas entre les paires de commentaires USER CODE BEGIN et
  * USER CODE END. Les autres parties de ce fichier, qu'elles soient
  * ins�r�es par l'utilisateur ou par des outils de d�veloppement de logiciels,
  * sont la propri�t� de leurs d�tenteurs respectifs des droits d'auteur.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution et utilisation sous forme de code source et binaire, avec ou sans modification,
  * sont autoris�es sous les conditions suivantes :
  *   1. Les redistributions de code source doivent conserver la mention de droit d'auteur ci-dessus,
  *      cette liste de conditions et la clause de non-responsabilit� suivante.
  *   2. Les redistributions sous forme binaire doivent reproduire la mention de droit d'auteur ci-dessus,
  *      cette liste de conditions et la clause de non-responsabilit� suivante dans la documentation
  *      et/ou les autres mat�riaux fournis avec la distribution.
  *   3. Ni le nom de STMicroelectronics ni les noms de ses contributeurs
  *      ne peuvent �tre utilis�s pour approuver ou promouvoir des produits d�riv�s de ce logiciel
  *      sans autorisation �crite pr�alable sp�cifique.
  *
  * CE LOGICIEL EST FOURNI PAR LES TITULAIRES DES DROITS D'AUTEUR ET LES CONTRIBUTEURS "EN L'�TAT"
  * ET TOUTES LES GARANTIES EXPRESSES OU IMPLICITES, Y COMPRIS, MAIS SANS S'Y LIMITER, LES
  * GARANTIES IMPLICITES DE QUALIT� MARCHANDE ET D'AD�QUATION � UN USAGE PARTICULIER SONT
  * D�CLIN�ES. EN AUCUN CAS, LE TITULAIRE DU DROIT D'AUTEUR OU LES CONTRIBUTEURS NE SAURAIENT �TRE TENUS RESPONSABLES
  * DES DOMMAGES DIRECTS, INDIRECTS, ACCESSOIRES, SP�CIAUX, EXEMPLAIRES OU CONS�CUTIFS
  * (Y COMPRIS, MAIS SANS S'Y LIMITER, L'ACQUISITION DE BIENS OU DE SERVICES DE SUBSTITUTION ; LA PERTE D'UTILISATION, DE DONN�ES OU DE PROFITS ; OU L'INTERRUPTION D'ACTIVIT�) TOUTEFOIS CAUS�S ET SUR TOUTE TH�ORIE DE RESPONSABILIT�, QU'ELLE SOIT CONTRACTUELLE, STRICTE OU D�LICTUELLE
  * (Y COMPRIS LA N�GLIGENCE OU AUTRE) D�COULANT DE TOUTE UTILISATION DE CE LOGICIEL, M�ME SI
  * CONSEILL� DE LA POSSIBILIT� DE TELS DOMMAGES.
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

/* Variables priv�es ---------------------------------------------------------*/
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

/* D�finir les alias CAN */
#define CAN_TX   &hcan1
#define CAN_RX   &hcan1
/* USER CODE BEGIN PV */
/* Variables priv�es ---------------------------------------------------------*/

/* USER CODE END PV */

/* Prototypes de fonctions priv�es -------------------------------------------*/
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
 * @brief Fonction de test du Flash, d�finissant des protections en lecture et �criture.
 *        Cette fonction active la protection en lecture/�criture sur les secteurs sp�cifi�s
 *        de la m�moire Flash du microcontr�leur.
 */
void flash_testing(void)
{
    uint8_t protection_mode = 2;  // Mode de protection (1 = �criture seule, 2 = Lecture/�criture)
    uint8_t sector_details = 0x80;  // Masque des secteurs � prot�ger (exemple : secteur 7)

    // Adresse du registre de contr�le des options de Flash (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

    // 1. D�verrouillage des options de configuration de la Flash
    HAL_FLASH_OB_Unlock();

    // 2. Attendre que l'interface Flash ne soit plus occup�e (bit BSY)
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

    // 3. Configurer la protection en fonction du mode s�lectionn�
    if (protection_mode == 1)  // Protection en �criture seule
    {
        // D�sactiver la protection en lecture
        *pOPTCR &= ~(1 << 31);
        // Configurer les secteurs � prot�ger en �criture
        *pOPTCR &= ~(0xff << 16);  // Effacer les anciens bits de protection
        *pOPTCR |= (sector_details << 16);  // Appliquer la protection d'�criture
    }
    else if (protection_mode == 2)  // Protection en lecture/�criture
    {
        // Activer la protection en lecture et en �criture
        *pOPTCR |= (1 << 31);  // Activer le bit de protection lecture/�criture
        *pOPTCR &= ~(0xff << 16);  // Effacer les anciens bits de protection
        *pOPTCR |= (sector_details << 16);  // Appliquer la protection lecture/�criture
    }

    // 4. D�marrer l'op�ration en d�finissant le bit OPTSTRT (bit 1)
    *pOPTCR |= (1 << 1);

    // 5. Attendre que l'interface Flash ne soit plus occup�e
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

    // 6. Verrouiller les options de configuration de la Flash apr�s l'op�ration
    HAL_FLASH_OB_Lock();
}


/**
 * @brief Fonction principale du programme.
 */
int main(void)
{
  /* R�initialisation de tous les p�riph�riques, initialisation de l'interface Flash et du Systick */
  HAL_Init();

  /* Configurer l'horloge du syst�me */
  SystemClock_Config();

  /* Initialiser tous les p�riph�riques configur�s */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_CAN1_Init();  // Initialiser le CAN au lieu de l'UART

  /*
   * V�rifier l'�tat du bouton utilisateur pour d�terminer s'il faut rester en mode bootloader ou
   * sauter � l'application utilisateur :
   * - Si le bouton est press�, le bootloader reste actif et attend des commandes via CAN.
   * - Sinon, le bootloader transf�re le contr�le � l'application utilisateur stock�e dans la m�moire Flash.
   */
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
  {
      printmsg_can("BL_DEBUG_MSG:Le bouton est press�... passage en mode BL\n\r");
      bootloader_can_read_data();  // Rester en mode bootloader et attendre les commandes via CAN.
  }
  else
  {
      printmsg_can("BL_DEBUG_MSG:Le bouton n'est pas press�... ex�cution de l'application utilisateur\n");
      bootloader_jump_to_user_app();  // Transf�rer le contr�le � l'application utilisateur.
  }
}


/**
 * @brief Fonction pour lire les commandes envoy�es par l'h�te via l'interface CAN.
 * Cette fonction attend continuellement les commandes entrantes, les traite et
 * appelle la fonction de gestion de commande correspondante en fonction du code de commande.
 */
void bootloader_can_read_data(void)
{
    CAN_RxHeaderTypeDef rxHeader;  // Structure pour stocker l'en-t�te des messages CAN re�us
    uint8_t rcv_len = 0;           // Variable pour stocker la longueur du paquet de commande re�u
    uint8_t can_rx_buffer[8];      // Tampon de r�ception pour les donn�es CAN

    while (1)
    {
        // R�initialiser le tampon de r�ception pour effacer les anciennes donn�es
        memset(bl_rx_buffer, 0, 200);

        // Attendre un message CAN et lire les donn�es
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, can_rx_buffer) == HAL_OK)
        {
            // La premi�re donn�e contient la longueur du paquet de commande
            rcv_len = can_rx_buffer[0];

            // Copier les donn�es re�ues dans le tampon de r�ception principal
            memcpy(bl_rx_buffer, can_rx_buffer, rcv_len);

            // D�coder la commande envoy�e par l'h�te, qui se trouve � l'index 1 du tampon
            switch (bl_rx_buffer[1])
            {
                case BL_GET_VER:
                    bootloader_handle_getver_cmd(bl_rx_buffer);  // G�rer la commande 'GET Version'
                    break;
                case BL_GET_HELP:
                    bootloader_handle_gethelp_cmd(bl_rx_buffer);  // G�rer la commande 'GET Help'
                    break;
                case BL_GET_CID:
                    bootloader_handle_getcid_cmd(bl_rx_buffer);  // G�rer la commande 'GET Chip ID'
                    break;
                case BL_GET_RDP_STATUS:
                    bootloader_handle_getrdp_cmd(bl_rx_buffer);  // G�rer la commande 'GET Read Protection Status'
                    break;
                case BL_GO_TO_ADDR:
                    bootloader_handle_go_cmd(bl_rx_buffer);  // G�rer la commande 'Go to Address'
                    break;
                case BL_FLASH_ERASE:
                    bootloader_handle_flash_erase_cmd(bl_rx_buffer);  // G�rer la commande 'Flash Erase'
                    break;
                case BL_MEM_WRITE:
                    bootloader_handle_mem_write_cmd(bl_rx_buffer);  // G�rer la commande 'Memory Write'
                    break;
                case BL_EN_RW_PROTECT:
                    bootloader_handle_en_rw_protect(bl_rx_buffer);  // G�rer la commande 'Enable Read/Write Protect'
                    break;
                case BL_MEM_READ:
                    bootloader_handle_mem_read(bl_rx_buffer);  // G�rer la commande 'Memory Read'
                    break;
                case BL_READ_SECTOR_P_STATUS:
                    bootloader_handle_read_sector_protection_status(bl_rx_buffer);  // G�rer la commande 'Read Sector Protection Status'
                    break;
                case BL_OTP_READ:
                    bootloader_handle_read_otp(bl_rx_buffer);  // G�rer la commande 'Read OTP Memory'
                    break;
                case BL_DIS_R_W_PROTECT:
                    bootloader_handle_dis_rw_protect(bl_rx_buffer);  // G�rer la commande 'Disable Read/Write Protection'
                    break;
                default:
                    printmsg_can("BL_DEBUG_MSG:Code de commande invalide re�u de l'h�te\n");  // Imprimer un message d'erreur pour une commande invalide
                    break;
            }
        }
    }
}


/**
 * @brief Fonction pour sauter � l'application utilisateur stock�e dans la m�moire flash.
 * Cette fonction suppose que l'application utilisateur est stock�e � une adresse sp�cifique,
 * d�finie par `FLASH_SECTOR2_BASE_ADDRESS`. Elle lit le MSP (Main Stack Pointer) et
 * l'adresse du Reset Handler � partir de cet emplacement et transf�re le contr�le � l'application utilisateur.
 */
void bootloader_jump_to_user_app(void)
{
    /* 1. Ce pointeur est utilis� pour stocker l'adresse du Reset Handler de l'application utilisateur.
     * Le reset handler est la premi�re fonction ex�cut�e lors du d�marrage de l'application.
    */
    void (*app_reset_handler)(void);

    // 2. Message de d�bogage : Ce message est envoy� via CAN pour indiquer que l'on est entr� dans la fonction 'bootloader_jump_to_user_app'
    printmsg_can("BL_DEBUG_MSG:bootloader_jump_to_user_app\n");

    /* 3. Lecture de la valeur du Main Stack Pointer (MSP):
     * Cette ligne lit la valeur du MSP stock�e � la premi�re adresse du secteur de m�moire o� l'application utilisateur est enregistr�e.
     * Le MSP indique o� la pile principale doit commencer.
    */
    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;

    // 4. Message de d�bogage pour la valeur du MSP
    printmsg_can("BL_DEBUG_MSG:MSP value : %#x\n", msp_value);

    // 5. Configuration du MSP : on utilise une fonction CMSIS (__set_MSP) pour d�finir
    // la valeur du MSP.
    __set_MSP(msp_value);

    // 6. Lecture de l'adresse du Reset Handler, situ�e apr�s l'adresse du MSP (offset +4 de l'adresse de base)
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    // 7. Initialisation du pointeur de fonction
    app_reset_handler = (void*) resethandler_address;

    // 8. Message de d�bogage pour l'adresse du reset handler
    printmsg_can("BL_DEBUG_MSG: app reset handler addr : %#x\n", app_reset_handler);

    // 9. Cette fonction fait un saut vers l'application utilisateur, une fois cette ligne ex�cut�e, le bootloader transf�re le contr�le � l'application utilisateur
    app_reset_handler();
}

/**
 * @brief Imprime des messages de d�bogage format�s via l'interface CAN.
 * Cette fonction est utilis�e pour imprimer des messages de d�bogage via l'interface CAN,
 * ce qui permet de surveiller les activit�s du bootloader.
 *
 * @param format: La cha�ne de format pour le message � imprimer.
 */
void printmsg_can(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
    char str[80];  // Tampon pour stocker la cha�ne de message format�e

    // Extraire la liste des arguments en utilisant les macros VA (Variable Argument)
    va_list args;
    va_start(args, format);

    // Formater la cha�ne en fonction du format et des arguments fournis
    vsprintf(str, format, args);

    // Transmettre la cha�ne format�e via l'interface CAN (simulation via HAL_CAN_AddTxMessage)
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t can_data[8];  // Un message CAN ne peut transporter que 8 octets, donc nous envoyons le message en plusieurs morceaux

    // Configurer l'en-t�te du message CAN
    txHeader.StdId = 0x65D;  // Identifiant standard pour les messages de d�bogage
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 8;  // Taille des donn�es � envoyer

    for (int i = 0; i < strlen(str); i += 8)
    {
        // Copier 8 octets de la cha�ne dans le tampon can_data
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
 * @brief Configure l'horloge du syst�me pour le microcontr�leur.
 *
 * Cette fonction configure l'oscillateur interne � haute vitesse (HSI) et les param�tres de la PLL
 * pour g�n�rer une source d'horloge stable pour le CPU et les p�riph�riques.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;    // Structure pour la configuration des oscillateurs
    RCC_ClkInitTypeDef RCC_ClkInitStruct;    // Structure pour la configuration de l'horloge

    /** Activer l'horloge de contr�le de l'alimentation pour la r�gulation de la tension */
    __HAL_RCC_PWR_CLK_ENABLE();

    /** Configurer la tension de sortie du r�gulateur principal */
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
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Utiliser la sortie de la PLL comme horloge syst�me
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // Diviseur de l'horloge AHB
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // Diviseur de l'horloge APB1
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // Diviseur de l'horloge APB2

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);                    // Gestion d'erreur de configuration
    }

    /** Configurer le temps d'interruption du Systick */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);          // Configurer le SysTick pour g�n�rer des interruptions toutes les 1 ms

    /** Configurer la source d'horloge du Systick */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);       // Utiliser HCLK comme source d'horloge SysTick

    /* D�finir la priorit� de l'interruption SysTick � 0 (priorit� la plus �lev�e) */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
 * @brief  Initialise le p�riph�rique CRC.
 *
 * Cette fonction configure et initialise le p�riph�rique CRC (Cyclic Redundancy Check),
 * qui est utilis� pour calculer les valeurs CRC afin de v�rifier l'int�grit� des donn�es dans le bootloader.
 */
static void MX_CRC_Init(void)
{
    hcrc.Instance = CRC;               // S�lectionner l'instance du p�riph�rique CRC
    if (HAL_CRC_Init(&hcrc) != HAL_OK)  // Initialiser le p�riph�rique CRC
    {
        _Error_Handler(__FILE__, __LINE__);  // Gestion d'erreur lors de l'initialisation
    }
}

/**
 * @brief  Initialise le p�riph�rique CAN1 pour la communication CAN en mode loopback.
 *
 * Cette fonction configure les param�tres pour l'interface CAN1 en mode loopback,
 * y compris le d�bit en bauds, les modes et filtres de r�ception.
 */
static void MX_CAN1_Init(void)
{
    CAN_HandleTypeDef hcan1;

    hcan1.Instance = CAN1;                                // S�lectionner l'instance du p�riph�rique CAN1
    hcan1.Init.Prescaler = 16;                            // Configurer le prescaler pour la vitesse de communication
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;                  // Configurer le CAN en mode loopback
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;               // Largeur de saut de synchronisation
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;                   // Configuration du segment de temps 1
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;                    // Configuration du segment de temps 2
    hcan1.Init.TimeTriggeredMode = DISABLE;               // Mode d�clench� par le temps d�sactiv�
    hcan1.Init.AutoBusOff = DISABLE;                      // D�sactiver la gestion automatique de l'arr�t du bus
    hcan1.Init.AutoWakeUp = DISABLE;                      // D�sactiver le r�veil automatique
    hcan1.Init.AutoRetransmission = ENABLE;               // Activer la retransmission automatique
    hcan1.Init.ReceiveFifoLocked = DISABLE;               // D�verrouiller le FIFO de r�ception
    hcan1.Init.TransmitFifoPriority = DISABLE;            // D�sactiver la priorit� du FIFO de transmission

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
    sFilterConfig.SlaveStartFilterBank = 14;              // Configurer le filtre de d�part esclave

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);               // Gestion d'erreur lors de la configuration du filtre
    }
}



/**
 * @brief Configure les broches GPIO comme Analog, Input, Output, EVENT_OUT, ou EXTI.
 *
 * Cette fonction configure les broches GPIO pour diff�rentes fonctions comme entr�e num�rique, sortie,
 * fonctions analogiques, sources d'interruptions externes (EXTI), ou autres configurations personnalis�es.
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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       // Vitesse tr�s �lev�e
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;               // Fonction alternative CAN1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                  // Initialiser les broches GPIOA pour CAN1

    /* Configurer le niveau de sortie GPIO pour LD2 (LED embarqu�e) */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  // D�finir la broche LED2 � un �tat bas

    /* Configurer la broche GPIO : B1_Pin (bouton utilisateur) */
    GPIO_InitStruct.Pin = B1_Pin;                // S�lectionner la broche B1
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Configurer comme interruption externe sur front descendant
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // Pas de r�sistances pull-up ou pull-down
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);  // Initialiser la configuration de la broche

    /* Configurer la broche GPIO : LD2_Pin (LED) */
    GPIO_InitStruct.Pin = LD2_Pin;               // S�lectionner la broche LD2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Configurer comme sortie push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // Pas de r�sistances pull-up ou pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Vitesse de sortie basse
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);  // Initialiser la configuration de la broche
}

/**
 * @brief Fonction ex�cut�e en cas d'erreur.
 *
 * Cette fonction est utilis�e pour signaler des erreurs ou des comportements inattendus.
 * Lorsqu'elle est appel�e, elle entre dans une boucle infinie, arr�tant l'ex�cution du code.
 * Les utilisateurs peuvent personnaliser cette fonction pour g�rer les erreurs plus gracieusement.
 *
 * @param file: Nom du fichier source o� l'erreur s'est produite.
 * @param line: Num�ro de ligne dans le fichier source o� l'erreur s'est produite.
 */
void _Error_Handler(char * file, int line)
{
  while(1)
  {
      // Boucle infinie pour indiquer l'�tat d'erreur
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  // Clignoter la LED pour indiquer une erreur
      HAL_Delay(500);                              // D�lai de 500 ms
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Signale le nom du fichier source et le num�ro de ligne
 *        o� l'erreur assert_param s'est produite.
 *
 * Cette fonction est utilis�e par la macro `assert_param` pour signaler des erreurs lorsque
 * une valeur de param�tre est invalide. Elle imprime le fichier source et le num�ro de ligne
 * o� l'erreur a �t� d�tect�e.
 *
 * @param file: Pointeur vers le nom du fichier source.
 * @param line: Num�ro de ligne o� l'erreur assert_param a �t� d�clench�e.
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  // L'utilisateur peut impl�menter ici un rapport d'erreur personnalis�
  // par exemple l'impression d'un message d'erreur sur le terminal s�rie ou un log
  printf("Erreur dans le fichier %s � la ligne %d\n", file, line);
}

#endif


/************** Impl�mentation des fonctions de gestion des commandes Bootloader *********/

/**
 * @brief G�re la commande 'Get Version' (BL_GET_VER) envoy�e par l'h�te via CAN.
 *
 * Cette fonction est utilis�e pour r�pondre � l'h�te avec la version actuelle du bootloader.
 * Elle lit le paquet de commande, v�rifie son int�grit� en utilisant le CRC,
 * et envoie la version du bootloader si le test CRC est r�ussi. Sinon, elle r�pond avec un NACK.
 *
 * @param bl_rx_buffer: Pointeur vers le tampon de commande re�u contenant le paquet de commande.
 */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;  // Variable pour stocker la version du bootloader

    // 1) Imprimer un message de d�bogage indiquant l'entr�e dans la fonction.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_getver_cmd\n");

    // Calculer la longueur totale du paquet de commande re�u.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoy� par l'h�te � la fin du paquet de commande.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier l'int�grit� du paquet re�u en calculant le CRC local
    // et en le comparant avec celui re�u. Si bootloader_verify_crc() retourne 0, le CRC est valide.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Si le CRC est correct, imprimer un message indiquant la r�ussite de la v�rification du checksum.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN.
        bootloader_send_ack(bl_rx_buffer[0], 1);

        // R�cup�rer la version actuelle du bootloader en utilisant la fonction get_bootloader_version().
        bl_version = get_bootloader_version(); // R�cup�rer la version du bootloader.

        // Imprimer la version du bootloader dans le terminal de d�bogage.
        printmsg_can("BL_DEBUG_MSG:BL_VER : %d %#x\n", bl_version, bl_version);

        // Envoyer la version du bootloader � l'h�te via l'interface CAN.
        bootloader_can_write_data(&bl_version, 1);  // Envoyer la r�ponse � l'h�te via CAN.
    }
    else
    {
        // Si le CRC est incorrect, imprimer un message indiquant un �chec de la v�rification du checksum.
      printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accus� de r�ception n�gatif (NACK) � l'h�te via CAN.
        bootloader_send_nack();
    }
}

/**
 * @brief G�re la commande 'Get Help' (BL_GET_HELP) envoy�e par l'h�te via CAN.
 *
 * Cette commande informe l'h�te de toutes les commandes support�es par le bootloader.
 * Lorsqu'elle est appel�e, cette fonction renvoie la liste de toutes les commandes support�es � l'h�te.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u de l'h�te.
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
    // Imprimer un message de d�bogage indiquant que la commande 'Get Help' est en cours de traitement.
  printmsg_can("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoy� par l'h�te.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier la validit� du CRC32 pour garantir que les donn�es n'ont pas �t� alt�r�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Si le CRC est valide, envoyer un message de succ�s pour la v�rification du checksum.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) avec le nombre d'octets que le bootloader enverra en r�ponse via CAN.
        bootloader_send_ack(pBuffer[0], sizeof(supported_commands));

        // Envoyer la liste des commandes support�es � l'h�te via l'interface CAN.
        bootloader_can_write_data(supported_commands, sizeof(supported_commands));
    }
    else
    {
        // Si le CRC est incorrect, envoyer un message d'erreur de v�rification.
      printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accus� de r�ception n�gatif (NACK) via CAN.
        bootloader_send_nack();
    }
}


/**
 * @brief G�re la commande 'Get Chip ID' (BL_GET_CID) envoy�e par l'h�te via CAN.
 *
 * Cette commande est utilis�e pour lire l'identifiant unique de la puce du microcontr�leur.
 * L'ID de la puce permet � l'h�te de d�terminer le num�ro de pi�ce ST et la r�vision du silicium du p�riph�rique.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u de l'h�te.
 */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
    uint16_t bl_cid_num = 0;  // Variable pour stocker l'ID unique de la puce du microcontr�leur.

    // Imprimer un message de d�bogage indiquant que la commande est en cours de traitement.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoy� par l'h�te.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier la validit� du CRC32.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Si le CRC est valide, imprimer un message indiquant la r�ussite de la v�rification du checksum.
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) avec la longueur de la r�ponse (2 octets pour l'ID de la puce) via CAN.
        bootloader_send_ack(pBuffer[0], 2);

        // R�cup�rer l'ID de la puce du microcontr�leur.
        bl_cid_num = get_mcu_chip_id();
        printmsg_can("BL_DEBUG_MSG:MCU id : %d %#x !!\n", bl_cid_num, bl_cid_num);

        // Envoyer l'ID de la puce (2 octets) � l'h�te via CAN.
        bootloader_can_write_data((uint8_t *)&bl_cid_num, 2);
    }
    else
    {
        // Si le CRC n'est pas valide, imprimer un message indiquant un �chec de v�rification.
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accus� de r�ception n�gatif (NACK) � l'h�te via CAN.
        bootloader_send_nack();
    }
}

/**
 * @brief G�re la commande 'Get Read Protection Status' (BL_GET_RDP_STATUS) envoy�e par l'h�te via CAN.
 *
 * Cette fonction r�cup�re le niveau de protection en lecture (RDP) de la m�moire Flash du microcontr�leur
 * et envoie cette information au syst�me h�te via l'interface CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u de l'h�te.
 */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    // Variable pour stocker le niveau de protection en lecture (RDP) de la m�moire Flash.
    uint8_t rdp_level = 0x00;

    // Imprimer un message de d�bogage indiquant l'entr�e dans la fonction de gestion de la commande RDP.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoy� par l'h�te.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour s'assurer de l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Imprimer un message de d�bogage indiquant que le CRC est valide.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) avec la longueur de la r�ponse (1 octet pour le niveau RDP) via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // R�cup�rer le niveau actuel de protection en lecture de la m�moire Flash.
        rdp_level = get_flash_rdp_level();

        // Imprimer le niveau de protection RDP dans le terminal de d�bogage.
        printmsg_can("BL_DEBUG_MSG:RDP level: %d %#x\n", rdp_level, rdp_level);

        // Envoyer le niveau de protection RDP � l'h�te via CAN.
        bootloader_can_write_data(&rdp_level, 1);
    }
    else
    {
        // Si le CRC est incorrect, imprimer un message d'erreur.
      printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accus� de r�ception n�gatif (NACK) � l'h�te via CAN.
        bootloader_send_nack();
    }
}

/**
 * @brief G�re la commande 'Go to Address' (BL_GO_TO_ADDR) envoy�e par l'h�te via CAN.
 *
 * Cette commande est utilis�e pour sauter � une adresse sp�cifi�e par l'h�te.
 * La fonction v�rifie l'int�grit� du paquet de commande via le CRC, valide l'adresse,
 * et si tout est correct, elle saute � l'adresse sp�cifi�e.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u de l'h�te.
 */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    // Variable pour stocker l'adresse vers laquelle le bootloader doit sauter.
    uint32_t go_address = 0;

    // Variables pour indiquer si l'adresse est valide ou non.
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;

    // Imprimer un message de d�bogage indiquant l'entr�e dans la fonction de gestion de la commande GO.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_go_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoy� par l'h�te.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour s'assurer de l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Imprimer un message indiquant que le CRC est valide.
      printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN pour confirmer la r�ception correcte de la commande.
        bootloader_send_ack(pBuffer[0], 1);

        // Extraire l'adresse vers laquelle l'h�te souhaite que le bootloader saute.
        go_address = *((uint32_t *)&pBuffer[2]);
        printmsg_can("BL_DEBUG_MSG:GO addr: %#x\n", go_address);

        // V�rifier si l'adresse extraite est valide pour l'ex�cution.
        if (verify_address(go_address) == ADDR_VALID)
        {
            // Indiquer � l'h�te que l'adresse est valide via CAN.
            bootloader_can_write_data(&addr_valid, 1);

            // Ajouter 1 � l'adresse pour d�finir le bit Thumb (T bit) � 1, n�cessaire pour les processeurs ARM Cortex M.
            go_address += 1;

            // D�clarer un pointeur de fonction pour sauter � l'adresse sp�cifi�e.
            void (*lets_jump)(void) = (void *)go_address;

            // Imprimer un message indiquant que le bootloader saute � l'adresse sp�cifi�e.
            printmsg_can("BL_DEBUG_MSG: jumping to go address! \n");

            // Sauter � l'adresse sp�cifi�e et ex�cuter le code � cet emplacement.
            lets_jump();
        }
        else
        {
            // Si l'adresse n'est pas valide, imprimer un message d'erreur.
            printmsg_can("BL_DEBUG_MSG:GO addr invalid ! \n");

            // Indiquer � l'h�te que l'adresse est invalide via CAN.
            bootloader_can_write_data(&addr_invalid, 1);
        }
    }
    else
    {
        // Si le CRC est incorrect, imprimer un message d'erreur.
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accus� de r�ception n�gatif (NACK) via CAN � l'h�te.
        bootloader_send_nack();
    }
}


/**
 * @brief G�re la commande 'Flash Erase' (BL_FLASH_ERASE) envoy�e par l'h�te via CAN.
 *
 * Cette fonction traite la commande 'Flash Erase' re�ue de l'h�te.
 * Elle v�rifie l'int�grit� du paquet de commande via le CRC, extrait les d�tails de l'effacement,
 * effectue l'op�ration d'effacement de la m�moire Flash, et envoie le statut � l'h�te via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande.
 */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;  // Variable pour capturer le statut de l'effacement.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoy�e par l'h�te.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour garantir l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Extraire les informations d'effacement : secteur initial et nombre de secteurs.
        printmsg_can("BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d\n", pBuffer[2], pBuffer[3]);

        // Allumer une LED pendant l'op�ration d'effacement pour indiquer que l'op�ration est en cours.
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
        erase_status = execute_flash_erase(pBuffer[2], pBuffer[3]);  // Appeler la fonction d'effacement de la Flash.
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);  // �teindre la LED apr�s l'effacement.

        // Afficher le statut de l'effacement.
        printmsg_can("BL_DEBUG_MSG: flash erase status: %#x\n", erase_status);

        // Envoyer le statut de l'effacement � l'h�te via CAN.
        bootloader_can_write_data(&erase_status, 1);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");
        // Si le CRC est invalide, envoyer un NACK � l'h�te via CAN.
        bootloader_send_nack();
    }
}



/**
 * @brief G�re la commande 'Memory Write' (BL_MEM_WRITE) envoy�e par l'h�te via CAN.
 *
 * Cette fonction traite la commande 'Memory Write' re�ue de l'h�te.
 * Elle v�rifie le CRC du paquet de commande, v�rifie si l'adresse m�moire est valide,
 * effectue l'op�ration d'�criture, et envoie le statut de l'�criture � l'h�te via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u.
 */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
    // Initialisation des variables de statut.
    // uint8_t addr_valid = ADDR_VALID;  // Indicateur si l'adresse est valide.
    uint8_t write_status = 0x00;      // Statut de l'op�ration d'�criture.
    uint8_t chksum = 0;
    uint8_t len = 0;      // Variables pour la longueur et la v�rification du checksum.

    // Lire la longueur du paquet de commande.
    len = pBuffer[0];

    // Longueur de la charge utile (les donn�es � �crire).
    uint8_t payload_len = pBuffer[6];

    // Extraire l'adresse m�moire o� l'�criture doit commencer.
    uint32_t mem_address = *((uint32_t *) (&pBuffer[2]));

    // Extraire le checksum envoy� par l'h�te pour v�rifier l'int�grit� des donn�es.
    if (chksum != pBuffer[len]) {
        // Gestion de l'erreur de checksum
        printmsg_can("Erreur de checksum\n");
    }


    // Imprimer un message de d�bogage indiquant l'ex�cution de la commande.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire le CRC32 envoy� par l'h�te pour v�rification.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour garantir l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Imprimer l'adresse m�moire cible pour l'op�ration d'�criture.
        printmsg_can("BL_DEBUG_MSG: mem write address : %#x\n", mem_address);

        // V�rifier si l'adresse m�moire cible est valide.
        if (verify_address(mem_address) == ADDR_VALID)
        {
            printmsg_can("BL_DEBUG_MSG: valid mem write address\n");

            // Allumer la LED pour indiquer que l'op�ration d'�criture est en cours.
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            // Effectuer l'op�ration d'�criture en m�moire.
            write_status = execute_mem_write(&pBuffer[7], mem_address, payload_len);

            // �teindre la LED pour indiquer la fin de l'op�ration d'�criture.
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

            // Envoyer le statut de l'�criture � l'h�te (succ�s ou �chec) via CAN.
            bootloader_can_write_data(&write_status, 1);
        }
        else
        {
            printmsg_can("BL_DEBUG_MSG: invalid mem write address\n");
            write_status = ADDR_INVALID;

            // Envoyer le statut d'adresse invalide � l'h�te via CAN.
            bootloader_can_write_data(&write_status, 1);
        }
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un NACK si la v�rification du CRC �choue via CAN.
        bootloader_send_nack();
    }
}


/**
 * @brief G�re la commande 'Enable/Disable Read/Write Protection' (BL_EN_RW_PROTECT) envoy�e par l'h�te via CAN.
 *
 * Cette fonction traite la commande pour activer ou d�sactiver la protection en lecture/�criture sur les secteurs de la Flash.
 * Elle v�rifie le CRC du paquet de commande re�u, applique la protection souhait�e,
 * et envoie le statut � l'h�te via le bus CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u.
 */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;  // Variable pour stocker le statut de protection.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\n");  // Message de d�bogage indiquant le traitement de la commande.

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoy�e par l'h�te.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour garantir l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");  // Le CRC est valide, poursuivre le traitement.

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // Activer ou d�sactiver la protection selon les param�tres de la commande.
        status = configure_flash_sector_rw_protection(pBuffer[2], pBuffer[3], 0);

        printmsg_can("BL_DEBUG_MSG: flash erase status: %#x\n", status);

        // Envoyer le statut de protection � l'h�te via CAN.
        bootloader_can_write_data(&status, 1);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");  // Le CRC est invalide, envoyer un NACK via CAN.
        bootloader_send_nack();
    }
}


/**
 * @brief G�re la commande 'Disable Read/Write Protection' (BL_DIS_RW_PROTECT) envoy�e par l'h�te via CAN.
 *
 * Cette fonction traite la commande pour d�sactiver la protection en lecture/�criture sur les secteurs de la Flash.
 * Elle v�rifie le CRC du paquet de commande re�u, d�sactive la protection,
 * et envoie le statut � l'h�te via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u.
 */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoy�e par l'h�te.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour garantir l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN.
        bootloader_send_ack(pBuffer[0], 1);

        // D�sactiver la protection en lecture/�criture.
        status = configure_flash_sector_rw_protection(0, 0, 1);

        printmsg_can("BL_DEBUG_MSG: flash erase status: %#x\n", status);

        // Envoyer le statut de d�sactivation de la protection � l'h�te via CAN.
        bootloader_can_write_data(&status, 1);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un accus� de r�ception n�gatif (NACK) via CAN si la v�rification du CRC �choue.
        bootloader_send_nack();
    }
}


/**
 * @brief G�re la commande 'Memory Read' (BL_MEM_READ) envoy�e par l'h�te via CAN.
 *
 * Cette fonction est un espace r�serv� pour l'impl�mentation de la fonctionnalit� de lecture m�moire
 * afin de lire une adresse m�moire sp�cifi�e et renvoyer les donn�es � l'h�te via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u.
 */
void bootloader_handle_mem_read(uint8_t *pBuffer)
{
    // � impl�menter dans le futur pour la lecture du contenu de la m�moire via CAN.
}

/**
 * @brief G�re la commande 'Read Sector Protection Status' (BL_READ_SECTOR_P_STATUS) envoy�e par l'h�te via CAN.
 *
 * Cette fonction lit l'�tat de la protection en lecture/�criture des secteurs.
 * Elle v�rifie le CRC du paquet de commande re�u, r�cup�re l'�tat de protection,
 * et l'envoie � l'h�te via CAN.
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u.
 */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
    uint16_t status;  // Variable pour stocker l'�tat de protection.
    printmsg_can("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n");

    // Calculer la longueur totale du paquet de commande.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extraire la valeur CRC32 envoy�e par l'h�te.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // V�rifier le CRC pour garantir l'int�grit� des donn�es.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg_can("BL_DEBUG_MSG:checksum success !!\n");

        // Envoyer un accus� de r�ception (ACK) � l'h�te via CAN.
        bootloader_send_ack(pBuffer[0], 2);

        // Lire l'�tat de protection depuis les Bytes d'Option (OB).
        status = read_OB_rw_protection_status();

        printmsg_can("BL_DEBUG_MSG: nWRP status: %#x\n", status);

        // Envoyer l'�tat de protection en lecture/�criture � l'h�te via CAN.
        bootloader_can_write_data((uint8_t *)&status, 2);
    }
    else
    {
        printmsg_can("BL_DEBUG_MSG:checksum fail !!\n");

        // Envoyer un NACK � l'h�te via CAN en cas d'�chec de la v�rification du CRC.
        bootloader_send_nack();
    }
}


/**
 * @brief G�re la commande 'Read OTP Memory' (BL_OTP_READ) envoy�e par l'h�te via CAN.
 *
 * Cette fonction est un espace r�serv� pour l'impl�mentation de la fonctionnalit� de lecture de la m�moire OTP (One-Time Programmable).
 *
 * @param pBuffer: Pointeur vers le tampon contenant le paquet de commande re�u via CAN.
 */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{
    // � impl�menter dans le futur pour la lecture du contenu de la m�moire OTP via CAN.
}

/**
 * @brief Envoie un message d'accus� de r�ception (ACK) avec une longueur de donn�es de suivi via CAN.
 *
 * Cette fonction est utilis�e pour indiquer que la commande envoy�e par l'h�te a �t� correctement
 * re�ue et trait�e par le bootloader via CAN.
 *
 * @param command_code: Le code de la commande pour laquelle l'ACK est envoy�.
 * @param follow_len: Longueur des donn�es de suivi � envoyer.
 */
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
    // On envoie ACK + Length (2 octets)
    uint8_t ack_buf[2];

    ack_buf[0] = BL_ACK;       // Code ACK
    ack_buf[1] = follow_len;   // Longueur des donn�es de suivi

    // Envoyer le message ACK via CAN
    bootloader_can_transmit(ack_buf, 2);  // Envoyer via l'interface CAN
}

/**
 * @brief Envoie un message de Non Accus� de R�ception (NACK) via CAN.
 *
 * Cette fonction est utilis�e pour indiquer que le CRC de la commande envoy�e par l'h�te est incorrect
 * ou que la commande n'a pas pu �tre trait�e.
 */
void bootloader_send_nack(void)
{
    // On envoie 1 octet NACK
    uint8_t nack = BL_NACK;

    // Envoyer le message NACK via CAN
    bootloader_can_transmit(&nack, 1);  // Envoyer via l'interface CAN
}

/**
 * @brief V�rifie le CRC (Cyclic Redundancy Check) d'un tampon de donn�es re�u via CAN.
 *
 * Cette fonction prend en entr�e un pointeur vers les donn�es (`pData`), la longueur des donn�es (`len`),
 * et la valeur CRC envoy�e par l'h�te (`crc_host`).
 * Si le CRC calcul� correspond au CRC de l'h�te, elle renvoie `VERIFY_CRC_SUCCESS`.
 * Sinon, elle renvoie `VERIFY_CRC_FAIL`.
 *
 * @param pData: Pointeur vers le tampon de donn�es � v�rifier.
 * @param len: Longueur des donn�es � traiter.
 * @param crc_host: Valeur CRC re�ue de l'h�te.
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

    __HAL_CRC_DR_RESET(&hcrc);  // R�initialiser l'unit� de calcul du CRC

    return (uwCRCValue == crc_host) ? VERIFY_CRC_SUCCESS : VERIFY_CRC_FAIL;
}


/**
 * @brief Envoie des donn�es au terminal s�rie via UART.
 * Cette fonction utilise l'API HAL_UART_Transmit pour envoyer des donn�es via l'UART sp�cifi�e.
 *
 * @param pBuffer: Pointeur vers le tampon contenant les donn�es � transmettre.
 * @param len: Longueur des donn�es � transmettre (en octets).
 */
/**
 * @brief Envoie des donn�es via CAN.
 * Cette fonction utilise l'API HAL_CAN_AddTxMessage pour envoyer des donn�es via l'interface CAN sp�cifi�e.
 *
 * @param pBuffer: Pointeur vers le tampon contenant les donn�es � transmettre.
 * @param len: Longueur des donn�es � transmettre (en octets).
 */
void bootloader_can_write_data(uint8_t *pBuffer, uint32_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    // Configurer l'en-t�te du message CAN
    txHeader.DLC = len;          // Longueur du message (Data Length Code)
    txHeader.StdId = 0x65D;      // Identifiant standard (modifiable selon votre syst�me)
    txHeader.IDE = CAN_ID_STD;   // Identifiant standard
    txHeader.RTR = CAN_RTR_DATA; // Type de trame (trame de donn�es)
    txHeader.TransmitGlobalTime = DISABLE;  // D�sactiver la temporisation globale

    // Envoyer les donn�es via CAN
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

    // Configuration de l'en-t�te CAN
    txHeader.DLC = len;          // Longueur des donn�es (0-8 octets)
    txHeader.StdId = 0x65D;      // Identifiant CAN (modifiable selon votre syst�me)
    txHeader.IDE = CAN_ID_STD;   // Format standard d'identifiant
    txHeader.RTR = CAN_RTR_DATA; // Trame de donn�es
    txHeader.TransmitGlobalTime = DISABLE;

    // D�coupage des donn�es en blocs de 8 octets (taille max d'une trame CAN)
    for (uint32_t i = 0; i < len; i += 8)
    {
        // Copier jusqu'� 8 octets dans can_data
        memcpy(can_data, &pData[i], (len - i > 8) ? 8 : (len - i));

        // Ajouter le message CAN � la file de transmission
        if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, can_data, &txMailbox) != HAL_OK)
        {
            printmsg_can("BL_DEBUG_MSG: CAN transmission failed\n");
            break;
        }

        // Attendre que le message soit envoy�
        while (HAL_CAN_IsTxMessagePending(&hcan1, txMailbox));
    }
}


/**
 * @brief Renvoie la version du bootloader.
 * Cette fonction renvoie simplement la valeur d�finie dans la macro `BL_VERSION`.
 *
 * @retval La version du bootloader sous forme d'entier 8 bits.
 */
uint8_t get_bootloader_version(void)
{
    // Retourner la version du bootloader d�finie par la macro `BL_VERSION`.
    return (uint8_t)BL_VERSION;
}

/**
 * @brief Lit l'identifiant unique de la puce du microcontr�leur.
 * L'identifiant unique du microcontr�leur est stock� dans le registre `IDCODE` du composant `DBGMCU`.
 * Les 12 premiers bits de ce registre contiennent l'ID du p�riph�rique, qui identifie le num�ro de s�rie du MCU.
 *
 * @retval uint16_t: L'identifiant unique du microcontr�leur.
 */
uint16_t get_mcu_chip_id(void)
{
    /*
     * Les MCU STM32F4xx int�grent un code d'identification (ID code).
     * Ce code est contenu dans le registre `DBGMCU->IDCODE`.
     * Les bits [11:0] de ce registre identifient l'ID du p�riph�rique.
     * Le registre `DBGMCU->IDCODE` est mapp� � l'adresse 0xE0042000.
     */
    uint16_t cid;

    // Lire le registre `IDCODE` et masquer les 12 bits inf�rieurs pour obtenir l'ID du p�riph�rique.
    cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;

    // Retourner l'identifiant unique.
    return cid;
}


/**
 * @brief Lit le niveau de protection en lecture (RDP) de la m�moire Flash.
 * Le niveau de protection en lecture est utilis� pour prot�ger le contenu de la m�moire Flash contre
 * les op�rations de lecture, d'�criture et d'effacement.
 *
 * @retval Un `uint8_t` repr�sentant le niveau actuel de protection RDP :
 *         - 0xAA : Niveau 0 (Pas de protection)
 *         - 0xBB : Niveau 1 (Protection activ�e)
 *         - 0xCC : Niveau 2 (Protection maximale, irr�versible)
 */
uint8_t get_flash_rdp_level(void)
{
    // Variable pour stocker le niveau RDP lu.
    uint8_t rdp_status = 0;

    // Directive de compilation pour s�lectionner la m�thode d'impl�mentation bas�e sur la pr�f�rence.
#if 0
    // M�thode 1 : Utiliser la biblioth�que HAL pour lire les octets d'option.

    // Structure HAL pour contenir la configuration des octets d'option.
    FLASH_OBProgramInitTypeDef ob_handle;

    // R�cup�rer la configuration actuelle des octets d'option, y compris le niveau RDP.
    HAL_FLASHEx_OBGetConfig(&ob_handle);

    // Le niveau RDP est stock� dans le champ `RDPLevel` de la structure `ob_handle`.
    rdp_status = (uint8_t)ob_handle.RDPLevel;

#else
    // M�thode 2 : Lire directement depuis l'adresse m�moire o� les octets d'option sont stock�s.

    // Adresse de d�part des octets d'option dans le STM32F446xx.
    // Les octets d'option sont stock�s � l'adresse 0x1FFFC000 dans la m�moire Flash.
    volatile uint32_t *pOB_addr = (uint32_t*)0x1FFFC000;

    // Lire les bits 8 � 15 pour obtenir le niveau RDP.
    // Effectuer un d�calage de 8 bits vers la droite pour isoler le deuxi�me ensemble de 8 bits dans le registre 32 bits.
    rdp_status = (uint8_t)(*pOB_addr >> 8);

#endif

    // Retourner le niveau RDP lu.
    return rdp_status;
}

/**
 * @brief V�rifie l'adresse envoy�e par l'h�te.
 * Retourne ADDR_VALID si l'adresse est valide pour l'ex�cution de code.
 * Sinon, retourne ADDR_INVALID.
 *
 * @param go_address: Adresse � v�rifier.
 * @retval ADDR_VALID si l'adresse est valide, ADDR_INVALID sinon.
 */
uint8_t verify_address(uint32_t go_address)
{
    // V�rifier si l'adresse appartient � la m�moire SRAM1
    if (go_address >= SRAM1_BASE && go_address <= SRAM1_END)
    {
        return ADDR_VALID;
    }
    // V�rifier si l'adresse appartient � la m�moire SRAM2
    else if (go_address >= SRAM2_BASE && go_address <= SRAM2_END)
    {
        return ADDR_VALID;
    }
    // V�rifier si l'adresse appartient � la m�moire Flash (m�moire utilisateur)
    else if (go_address >= FLASH_BASE && go_address <= FLASH_END)
    {
        return ADDR_VALID;
    }
    // V�rifier si l'adresse appartient � la m�moire SRAM de sauvegarde
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
 * @brief Efface les secteurs de m�moire Flash.
 *
 * Nous avons un total de 8 secteurs (de 0 � 7) dans ce microcontr�leur.
 * Le nombre de secteurs doit �tre compris entre 0 et 7.
 * Si le secteur = 0xFF, cela indique qu'un effacement de masse (effacement complet) est demand�.
 *
 * @param sector_number: Le premier secteur � effacer.
 * @param number_of_sector: Nombre de secteurs � effacer.
 * @retval Le statut de l'op�ration (OK, erreur, etc.).
 */
uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector)
{
    FLASH_EraseInitTypeDef flashErase_handle; // Structure pour g�rer le processus d'effacement
    uint32_t sectorError; // Variable pour capturer les erreurs de secteur
    HAL_StatusTypeDef status; // Statut de retour de l'API HAL

    // Si le nombre de secteurs d�passe 8, retourner une erreur de secteur non valide
    if (number_of_sector > 8)
        return INVALID_SECTOR;

    // Assurez-vous que le secteur est soit 0xFF (effacement de masse) soit compris entre 0 et 7
    if ((sector_number == 0xFF) || (sector_number <= 7))
    {
        // Si c'est un effacement de masse
        if (sector_number == 0xFF)
        {
            flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE; // Indique un effacement complet de la m�moire
        }
        // Si c'est un effacement de secteurs sp�cifiques
        else
        {
            // Calculer les secteurs restants � partir du secteur de d�part
            uint8_t remaining_sector = 8 - sector_number;
            if (number_of_sector > remaining_sector)
            {
                number_of_sector = remaining_sector;
            }
            flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS; // Indique un effacement secteur par secteur
            flashErase_handle.Sector = sector_number; // Premier secteur � effacer
            flashErase_handle.NbSectors = number_of_sector; // Nombre de secteurs � effacer
        }
        flashErase_handle.Banks = FLASH_BANK_1; // Effacement sur la banque Flash 1

        // D�verrouiller la m�moire Flash pour les acc�s en �criture
        HAL_FLASH_Unlock();
        flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Plage de tension utilis�e par le microcontr�leur
        // Appeler l'API HAL pour effectuer l'op�ration d'effacement
        status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
        // Verrouiller la m�moire Flash apr�s l'effacement
        HAL_FLASH_Lock();

        return status; // Retourner le statut de l'op�ration (OK, erreur, etc.)
    }

    return INVALID_SECTOR; // Si le num�ro de secteur est invalide, retourner une erreur
}

/**
 * @brief �crit le contenu de pBuffer � l'adresse "mem_address" octet par octet.
 *
 * Remarque 1 : Actuellement, cette fonction ne prend en charge que l'�criture en Flash.
 * Remarque 2 : Cette fonction ne v�rifie pas si "mem_address" est une adresse valide dans la plage Flash.
 *
 * @param pBuffer: Tampon contenant les donn�es � �crire.
 * @param mem_address: Adresse m�moire o� l'�criture doit commencer.
 * @param len: Longueur des donn�es � �crire.
 * @retval Statut de l'op�ration (HAL_OK ou erreur).
 */
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status = HAL_OK;

    // D�verrouiller la m�moire Flash pour les op�rations d'�criture
    HAL_FLASH_Unlock();

    // �crire chaque octet dans la m�moire
    for (uint32_t i = 0; i < len; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_address + i, pBuffer[i]);
    }

    // Verrouiller la m�moire Flash apr�s l'op�ration
    HAL_FLASH_Lock();

    return status;
}




/**
 * @brief Modifie les octets d'option utilisateur pour configurer la protection en lecture/�criture.
 * Pour modifier la valeur des octets d'option utilisateur, suivez la s�quence ci-dessous :
 * 1. V�rifiez qu'aucune op�ration de m�moire Flash n'est en cours en v�rifiant le bit BSY dans le registre FLASH_SR.
 * 2. �crivez la valeur d'option souhait�e dans le registre FLASH_OPTCR.
 * 3. D�finissez le bit de d�marrage d'option (OPTSTRT) dans le registre FLASH_OPTCR.
 * 4. Attendez que le bit BSY soit d�sactiv�.
 *
 * Cette fonction configure la protection en lecture/�criture au niveau des secteurs en fonction du mode fourni :
 * - Si `disable` est d�fini, il d�sactive la protection en lecture/�criture sur tous les secteurs.
 * - Si `protection_mode` est 1, il active la protection en �criture uniquement pour les secteurs sp�cifi�s.
 * - Si `protection_mode` est 2, il active la protection en lecture/�criture pour les secteurs sp�cifi�s.
 *
 * @param sector_details : Masque de bits pour les secteurs � prot�ger.
 * @param protection_mode : Le type de protection (1 : Protection en �criture, 2 : Protection en lecture/�criture).
 * @param disable : Si d�fini, d�sactive la protection.
 *
 * @return 0 en cas de succ�s.
 */
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
    // Pointeur vers le registre de contr�le des options de la m�moire Flash (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

    // D�sactiver la protection en lecture/�criture si l'argument `disable` est d�fini
    if (disable)
    {
        HAL_FLASH_OB_Unlock();  // D�verrouiller les octets d'option Flash

        // Attendre jusqu'� ce que la m�moire Flash ne soit plus occup�e
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

        // D�sactiver la protection sur tous les secteurs
        *pOPTCR &= ~(1 << 31);  // D�sactiver le bit SPRMOD (protection en lecture/�criture)
        *pOPTCR |= (0xFF << 16);  // D�sactiver la protection sur tous les secteurs

        *pOPTCR |= (1 << 1);  // D�marrer la programmation des options

        // Attendre jusqu'� ce que la m�moire Flash ne soit plus occup�e
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

        HAL_FLASH_OB_Lock();  // Verrouiller les octets d'option Flash
        return 0;
    }

    // Configurer la protection en fonction du mode s�lectionn�
    if (protection_mode == (uint8_t) 1)  // Mode 1 : Protection en �criture uniquement
    {
        HAL_FLASH_OB_Unlock();  // D�verrouiller les octets d'option Flash

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occup�e

        *pOPTCR &= ~(1 << 31);  // D�sactiver la protection en lecture (bit 31 � 0)
        *pOPTCR &= ~(sector_details << 16);  // Activer la protection en �criture pour les secteurs sp�cifi�s

        *pOPTCR |= (1 << 1);  // D�marrer la programmation des options

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occup�e

        HAL_FLASH_OB_Lock();  // Verrouiller les octets d'option Flash
    }
    else if (protection_mode == (uint8_t) 2)  // Mode 2 : Protection en lecture/�criture
    {
        HAL_FLASH_OB_Unlock();  // D�verrouiller les octets d'option Flash

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occup�e

        *pOPTCR |= (1 << 31);  // Activer la protection en lecture (bit 31 � 1)
        *pOPTCR &= ~(0xff << 16);  // Effacer tous les bits de protection
        *pOPTCR |= (sector_details << 16);  // Activer la protection en lecture/�criture pour les secteurs sp�cifi�s

        *pOPTCR |= (1 << 1);  // D�marrer la programmation des options

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Attendre que la Flash ne soit plus occup�e

        HAL_FLASH_OB_Lock();  // Verrouiller les octets d'option Flash
    }

    return 0;
}

/**
 * @brief Lit l'�tat de la protection en lecture/�criture des octets d'option Flash.
 *
 * Cette fonction lit la configuration des octets d'option (OB) pour d�terminer l'�tat actuel de la protection
 * en lecture/�criture. L'�tat de la protection est stock� dans le champ `WRPSector` de la structure OB.
 *
 * @return Une valeur de 16 bits repr�sentant les secteurs avec une protection en lecture/�criture active.
 */
uint16_t read_OB_rw_protection_status(void)
{
    // Cette structure est fournie par le pilote Flash de ST pour contenir le contenu des OB (Option Bytes).
    FLASH_OBProgramInitTypeDef OBInit;

    // Tout d'abord, d�verrouiller l'acc�s m�moire des OB (Option Bytes)
    HAL_FLASH_OB_Unlock();
    // R�cup�rer les d�tails de la configuration des OB
    HAL_FLASHEx_OBGetConfig(&OBInit);
    // Verrouiller l'acc�s m�moire des OB
    HAL_FLASH_Lock();

    // Nous nous int�ressons uniquement � l'�tat de protection en lecture/�criture des secteurs.
    return (uint16_t)OBInit.WRPSector;
}
