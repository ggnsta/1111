#include <msp430.h>
#include <math.h>

typedef unsigned char uint8_t;

#define SET_COLUMN_ADDRESS_LSB        0x00 // младший байт столбца 0
#define SET_COLUMN_ADDRESS_MSB        0x10 // старший байт столбца 0
#define SET_PAGE_ADDRESS              0xB0 // страница 0(pa)
#define SET_SEG_DIRECTION             0xA0 // нормальный порядок записи столбцов
#define SET_COM_DIRECTION             0xC0 // нормальный порядок отображения строк
#define SET_POWER_CONTROL             0x2F // Управление питанием. PC[0] – усилитель, PC[1] — регулятор, PC[2] — повторитель. 0 — отключено, 1 — включено
#define SET_SCROLL_LINE               0x40 // Установка начальной линии скроллинга SL=0..63
#define SET_VLCD_RESISTOR_RATIO       0x27 // Установка уровня внутреннего резисторного делителя PC = [0..7].Используется для управления контрастом.
#define SET_ELECTRONIC_VOLUME_MSB     0x81 // Регулировка контраста. Двухбайтная команда. PM[5..0] PM = 0..63.
#define SET_ELECTRONIC_VOLUME_LSB     0x0F
#define SET_ALL_PIXEL_ON              0xA4 // Включение всех пикселей. 0 – отображение содержимого памяти, 1 – все пиксели включены (содержимое памяти сохраняется).
#define SET_INVERSE_DISPLAY           0xA6 // Инверсный режим. 0 — нормальное отображение содержимого памяти, 1 — инверсное.
#define SET_DISPLAY_ENABLE            0xAF // Отключение экрана. 0 — экран отключен, 1 — включен.
#define SET_LCD_BIAS_RATIO            0xA2 // Смещение напряжения делителя: 0 – 1/9, 1 – 1/7.
#define SET_ADV_PROGRAM_CONTROL0_MSB  0xFA // Расширенное управление. ТС — температурная компенсация 0 = -0.05, 1 = -0.11 % / °С;
#define SET_ADV_PROGRAM_CONTROL0_LSB  0x90 // WC – циклический сдвиг столбцов 0 = нет, 1 = есть; WP –циклический сдвиг страниц 0 = нет, 1 = есть.
#define READ_Y_AXIS_DATA              0x1C
#define READ_Z_AXIS_DATA              0x20

uint8_t initMacro[] = {
    SET_SCROLL_LINE,
    SET_SEG_DIRECTION,
    SET_COM_DIRECTION,
    SET_ALL_PIXEL_ON,
    SET_INVERSE_DISPLAY,
    SET_LCD_BIAS_RATIO,
    SET_POWER_CONTROL,
    SET_VLCD_RESISTOR_RATIO,
    SET_ELECTRONIC_VOLUME_MSB,
    SET_ELECTRONIC_VOLUME_LSB,
    SET_ADV_PROGRAM_CONTROL0_MSB,
    SET_ADV_PROGRAM_CONTROL0_LSB,
    SET_DISPLAY_ENABLE,
    SET_PAGE_ADDRESS,
    SET_COLUMN_ADDRESS_MSB,
    SET_COLUMN_ADDRESS_LSB
};

int COLUMN_START_ADDRESS = 126;
int MAPPING_VALUES[] = { 4571, 2286, 1142, 571, 286, 143, 71 };
uint8_t BITx[] = { BIT6, BIT5, BIT4, BIT3, BIT2, BIT1, BIT0 };

unsigned char tx = 0;
unsigned char rx = 0;
unsigned char busy = 0;
unsigned char isRxReady = 0;
unsigned char isTxReady = 0;
unsigned char isBusy = 1;

uint8_t symbols[12][6] = {
        {0x1C, 0x1C, 0xFF, 0xFF, 0x1C, 0x1C}, // +
        {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C}, // -
        {0xFF, 0xFF, 0xC1, 0xC1, 0xFF, 0xFF}, // 0
        {0x80, 0x80, 0xFF, 0xFF, 0x83, 0x82}, // 1
        {0xDF, 0xDF, 0xD9, 0xD9, 0xF9, 0xF9}, // 2
        {0xFF, 0xFF, 0xC9, 0xC9, 0xC9, 0xC9}, // 3
        {0xFF, 0xFF, 0x0C, 0x0C, 0x0F, 0x0F}, // 4
        {0xFD, 0xFD, 0xCD, 0xCD, 0xCF, 0xCF}, // 5
        {0xFD, 0xFD, 0xCD, 0xCD, 0xFF, 0xFF}, // 6
        {0xFF, 0xFF, 0x01, 0x01, 0x01, 0x01}, // 7
        {0xFF, 0xFF, 0xC9, 0xC9, 0xFF, 0xFF}, // 8
        {0xFF, 0xFF, 0xC9, 0xC9, 0xCF, 0xCF}  // 9
};

uint8_t symbols2[12][6] = {
        {0x00, 0x00, 0x80, 0x80, 0x00, 0x00}, // +
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // -
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 0
        {0x00, 0x00, 0x80, 0x80, 0x00, 0x00}, // 1
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 2
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 3
        {0x80, 0x80, 0x00, 0x00, 0x80, 0x80}, // 4
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 5
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 6
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 7
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, // 8
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}  // 9
};

int getNumberLength(long int number);
void printNumber(long int number);
void clearScreen(void);
void setAddress(uint8_t pa, uint8_t ca);
void writeData(uint8_t* sData, uint8_t i);
void writeCommand(uint8_t* sCmd, uint8_t i);
uint8_t CMA3000_writeCommand(uint8_t byte_one, uint8_t byte_two);
void CMA3000_init(void);
int calculateAngleFromProjection(double projection);
long int parseProjectionByte(uint8_t projection_byte);

void startTimerA1(){
    TA1CCTL0 = CCIE;
    TA1CCR0 = 1240;
    TA1EX0 = TAIDEX_4;
    TA1CTL = TASSEL_2 | ID__4 | MC_1 | TACLR;
}

void stopTimerA1() {
    TA1CCTL0 &= ~CCIE;
    TA1CTL = MC_0;
}

int main(void) {
     WDTCTL = WDTPW | WDTHOLD;

    P1DIR |= BIT2;
    P1OUT &= ~BIT2;

    P5DIR |= BIT7; // установка на выход RST
    P5OUT |= BIT7;

    P7DIR |= BIT4; // установка на выход CS

    P5DIR |= BIT6;
    P5OUT &= ~BIT6; // CD - команда

    P4SEL |= BIT1; // передача данных LCD_SIMO
    P4DIR |= BIT1;

    P4SEL |= BIT3; // синхросигнал SCLK
    P4DIR |= BIT3;

    UCB1CTL1 = UCSSEL_2 | UCSWRST; // SMCLK, разрешение программного сброса
    UCB1CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC; // фаза ти, порядок передачи - MSB, Master, 3-pin, синхронный режим
    UCB1CTL1 &= ~UCSWRST; // разрешение работы модуля USCI
    UCB1IFG &= ~UCRXIFG; //флаг прерывания приёмника

    writeCommand(initMacro, 13);

    P7DIR |= BIT6; // питание подсветки
    P7SEL &= ~BIT6;
    P7OUT |= BIT6;

    clearScreen();
    CMA3000_init();

    __bis_SR_register(LPM0_bits + GIE);

    return 0;
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void TIMER_1 (void) {
    if(tx == 1)
    {
            if(UCA0IFG & UCTXIFG)
            {
                tx = 0;
                isTxReady = 1;
                __bic_SR_register_on_exit(LPM0_bits + GIE);
            }
        }
        else if(rx == 1)
        {
            if(UCA0IFG & UCRXIFG)
            {
                rx = 0;
                isRxReady = 1;
                __bic_SR_register_on_exit(LPM0_bits + GIE);
            }
        }
        else if(busy == 1)
        {
            if(!(UCA0STAT & UCBUSY))
            {
                busy = 0;
                isBusy = 0;
                __bic_SR_register_on_exit(LPM0_bits + GIE);
            }
        }
}

#pragma vector = PORT2_VECTOR
__interrupt void accelerometerInterrupt(void) {
    volatile uint8_t yProjectionByte = CMA3000_writeCommand(READ_Y_AXIS_DATA, 0);
    volatile long int yAxisProjection = parseProjectionByte(yProjectionByte);

    volatile long int g = yAxisProjection * 9.81 * 3.3/10;

    clearScreen();
    printNumber(g);

    int angle = calculateAngleFromProjection((double) yAxisProjection);

    //-2803
    if (-60 >= angle && angle >= -120) {
        P1OUT |= BIT2;
    }
    else {
        P1OUT &= ~BIT2;
    }
}

void CMA3000_init(void) {

    P2DIR  &= ~BIT5;// mode: input
    P2OUT  |=  BIT5;
    P2REN  |=  BIT5;// enable pull up resistor
    P2IE   |=  BIT5;// interrupt enable
    P2IES  &= ~BIT5;// process on interrupt's front
    P2IFG  &= ~BIT5;// clear interrupt flag

    P3DIR  |=  BIT5; //выбор устройства CSB
    P3OUT  |=  BIT5;

    P2DIR  |=  BIT7;
    P2SEL  |=  BIT7;    // UCA0CLK

    P3DIR  |= (BIT3 | BIT6);    // режим SIMO и напряжение питания PWM
    P3DIR  &= ~BIT4;        // SOMI
    P3SEL  |= (BIT3 | BIT4);    // P3.3 - UCA0SIMO , P3.4 - UCA0SOMI
    P3OUT  |= BIT6;// Power cma3000

    UCA0CTL1 = UCSSEL_2 | UCSWRST;
    UCA0CTL0 = UCCKPH & ~UCCKPL | UCMSB | UCMST | UCSYNC | UCMODE_0;
    UCA0CTL1 &= ~UCSWRST;// enable USCI

    // холостое чтение из REVID
    CMA3000_writeCommand(0x04, 0);
   __delay_cycles(1250);
    // запись в CTRL
    CMA3000_writeCommand(0x0A, BIT7 | BIT4 | BIT2 | BIT1);
}

uint8_t CMA3000_writeCommand(uint8_t firstByte, uint8_t secondByte) {
    char indata;
    P3OUT &= ~BIT5;// enable cma3000 SPI data transfer
    P2IE &= ~BIT5;

    indata = UCA0RXBUF;
    startTimerA1();

    tx = 1;
    __bis_SR_register(LPM0_bits + GIE);
    UCA0TXBUF = firstByte;
    isTxReady = 0;

    rx = 1;
    __bis_SR_register(LPM0_bits + GIE);
    indata = UCA0RXBUF;
    isRxReady = 0;

    tx = 1;
    __bis_SR_register(LPM0_bits + GIE);
    UCA0TXBUF = secondByte;
    isTxReady = 0;

    rx = 1;
    __bis_SR_register(LPM0_bits + GIE);
    indata = UCA0RXBUF;
    isRxReady = 0;

    busy = 1;
    __bis_SR_register(LPM0_bits + GIE);
    isBusy = 1;

    stopTimerA1();
    P3OUT |= BIT5;
    P2IE |= BIT5;
    return indata;
}

long int parseProjectionByte(uint8_t projectionByte) {
    int i = 0;
    long int projectionValue = 0;

    int isNegative = projectionByte & BIT7;

    for (; i < 7; i++) {
        if (isNegative) {
            projectionValue += (BITx[i] & projectionByte) ? 0 : MAPPING_VALUES[i];
        }
        else {
            projectionValue += (BITx[i] & projectionByte) ? MAPPING_VALUES[i] : 0;
        }
    }

    projectionValue *= isNegative ? -1 : 1;

    return projectionValue;
}

int calculateAngleFromProjection(double projection) {
    projection /= 1000;
    projection = projection > 1 ? 1 : projection < -1 ? -1 : projection;

    double angle = asin(projection);
    angle *= 57.3;

    return (int) angle;
}


void printNumber(long int number) {
    int nDigits = getNumberLength(number);

    setAddress(7, COLUMN_START_ADDRESS);
    writeData(number > 0 ? symbols[0] : symbols[1], 6);
    setAddress(6, COLUMN_START_ADDRESS);
    writeData(number > 0 ? symbols2[0] : symbols2[1], 6);

    int i = 0;
    long int divider = pow(10, nDigits - 1);
    number = fabsl(number);

    for (i = 1; i <= nDigits; i++) {
        int digit = number / divider;
        setAddress(7, COLUMN_START_ADDRESS - i*8);
        writeData(symbols[digit + 2],6);
        setAddress(6, COLUMN_START_ADDRESS - i*8);
        writeData(symbols2[digit + 2],6);
        number = number % divider;
        divider /= 10;
    }
}

int getNumberLength(long int number) {
    int length = 0;
    number = fabsl(number);

    if(number == 0) {
        return 1;
    }

    while(number) {
        number /= 10;
        length++;
    }

    return length;
}

void clearScreen(void)
{
    uint8_t Data[] = { 0x00 };
    uint8_t p, c;

    for (p = 0; p < 8; p++)
    {
        setAddress(p, 0);
        for (c = 0; c < 132; c++)
        {
            writeData(Data, 1);
        }
    }
}

void setAddress(uint8_t pa, uint8_t ca)
{
    uint8_t cmd[1];
    cmd[0] = SET_PAGE_ADDRESS + pa;
    uint8_t H = 0x00;
    uint8_t L = 0x00;
    uint8_t ColumnAddress[] = { SET_COLUMN_ADDRESS_MSB, SET_COLUMN_ADDRESS_LSB };
    L = (ca & 0x0F);
    H = (ca & 0xF0);
    H = (H >> 4);
    ColumnAddress[0] = SET_COLUMN_ADDRESS_LSB + L;
    ColumnAddress[1] = SET_COLUMN_ADDRESS_MSB + H;
    writeCommand(cmd, 1);
    writeCommand(ColumnAddress, 2);
}

void writeData(uint8_t* sData, uint8_t i)
{
    P7OUT &= ~BIT4;
    P5OUT |= BIT6;

    while (i)
    {
        while (!(UCB1IFG & UCTXIFG));
        UCB1TXBUF = *sData;
        sData++;
        i--;
    }

    while (UCB1STAT & UCBUSY);

    P7OUT |= BIT4;
}

void writeCommand(uint8_t* sCmd, uint8_t i)
{
    P7OUT &= ~BIT4;
    P5OUT &= ~BIT6;

    while (i)
    {
        while (!(UCB1IFG & UCTXIFG));
        UCB1TXBUF = *sCmd;
        sCmd++;
        i--;
    }

    while (UCB1STAT & UCBUSY);

    P7OUT |= BIT4;
}

