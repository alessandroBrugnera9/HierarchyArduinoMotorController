#include<SoftwareSerial.h>
#define rx1Pin 10
#define tx1Pin 9
#define rx2Pin 11
#define tx2Pin 12
    // Comunicacao modbus
    char c;
String recData;
String query = ":";
String sendData;
SoftwareSerial mySerial1 = SoftwareSerial(rx1Pin, tx1Pin);
SoftwareSerial mySerial2 = SoftwareSerial(rx2Pin, tx2Pin);
char dados[21];
int dec_lrc[4];
;
int lrc;
int lrc_bin[16];
// calculo de trajetoria
int x = 1, y = 1, angle = 1;
// ============================== Fim das variaveis globais
// Funcoes utilizadas para o protocolo MODBUS
int BintoDec(int val1, int val2, int val3, int val4)
{
    int result = 0;
    result += val4;
    result += val3 * 2;
    result += val2 * 4;
    result += val1 * 8;
    return result;
}
String formata_query(String query, int coord, int slave)
{
    int s = 0;
    // 1 e 2 - Slave Adress (0x)
    query += "0";
    query += slave;
    // 3 e 4 - Function Code (06)
    query += "06";
    // 5 ,6 ,7 ,8 - Controlador (0001)
    query += " 0001 ";
    // 9 ,10 ,11 ,12 - coord setpoint ( ABCD )
    query += "00";
    query += coord;
    // 13 ,14 ,15 ,16 - LRC
    lrc = query[1] ^ query[2] ^ query[3] ^ query[4] ^ query[5] ^ query[6] ^ query[7] ^
          query[8] ^ query[9] ^ query[10] ^
          query[11] ^ query[12];
    for (int k = 0; lrc > 0; k++)
    {
        lrc_bin[k] = lrc % 2;
        lrc = lrc / 2;
    }
    int dec_lrc[4];
    dec_lrc[3] = BintoDec(lrc_bin[3], lrc_bin[2], lrc_bin[1], lrc_bin[0]); // lsb
    dec_lrc[2] = BintoDec(lrc_bin[7], lrc_bin[6], lrc_bin[5], lrc_bin[4]);
    dec_lrc[1] = BintoDec(lrc_bin[11], lrc_bin[10], lrc_bin[9], lrc_bin[8]);
    dec_lrc[0] = BintoDec(lrc_bin[15], lrc_bin[14], lrc_bin[13], lrc_bin[12]); // msb
    int m = 0;
    for (int j = 0; j < 4; j++)
    {
        if (dec_lrc[m] > 10)
        {
            query += dec_lrc[m];
        }
        else
        {
            query += dec_lrc[m];
        }
        m++;
    }
    // 17 e 18 - CR ,LF
    query += '\r';
    query += '\n';
    return query;
}
// TASK1
#define INTERVALO1 1000
void task1()
{
    x = round(10 * (1 + cos(angle * M_PI / 180)));
    y = round(10 * (1 + sin(angle * M_PI / 180)));
    angle++;
} // task1
// TASK2
#define INTERVALO2 200 // Tarefa2 a cada 0.005 s
void task2()
{
    sendData = formata_query(query, x, 1);
    Serial.println(" Transmitting : " + sendData);
    mySerial1.print(sendData);
    sendData = "";
    query = ":";
    sendData = formata_query(query, y, 2);
    Serial.println(" Transmitting : " + sendData);
    mySerial2.print(sendData);
    sendData = "";
    query = ":";
    delay(50);
    if (mySerial1.available() > 0)
    {
        recData = "";
        while (mySerial1.available())
        {
            c = mySerial1.read();
            recData += c;
        }
    }
    delay(50);
    if (mySerial2.available() > 0)
    {
        recData = "";
        while (mySerial2.available())
        {
            c = mySerial2.read();
            recData += c;
        }
    }
} // task2
// TASK SWITCHER
typedef struct
{
    void (*task)();
    long interval;
    long current_time;
    int status;
} TaskControl;
#define MAX_TASKS 2
#define READY 1
#define WAIT 0
TaskControl taskList[MAX_TASKS];
void createTask(int taskNum, void (*t)(), long interval)
{
    // Cria uma task
    taskList[taskNum].task = t;
    taskList[taskNum].interval = interval;
    taskList[taskNum].current_time = 0;
    taskList[taskNum].status = WAIT;
} // createTask
void runCurrentTask()
{
    // Executa a task atual
    int i;
    void (*task)();
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (taskList[i].status == READY)
        {
            task = taskList[i].task;
            (*task)();
            noInterrupts();
            taskList[i].status = WAIT;
            taskList[i].current_time = 0;
            interrupts();
        } // if task is READY
    }     // for each task
} // runCurrentTask
void updateTickCounter()
{
    // Atualiza o contador para os intervalos
    // das tasks
    int i;
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (taskList[i].status == WAIT)
        {
            taskList[i].current_time++;
            if (taskList[i].current_time >= taskList[i].interval)
            {
                taskList[i].status = READY;
            }
        } // if task is WAITing
    }     // for each task
} // updateTickCounter
void setup()
{
    pinMode(rx1Pin, INPUT);
    pinMode(tx1Pin, OUTPUT);
    pinMode(rx2Pin, INPUT);
    pinMode(tx2Pin, OUTPUT);
    // set the data rate for the SoftwareSerial port
    mySerial1.begin(2400);
    mySerial2.begin(2400);
    Serial.begin(2400);
    // Cria as tarefas
    createTask(0, &task1, INTERVALO1);
    createTask(1, &task2, INTERVALO2);
    // Inicia interrupcao do timer
    // para 1ms
    setTimerInterrupt(1000); // int @1ms (1000 us)
}
void loop()
{
    runCurrentTask(); // executa tarefa atual
}
// Interrupcao do Timer
ISR(TIMER1_COMPA_vect)
{
    updateTickCounter();
} // ISR
void setTimerInterrupt(long uSecs)
{
    noInterrupts(); // Desabilita interrupcoes
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // compare match register 16 MHz /256 * t(s) - 1
    OCR1A = (16e6 / 256L * uSecs) / 1e6 - 1;
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    interrupts();            // enable all interrupts
}