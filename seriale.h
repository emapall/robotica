#include "arduino-serial.h"
int Serial_start ()
{
  id_message = 0;

  cout<<"Inizializzazione Porta Seriale...";
  int status = serialport_init("/dev/ttyUSB0", 9600);
  if ( status == -1){
    printf("ERRORE APERTURA PORTA SERIALE");
    exit(-1);}
  cout<<"ok!"<<endl;

  //scrivo
  int mes =3;
  if (serialport_writebyte(status, mes) == -1)
  {
		printf ("ERRORE SCRITTURA porta seriale \n");
		exit(-1);
  }

  //leggo
  char eolchar = '\n';
  int timeout = 1000;
  char buf[4];
  serialport_read_until(status, buf, eolchar, 4, timeout);
  sleep(1);
  return status;
}

// ↓ molto brutta ma potrebbe essere efficace
bool validation (char mes, char buf[])
{
	const int leng1 = 2;
	const char type1[leng1] = {'0','1'};

	const int leng2 = 4;
	const char type2 [leng2] = {'0','1','2','3'};
	/*
	 * Muri (m) e rotazione (r) accettano come valori solo [0,1] → type1
	 * Avanti (a) accetta su entrambe le cifre solo [0,1,2,3] → type2
	 */

	bool ok = false;

	if ( (mes == 'm') || (mes == 'r') )
	{
		for (int i = 0; i<leng1; i++)
			if (buf[0] == type1[i])
				ok = true;
	}

	else if (mes == 'a')
	{
		for (int i = 0; i<leng2; i++)
			for (int j = 0; j<leng2; j++)
				if ( (buf[0] == type2[i]) && (buf[1] == type2[j]) )
					ok = true;
	}
	return ok;
}


int arduino (char mes, int arg)
{

	char eolchar = '\n';
	int timeout = 5000;
	char buf[5];

	bool error;

	do {

	 serialport_flush(fd);
	 error = false;

		if (serialport_writebyte(fd, mes) == -1) {
			printf ("ERRORE SCRITTURA");
			exit(-1); }

		if (serialport_writebyte(fd, arg) == -1) {
			printf ("ERRORE SCRITTURA");
			exit(-1); }

		if (serialport_writebyte(fd, id_message) == -1) {
			printf ("ERRORE SCRITTURA");
			exit(-1); }


		if (serialport_read_until(fd, buf, eolchar, 5, timeout) == -2) {
			printf("ERRORE LETTURA");
			error = true; }

		if ( !validation(mes, buf) )
			error = true;

		//cout<< "id: "<<id_message<<" mes: "<<mes<<" arg: "<<arg <<" error: "<<error<<" buf: "<<buf[0]<<buf[1]<<endl;
		//cin.get();
		//sleep(1);
    } while (error == true);

    id_message ++;
/*
	//atoi() dovrebbe convertire da char a int i numeri
    if (mes == 'a') //servono due risposte
		return atoi(buf[0]) * 10 + atoi(buf[1]);
	else
		return atoi(buf[0]);
  */

  return atoi(buf);
}


/*
bool leggi_muro_old(butgold) (int mur)
{
	cout<<mur<<endl;
	bool muro;

	char eolchar = '\n';
	int timeout = 5000;
	char buf[3];


  	do{

 serialport_flush(fd);

	if (serialport_writebyte(fd, mur) == -1)
	{
		printf ("ERRORE SCRITTURA porta seriale");
		exit(-1);
	}

	if (serialport_read_until(fd, buf, eolchar, 2, timeout) == -2) {
		printf("ERRORE LETTURA");
		//exit (-2);
    }

	if (buf[0] == '1') //c'è il muro
	{
		cout<<"C'è il muro"<<endl;
		muro = true;
		//cin.get();
	}
	else if (buf[0] == '0') {
		cout<<"No muro"<<endl;
		muro = false;
		//cin.get();
	}
	else {
		cout<<"Ho ricevuto merda... Dovrei rileggere... Peace <3"<<endl<<"PS: Ho ricevuto: "<<buf[0]<<endl;
		muro = false;
		//cin.get();
	}

	} while ( (buf[0]!='1') && (buf[0]!='0') );
		//sleep(2);
	//serialport_flush(fd); //perché così? perché sembra funzionare! asd
	//fd = serialport_flush(fd); //<- altrimenti usare questo :D
  return muro; // non c'è il muro

}
*/

/* CODICE ARDUINO
 *
 * int inByte = 0;         // incoming serial byte


struct Sensore_US
{
  int trigger;
  int echo;
};

#define AC 0  //Avanti centro
#define DX 1  //destra
#define SX 2  //sinistra
#define BC 3  //dietro centro
#define AS 4  //avanti sinistra
#define AD 5  //avanti destra
#define BS 6  //dietro sinistra
#define BD 7  //dietro destra




void setup()
{


  Sensore_US US[8];

US[AC].trigger = 3;
US[AC].echo = 2;

US[DX].trigger = 32;
US[DX].echo = 33;

US[SX].trigger = 44;
US[SX].echo = 45;

US[BC].trigger = 3;
US[BC].echo = 2;

  for (int i=0; i<3; i++)
  {
    pinMode( US[i].trigger, OUTPUT );
    pinMode( US[i].echo, INPUT );
  }

  // start serial port at 9600 bps:
  Serial.begin(9600);

}

int leggi(int triggerPort, int echoPort)
{
  //porta bassa l'uscita del trigger
digitalWrite( triggerPort, LOW );

//invia un impulso di 10microsec su trigger
digitalWrite( triggerPort, HIGH );
delayMicroseconds( 10 );
digitalWrite( triggerPort, LOW );

long duration = pulseIn( echoPort, HIGH );

long r = 0.034 * duration / 2;
  return r;
}

void loop()
{

    Sensore_US US[8];

US[AC].trigger = 3;
US[AC].echo = 2;

US[DX].trigger = 32;
US[DX].echo = 33;

US[SX].trigger = 44;
US[SX].echo = 45;

US[BC].trigger = 3;
US[BC].echo = 2;

  int read1;
  while (Serial.available() <= 0);
  read1 = Serial.read();
  //int read2 = Serial.read(); //non dovrebbe andarci ma ecco
 if (read1 == 3)
  Serial.print(2);
  else
  Serial.print(4);
  Serial.println(read1);

  Serial.read();
  //establishContact();  // send a byte to establish contact until receiver responds

    while (true){

      int solution;
      if (Serial.available() > 0) {

        inByte = Serial.read();
        if (inByte >= 0) {
          if (
*/

/* new arduino
#define AC 0  //Avanti centro
#define DX 1  //destra
#define SX 2  //sinistra
#define BC 3  //dietro centro
#define AS 4  //avanti sinistra
#define AD 5  //avanti destra
#define BS 6  //dietro sinistra
#define BD 7  //dietro destra


int inByte = 0;         // incoming serial byte
char inChar;


typedef struct
{
  int trigger;
  int echo;
}Sensore_US;

  Sensore_US US[8] = { {3,2}, {32,33}, {44,45}, {2,3} };



void setup()
{
  for (int i=0; i<3; i++)
  {
    pinMode( US[i].trigger, OUTPUT );
    pinMode( US[i].echo, INPUT );
  }

  // start serial port at 9600 bps:
  Serial.begin(9600);

}

int leggi(int sensore)
{
  //porta bassa l'uscita del trigger
  digitalWrite( US[sensore].trigger, LOW );

  //invia un impulso di 10microsec su trigger
  digitalWrite( US[sensore].trigger, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( US[sensore].trigger, LOW );

  long duration = pulseIn( US[sensore].echo, HIGH );

  long r = 0.034 * duration / 2;
  return r;
}

int muovi (int direzione)
{
  return 0;
}

int rotate (int direzione)
{
  return 0;
}

void loop()
{
  // legge/scrive COOOSE
  int read1;
  while (Serial.available() <= 0);
  read1 = Serial.read();
  if (read1 == 3)
    Serial.print(2);
  else
    Serial.print(4);
  Serial.println(read1);

  Serial.read();


  int solution;
  int old_solution;
  int id_message;
  int old_message = -1;

  //INIZIA IL VERO PRAGRAMMA
    while (true)
    {

      if (Serial.available() > 0)
      {
        inChar = Serial.read();
        inByte = Serial.read();
        id_message = Serial.read();

        if (id_message == old_message) //messaggio vecchio, già eseguito
        {
          solution = old_solution;
          //i muri vanno riletti? mah
        }
        else
        {
          solution = 9;

          if (inChar == 'm')
          {
            int distance = leggi(inByte);
            if (distance < 15)
              solution = 1;
            else
              solution = 0;
          }
          else if (inChar == 'a')
          {
            solution = muovi(inByte);
          }

          else if (inChar == 'r')
          {
            solution = rotate(inByte);
          }
        }

        //invio i risultati
        Serial.println(solution);
        old_solution = solution;
        old_message = id_message;
      }
      delay(100);
    }
}
*/
