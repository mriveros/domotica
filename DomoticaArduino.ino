/*
	Name:       DomoticaArduino.ino
	Created:	13/10/2018 12:55:11
	Author:     Marcos Riveros
*/

/*
ESQUEMA PATILLAJE
======================
A0  --> Lectura an�logica del sensor de corriente.
A4  --> SDA (Conexi�n I2C con el reloj DS3231)
A5  --> SCL (Conexi�n I2C con el reloj DS3231)

PD2 --> Pin para la interrupci�n enviada desde el DS3231
*/

#include <Ethernet.h>				// Manejo del W5100 Ethernet Shield Arduino
#include <SD.h>						// Leer/grabar en tarjetas SD (Incorporado en W5100)
#include <RTClib.h>					// Manejo del reloj DS3231 por protocolo I2C


RTC_DS3231 reloj;					// Encargado de suministrar la fecha/hora.
DateTime tiempo;					// Almacena el dato de la fecha/hora.

const byte pinControlSD = PD4;		

// Par�metros red.
IPAddress ip(192, 168, 1, 122);					// Asignaci�n de IP fija a la placa. 
byte mac[] = { 0xD4, 0x7B,  0xB0, 0x32, 0x1D, 0xA1 };	// Direcci�n MAC de la placa Ethernet de Arduino.
byte servidor[] = { 192, 168, 1, 101 };			// Direcci�n IP del servidor.
EthernetClient cliente;							// Cliente web.

volatile bool escribir = false;					// Indica cuando enviar la lectura.
const char referencia[] = "Pruebas01";			// Referencia para el env�o de la lectura a la web.
const byte escala = 1;							// Escala utilizada (valor fijo, no utilizo escalas)
float corriente = 0;							// Total corriente consumida en un periodo.
float compensacion = 0;							// C�lculo del valor de compensaci�n para una medida mas exacta.
float consumoMedio = 0;							// C�lculo del consumo medio en un periodo de tiempo.
float potencia = 0;								// Potencia consumida en un periodo de tiempo.
const float corrienteMax = 20;					// Corriente m�xima soportada por el sensor Hall
const char tension[] = "220";					// Tensi�n de la red
unsigned int lecturas = 0;						// N� de lecturas efectuadas en un periodo.
const float r2 = sqrt(2);						// Raiz cuadrada de 2 (Para el c�lculo de la potencia)

char format[] = "01/01/2018 00:00:00";			// Utilizado para formatear Fecha/hora con la funci�n sprintf.
char fichero[] = "lecturas/XXXXXXXX.csv";		// Nombre del fichero csv.
File ficheroCSV;								// Fichero csv para guardar las lecturas
const char separador = ';';						// Separador utilizado en el fichero CSV.
bool escribirSD = true;
bool subidoWeb = false;

unsigned long milisegundos = 0;					// Control para tomar lecturas cada milisegundo.

// Configuraci�n inicial de la placa.
void setup()
{
	Serial.begin(9600);

	reloj.begin();								// Inicializamos el reloj.
	reloj.enableOscillator(true, false, 0);		// Configuramos una se�al cuadrada de 1 Hz que utilizamos como interrupci�n.
	tiempo = reloj.now();						// Leer la hora.
	pinMode(pinControlSD, OUTPUT);				// Configurar el pin de control de la tarjeta SD como salida.
	
	escribirSD = SD.begin(pinControlSD);		// Comprobamos si la tarjeta est� presente.

	if (escribirSD) {
		preparaFichero();						// Comprueba el fichero.
	}
	else
	{
		Serial.println(F("No se encuentra la tarjeta."));	// La tarjeta SD no est� preparada.
	}

	Serial.println(F("Empezamos."));
	Ethernet.begin(mac, ip);							// Configuraci�n m�dulo ethernet W5100.

	valorMedio();										// Calculamos el valor promedio de las lecturas cuando no hay consumo.

	// Configuramos interrupci�n lectura (1 por segundo)
	attachInterrupt(digitalPinToInterrupt(2), comprobarLectura, RISING);
	milisegundos = millis();							// Actualizamos la variable de control antes de empezar.
}

// Bucle principal.
void loop()
{
	if (millis() > milisegundos) {									// Una lectura cada milisegundo.
		milisegundos++;
		corriente += abs((float)analogRead(A0) - compensacion);		// Devuelve un valor entre -512 y 511
		lecturas++;													// N�mero de lecturas en un periodo.
	}
	if (escribir) {											// Escribir = true cuando se activa la interrupci�n (una vez por segundo)
		escribir = false;
		Serial.print(".");
		tiempo = reloj.now();
		if (tiempo.second() == 0) {							// Cada minuto enviamos la lectura a la web
			conversion();
			if (tiempo.hour() == 0 && tiempo.minute() == 0) { preparaFichero(); }	// Si hora = 0 y minutos = 0 hemos cambiado de d�a
			Serial.println(consumoMedio);
			;					// Conectamos con el servidor.
			if (cliente.connect(servidor, 80)) {						// Si la conexi�n es OK enviamos la informaci�n
				sprintf(format, "%02u/%02u/%04u %02u:%02u:%02u", tiempo.day(), tiempo.month(), tiempo.year(), tiempo.hour(), tiempo.minute(), tiempo.second());
				Serial.println(format);
				cliente.print(F("GET /guardar.php?"));		// Fichero con el c�digo en PHP para guardar la lectura
				cliente.print(F("r="));
				cliente.print(referencia);					// Valor del campo referencia
				cliente.print(F("&c="));
				cliente.print(consumoMedio);				// Valor de la corriente media durante la lectura.
				cliente.print(F("&e="));
				cliente.print(escala);						// Escala utilizada en la lectura.
				cliente.println(F(" HTTP/1.1"));			// Final de cadena de los datos.
				cliente.println(F("Host: 192.168.1.200"));	// Direci�n del servidor Web
				cliente.println();
				cliente.stop();								// Finalizamos la conexi�n
				subidoWeb = true;
			}
			else {
				Serial.println(F("No conectado..."));		// Ha fallado la conexi�n.
				subidoWeb = false;
			}
			preparaFichero();													// Preparamos la escritura en la tarjeta SD
			if (escribirSD) {
				ficheroCSV = SD.open(fichero, FILE_WRITE);						// Abrimos el fichero.
				ficheroCSV.print(referencia); ficheroCSV.print(separador);		// Escribimos los datos.
				sprintf(format, "%02u/%02u/%04u %02u:%02u:%02u", tiempo.day(), tiempo.month(), tiempo.year(), tiempo.hour(), tiempo.minute(), tiempo.second());
				ficheroCSV.print(format); ficheroCSV.print(separador);
				ficheroCSV.print(tension); ficheroCSV.print(separador);
				ficheroCSV.print(consumoMedio); ficheroCSV.print(separador);
				potencia = 220.0f * consumoMedio * r2;
				ficheroCSV.print(potencia); ficheroCSV.print(separador);
				ficheroCSV.print(lecturas); ficheroCSV.print(separador);
				ficheroCSV.println(subidoWeb ? 'S' : 'N');
				ficheroCSV.close();
			}
			lecturas = 0;									// Reiniciamos las variables
			corriente = 0;
		}
	}
}

// FUNCIONES AUXILARES

void comprobarLectura() {									// Interrupci�n una vez por segundo.
	escribir = true;
}

// Calcula el valor medio de las lecturas cuando no hay consumo.
void valorMedio() {
	while (lecturas < 10000)					// Leemos durante 10 segundos.
	{
		if (millis() > milisegundos) {
			milisegundos++;
			corriente += analogRead(A0);
			lecturas++;
		}
	}
	compensacion = corriente / lecturas;
	lecturas = 0;
	corriente = 0;
}

// Pasa de la lectura del pin A0 a los amperios consumidos.
void conversion() {
	consumoMedio = corriente / lecturas;				// Hallamos la media de las lecturas.
	consumoMedio *= (corrienteMax / 512);				// Pasamos de "Lectura" a amperios.
}

void preparaFichero() {
	// Calculamos el nombre del fichero.
	sprintf(fichero, "Lecturas/%04u%02u%02u.csv", tiempo.year(), tiempo.month(), tiempo.day());
	// Conectamos con la tarjeta.
	if (!SD.exists(fichero)) {
		// El fichero no existe. Lo abrimos y ponemos el encabezado.
		ficheroCSV = SD.open(fichero, FILE_WRITE);
		if (ficheroCSV) {
			ficheroCSV.println(F("Id;Fecha_Hora;Tension;Corriente;Potencia;Lecturas;Subido"));
			ficheroCSV.close();
			escribirSD = true;
		}
		else {
			// Ha falado la creaci�n del fichero.
			escribirSD = false;
			Serial.println(F("Imposible abrir fichero."));
		}
	}
	else {
		// EL fichero existe. No es necesario crearlo.
		Serial.println(F("Fichero encontrado"));
		escribirSD = true;
	}
}
