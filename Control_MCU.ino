
/**
 * @file Control_MCU.ino
 *
 * @mainpage Programa que permite el control de pruebas a un motor DC 
 * 
 * @section Programa que recibe por medio de la terminal comandos 
 * para activar el banco de pruebas de un motor DC aplicando una señal
 * pwm a un motor, de esta manera observando su comportamiento durante periodos de tiempo 
 * estipulados, con posibiliad de variar parametros configurables como periodo de aumento de la señal
 * pwm aplicada, periodo de muestreo, control de limite de corriente, observar los datos finales etc. 
 *
 * @section Notes
 * El programa solo tiene permitido 12 comandos con los cuales
 * el usuario puede interactuar con la terminal.
 * Estos son MODE STP, MODE TRG, PERIOD, FSAMPLE, AVERAGE, VERBOSE ON, VERBOSE OFF, STATE, MAXCURRENT, FDATA, STOP, RESET
 * 
 *
 * @section author Author
 * - Juan David Rios Rivera
 * - Mario Alejandro Tabares
 */
#include <String.h>                     /**< Se incluye libreria String para imprimir con formato por consola*/
#include "HX711.h"                      /**< Se incluye librería HX711 para manipular la celda de carga con su converson analogo digital de 24 bits*/
#include <stdint.h>                     /**< Se incluye libreria stdint para manejar variables tipo uint_t*/
#include <Arduino.h>                    /**< Se incluye libreria arduino para manejar variables tipo uint_t*/


//Definicion de pines de entradas y salidas
const int fuerzaPin = 8;                /**< Pin digital de entrada por el cual se lee el valor de la fuerza de empuje medida*/
const int CLK = 7;                      /**< Pin digital de salida que actúa como reloj para la libreria que controla la medida de fuerza de empuja de la celda de carga*/
const int temperaturePin = A0;          /**< Pin analógico de entrada de la variable temperatura */
const int currentPin = A1;              /**< Pin analógico de entrada para la medida de la corriente consumida por el motor*/
const int velocityPin = 2;              /**< Pin digital de entrada donde se conecta la señal de interrupción de las aspas del motor, para medir la velocidad*/
const int pwmPin = 5;                   /**< Pin digital de salida por donde sale la señal pwm para controlar la velocidad del motor*/

//Variables 
bool mode = 1;                             /**< Booleano que escoge los dos modos de operación*/
bool activate = 0;                         /**< Booleano que indica si esta activo o no el sistema */
bool verbose = 1;                          /**< Booleano que indica si escribir todo en consola a medida que se ejecuta el sistema y se toman las muestras*/
uint32_t vel = 0;                          /**< Variable que guarda la velocidad en RPM*/
uint8_t step_duty = 10;                    /**< Variable que controla el valor del duty de la señal pwm de 0 a 100%*/
uint32_t t_sample = 5;                     /**< Variable que guarda el periodo de muestreo en ms, por defecto son 200Hz = 5ms*/
uint32_t period = 1000;                    /**< Variable que guarda el periodo que dura un paso en modo STP, o todo el estimulo en modo TRG en ms*/
volatile unsigned long pulseCount = 0;     /**< Variable volátil para contar las interrupciones que generan las aspas al cruzar por el infrarrojo*/
uint8_t temperatura = 0;                   /**< Variable que guarda el valor de la temperatura medido por el sensor LM35 en grados*/
float corriente = 0;                       /**< Variable que guarda el valor de la corriente medida que consume el motor*/
float strong = 0;                          /**< Variable que gaurda el valor del empuje medido por la celda de carga*/
uint16_t limit = 1000;                     /**< Variable que establece el valor máximo de corriente que permite el sistema en miliamperios*/
uint8_t media_average = 1;                 /**< Variable que establece la cantidad de muestras que se promedian*/
uint32_t average[4][10];                   /**< Matriz que guarda los valor a promediar por cada variable, por ello son 4 filas, 4 variables medidas, máximo 10 valores se promedian*/
uint32_t tabla[40][5];                     /**< Matriz que guarda los valores leidos en cada muestra, ya sea promediada o no para luego mostrarla como una tabla en consola*/
uint8_t indice = 0;                        /**< Indice que recorre las posiciones de la matriz donde se guardan las mediciones de cada variable por cada  muestra*/

HX711 sensorFuerza;                        /**< Se define el sensor de fuerza usando la libreria HX711 para leer los valores de fuerza medidos*/
void setup() {
  /**
  Se inicializa los pines de entrada y salida, como entradas se tiene los sensores de temperatura,
  velocidad(interrupciones), corriente(puerto analógico), y fuerza. 
  Como salida se tiene el pin encargado de escribir de manera analógica un valor de pwm en el motor.
  Ademas se inicializa el monitor serial, para imprimir en consola y leer de la misma los comandos de usuario.
  Se inicializa la interrupción para el pin que cuenta las interrupciones de las aspas (velocidad),
  asignando una función encargada de ejecutarse cuando se presente un flanco de subida en este pin.
  Se inicializa la balanza y se le hace la tara, es decir se resta el valor de lo que tenga encima en ese momento, 
  esta balanza mide el empuje en gramos del motor, además se escala al valor previamente calculado para que mida en gramos.
  */

    pinMode(temperaturePin, INPUT);
    pinMode(velocityPin, INPUT); 
    pinMode(currentPin, INPUT);
    pinMode(pwmPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(velocityPin), countPulse, RISING); 
    Serial.begin(9600);                         
    sensorFuerza.begin(fuerzaPin, CLK);
    sensorFuerza.set_scale(489.4);
    sensorFuerza.tare(20);
    Serial.print(">>");
}

void loop() {

  /**
    Función que se ejecuta en loop infinito en el arduino, aqui el loop se pregunta constantemente que comandos ingresa el usuario,
    una vez ingresados los commandos cuando el sistema se activa con el booleano <activate> ingresa a la condicional que ejecuta la función correspondiente,
    ya sea en modo step, o en modo triangular. Una vez terminado de ejecutarse el comando, el sistema muestra un mensaje y se desactiva, esperando su póxima activación.
  */
  /*analogWrite(5, 255);
  Serial.println(analogRead(A1));  
  check_corriente();
  Serial.println(corriente);*/
  command();
  if(activate){
    if(mode){
        step(&period, &step_duty, &t_sample);
        Serial.println("STIMULUS COMPLETE");
        //activate = 0;
    }else{
        ramp(&period, &t_sample);
        Serial.println("STIMULUS COMPLETE");
        //activate = 0;
    }
  }

}

void check_temperatura()
{
   /**
    Función encargada de leer por el pin temperaturePin, el valor correspondiente a un valor analogo entre 0 y 1023 correspondiente a lso voltios
    que otorga el sensor LM35, aqui se hace la conversión y se asigna a la variable de temperatura el valor leído.  
   */
   float sen = analogRead(temperaturePin);
   temperatura = ((sen/1024.0)*500.0);

}

void check_rpm(uint32_t muestreo){
  /**
    Función encargada de evaluar el valor de la velocidad, mediante interrupciones. Lo hace llamando la funcion espera(), 
    pasándole como argumento el valor del perido durante el cual se toman las interrupciones, detiene las interrupciones, para que dentro de espera
    se activen una vez se empiecen a contar las interupciones, luego al salir de la función se vuelven a activar. 
    Vale resaltar que se reinician todas la variables de los sensores para evitar que si alguno no fuciona quede con un valor leido en
    una muestra anterior. 
    Luego de asignar el valor a la velocidad en la funcion espera(), se realiza la conversion a rpm, donde se tienen 2 aspas, 
    por tanto el valor de la velocidad sera multiplicado por 30 (60segundos en un minuto divido dos aspas) y se multiplica este valor por 
    1 segundo en micros  dividido el tiempo durante el cual se hallaron las interrupciones den microsegundos. Dicho valor para que no importa cual
    sea el valor de tiempo durante el cual se tomen las interrupciones. 
    @param muestreo variable que trae el valor en microsegundos durante el cual se toman las interrupciones y se leen valores de los sensores.
   */
  vel = 0;
  corriente = 0;
  strong = 0;
  temperatura = 0;
  noInterrupts();           
  espera(muestreo);
  interrupts();
  vel = (1000000.0/muestreo)*vel*30;
  
}

void check_corriente(){
  /**
    Función encargada de leer por el pin currentPin, el valor correspondiente a un valor analogo entre 0 y 1023 correspondiente a los voltios que
    caen sobre una resistencia en serie con el motor, aqui se hace la conversión a amperios y se asigna a la variable de corriente el valor leído.  
   */
  int lectura = analogRead(currentPin);
  corriente = ((lectura*1000.0)/1023.0)/0.5;

}

void check_empuje(){
    /**
    Función encargada de leer el valor de empuje con ayuda del la librería HX711,
    con la función preparada para tal fin "sensorFuerza.get_units()" 
    se lee la fuerza ejercida en gramossobre la celda de carga y se asigna a la variable strong
   */
   strong = sensorFuerza.get_units();
   if(strong<0){
     strong = 0;
   }

}


void espera(uint32_t valor)        
{
    /**
    Función que me permite esperar durante un tiempo para que se cuenten los pulsos proveniente de las interrupciones de las aspas
    sobre un sensor infrarrojo. Además se usa para que durante el tiempo que lee las interrupciones se lean las demás variables en 3 
    instante diferentes de tiempo, dando espacio suficiente para que el conversor ADC interno del arduino esté listo para ser usado 
    en otro pin una vez se haya desocupado con la anterior variable. Esto se hace con ayuda de un timer inicial, equivalente al 
    reloj del sistema y se le suman fracciones del tiempo que se esperará en total, además se usa una bandera entera para saber en que punto
    se lee cual variable, con el objetivo de solo leer una vez la variable cada tiempo total. Es decir solo en 3 momento se leen las variables,
    esto mejora el tiempo de ejecución y disminuye asi las pérdidas de tiempo en este loop, haciendo mas exactos los tiempos de muestreo.
  
    Cada vez que se inicia se reinicia el valor que tenga la variable pulseCount, se activan las interrupciones, y durante el tiempo
    se leen las mismas, luego se apagan las interrupciones y se asigna el valor de los pulsos contados a la velocidad. 

    @param valor variable de 32bits, la cual es un numero en microsegundos, el cual me indica el tiempo que se crea el loop para contar pulsos, 
    y durante el cual se leen las demás variable una única vez en cada caso.
    @return No retorna, pero cada vez que hay una interrupción se llama la función countPulse()
    */
    
    pulseCount = 0;               
    unsigned long Contador = micros() + valor;    /**contador < Variable contador local, que me indica el tiempo actual + la cantidad de tiempo en us que se hace el loop*/
    unsigned long cont_init = micros();
    uint8_t flag = 1;
    interrupts();   
    do { 
        if(micros()>cont_init+valor/4 && micros()<cont_init+valor/2 && flag ==1){
          check_temperatura();
          flag++;
        }else if(micros()>cont_init +  (valor/2) && micros()<cont_init + 3*(valor/4) && flag ==2){
          check_empuje();
          flag++;
        }else if(micros()>cont_init + 4* (valor/5) && flag ==3){
          check_corriente();
          flag++;
        }
        } while (Contador >= micros());
    noInterrupts();                                   /*! Desactivo interrupciones*/
    vel = pulseCount;
    
}

void countPulse() 
{
  /**
  Función que se ejecuta cada vez que hay una interrupción, es decir un flanco de subida en el pin 2 del arduino
  @param Ninguno
  @return No retorna nada, solo modifica la variable global volatil pulseCount, la cual se incrementa en uno cada vez que hay una interrupción
  */
  pulseCount++;  
}

uint8_t calc_average(uint32_t (&average)[4][10], uint8_t* i, uint8_t* dut){
  /**
    Función que calcula el promedio de los valores de una matriz, en este caso la matriz tiene tantas filas como variables, 
    y tantas columnas como muestras. La matriz se recorre por sus columnas hasta el punto indicado por la variable media_average, 
    la cual define la cantidad de muestras a promediar, por cada fila hay un resultado promedio resultado de sumar los valores de las 
    columnas de esa fila y dividir por el valor de media_average.
    Además de promediar, esta función permite guardar el valor promediado en una tabla para ser mostrado al finalizar mediante comandos.
    Los datos se pueden guardar en caso de haber sido promediados o no. Esto lo define si la variable de media_average es mayor a uno, 
    es decir se deben promediar valores. Los valores guardados por cada muestra se guardan en filas diferentes, hasta llenarse la tabla, 
    una vez llena la tabla se reescriben los valores en la tabla, empezando de arriba hacia abajo.

    @param average es una matriz de valores, con tantas filas como variables medidas y columnas como muestras
    @param i es un valor de indice que ayuda a conocer en que punto de la matriz estoy asignando un valor de muestra, 
    una vez se alcance el valor de media se calcula el promedio de valores y se reinicia este indice.
    @param dut el cual es el valor del duty que debe guardarse y dicho valor no se promedia
    @return i, el valor retornado es el valor del indice aumentado en uno, esto indica que la proxima vez que ingrese a guardar un valor se hará
    en la siguiente columna, esto con el objetivo de llevar la cuenta de en que punto debo hacer el promedio.

  */
  if(indice == 40){
      indice = 0;
  }
  if(media_average > 1){
    tabla[indice][4] = *dut;
    average[0][*i] = vel;
    average[1][*i] = corriente;
    average[2][*i] = temperatura;
    average[3][*i] = strong;
    if((*i) == media_average-1){
      for (int a = 0; a < 4; a++) {
        uint32_t avg = 0;
        for (int j = 0; j < media_average; j++) {
          avg += average[a][j];
        }
        avg = avg/media_average;
        tabla[indice][a] = avg;
      }
      indice++;
      return 0;
    }else{
      return ((*i) + 1);
    }
  }else{
    tabla[indice][0] = vel;
    tabla[indice][1] = corriente;
    tabla[indice][2] = temperatura;
    tabla[indice][3] = strong;
    tabla[indice][4] = *dut;
    indice++;
  }
}

void step(uint32_t* period_step, uint8_t* paso, uint32_t* samples){
  /**
  Función que realiza cambios del duty aplicado al motor, durante un periodo de tiempo y toma los datos de las variables de interés 
  en ciertos periodos. Esta función ejecuta dos loops, uno dentro de otro, el primero loop, es decir el mas interno se ejecuta cada
  vez que se alcanza el valor de un periodo  de paso de duty, es decir mediante un timer en micros + el periodo en micros, se compara 
  con el reloj en micros actual, y si dicho valor es superado por el reloj del sistema (micros()) entonces se evalúa si tomar una muestra, 
  esta evaluacion se hace con ayuda de otra variable tipo timer + periodo de muestreo, aqui se evalua la condición de que dicho periodo
  de muestreo se ha alcanzado entonces se llama a la función muestreo(). Luego se llama la funcion calc_average() para guardar los valores 
  del muestreo o en caso de necesitar promediarse, se guardan en la matriz hasta que se cumpla con la cantidad de muestras para realizar el promedio.
  En este punto se reinicia la variable timer1 para asignarle el nuevo valor de tiempo y que espere hasta que se cumpla el nuevo periodo para tomar la siguiente muestra.

  El segundo loop se ejecuta de manera similar al primero, la condición de terminación de este loop es alcanzar el máximo valor del duty, 
  puesto que este loop incrementan en pasos el duty sumandole el valor en la direccion a la que apunta la varibale paso, 
  cada vez que se cumple la condicion del anterior loop, es decir cada vez que se ha completado el periodo correspondiente a un paso. 
  En este loop, se evalúa el duty de 0 a 100% y se asigna un valor al pin de salida pwmPin entre 0 y 255,
  el cual aumenta la velocidad del motor. En este loop tambien se crea la variable de tiempo que se debe alcanzar hasta el nuevo paso de duty, 
  es decir el periodo en microsegundos que se cumple en el loop interno. 
  Vale aclara que una vez tomada una muestra se debe evaluar si el corriente no se ha superado, por ello se evalua si el valor de la corriente se
  encuentra en los limites, de no hacer el sistema se interrupto aburptamente con un break, saliendo de los loops y apagando el motor. Al finalizar este proceso
  el motor se apaga puesto que se ha alacanzado el máximo duty, asi que no se puede dejar encendido.

  @param period_step Variable tipo apuntador, cuya dirección guarda el valor en milisegundos para el tamaño del periodo del step
  @param paso  Variable tipo apuntador cuya dirección guarda un valor entre 0 y 100 para el tamaño del duty aplicado al motor para variar la velocidad
  @param samples Variable tipo apuntador cuya dirección guardae el valor de cada cuantos microsegundos se debe tomar los datos, es decir frecuencia de muestreo
  */

  uint8_t duty = 0;
  uint32_t period_us = (*period_step) * 1000; // periodo del step en microsegundos
  uint32_t  t_samples = (*samples) * 1000;    // periodo de muestreo en microsegundos
  uint8_t duty_analog;
  uint8_t index = 0;
  uint32_t timer1 = micros();   
  indice = 0;
  do{
    duty_analog =  map(duty, 0, 100, 0, 255);
    analogWrite(pwmPin, duty_analog);
    uint32_t timer = micros();
    do{
      command();
      if(!activate){
        analogWrite(pwmPin, 0);
        break;
      }
      if((timer1 + t_samples)<=micros()){
          muestreo(&t_samples, &duty);          //Cada cuantos microsegundos tomo una muestra  
          index = calc_average(average, &index, &duty);
          timer1 = micros();
          timer1 += t_samples;
        }
          /*muestreo(t_samples);          //Cada cuantos microsegundos tomo una muestra
          if(corriente >limit){
            break;
          }
          index = calc_average(average, &index, &duty);
          nsamples += t_samples;*/
    }while((timer + period_us) >= micros());
    if((corriente >limit)){
        analogWrite(pwmPin, 0);
        break;
    }
    duty += *paso;
  }while (duty<=100);
  analogWrite(pwmPin, 0);
  
}

void ramp(uint32_t* period, uint32_t* samples){
  /**
  Función que realiza un aumento lineal del duty de 0 a 100 y de 100 a 0, en un periodo establecido, y toma muestras cada que se cumple 
  un periodo de muestreo. Esta función ejecuta dos loops, en cada caso es decir en subid de la rampa y en bajada, de manera similar a la 
  funcion step(), usa timer con el objetivo de saber en que punto se debe tomar la muestra. En este caso el loop interior se ejecuta mientras
  uno de los timer+ periodo del paso de duty sea mayor al reloj actual. Mientras esta condicion se cumple se ejcuta dentro del loop la 
  condición de si otro timer + periodo de muestreo es menor al reloj actual, si esto es cierto quiere decir que se ha pasado por el periodo de muestreo
  entonces se llama a la función muestreo(), seguida de calc_average, si es necesario hacer muestreo, aunque fue verificado experimentalmente que
  no es adecuado, debido a que los periodos de cambio de duty son muy cortos, por tanto promediar muestras equivale a mezclar muestras de un valor
  de duty con otro totalmente diferente. Luego de esto se reinicia el timer de esta condición para contar a partir de aqui el nuevo valor del periodo
  de muestreo que debe alcanzarse para tomar una nueva muestra.

  En el loop externo, se evalua si se ha alcanzao el valor máximo de duty, en caso de hacerse se sale de este proceso y se pasa al siguiente el cual
  es similar, tan solo que en lugar de aumentar el duty en pasos de a 1% se decrementa. En este punto se evalúa si se ha superado la corriente
  en una de las muestras, deteniendo el sistema de inmediato y apagando el motor en caso de ser asi.
  En este loop se crea la variable de tiempo para evaluar cada cuanto aumentar el duty, es decir cada cuanto debe acabarse el loop interno para 
  aumentar en un porcentaje el duty. Se hac eun mapeo para asignar valores entre 0 y 255 al pwmPin.

  @param period Variable tipo apuntador cuya dirección guarda el valor en milisegundos para el tamaño del periodo total de ejecución de la rampa en ascenso y descenso
  @param samples Variable tipo apuntador cuya dirección guarda el valor en milisegundos del periodo de muesreto para tomar los datos, es decir frecuencia de muestreo
  */

  uint8_t duty = 0;
  uint32_t period_us = (*period) * 5;   // Multiplico el periodo por 1000 para que esté en micros y luego divido por 200, para 100% de subida y 100% de bajada
  uint32_t t_samples = (*samples) * 1000;
  uint8_t duty_analog;
  uint32_t timer1 = micros();           //Timer para esperar hasta la próxima muestra
  uint8_t index = 0;
  indice = 0;
  do{
    duty_analog =  map(duty, 0, 100, 0, 255);
    analogWrite(pwmPin, duty_analog);
    uint32_t timer = micros();             //Timer para esperar hasta el proximo aumento de duty
    do{
        command();
        if((timer1 + t_samples)<=micros()){
          muestreo(&period_us, &duty);          //Cada cuantos microsegundos tomo una muestra  
          index = calc_average(average, &index, &duty);
          timer1 = micros();
          timer1 += t_samples;
        }
    }while((timer + period_us) >= micros());
    if((corriente >limit) || !activate){
      analogWrite(pwmPin, 0);
      break;
    }
    duty++;
  }while (duty<=100);
  analogWrite(pwmPin, duty_analog);
  do{
    duty_analog =  map(duty, 0, 100, 0, 255);
    analogWrite(pwmPin, duty_analog);
    uint32_t timer = micros();
    do{ 
        command();
        if((timer1 + t_samples)<=micros()){
          muestreo(&period_us, &duty);          //Cada cuantos microsegundos tomo una muestra  
          index = calc_average(average, &index, &duty);
          timer1 = micros();
          timer1 += t_samples;
        }
    }while((timer + period_us) >= micros());
    if((corriente >limit) || !activate){
      analogWrite(pwmPin, 0);
      break;
    }
    duty--;
  }while (duty>0);

}

void muestreo(uint32_t* samples, uint8_t* duty){
  /**
  Función que realiza una muestra de las variables sensadas, llamando a la funcion check_rpm() y entregandole el valor del periodo en micros
  durante el cual se calcularán los valores. 
  Después dependiendo de la variable booleana verbose muestra en consola las variables medidas o no. 

  @param samples variable tipo puntero cuya direccion apunta a un valor de 32bits con el valor en microsegundos durante el cual se hará
  la evaluación de los sensores.
  */
  check_rpm((*samples));
  if(verbose){
    Serial.println(String(corriente) + "," + String(vel)+ "," +String(strong) + "," + String(temperatura) + "," + String(*duty)); 
  }

}

void command(){
  /**
    Función que lee los comandos que se escriben por consola serial.
    if(Serial. available)  Verifica si la terminal esta activa y a la espera de algun comando, despues de esto recupera el dato escrito en consola
    y lo configura para que todo quede en mayúscula, luego evalua en una serie de condicionales si cumple alguno y dependiendo de esto,
    asigna valores a variables, o activa banderas, incia sistemas, reestablece valores a variables, o asigna valores nuevos a variable existentes.
  */

  if (Serial.available()>0){          
  
    String command = Serial.readStringUntil('\n');    /*! Pide el comando de la terminal */
    command.toUpperCase();                            /*! define siempre mayuscula el comando del usuario */
    if(command.indexOf("MODE STP ") != -1){
         /**if(comand=="MODE STP") Realiza estimulo a través de escalones continuos del porcentaje seleccionado*/
        int step = command.substring(command.indexOf(" ")+4, command.lastIndexOf("\n")).toInt(); 
        step_duty = step;
        mode = 1;
        Serial.print("DONE"); 
    }

    if(command=="MODE TRG"){
         /**if(comand=="MODE TRG") Realiza estimulo en forma triangular variando la señal de forma lineal de 0 a 100% y de 100 a 0%*/
        mode = 0;
        Serial.print("DONE");
    }

    if(command.indexOf("PERIOD") != -1){
         /**if(comand=="PERIOD") Establece el tiempo de duración de los escalones en el modo STEP en milisegundos, y en el modo TRG por defecto son 500ms*/
        uint32_t per = command.substring(command.indexOf(" ")+1, command.lastIndexOf("\n")).toInt(); 
        period = per;
        Serial.print("DONE");
        
    }
     if(command.indexOf("FSAMPLE") != -1){
         /**if(comand=="FSAMPLE") Establece un nuevo valor de frecuencia de muestreo, la frecuencia por defecto son 200Hz*/
        int freq = command.substring(command.indexOf(" ")+1, command.lastIndexOf("\n")).toInt(); 
        t_sample = 1000/freq; //en milisegundos
        Serial.print("DONE");
        
    }
    if(command.indexOf("AVERAGE") != -1){
         /**if(comand=="AVERAGE") Establece el numero de muestras que se promedian por un dato almacenado, el numero de muestras promedio es 1, sin promedio*/
        uint8_t media = command.substring(command.indexOf(" ")+1, command.lastIndexOf("\n")).toInt(); 
        media_average = media;
        Serial.print("DONE");
        
    }
    if(command.indexOf("MAXCURRENT") != -1){
         /**if(comand=="MAXCURRENT") Establece un valor de corriente máximo que la prueba no debe superar, si lo hace el sistema se detiene, por defecto es 1 amperio*/
        uint16_t lim = command.substring(command.indexOf(" ")+1, command.lastIndexOf("\n")).toInt(); 
        limit = lim;
        Serial.print("DONE");
        
    }
     if(command=="VERBOSE ON"){
         /**if(comand=="VERBOSE") Establece si durante un periodoo de captura los datos se transmiten inmediato a consola, apagado por defecto*/
        verbose = 1;
        Serial.print("DONE");
        
    }
    if(command=="VERBOSE OFF"){
         /**if(comand=="VERBOSE") Establece si durante un periodoo de captura los datos se almacenan para enviarse si se usa el comando FDATA, encendido por defecto*/
        verbose = 0;
        Serial.print("DONE");
        
    }
    if(command=="START"){
      activate = 1;
    }
    if(command=="STATE"){
         /**if(comand=="STATE") Imprime el modo en el que se esta utilizando el sistema de banco de pruebas
          modo, estado, periodo, frecuencia, average, verbose, y max current
        */
        if(mode){
          Serial.println("Modo: STEP");
        }else{
          Serial.println("Modo: TRG");
        }
        Serial.println("Periodo: " + String(period));
        Serial.println("Frecuencia de muestreo: "  + String(1000000/(t_sample*1000)) + "Hz - " + "Periodo Muestreo: " + String(t_sample) + "ms");
        Serial.println("Average : " + String(media_average));
        Serial.println("Paso ciclo Dureza: " + String(step_duty));
        if(verbose){
          Serial.println("Verbose: ON");
        }else{
          Serial.println("Verbose: OFF");
        }
        Serial.println("Corriente Máxima: " + String(limit) + "mA");
        Serial.print("DONE");
        
    }
    if(command=="FDATA"){
         /**if(comand=="FDATA") Imprime una tabla con los datos que seleccionemos*/
        Serial.println("-------------------------TABLA--------------------------");
        Serial.println("| Corriente  | Velocidad | Fuerza | Temperatura | Duty |");
        for(int i = 0; i<40; i++){
          Serial.println(String(tabla[i][1]) + "," + String(tabla[i][0]) + "," + String(tabla[i][3]) + "," +  String(tabla[i][2]) + "," + String(tabla[i][4]));
        }
        
    }
    if(command=="STOP"){
       /**if(comand=="STOP")
        Detiene el estimulo aplicado al motor, este es el estado por defecto con el estímulo desactivado
      */
        activate = 0;
        Serial.print("STIMULUS DISABLE");
    }
    
    if(command=="RESET"){
      /**if(comand=="RESET")) Restablece los valores por defecto que se encuentran en el state.
      */
      mode = 1;
      period = 1000;
      step_duty = 10;
      t_sample = 5;
      limit = 1000;
      for(int i = 0; i<40; i++){
        for(int j = 0; j<5; j++){
          tabla[i][j] = 0;
        }
      }
    }
    Serial.print('\n');   /*! Salto de linea */
    Serial.print(">>");   /*! imprime >> en terminal */
  }
    
}
