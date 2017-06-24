#ifndef TLC5952_H

#define TLC5952_H

#include <mbed.h>

// Max-Ströme vorgegeben durch R1 (2,4k)
#define max_strom_rot 20
#define max_strom_blau 15
#define max_strom_gruen 20


class TLC5952 {

  public :
    // Konstruktor:
    // Pin-Belegung +  Strom pro LED
    TLC5952(PinName , PinName , PinName , PinName ,  uint8_t ) ;

    // Destruktor:
    ~TLC5952();

    /** Writes the given integer to the shift register
    */
    void write(int ) ;

    // Balkenanzeige 1=LED1 an
    // Anzahl leuchtender LED's übergeben
    void LED_bar_down(int );

    // Balkenanzeige 1=LED24 an
    // Anzahl leuchtender LED's übergeben
    void LED_bar_up(int );

    // 6 LED-Lauflicht für Ventile (pro IC 4 Stück)
    void ventil_1_mov(int);

    void ventil_2_mov(int);

    void ventil_3_mov(int);

    void ventil_4_mov(int);


  private :
    DigitalOut *clkout;
    DigitalOut *dataout;
    DigitalOut *latchout;
    DigitalOut *blankout;
    int8_t registerCount;
    int ventil_1;
    int ventil_2;
    int ventil_3;
    int ventil_4;

    // Einrichten und Strom für LED's setzen
    void init_TLC5952(int );

    // Hilfsfunktionen für Ventil-Lauflicht
    int mov(int );


    void ventil_out();

};

#endif
