#include <TLC5952.h>


// Konstruktor:
// Pin-Belegung +  Strom pro LED
TLC5952::TLC5952(PinName clk, PinName data, PinName latch, PinName blank,  uint8_t strom = 5) {
  int32_t registerCount = 0x25;
  clkout = new DigitalOut(clk);
  dataout = new DigitalOut(data);
  latchout = new DigitalOut(latch);
  blankout = new DigitalOut(blank);
  this->registerCount = registerCount;
  init_TLC5952(strom);
}

// Destruktor:
TLC5952::~TLC5952() {
  delete clkout;
  delete dataout;
  delete latchout;
  delete blankout;
}

/** Writes the given integer to the shift register
*/
void TLC5952::write(int data) {
  *latchout = 0;
  for (int i = registerCount - 1; i >=  0; i--) {
    wait_us(15);
    *clkout = 0;
    *dataout = (data & (1 << i)) != 0;
    *clkout = 1;
  }
  *latchout = 1;
}

// Balkenanzeige 1=LED1 an
// Anzahl leuchtender LED's übergeben
void TLC5952::LED_bar_down(int num) {
  num=(uint8_t)num;
  int c=0;
  long out=0;

  if(num>24 && num<126) {
    num=24;
  }
  else if (num>127 ) {
    num=0;
  }

  for(c=0;c<num;c++) {
    out=(out<<1)+1  ;
  }

  for(c=0;c<(24-num);c++) {
    out=(out<<1)+0  ;
  }
  c=0;

  write(out);
}

// Balkenanzeige 1=LED24 an
// Anzahl leuchtender LED's übergeben
void TLC5952::LED_bar_up(int num) {
  num=(uint8_t)num;
  int c=0;
  long out=0;

  if(num>24 && num<126) {
    num=24;
  }
  else if (num>127 ) {
    num=0;
  }

  for(c=0;c<num;c++) {
    out=(out<<1)+1  ;
  }
  write(out);
}

// 6 LED-Lauflicht für Ventile (pro IC 4 Stück)
void TLC5952::ventil_1_mov(int on_off) {
  if(on_off==1) {
    ventil_1 = mov(ventil_1);
    ventil_out();
  }
}

void TLC5952::ventil_2_mov(int on_off) {
  if(on_off==1) {
    ventil_2 = mov(ventil_2);
    ventil_out();
  }
}

void TLC5952::ventil_3_mov(int on_off) {
  if(on_off==1) {
    ventil_3 = mov(ventil_3);
    ventil_out();
  }
}

void TLC5952::ventil_4_mov(int on_off) {
  if(on_off==1) {
    ventil_4 = mov(ventil_4);
    ventil_out();
  }
}

// Einrichten und Strom für LED's setzen
void TLC5952::init_TLC5952(int strom) {
  int strom_blau=((strom*127)/max_strom_blau);
  int strom_rot=((strom*127)/max_strom_rot);
  int strom_gruen=((strom*127)/max_strom_gruen);
  long control=8;
  control = (control<<7)+strom_blau;
  control = (control<<7)+strom_gruen;
  control = (control<<7)+strom_rot;

  *blankout=1;
  write(control);
  write(0);
  *blankout=0;
}

// Hilfsfunktionen für Ventil-Lauflicht
int TLC5952::mov(int v) {
  if(v>16) {
    v=1;
  }
  else if (v==0) {
    v=1;
  }
  else {
    v=v<<1;
  }

  return v;
}

void TLC5952::ventil_out() {
  long out=0;
  out = ventil_1;
  out = (out<<6)+ventil_2;
  out = (out<<6)+ventil_3;
  out = (out<<6)+ventil_4;
  write(out);
}
