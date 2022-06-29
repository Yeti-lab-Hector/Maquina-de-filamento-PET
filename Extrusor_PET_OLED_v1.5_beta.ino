/*
 * Extrusor de botellas PET versión v1.5 ultimo debug (16/06/2022)
 * Software y hardwar implementados por Hector santos
 * yeti-lab.blogspot.com
 * yeti-lab en youtube
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <PIDController.hpp>

#include <EEPROM.h>
#include <avr/pgmspace.h> 

const uint16_t PROGMEM temptable  [102][2] =
// Thermistor: 100K nct chino pull-up 4k7
{
{1, 3318}, // 829.50 C
{11, 1582}, // 395.50 C
{21, 1189}, // 297.25 C
{31, 1125}, // 281.25 C
{41, 1075}, // 268.75 C
{51, 1031}, // 257.75 C
{61, 996}, // 249.00 C
{71, 960}, // 240.00 C
{81, 935}, // 233.75 C
{91, 911}, // 227.75 C
{101, 883}, // 220.75 C
{111, 866}, // 216.50 C
{121, 847}, // 211.75 C
{131, 830}, // 207.50 C
{141, 813}, // 203.25 C
{151, 799}, // 199.75 C
{161, 786}, // 196.50 C
{171, 771}, // 192.75 C
{181, 756}, // 189.00 C
{191, 744}, // 186.00 C
{201, 734}, // 183.50 C
{211, 721}, // 180.25 C
{221, 709}, // 177.25 C
{231, 699}, // 174.75 C
{241, 692}, // 173.00 C
{251, 681}, // 170.25 C
{261, 671}, // 167.75 C
{271, 662}, // 165.50 C
{281, 654}, // 163.50 C
{291, 648}, // 162.00 C
{301, 638}, // 159.50 C
{311, 627}, // 156.75 C
{321, 622}, // 155.50 C
{331, 612}, // 153.00 C
{341, 606}, // 151.50 C
{351, 598}, // 149.50 C
{361, 589}, // 147.25 C
{371, 579}, // 144.75 C
{381, 575}, // 143.75 C
{391, 568}, // 142.00 C
{401, 560}, // 140.00 C
{411, 552}, // 138.00 C
{421, 547}, // 136.75 C
{431, 541}, // 135.25 C
{441, 534}, // 133.50 C
{451, 527}, // 131.75 C
{461, 520}, // 130.00 C
{471, 512}, // 128.00 C
{481, 505}, // 126.25 C
{491, 502}, // 125.50 C
{501, 495}, // 123.75 C
{511, 486}, // 121.50 C
{521, 484}, // 121.00 C
{531, 478}, // 119.50 C
{541, 471}, // 117.75 C
{551, 468}, // 117.00 C
{561, 460}, // 115.00 C
{571, 454}, // 113.50 C
{581, 448}, // 112.00 C
{591, 441}, // 110.25 C
{601, 435}, // 108.75 C
{611, 428}, // 107.00 C
{621, 423}, // 105.75 C
{631, 418}, // 104.50 C
{641, 410}, // 102.50 C
{651, 406}, // 101.50 C
{661, 400}, // 100.00 C
{671, 390}, // 97.50 C
{681, 387}, // 96.75 C
{691, 378}, // 94.50 C
{701, 372}, // 93.00 C
{711, 367}, // 91.75 C
{721, 358}, // 89.50 C
{731, 352}, // 88.00 C
{741, 344}, // 86.00 C
{751, 336}, // 84.00 C
{761, 328}, // 82.00 C
{771, 321}, // 80.25 C
{781, 315}, // 78.75 C
{791, 312}, // 78.00 C
{801, 304}, // 76.00 C
{811, 298}, // 74.50 C
{821, 292}, // 73.00 C
{831, 280}, // 70.00 C
{841, 276}, // 69.00 C
{851, 266}, // 66.50 C
{861, 258}, // 64.50 C
{871, 248}, // 62.00 C
{881, 236}, // 59.00 C
{891, 226}, // 56.50 C
{901, 212}, // 53.00 C
{911, 203}, // 50.75 C
{921, 192}, // 48.00 C
{931, 179}, // 44.75 C
{941, 165}, // 41.25 C
{951, 150}, // 37.50 C
{961, 136}, // 34.00 C
{971, 112}, // 28.00 C
{981, 87}, // 21.75 C
{991, 53}, // 13.25 C
{1001, 41}, // 10.25 C
{1011, 4}, // 1.00 C
};
/*const uint16_t PROGMEM temptable[102][2] = 
// Table for the Extruder.
// Thermistor: EPCOS B57560G104F
{
   {1, 3324}, // 831.245067985 C
   {11, 1585}, // 396.252474229 C
   {21, 1325}, // 331.32932964 C
   {31, 1190}, // 297.645114323 C
   {41, 1101}, // 275.474490595 C
   {51, 1036}, // 259.168154846 C
   {61, 985}, // 246.369362718 C
   {71, 943}, // 235.884012164 C
   {81, 908}, // 227.028859934 C
   {91, 877}, // 219.378723164 C
   {101, 850}, // 212.652165901 C
   {111, 826}, // 206.653862026 C
   {121, 804}, // 201.243058701 C
   {131, 785}, // 196.3151827 C
   {141, 767}, // 191.790546509 C
   {151, 750}, // 187.607114114 C
   {161, 734}, // 183.715699896 C
   {171, 720}, // 180.076684692 C
   {181, 706}, // 176.657710401 C
   {191, 693}, // 173.432024496 C
   {201, 681}, // 170.377267302 C
   {211, 669}, // 167.474567825 C
   {221, 658}, // 164.70785891 C
   {231, 648}, // 162.063351148 C
   {241, 638}, // 159.529123527 C
   {251, 628}, // 157.094801199 C
   {261, 619}, // 154.751299129 C
   {271, 609}, // 152.49061616 C
   {281, 601}, // 150.305668094 C
   {291, 592}, // 148.190151275 C
   {301, 584}, // 146.138430239 C
   {311, 576}, // 144.145444526 C
   {321, 568}, // 142.206630857 C
   {331, 561}, // 140.317857748 C
   {341, 553}, // 138.475370233 C
   {351, 546}, // 136.675742876 C
   {361, 539}, // 134.915839614 C
   {371, 532}, // 133.192779262 C
   {381, 526}, // 131.503905728 C
   {391, 519}, // 129.846762188 C
   {401, 512}, // 128.219068561 C
   {411, 506}, // 126.618701801 C
   {421, 500}, // 125.043678555 C
   {431, 493}, // 123.492139831 C
   {441, 487}, // 121.962337391 C
   {451, 481}, // 120.452621604 C
   {461, 475}, // 118.961430553 C
   {471, 469}, // 117.487280209 C
   {481, 464}, // 116.028755524 C
   {491, 458}, // 114.584502302 C
   {501, 452}, // 113.153219732 C
   {511, 446}, // 111.733653483 C
   {521, 441}, // 110.324589267 C
   {531, 435}, // 108.924846792 C
   {541, 430}, // 107.533274022 C
   {551, 424}, // 106.148741686 C
   {561, 419}, // 104.770137965 C
   {571, 413}, // 103.396363295 C
   {581, 408}, // 102.026325232 C
   {591, 402}, // 100.658933307 C
   {601, 397}, // 99.2930938245 C
   {611, 391}, // 97.9277045264 C
   {621, 386}, // 96.5616490636 C
   {631, 380}, // 95.193791196 C
   {641, 375}, // 93.8229686397 C
   {651, 369}, // 92.4479864717 C
   {661, 364}, // 91.0676099844 C
   {671, 358}, // 89.6805568711 C
   {681, 353}, // 88.2854885992 C
   {691, 347}, // 86.8810008054 C
   {701, 341}, // 85.4656125124 C
   {711, 336}, // 84.0377539283 C
   {721, 330}, // 82.5957525369 C
   {731, 324}, // 81.1378171244 C
   {741, 318}, // 79.6620193031 C
   {751, 312}, // 78.1662719881 C
   {761, 306}, // 76.6483041443 C
   {771, 300}, // 75.1056309408 C
   {781, 294}, // 73.5355182138 C
   {791, 287}, // 71.9349398223 C
   {801, 281}, // 70.3005260563 C
   {811, 274}, // 68.6285006809 C
   {821, 267}, // 66.9146034001 C
   {831, 260}, // 65.153993415 C
   {841, 253}, // 63.3411281701 C
   {851, 245}, // 61.4696091069 C
   {861, 238}, // 59.5319829034 C
   {871, 230}, // 57.5194816754 C
   {881, 221}, // 55.4216779533 C
   {891, 212}, // 53.22601822 C
   {901, 203}, // 50.9171793761 C
   {911, 193}, // 48.4761601261 C
   {921, 183}, // 45.8789633185 C
   {931, 172}, // 43.094624341 C
   {941, 160}, // 40.0821493573 C
   {951, 147}, // 36.7855424039 C
   {961, 132}, // 33.1252686346 C
   {971, 115}, // 28.9825340427 C
   {981, 96}, // 24.1675388389 C
   {991, 73}, // 18.3466418647 C
   {1001, 43}, // 10.8401360547 C
   {1011, 0}, // -0.157748974426 C
};*/
// Definir constantes
#define ANCHO_PANTALLA 128 // ancho pantalla OLED
#define ALTO_PANTALLA 64 // alto pantalla OLED
 
// Objeto de la clase Adafruit_SSD1306
Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, -1);

static const unsigned char PROGMEM icono_config[120] = {
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x03, 0x80, 0x00, 
    0x00, 0x03, 0x80, 0x00, 
    0x00, 0x07, 0xc0, 0x80, 
    0x03, 0x07, 0xc1, 0xc0, 
    0x07, 0x9f, 0xf3, 0xe0, 
    0x0f, 0xff, 0xff, 0xf0, 
    0x0f, 0xff, 0xff, 0xe0, 
    0x07, 0xf0, 0x1f, 0xc0, 
    0x03, 0xc0, 0x07, 0x80, 
    0x03, 0x80, 0x03, 0x80, 
    0x07, 0x81, 0xc3, 0xc0, 
    0x07, 0x00, 0xe1, 0xc0, 
    0x1f, 0x00, 0x71, 0xf8, 
    0x7f, 0x00, 0x71, 0xfc, 
    0x7f, 0x10, 0xf1, 0xfc, 
    0x7f, 0x1d, 0xf1, 0xfc, 
    0x1f, 0x1f, 0xf0, 0xf8, 
    0x07, 0x0f, 0xf8, 0x40, 
    0x07, 0x87, 0xfc, 0x00, 
    0x03, 0xc0, 0x1e, 0x00, 
    0x03, 0xe0, 0x0f, 0x00, 
    0x07, 0xf0, 0x07, 0x80, 
    0x0f, 0xff, 0xc3, 0xc0, 
    0x1f, 0xff, 0xe1, 0xe0, 
    0x07, 0x8f, 0xe0, 0xf0, 
    0x03, 0x07, 0xc0, 0xf8, 
    0x02, 0x07, 0xc0, 0x78, 
    0x00, 0x03, 0x80, 0x10, 
    0x00, 0x03, 0x80, 0x00
};

static const unsigned char PROGMEM icono_Down[32] = {
    0x00, 0x00, 
    0x00, 0x00, 
    0x00, 0x00, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x3f, 0xf8, 
    0x1f, 0xf0, 
    0x0f, 0xe0, 
    0x07, 0xc0, 
    0x03, 0x80, 
    0x01, 0x00, 
    0x00, 0x00, 
    0x00, 0x00
};


static const unsigned char PROGMEM icono_Up[32] = {
    0x00, 0x00, 
    0x00, 0x00, 
    0x01, 0x00, 
    0x03, 0x80, 
    0x07, 0xc0, 
    0x0f, 0xe0, 
    0x1f, 0xf0, 
    0x3f, 0xf8, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x07, 0xc0, 
    0x00, 0x00, 
    0x00, 0x00, 
    0x00, 0x00
};

 static const unsigned char PROGMEM icono_nozzle[32] = {
      0x3f, 0xf0, 
    0x7f, 0xf8, 
    0xff, 0xfc, 
    0xff, 0xfc, 
    0xff, 0xfc, 
    0x3f, 0xf0, 
    0x3f, 0xf0, 
    0x3f, 0xf0, 
    0xff, 0xfc, 
    0xff, 0xfc, 
    0xff, 0xfc, 
    0xff, 0xfc, 
    0x1c, 0xe0, 
    0x08, 0x40, 
    0x0c, 0xc0, 
    0x03, 0x00
 };
static const unsigned char PROGMEM Logo[896] = {
0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 
    0x00, 0x0f, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 
    0x00, 0x3f, 0xff, 0xff, 0xc0, 0x30, 0x01, 0xff, 0xf8, 0x00, 0xff, 0xff, 0x80, 0x00, 
    0x00, 0x7f, 0xff, 0xff, 0xc0, 0x30, 0x00, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xc0, 0x00, 
    0x00, 0xff, 0xff, 0xff, 0x80, 0x18, 0x00, 0xff, 0xf0, 0x01, 0xff, 0xff, 0xe0, 0x00, 
    0x01, 0xff, 0xff, 0xff, 0x80, 0x1c, 0x00, 0x7f, 0xe0, 0x01, 0xff, 0xff, 0xf0, 0x00, 
    0x01, 0xff, 0xff, 0xff, 0xc0, 0x0f, 0x00, 0x7f, 0xe0, 0x03, 0xff, 0xff, 0xf0, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xe0, 0x01, 0xc0, 0x3f, 0xc0, 0x03, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x40, 0x3f, 0xc0, 0x07, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x60, 0x1f, 0x80, 0x07, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x20, 0x1f, 0x80, 0x0f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x20, 0x0f, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x30, 0x0f, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x18, 0x06, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x08, 0x07, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x06, 0x03, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x03, 0x03, 0x80, 0x7f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x01, 0x81, 0x80, 0x7f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x81, 0xc0, 0x7f, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x80, 0xc0, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0xc0, 0xe0, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x40, 0x61, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x60, 0x71, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x20, 0x33, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x20, 0x3b, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xfc, 0x02, 0x00, 0x00, 0x10, 0x1f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xf8, 0x07, 0x80, 0x00, 0x18, 0x1f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xf0, 0x0f, 0x80, 0x00, 0x08, 0x1f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xe0, 0x1f, 0x80, 0x00, 0x0c, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xc0, 0x7f, 0x80, 0x00, 0x07, 0xef, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xc0, 0xff, 0x80, 0x00, 0x01, 0xaf, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0x87, 0xff, 0xc0, 0x01, 0x80, 0x2f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0x8f, 0xff, 0xc0, 0x00, 0xc0, 0x6f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xcf, 0xff, 0xe0, 0x00, 0xf0, 0xcf, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xef, 0xff, 0xe0, 0x00, 0xdf, 0x8f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0xc7, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0xc0, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x00, 0xc0, 0x40, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x01, 0xc0, 0x60, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xe0, 0x30, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x0f, 0xe0, 0x10, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xf0, 0x08, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xf0, 0x0c, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xf8, 0x04, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xfc, 0x06, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xfe, 0x02, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xfe, 0x01, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0x01, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x03, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0x80, 0xcf, 0xff, 0xff, 0xff, 0xf8, 0x00, 
    0x01, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0x80, 0x4f, 0xff, 0xff, 0xff, 0xf0, 0x00, 
    0x00, 0xff, 0xff, 0xff, 0xe0, 0x1f, 0xfe, 0x00, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 
    0x00, 0xff, 0xff, 0xff, 0xc0, 0x3f, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 
    0x00, 0x3f, 0xff, 0xfc, 0x00, 0x3f, 0xf8, 0x07, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 
    0x00, 0x1f, 0xff, 0xfc, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 
    0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};




 //////PID//////

const byte PIN_OUTPUT = 6;
float kp=23.05, ki=2.0 ,kd=66.47;
PID::PIDParameters<double> parameters(kp,ki,kd);
PID::PIDController<double> pidController(parameters);

/////variables para el termistor///
uint16_t Vo;


unsigned long Time1=0;
unsigned long Time2=0;
unsigned long Time3=0;

uint8_t tempGraf=0;

float temp=0;
uint16_t tempMeta; /////////Temperatura del bloque/////////


const uint8_t okBoton=3;
const uint8_t LBoton=4;
const uint8_t RBoton=5;
uint8_t menu=2;

uint16_t lecturaEstable=0;
float lectura;
float temperatura;

bool disMotor=true;
uint16_t velMotor;  // Velocidad del motor

///////////motor PAP//////////
const uint8_t disableMotorPin=11;
const uint8_t dirMotorPin=10;
const uint8_t stepMotorPin=9;
const uint8_t SFilamento=7;

bool EnCalefactor=false;

uint8_t dataGrafic[128]; /////Varibles para la grafica+++++++++++++++++++++++++++++
uint8_t scroll=1;
uint8_t bufferScroll;

bool ok=false;
bool Config=false;
volatile unsigned int pasos=0;
unsigned long tiempoFucnionando=0; 
bool reloj=false;
bool Auto=true;
bool Auto1=true;
bool fin=true;
uint8_t horas=0,minutos=0,segundos=0;
int tiempo=0;
uint8_t metros=0;
float distancia;


//////////////////////////////////////////////////////////////////////////
void setup() {
for(int i=0;i<128;i++)dataGrafic[i]=100; // pone a 0 la grafica

//Serial.begin(9600);
actualizaVariables();

pinMode(disableMotorPin,OUTPUT);
pinMode(dirMotorPin,OUTPUT);
pinMode(stepMotorPin,OUTPUT);

digitalWrite(disableMotorPin,HIGH);
digitalWrite(dirMotorPin,HIGH);
//tone(stepMotorPin,velMotor);
  
  delay(100);
  // Iniciar pantalla OLED en la dirección 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  display.clearDisplay();
  
  
   display.drawBitmap(10,0, Logo, 108, 64, SSD1306_WHITE);
   display.setTextSize(1);
   display.setCursor(80, 55);
   display.setTextColor(SSD1306_INVERSE);
   display.print("v1.5");             ////La version
  display.display();
   
  display.setTextSize(2);
  pinMode(2,INPUT); //interrupciones
  pinMode(PIN_OUTPUT,OUTPUT);
  pinMode(okBoton,INPUT_PULLUP);
  pinMode(RBoton,INPUT_PULLUP);
  pinMode(LBoton,INPUT_PULLUP);
  pinMode(SFilamento,INPUT_PULLUP);
  




delay(1500);
  display.clearDisplay();
  Time1=millis();
  
  attachInterrupt(digitalPinToInterrupt(2), interrupcion, FALLING);

   pidController.Input = temperatura;
  pidController.Setpoint = tempMeta;
  pidController.TurnOn();
 
}
/////////////////////////////////////////////////////////////////////////////////



void loop() {

  PIDtemp(); 
if(disMotor)noTone(stepMotorPin);
 
 if (reloj){
    tiempo=((millis()-tiempoFucnionando)/1000);
    
    horas=0;
    minutos=0;
    segundos=0;
    tiempo;
    while(tiempo>=3600){
      horas++;
      tiempo=tiempo-3600;
    }
     while(tiempo>=60){
      minutos++;
      tiempo=tiempo-60;
    }
    segundos=tiempo;
  }
  
  if(Auto1 && Auto && setTemp()>tempMeta){ ///arranque automatico
    Auto1=false;
    tone(stepMotorPin,velMotor); 
    tiempoFucnionando=millis(); 
    reloj=true;
    disMotor=false;
    digitalWrite(disableMotorPin,disMotor);
    }
    
 if (digitalRead(SFilamento)&& fin && minutos>0){
  disMotor=true;
  noTone(stepMotorPin);
  EnCalefactor=false;
  reloj=false;
  digitalWrite(disableMotorPin,disMotor);
  }

ok=false;
    
    if(!digitalRead(RBoton)){
    while(!digitalRead(RBoton))delay(100);
    menu++; 
    Time2=millis()-1100; 
  display.clearDisplay();
  display.display();
    }
    if(!digitalRead(LBoton)){
    while(!digitalRead(LBoton))delay(100);  
    menu--;
    Time2=millis()-1100;
  display.clearDisplay();
  display.display();
  }
    if(!digitalRead(okBoton)){
    while(!digitalRead(okBoton))delay(100); 
    ok=true;
    Time2=millis()-1100;
    }

  if(millis()>Time1+100){ //cada 100ms
  Time1=millis()+100;

  menu=constrain(menu,1,8);
switch(menu){
  case 1:  Pantalla2(); //Grafica
  break;
  case 2: if(millis()>Time2+1000){ // Velocidad de muestreo 
  Time2=millis(); Pantalla1();} //inicio
  break;
  case 3:  if(millis()>Time2+1000){
   Time2=millis(); Pantalla3();}// I/O motor 
  break;
  case 4: if(millis()>Time2+1000){
   Time2=millis();Pantalla4();}// I/O calefactor
  break;
  case 5: if(millis()>Time2+1000){
   Time2=millis();Pantalla5();}//config motor
  break;
  case 6:if(millis()>Time2+1000){
   Time2=millis(); Pantalla6();}//config calefactor
  break;
  case 7: if(millis()>Time2+1000){
   Time2=millis();Pantalla8();}//config Manual/automatico
  break;
  case 8: if(millis()>Time2+1000){
   Time2=millis();Pantalla7();}//config guardar
  break;
  
  }

}
}
/////////////////////////////////////////////////////////////////////////////////////////



  void Pantalla1(){ 
    //Serial.println(analogRead(A0));
   if (digitalRead(SFilamento)){
    display.setCursor(2, 26);
    display.print("X");
    //display.setCursor(2, 28);
    //display.print("X");
    
    }else{
      display.setCursor(2, 18);
    display.print("|");
    display.setCursor(2, 36);
    display.print("|");
    
      }

      
  display.setCursor(5 , 0);
    display.print(" ");
   display.print( (int)setTemp());
   display.print("/");
    if(EnCalefactor)display.print(tempMeta);
  else display.print("0");
 

 
  
   display.setCursor(18, 18);
  display.print(horas);  
  display.print(":");
  if(minutos<10)display.print("0");
  display.print(minutos);
   display.print(":");
  if(segundos<10)display.print("0");
  display.print(segundos);
  
  display.drawBitmap(0,0, icono_nozzle, 14, 16, SSD1306_WHITE);

 distancia=(double)(pasos*0.01775)/1000;
 if(distancia>=1){pasos=0; metros++; distancia=0;}
  display.setCursor(18, 36);
  display.print(metros+distancia);
   display.print("m");

  display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
  display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
  display.display();
  display.clearDisplay();
  }
//////////////////////////////////////////////////////////////////////////


///*
  void Pantalla2(){ ///GRAFICA
     
    
if(bufferScroll != scroll){
  bufferScroll=scroll;
    display.setTextSize(2);

    display.drawBitmap(0,0, icono_nozzle, 14, 16, SSD1306_WHITE);
    display.setCursor(20, 0);
    display.print( (int)setTemp());
    display.print("/");
    display.print(tempMeta);
    
    display.drawRoundRect(0, 18, 128, 46, 2, SSD1306_WHITE);
      for(int i=1;i<=128;i++){   
      if(i%2)display.drawPixel(i, 40, SSD1306_WHITE);////Linea orizontal de pintos
      if(i>=2)display.drawLine(i,dataGrafic[i] ,i+1 , dataGrafic[i+1] , SSD1306_WHITE); // grafica
      }
      display.drawLine(scroll, 18, scroll,62, SSD1306_WHITE); // linea vertical
      
    display.display(); 
    display.clearDisplay();
  }}
    //////////////////////////////////////////////////////////////////////
   void Pantalla3(){ ///I/O motor
    if(ok){disMotor=!disMotor; 
    if(!disMotor){tone(stepMotorPin,velMotor); tiempoFucnionando=millis(); reloj=true;}
    else reloj=false;
    digitalWrite(disableMotorPin,disMotor);
    }
    display.setCursor(0, 0);
    display.print("   Motor");
    display.setCursor(0,18);
    display.print(pasos);
      display.setTextSize(1);
    display.print(" Pasos");
      display.setTextSize(2); 
    display.setCursor(0,36);
    display.print(velMotor);
    display.setTextSize(1);
    display.print("Pasos/s");
    display.setTextSize(2);
    display.setCursor(52, 49);
    if(!disMotor)display.print("OFF");
    else display.print("ON");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    display.clearDisplay();
}
    
   /////////////////////////////////////////////////////////////////////////////
    
    void Pantalla4(){ ////I/O calefactor
     if(ok){EnCalefactor=!EnCalefactor;}
    display.setCursor(0, 0);
    display.print("Nozzle");
    display.setCursor(0,18);
    display.print("Set:");    
    display.print(tempMeta);
    display.print(" C");
    display.setCursor(4, 49);
    if(EnCalefactor)display.print(" Enfriar");
    else display.print(" Calentar");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    display.clearDisplay();
    
    }

///////////////////////////////////////////////////////////////////////////

    void Pantalla5(){  ////Config Motor
      display.drawBitmap(90,8, icono_config, 30, 30, SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Motor");
    display.setCursor(0, 18);
     display.print(velMotor);
    display.setCursor(24, 49);
    display.print("config");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    display.clearDisplay();
    
    if (ok){
    
    display.setCursor(0, 0);
    display.println("Motor");
    display.setCursor(0, 18);
     display.print(velMotor);
     display.setCursor(52, 49);
    display.print("OK");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
      Config=true;
      while(Config){
      display.clearDisplay();
      PIDtemp(); 
    if(!digitalRead(RBoton)){
    while(!digitalRead(RBoton))delay(100);
    velMotor--;if(!disMotor)tone(stepMotorPin,velMotor);
    display.clearDisplay();
   display.setCursor(0, 0);
    display.println("Motor");
    display.setCursor(0, 18);
     display.print(velMotor);
     display.setCursor(52, 49);
    display.print("OK");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    }
    else if(!digitalRead(LBoton)){
    while(!digitalRead(LBoton))delay(100);  
    velMotor++;if(!disMotor)tone(stepMotorPin,velMotor);
    display.clearDisplay();
   display.setCursor(0, 0);
    display.println("Motor");
    display.setCursor(0, 18);
     display.print(velMotor);
    display.setCursor(52, 49);
    display.print("OK");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
  }
    else  if(!digitalRead(okBoton)){
    while(!digitalRead(okBoton)){ 
    delay(100);
    Config=false;;
    } 
      }
      } }
    
    }
       
       

///////////////////////////////////////////////////////////////////////////       
    void Pantalla6(){ ///confog calefactor
       display.drawBitmap(90,8, icono_config, 30, 30, SSD1306_WHITE);
    tempMeta=constrain(tempMeta,0,350);// Limites de temperatura
        display.setCursor(0, 0);
    display.println("Nuzzle");
    display.setCursor(0, 18);
     display.print(tempMeta);
    display.setCursor(24, 49);
    display.print("config");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    display.clearDisplay();
    
if (ok){
      display.clearDisplay();
   display.setCursor(0, 0);
    display.println("Nuzzle");
    display.setCursor(0, 18);
     display.print(tempMeta);
     display.setCursor(52, 49);
    display.print("OK");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
      Config=true;
      while(Config){
              display.clearDisplay();
      PIDtemp(); 
    if(!digitalRead(RBoton)){
    while(!digitalRead(RBoton))delay(100);
    tempMeta--;
    display.clearDisplay();
   display.setCursor(0, 0);
    display.println("Nuzzle");
    display.setCursor(0, 18);
     display.print(tempMeta);
     display.setCursor(52, 49);
    display.print("OK");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    }
    else if(!digitalRead(LBoton)){
    while(!digitalRead(LBoton))delay(100);  
    tempMeta++;
    display.clearDisplay();
   display.setCursor(0, 0);
    display.println("Nuzzle");
    display.setCursor(0, 18);
     display.print(tempMeta);
    display.setCursor(52, 49);
    display.print("OK");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
  }

    else  if(!digitalRead(okBoton)){
    while(!digitalRead(okBoton)){ 
    delay(100);
    pidController.Setpoint = tempMeta;
    Config=false;
    } }
      }
      }
    display.clearDisplay();
    }


///////////////////////////////////////////////////////////////////////////
   
    void Pantalla7(){ ///guardar cambios
    display.drawBitmap(48,16, icono_config, 30, 30, SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("  Guardar");
    display.setCursor(52, 49);
    display.print("SI");
    display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.display();
    display.clearDisplay();
    if(ok){menu=2; Guardar();}
    }
/////////////////////////////////////////////////////////////////////////////    
void Pantalla8(){ //config Manual/automatico
      display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("In:");
    if(Auto)display.println("Auto");
    else display.println("Manual");
    display.setCursor(0, 18);
    display.print("Fin:");
    if(fin)display.println("Auto");
    else display.println("Manual");
    display.setCursor(24, 49);
    display.print("config");
      display.display();
    display.clearDisplay();
    if(ok){
      delay(200);
      for(int i=0;i<2;){

      display.drawBitmap(0,50, icono_Up, 14, 16, SSD1306_WHITE);
    display.drawBitmap(110,50, icono_Down, 14, 16, SSD1306_WHITE);
       display.setCursor(52, 49);
    display.print("OK");
    display.setCursor(0, 0);
    if(i==0){
    display.print("In:");
    if(Auto)display.println("Auto");
    else display.println("Manual");
    if(!digitalRead(RBoton)){
    while(!digitalRead(RBoton))delay(100);
    Auto = true;
    }
    if(!digitalRead(LBoton)){
    while(!digitalRead(LBoton))delay(100);
    Auto = false;
    }
    }else{
    display.print("Fin:");
    if(fin)display.println("Auto");
    else display.println("Manual");
       if(!digitalRead(RBoton)){
    while(!digitalRead(RBoton))delay(100);
    fin = true;
    }
    if(!digitalRead(LBoton)){
    while(!digitalRead(LBoton))delay(100);
    fin = false;
    }
     }
    display.display();
    display.clearDisplay();

    
    if(!digitalRead(okBoton)){i++; delay(200);}
        
        
        }
      }
  }



 ////////////////////////////////////////////////////////////////////////////////    
 
 void actualizaVariables(){
  
  if(EEPROM.read(255)>1){ // configura valores en el primer arranque
    //if(true){        //Para RESET descomentar esta linea y comentar la anterios
    EEPROM.update(255,0);
  EEPROM.put(0, 200-30);
  EEPROM.put(4, 150-30);
 // EEPROM.put(8, 23.05); //kp
  //EEPROM.put(12, 2.00);//ki
  //EEPROM.put(16, 66.47);//kd
  EEPROM.put(20, true);
  EEPROM.put(22, true);
    
    }
  EEPROM.get(0, tempMeta); 
  EEPROM.get(4, velMotor);
  //EEPROM.get(8, kp);//Las variables float ocupan 4 bytes
  //EEPROM.get(12, ki);
 // EEPROM.get(16, kd);
  EEPROM.get(20, Auto);
  EEPROM.get(22, fin);
  }
  
 void Guardar(){
  EEPROM.put(0, tempMeta);
  EEPROM.put(4, velMotor);
  //EEPROM.put(8, kp);
  //EEPROM.put(12, ki);
  //EEPROM.put(16, kd);
  EEPROM.put(20, Auto);
  EEPROM.put(22, fin);
  }

  //////////////////////////////////////////////////////////////////////////
       void PIDtemp(){
         


  if(millis()>Time3+1500){ // Velocidad de muestreo grafica
  Time3=millis()+1500;
   if(scroll>126) scroll=1;
   else scroll++;
   
    dataGrafic[scroll]=constrain(map( setTemp(),tempMeta-7,tempMeta+7,0,45),0,45); // se guardan y reducen los datos
    dataGrafic[scroll]=(45-dataGrafic[scroll])+18;
  }

        if(EnCalefactor){
  pidController.Input = (int)setTemp();
  //pidController.Setpoint = tempMeta;
  pidController.Update();
  analogWrite(PIN_OUTPUT, pidController.Output);
        } else{ analogWrite(PIN_OUTPUT, 0);}
      }
////////////////////////////////////////////////////////////

 float  setTemp(){
  uint16_t a,b;
  int Vo = analogRead(A0);      
  uint16_t i=0;
  while(Vo > pgm_read_word ( &temptable[i][0] ) && i<102){
    i++;
  }
  Vo= pgm_read_word(&temptable[i][0])-Vo;
  a=uint16_t(pgm_read_word(&temptable[i][1]));
  b=uint16_t(pgm_read_word(&temptable[i-1][1]));
  Vo = map(Vo,0,9,a,b);

 return float(Vo*0.25);
  }
  ///////////////////////////////////////////////////////////////////
    void interrupcion(){
      pasos++;
      }
      /////////////////////////////////////

    /////////////////////FIN///////////////////////////////
