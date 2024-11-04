//Masukkan Library
#include "freertos/FreeRTOS.h"                  //library bawaan Free RTOS
#include "freertos/task.h"                      //library bawaan Free RTOS
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>   //library untuk Panel Dot (pakai virtual karena banyak panel)
#include <Adafruit_ADS1X15.h>                   //library untuk ADS1115
#include <SD.h>                                 //library untuk SD card module
#include <SPI.h>                                //library SPI (memastikan terpanggil untuk settingan SD.h)

// Definisi pin yang digunakan
#define R1_PIN 4    //pin Red 1
#define G1_PIN 17   //pin Green 1
#define B1_PIN 2    //pin Blue 1
#define R2_PIN 15   //pin Red 2
#define G2_PIN 16   //pin Green 2
#define B2_PIN 13   //pin Blue 2
#define A_PIN 25    //pin A
#define B_PIN 32    //pin B
#define C_PIN 27    //pin C
#define D_PIN 33    //pin D
#define E_PIN -1    //pin E tidak digunakan, diisi -1
#define LAT_PIN 26  //pin Latching(LAT)
#define OE_PIN 12   //pin enable(OE)
#define CLK_PIN 14  //pin clock(CLK)
#define CS_PIN 5    //pin untuk selectpin i2c ads

// Definisi ukuran panel dan konfigurasi chaining
#define PANEL_RES_X 64  //Lebar per panel dalam piksel
#define PANEL_RES_Y 32  //Tinggi per panel dalam piksel
// sesuaikan dengan formasi(baca di dokumen library)
#define NUM_ROWS 2      //Jumlah baris panel dalam rantai
#define NUM_COLS 3      //Jumlah panel per baris
#define PANEL_CHAIN NUM_ROWS*NUM_COLS // Total panel chaining

//definisi font
#ifndef NO_GFX
#include <Fonts\ARLRDBD9pt7b.h>
#include <Fonts\ARLRDBD10pt7b.h>
#include <Fonts\ARLRDBD11pt7b.h>
#include <Fonts\ARLRDBD12pt7b.h>
#include <Fonts\ARLRDBD13pt7b.h>
#include <Fonts\ARLRDBD14pt7b.h>
#include <Fonts\ARLRDBD15pt7b.h>
#include <Fonts\ARLRDBD16pt7b.h>
#include <Fonts\ARLRDBD17pt7b.h>
#include <Fonts\ARLRDBD18pt7b.h>
#include <Fonts\ARLRDBD19pt7b.h>
#include <Fonts\ARLRDBD20pt7b.h>
#include <Fonts\ARLRDBD21pt7b.h>
#include <Fonts\ARLRDBD22pt7b.h>
#endif

//Konfigurasi pin untuk port X1
HUB75_I2S_CFG::i2s_pins _pins_x1 = {
  R1_PIN, G1_PIN, B1_PIN, 
  R2_PIN, G2_PIN, B2_PIN,
  A_PIN, B_PIN, C_PIN, D_PIN, 
  E_PIN, LAT_PIN, OE_PIN, CLK_PIN
};


//Tipe chaining panel dari kiri bawah ke atas, sesuaikan dengan formasi(baca di dokumen library)
#define VIRTUAL_MATRIX_CHAIN_TYPE CHAIN_BOTTOM_LEFT_UP

//Deklarasi varibale pointer
MatrixPanel_I2S_DMA *dma_display = nullptr; //Pointer untuk objek MatrixPanel_I2S_DMA
VirtualMatrixPanel *virtualDisp = nullptr;  //Pointer untuk objek VirtualMatrixPanel
Adafruit_ADS1115 ads;                       //Pointer untuk objek Adafruit_ADS1115
File file;                                  //Pointer untuk objek File Sd card

//inisialisasi warna
uint16_t myBLACK = virtualDisp->color444(0, 0, 0);
uint16_t myWHITE = virtualDisp->color444(255, 255, 255);
uint16_t myRED = virtualDisp->color444(255, 0, 0);
uint16_t myGREEN = virtualDisp->color444(0, 255, 0);
uint16_t myBLUE = virtualDisp->color444(0, 0, 255);
uint16_t myYELLOW = virtualDisp->color444(255, 255, 0);
uint16_t textColor = virtualDisp->color444(255, 0, 0);


//variabel global untuk autocenter
int max_x = 160;              //Nilai maksimum sumbu x yang mau dibatasi
int max_y = 64;               //Nilai maksimum sumbu y yang mau dibatasi
//variabel global untuk Panel
String Z = "";                //Nilai awal untuk Z
String ZA = "";               //Nilai awal untuk ZA
String previousZ = "";        //Nilai sebelumnya untuk Z
String previousZA = "";       //Nilai sebelumnya untuk ZA
int i = 0;                    //Variabel kontrol status
String Z_1, Z_2, ZA_1, ZA_2;  //Variabel indexing
//variabel global untuk ADS
int16_t adc0 = 0, adc1 = 0, adc2 = 0;           //variabel untuk pembacaan nilai adc
float volts0 = 0.0, volts1 = 0.0, volts2 = 0.0; //variabel untuk Voltase
int AN0 = 0, AN1 = 0, AN2 = 0;                  //Variabel hasil mapping 0-9
//variabel global untuk selector sdcard
String a = "";          //Variabel string untuk menyimpan AN0
String previousA = "";  //Variabel string untuk pembanding a
String b = "";          //Variabel string untuk menyimpan AN1
String previousB = "";  //Variabel string untuk pembanding b
String c = "";          //Variabel string untuk menyimpan AN2
String previousC = "";  //Variabel string untuk pembanding c

void setup() {
  // put your setup code here, to run once:
  //Inisialisasi serial untuk debugging(serial monitor)
  Serial.begin(115200);
  
  //Membuat objek konfigurasi HUB75
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X, //Lebar modul panel
    PANEL_RES_Y, //Tinggi modul panel
    PANEL_CHAIN, //Jumlah total panel dalam rantai
    _pins_x1 // mapping pin untuk port X1
  );
  
  //Konfigurasi kecepatan i2s, blanking, dan driver IC(baca di dokumen library)
  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_20M; //Kecepatan i2s diset ke 20 MHz
  mxconfig.latch_blanking = 4;               //Siklus blanking untuk sinkronisasi latch
  mxconfig.driver = HUB75_I2S_CFG::FM6124;   //Menetapkan driver IC sebagai FM6124(atur sesuai chip yang digunakan)
  mxconfig.double_buff = 0;                  //Menonaktifkan double buffering
  
  //Membuat objek untuk mengelola tampilan LED
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->setBrightness8(255); //Mengatur kecerahan ke nilai maksimal (0-255)
  dma_display->begin();             //Memulai tampilan LED
  dma_display->clearScreen();       //Membersihkan layar LED
  
  //Membuat objek VirtualMatrixPanel untuk mengelola tampilan virtual pada panel LED
  virtualDisp = new VirtualMatrixPanel((*dma_display), NUM_ROWS, NUM_COLS, PANEL_RES_X, PANEL_RES_Y, VIRTUAL_MATRIX_CHAIN_TYPE);
  virtualDisp->fillScreen(virtualDisp->color444(0,0,0));  //Mengisi layar dengan warna hitam (RGB: 0, 0, 0)
  virtualDisp->setPhysicalPanelScanRate(NORMAL_TWO_SCAN); //Mengatur tingkat pemindaian panel (scan rate)
  virtualDisp->setTextWrap(false);                        //Settingan agar teks tidak terpotong ke bawah(wrapping text)

  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV(untuk ads1015), 0.1875mV (ads1115) (default)
  SD.begin(CS_PIN);             //Memulai sd dengan pin cs disertakan

  if (!ads.begin()){
    Serial.println("gagal inisialisai ads");
    while(1);
  }
  if (!SD.begin()){
    Serial.println("gagal inisialisasi SD card");
    return;
  }
  Serial.println("sd card & ads done");
  file = SD.open("/DATABASE_JR205.txt");
  if (!file){
    Serial.println("gagal membuka database");
    return;
  }

  //membuat Task untuk RTOS
  xTaskCreate(displayTask, "Display Task", 4096, NULL, 1, NULL);
  xTaskCreate(databaseTask, "Database Task", 4096, NULL, 1, NULL);
}

void displayTask (void *pvParameters){
  while (true) {
    autoCenter();
    vTaskDelay(1);
  }
}

void databaseTask (void *pvParameters){
  while (true) {
    adsRead();
    sdcardRead();
    vTaskDelay(1);
  }
}

void autoCenter() {
  if (i == 0) { //Hapus tampilan saat nilai dari ads berganti
    virtualDisp->clearScreen(); 
  }
  else if (i == 2) {
    virtualDisp->setTextColor(textColor);   //Set warna huruf
    
    //Cek apakah ada spasi di dalam Z
    int spaceIndexZ = Z.indexOf(' ');
    if (Z.length() > 0 && ZA.length() == 0 ){ //Jika hanya Z dan tidak ada ZA
      if (spaceIndexZ != -1) { //Jika ada spasi
        if (Z == "UJI COBA"){ //Khusus Kalimat UJI COBA
          virtualDisp->setFont(&ARLRDBD17pt7b); //Set Font
          virtualDisp->setTextSize(1);          //Set pengali font

          //Mendapatkan ukuran untuk perhitungan posisi
          int16_t x1_5 = 0, y1_5 = 0;         //Variabel untuk posisi x dan y dari teks
          uint16_t textWidth5, textHeight5;   //Variabel untuk lebar dan tinggi teks

          //Menghitung bounding box (ukuran dan posisi) teks pada posisi (0, 0)
          //virtualDisp->getTextBounds() mengembalikan posisi dan ukuran teks dalam variabel yang diberikan
          virtualDisp->getTextBounds(Z.c_str(), 0, 0, &x1_5, &y1_5, &textWidth5, &textHeight5);

          //Hitung posisi X untuk menampilkan ZA secara terpusat
          int16_t xCenter5 = (max_x - textWidth5) / 2 - x1_5;   //Menghitung posisi horizontal (X) agar teks berada di tengah layar
          int16_t yCenter5 = (max_y - textHeight5) / 2 - y1_5;  //Menghitung posisi vertikal (Y) agar teks berada di tengah layar

          //Tampilkan Z di tengah display
          virtualDisp->setCursor(xCenter5, yCenter5);
          virtualDisp->print(Z);
          i = 3; // Set flag agar hanya tampil sekali          
        }
        else{ //Selain kalimat uji coba
          /*-------------------------------Z_1--------------------------------------------*/
          // Pisahkan Z menjadi Z_1 dan Z_2
          Z_1 = Z.substring(0, spaceIndexZ);  //Kata sebelum spasi
          Z_2 = Z.substring(spaceIndexZ + 1); //Kata setelah spasi

          // Mendapatkan ukuran teks dan menentukan font Z_1
          if (Z_1.length() <= 6) {
            virtualDisp->setFont(&ARLRDBD18pt7b);   //Set Font
            virtualDisp->setTextSize(1);            //Set pengali font
          } else if (Z_1.length() == 7) {
            virtualDisp->setFont(&ARLRDBD15pt7b);   //Set Font
            virtualDisp->setTextSize(1);            //Set pengali font
          }

          // Mendapatkan ukuran Z_1 untuk perhitungan posisi
          int16_t x1_1 = 0, y1_1 = 0;           //Variabel untuk posisi x dan y dari teks
          uint16_t textWidth1, textHeight1;     //Variabel untuk lebar dan tinggi teks

          //Menghitung bounding box (ukuran dan posisi) teks pada posisi (0, 0)
          //virtualDisp->getTextBounds() mengembalikan posisi dan ukuran teks dalam variabel yang diberikan
          virtualDisp->getTextBounds(Z_1.c_str(), 0, 0, &x1_1, &y1_1, &textWidth1, &textHeight1);

          // Hitung posisi X untuk menampilkan Z_1 secara terpusat
          int16_t xCenter1 = (max_x - textWidth1) / 2 - x1_1; //Menghitung posisi horizontal (X) agar teks berada di tengah layar

          // Tampilkan Z_1 di bagian atas (0-32 pixel)
          virtualDisp->setCursor(xCenter1, (32 - textHeight1) / 2 - y1_1);
          virtualDisp->print(Z_1);

          /*-------------------------------Z_2--------------------------------------------*/
          // Mendapatkan ukuran teks dan menentukan font Z_2
          if (Z_2.length() <= 6) {
            virtualDisp->setFont(&ARLRDBD18pt7b);   //Set Font
            virtualDisp->setTextSize(1);            //Set pengali font
          } else if (Z_2.length() == 7) {
            virtualDisp->setFont(&ARLRDBD15pt7b);   //Set Font
            virtualDisp->setTextSize(1);            //Set pengali font
          }

          // Mendapatkan ukuran Z_2 untuk perhitungan posisi
          int16_t x1_2 = 0, y1_2 = 0;         //Variabel untuk posisi x dan y dari teks
          uint16_t textWidth2, textHeight2;   //Variabel untuk lebar dan tinggi teks

          //Menghitung bounding box (ukuran dan posisi) teks pada posisi (0, 0)
          //virtualDisp->getTextBounds() mengembalikan posisi dan ukuran teks dalam variabel yang diberikan
          virtualDisp->getTextBounds(Z_2.c_str(), 0, 0, &x1_2, &y1_2, &textWidth2, &textHeight2);

          // Hitung posisi X untuk menampilkan Z_2 secara terpusat
          int16_t xCenter2 = (max_x - textWidth2) / 2 - x1_2; //Menghitung posisi horizontal (X) agar teks berada di tengah layar

          // Tampilkan Z_2 di bagian bawah (33-64 pixel)
          virtualDisp->setCursor(xCenter2, 33 + (32 - textHeight2) / 2 - y1_2);
          virtualDisp->print(Z_2);
          i = 3; // Set flag agar hanya tampil sekali
        }
      }
      else { // Jika tidak ada spasi, tampilkan teks di tengah layar
        // Pilih ukuran font berdasarkan panjang teks
        if (Z.length() <= 4) {
          virtualDisp->setFont(&ARLRDBD15pt7b);   //Set Font
          virtualDisp->setTextSize(2);            //Set pengali font
        } else if (Z.length() == 5) {
          virtualDisp->setFont(&ARLRDBD22pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (Z.length() == 6) {
          virtualDisp->setFont(&ARLRDBD18pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (Z.length() == 7) {
          virtualDisp->setFont(&ARLRDBD16pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (Z.length() == 8) {
          virtualDisp->setFont(&ARLRDBD15pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (Z.length() == 9) {
          virtualDisp->setFont(&ARLRDBD12pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else {
          virtualDisp->setFont(&ARLRDBD11pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        }

        // Mendapatkan ukuran teks Z
        int16_t x1 = 0, y1 = 0;           //Variabel untuk posisi x dan y dari teks
        uint16_t textWidth, textHeight;   //Variabel untuk lebar dan tinggi teks

        //Menghitung bounding box (ukuran dan posisi) teks pada posisi (0, 0)
          //virtualDisp->getTextBounds() mengembalikan posisi dan ukuran teks dalam variabel yang diberikan
        virtualDisp->getTextBounds(Z.c_str(), 0, 0, &x1, &y1, &textWidth, &textHeight);

        // Hitung posisi X dan Y untuk menampilkan teks di tengah layar
        int16_t xCenter = (max_x - textWidth) / 2 - x1;   //Menghitung posisi horizontal (X) agar teks berada di tengah layar
        int16_t yCenter = (max_y - textHeight) / 2 - y1;  //Menghitung posisi vertikal (Y) agar teks berada di tengah layar

        // Tampilkan teks di tengah layar
        virtualDisp->setCursor(xCenter, yCenter);
        virtualDisp->print(Z.c_str());
        i = 3;// Set flag agar hanya tampil sekali
      }
    }
    else {//jika ada ZA
        // Mendapatkan ukuran teks dan menentukan font Z
        if (Z.length() == 9) {//KP BANDAN
          virtualDisp->setFont(&ARLRDBD13pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (Z.length() >= 11) {//TIDAK UNTUK
          virtualDisp->setFont(&ARLRDBD11pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        }

        // Mendapatkan ukuran Z untuk perhitungan posisi
        int16_t x1_3 = 0, y1_3 = 0;         //Variabel untuk posisi x dan y dari teks
        uint16_t textWidth3, textHeight3;   //Variabel untuk lebar dan tinggi teks

        //Menghitung bounding box (ukuran dan posisi) teks pada posisi (0, 0)
          //virtualDisp->getTextBounds() mengembalikan posisi dan ukuran teks dalam variabel yang diberikan
        virtualDisp->getTextBounds(Z.c_str(), 0, 0, &x1_3, &y1_3, &textWidth3, &textHeight3);

        // Hitung posisi X untuk menampilkan Z secara terpusat
        int16_t xCenter3 = (max_x - textWidth3) / 2 - x1_3; //Menghitung posisi horizontal (X) agar teks berada di tengah layar

        // Tampilkan Z di bagian tengah
        virtualDisp->setCursor(xCenter3, (32 - textHeight3) / 2 - y1_3); //karena y data Z akan ditampilkan dari senter bagian display atas
        virtualDisp->print(Z);

        // Mendapatkan ukuran teks dan menentukan font ZA
        if (ZA.length() == 9) {//PENUMPANG
          virtualDisp->setFont(&ARLRDBD12pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (ZA.length() == 12) {//VIA PS SENEN
          virtualDisp->setFont(&ARLRDBD11pt7b);   //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        } else if (ZA.length() == 13) {//VIA MANGGARAI
          virtualDisp->setFont(&ARLRDBD9pt7b);    //Set Font
          virtualDisp->setTextSize(1);            //Set pengali font
        }

        // Mendapatkan ukuran ZA untuk perhitungan posisi
        int16_t x1_4 = 0, y1_4 = 0;         //Variabel untuk posisi x dan y dari teks
        uint16_t textWidth4, textHeight4;   //Variabel untuk lebar dan tinggi teks
        virtualDisp->getTextBounds(ZA.c_str(), 0, 0, &x1_4, &y1_4, &textWidth4, &textHeight4);

        // Hitung posisi X untuk menampilkan ZA secara terpusat
        int16_t xCenter4 = (max_x - textWidth4) / 2 - x1_4; //Menghitung posisi horizontal (X) agar teks berada di tengah layar

        // Tampilkan ZA di bagian tengah
        virtualDisp->setCursor(xCenter4, 33 + (32 - textHeight4) / 2 - y1_4); //karena y data Z akan ditampilkan dari senter bagian display atas
        virtualDisp->print(ZA);
        i = 3; // Set flag agar hanya tampil sekali
    }
  }
  else {
    // tidak melakukan apa-apa (menunggu pergantian data)
  }
}

void adsRead(){
  adc0 = ads.readADC_SingleEnded(0);  //pembacaan analog pada pin A0 ADS
  adc1 = ads.readADC_SingleEnded(1);  //pembacaan analog pada pin A0 ADS
  adc2 = ads.readADC_SingleEnded(2);  //pembacaan analog pada pin A0 ADS

  volts0 = ads.computeVolts(adc0);    //komputasi voltase pada A0
  volts1 = ads.computeVolts(adc1);    //komputasi voltase pada A0
  volts2 = ads.computeVolts(adc2);    //komputasi voltase pada A0

  AN0 = map(adc0, 0, 24900, 0, 9);    //Mapping A0 dari 0-24900 ke 0-9
  AN1 = map(adc1, 0, 24900, 0, 9);    //Mapping A1 dari 0-24900 ke 0-9
  AN2 = map(adc2, 0, 24900, 0, 9);    //Mapping A2 dari 0-24900 ke 0-9

  if(AN0 >= 9) {AN0 = 9;}             //Menjaga agar nilai tetap 9 saat leih dari 4.5 V
  if(AN1 >= 9) {AN1 = 9;}             //Menjaga agar nilai tetap 9 saat leih dari 4.5 V
  if(AN2 >= 9) {AN2 = 9;}             //Menjaga agar nilai tetap 9 saat leih dari 4.5 V

  Serial.println("-----------------------------------------------------------");
  // Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.print("V");Serial.print("  "); Serial.print(AN0); Serial.println("V");
  // Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.print("V");Serial.print("  "); Serial.print(AN1); Serial.println("V");
  // Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.print("V");Serial.print("  "); Serial.print(AN2); Serial.println("V");
  // delay(100);

  a = String(AN0);    //mengisi string a dengan cara mengubah AN0 ke string
  b = String(AN1);    //mengisi string b dengan cara mengubah AN1 ke string
  c = String(AN2);    //mengisi string c dengan cara mengubah AN2 ke string
  if (a == previousA && b == previousB) {
    i = 1;  //jika tidak ada pergantian string a b
  } else {
    i = 0;  //jika ada pergantian di string a b
  }
  previousA = a;      //dimasukkan ke prev Agar nanti bisa dibuat perbandingan apakah ada pergantian isi atau tidak
  previousB = b;      //dimasukkan ke prev Agar nanti bisa dibuat perbandingan apakah ada pergantian isi atau tidak
  previousC = c;      //dimasukkan ke prev Agar nanti bisa dibuat perbandingan apakah ada pergantian isi atau tidak
  Serial.print("i="); Serial.println(i);
  Serial.print("a="); Serial.println(a);
  Serial.print("b="); Serial.println(b);
  Serial.print("c="); Serial.println(c);
  Serial.print("AN0="); Serial.println(AN0);
  Serial.print("AN1="); Serial.println(AN1);
  Serial.print("AN2="); Serial.println(AN2);
}

void sdcardRead(){
  String hasil = a+b;   //Menggabungkan string a dan b ke variabel lokal hasil
  if(i==0){   //Jika ada pergantian x/y
    //Reset variabel Z dan ZA
    Z = "";
    ZA = "";

    //Buka kembali file(biar gak stuck ketika gagal)
    file = SD.open("/DATABASE_JR205.txt");
    if (!file) {
      Serial.println("Failed to open file!");
      return;
    }

    //Baca file tiap baris
    while (file.available()) {
      String line = file.readStringUntil('\n'); //baca file yang dituju sampai enter/ganti baris

      //Mencocokkan nilai dengan string hasil
      if (line.startsWith(hasil)) {
        Serial.print("Found match: ");
        Serial.println(line);

        //Ambil data setelah '|' menggunakan substring
        int delimiterIndex = line.indexOf('|');
        String data = line.substring(delimiterIndex + 1);  //Ambil data setelah '|'
        data.trim();  //Hapus spasi tambahan di akhir/beginning string

        //Pisahkan data jadi beberapa kata(ini dilimit 2 kata)
        int firstSpaceIndex = data.indexOf(' ');
        int secondSpaceIndex = data.indexOf(' ', firstSpaceIndex + 1);

        //Simpan dua kata pertama di variabel z, sisanya di za
        if (firstSpaceIndex != -1 && secondSpaceIndex != -1) {
          Z = data.substring(0, secondSpaceIndex);    // Dua kata pertama
          ZA = data.substring(secondSpaceIndex + 1);  // Kata-kata sisanya
        } else {
          // Jika kurang dari dua kata, simpan seluruhnya di z
          Z = data;
        }
        break;  // Keluar dari loop setelah menemukan data
      }
    }
    file.close(); //menutup file
    i=2;    //ini berpengaruh pada autocenter() untuk memindahkan kondisi i ke 2 agar menampilkan data ke panel
  }

  // Tampilkan variabel hasil, z dan za di Serial
  Serial.print("hasil: "); Serial.println(hasil);
  Serial.print("Data in variable Z: "); Serial.println(Z);
  Serial.print("Data in variable ZA: "); Serial.println(ZA);
}

void loop() {
  // put your main code here, to run repeatedly:
}