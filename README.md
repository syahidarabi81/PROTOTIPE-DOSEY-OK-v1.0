
// ========================================================================================
//                              SISTEM DOSING HIDROPONIK v1.0
// ========================================================================================
// oleh Syahid Tazia Arabi_F1401221114_Departemen Teknik Mesin dan Biosistem_IPB University

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <avr/wdt.h>             // reset pabrik via watchdog (reset perangkat keras penuh)
#include <Adafruit_ADS1X15.h>   // ADS1115: ADC eksternal 16-bit via I2C

// Matikan watchdog sedini mungkin setelah reset (sebelum setup), agar reset via watchdog (Reset Sistem)
// tidak memicu reset berulang. Dijalankan otomatis oleh startup AVR (
uint8_t mcusrSaatBoot __attribute__((section(".noinit")));   // penyebab reset terakhir (dibaca sebelum dihapus)
void wdtMatikanSaatBoot(void) __attribute__((naked, used, section(".init3")));
void wdtMatikanSaatBoot(void)
{
  mcusrSaatBoot = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
// Watchdog SAAT BERJALAN bila program hang > 8 dtk (noise relay, I2C macet, memori habis) mikrokontroler
// reset otomatis -> semua relay kembali OFF di setup() & siklus dilanjutkan oleh logika pemulihan daya.
#define WDT_RUNTIME WDTO_8S

// ── PIN SENSOR DAN RTC ──
#define PIN_DS18B20      8
#define PIN_TDS          A15
#define PIN_PH           A14
#define PIN_WATER_LEVEL  10

// ── PIN RELAY 8 CHANNEL – Pompa & Katup Solenoid ─────
#define PIN_RELAY_D1        22   // Pompa D1            -> IN1
#define PIN_RELAY_D2        19   // Pompa D2            -> IN2
#define PIN_RELAY_D3        35   // Pompa D3            -> IN3
#define PIN_RELAY_P1        28   // Pompa P1            -> IN4
#define PIN_RELAY_P2        31   // Pompa P2            -> IN5
#define PIN_RELAY_P3        32   // Pompa P3            -> IN6
#define PIN_RELAY_SOLENOID  7   // Katup Solenoid NC ke kontak NO relay (fail-safe) -> IN7
#define PIN_RELAY_BUZZER    36   // Buzzer aktif        -> IN8 (relay ke-8)

#define PIN_RELAY_TDS       4   // Daya modul sensor TDS (relay ke-9, modul terpisah)

#define RELAY_AKTIF_LOW  true    // true = modul relay aktif LOW (LOW=ON, HIGH=OFF)

// ── OBJEK ──
OneWire              oneWireBus(PIN_DS18B20);
DallasTemperature    sensorSuhu(&oneWireBus);
RTC_DS3231            rtc;
bool                 rtcSiap = false;  // true jika DS3231 terdeteksi di bus I2C
//  selama layar TIBA WAKTU DOSING tampil, tulisan menu di belakangnya (yang tetap berjalan)
// tidak digambar -> prompt tidak tertimpa. Setelah prompt ditutup, layar menu digambar ulang.
bool lcdTertahan = false;
class LcdBertahan : public LiquidCrystal_I2C {
public:
  LcdBertahan(uint8_t a, uint8_t c, uint8_t r) : LiquidCrystal_I2C(a, c, r) {}
  void clear(){ if(!lcdTertahan) LiquidCrystal_I2C::clear(); }
  void setCursor(uint8_t c, uint8_t r){ if(!lcdTertahan) LiquidCrystal_I2C::setCursor(c, r); }
  size_t write(uint8_t ch){ if(lcdTertahan) return 1; return LiquidCrystal_I2C::write(ch); }
};
LcdBertahan          lcd(0x27, 20, 4);

// ── ADS1115 ──
Adafruit_ADS1115   ads;
bool               adsSiap = false;   // true jika ADS1115 terdeteksi di bus I2C
#define ADS_CH_PH    0     
#define ADS_SKALA    32      // faktor skala hitungan ADS1115 (15-bit) vs ADC internal (10-bit)

// ── KONSTANTA SENSOR ──
#define TDS_VREF           5.0f
#define TDS_SCOUNT         15    // 15 detik median
#define PH_SCOUNT          15    // 15 detik median
#define INTERVAL_SENSOR    1000UL
#define INTERVAL_KAL        500UL
#define INTERVAL_RTC       1000UL
#define INTERVAL_LCD_PANTAU 1500UL
#define KATUP_JEDA_BUKA_MS  3000UL   // TES_149: jeda minimum sebelum katup boleh dibuka lagi
#define LCD_REINIT_TUNDA_MS 300UL    // TES_149: re-init LCD sesudah lonjakan katup reda

// ── BATAS DETEKSI SENSOR TIDAK TERBACA / TIDAK TERPASANG ───
#define TDS_ADC_MIN_VALID   8
#define PH_ADC_MIN_VALID    8
#define PH_ADC_MAX_VALID    1015
#define PH_VALID_MIN        1.0f    // TES_148: hasil pH di luar 1-13 = sensor/kalibrasi tak wajar (bukan larutan tandon)
#define PH_VALID_MAKS       13.0f

// ── VARIABEL SENSOR ──
const float PH_SLOPE_BAWAAN  = -5.70f;   // rumus bawaan pH = slope x V + offset (tanpa kalibrasi)
const float PH_OFFSET_BAWAAN =  21.34f;
float phSlope    = PH_SLOPE_BAWAAN;
float phOffset   = PH_OFFSET_BAWAAN;
float calPhVolt1 = 0.0f, calPhVolt2 = 0.0f, calPhVolt3 = 0.0f;
float suhuKalibrasiPH = 25.0f;   // suhu larutan saat kalibrasi pH (dari sensor) - acuan ATC Nernst
float suhularutan = 25.0f;
float suhuRawTerakhir = 25.0f;
float suhuOffset = 0.0f;
bool  suhuInputNegatif = false;
int   ppmTerbaca  = 0;
float phTerbaca   = 7.0f;
bool  tdsSensorValid = true;
bool  phSensorValid  = true;
unsigned long tsBacaTerakhir = 0, tsKalTerakhir = 0, tsRtcTerakhir = 0;
unsigned long tsRtcHealth = 0;          // cek lostPower / validitas RTC berkala
#define INTERVAL_RTC_HEALTH 60000UL     // tiap 60 detik
unsigned long tsLcdPantau = 0;
bool lcdPerluReinit = false;   // dipicu HANYA saat on/off solenoid valve -> paksa re-init LCD
bool sensorAktifSebelumnya = false;
int tdsBuffer[TDS_SCOUNT], tdsBufferTemp[TDS_SCOUNT], tdsBufferIdx = 0;
int phBuffer[PH_SCOUNT], phBufferTemp[PH_SCOUNT], phBufferIdx = 0;
// -- Kalibrasi pH: jendela median bergulir (min 60 dtk, ambil median) --
#define STAB_ROLL_SEC 15          // jendela median 15 detik terakhir (kalkulator sensor & dosing)
#define KAL_PH_ROLL_N 20           // 20 sampel @500ms (INTERVAL_KAL) = 10 detik terakhir 
#define KAL_TDS_ROLL_N 20          // 20 sampel @500ms (INTERVAL_KAL) = 10 detik terakhir 
#define ROLL_MED_N 20               // 20 sampel @1000ms (INTERVAL_SENSOR) = jendela median 20 detik terakhir
#define KAL_PH_MIN_MS  120000UL    // pH: kalibrasi & kalkulator - tunggu stabil minimal 120 detik
#define KAL_TDS_MIN_MS  30000UL    // TDS/PPM: kalibrasi & kalkulator - tunggu stabil minimal 30 detik
int kalPhRoll[KAL_PH_ROLL_N];
int kalPhRollIdx = 0, kalPhRollCount = 0;
unsigned long tKalPhBufMulai = 0;
float suhuKalPhSum = 0.0f; int suhuKalPhCount = 0;
int kalTdsRoll[KAL_TDS_ROLL_N];
int kalTdsRollIdx = 0, kalTdsRollCount = 0;
unsigned long tKalTdsBufMulai = 0;
bool kalPhTungguTampil = false;   // true jika B ditekan sebelum 120 dtk tercapai (pH)
bool kalTdsTungguTampil = false;  // true jika B ditekan sebelum 30 dtk tercapai (TDS)
int   ppmRoll15[ROLL_MED_N]; int ppmRoll15Idx=0, ppmRoll15Count=0;
float phRoll15[ROLL_MED_N];  int phRoll15Idx=0,  phRoll15Count=0;

// ── VARIABEL PEMBACAAN SUHU NON-BLOCKING ──
bool suhuKonversiSedangBerjalan = false;
unsigned long tsSuhuMulai = 0;
const unsigned long INTERVAL_SUHU_KONVERSI = 750UL;

// ── KEYPAD ──
const byte ROWS=4, COLS=4;
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS]={6,18,17,16}, colPins[COLS]={15,14,2,3};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// ── STATE ──
enum MenuState {
  MENU_UTAMA, MENU_RINGKASAN_BARU, MULAI_DOSING_KONFIRMASI,

  MENU_BATASAN_PPM,
    MENU_BATASAN_PH,

  MENU_JENIS_DOSING,
    MENU_PERSEN_MAKRO,         // TES_121: input besar makro dosing (50-90%) setelah memilih jenis dosing
    DOSING_INPUT_LAMA_CEK,
    DOSING_INPUT_STABILISASI,
    DOSING_INPUT_JEDA,
    DOSING_TUNGGU_WAKTU,
    DOSING_ISI_TANDON,
    DOSING_JEDA_ISI,
    DOSING_TANDON_TIMEOUT,
    DOSING_SENSOR_INVALID,
    DOSING_CEK_PPM1,
    DOSING_MACRO_AB,
    DOSING_HOMOGEN1,
    DOSING_CEK_PPM2,
    DOSING_CEK_PPM3,           
    DOSING_MICRO_AB,
    DOSING_HOMOGEN2,
    DOSING_CEK_PH1,
    DOSING_MACRO_PH,
    DOSING_HOMOGEN3,
    DOSING_CEK_PH2,
    DOSING_MICRO_PH,
    DOSING_HOMOGEN4,
    DOSING_PI_CEK_PPM,
    DOSING_PI_POMPA_AB,
    DOSING_PI_HOMOGEN_AB,
    DOSING_PI_CEK_PH,
    DOSING_PI_POMPA_PH,
    DOSING_PI_HOMOGEN_PH,
    DOSING_CEK_AKHIR,
    DOSING_FINAL_STATUS,
    DOSING_JEDA_PULIH,        
    DOSING_REKAP,
    DOSING_TUNDA_KALIBRASI,   

  SUB_MENU_A,
    MENU_JENIS_LARUTAN,
      MENU_PENGENCERAN,
      MENU_EC_BEBAS,
      MENU_JENIS_ASAM, MENU_KONSENTRASI,
    MENU_VOL_TANDON,
    MENU_TARGET_PPM, MENU_TARGET_PPM_HASIL,
    MENU_TARGET_PH, MENU_TARGET_PH_HASIL,
    MENU_WAKTU_DOSING,
    MENU_DEBIT_POMPA_SEL, MENU_DEBIT_POMPA_VAL,

  MENU_PPM_PH_TANAMAN,
  MENU_TANAMAN_LIST,

  SUB_MENU_B,
    MENU_KAL_SENSOR,          // submenu kalibrasi TDS / pH (kalSensorSel)
      MENU_KAL_TDS_SAMPEL,    // ambil 1 sampel buffer TDS
      MENU_KAL_PH_SAMPEL,     // ambil 1 sampel buffer pH
      MENU_KAL_LIHAT,         // lihat data sampel draft
      MENU_KAL_ULANG_KONF,    // konfirmasi hapus sampel terakhir / hapus 1 buffer / mulai ulang draft
    MENU_SET_RTC_TAHUN, MENU_SET_RTC_BULAN, MENU_SET_RTC_TGL,
    MENU_SET_RTC_JAM,   MENU_SET_RTC_MENIT, MENU_SET_RTC_DETIK, MENU_SET_RTC_HASIL,
    MENU_WATER_LEVEL,
    MENU_SUHU_KALIBRASI,
    TES_SENSOR_INPUT_BILAS,
    TES_SENSOR_INPUT_STABIL,
    TES_SENSOR_PROSES,
    TES_SENSOR_HASIL,

  SUB_MENU_C,
    MENU_TES_POMPA_RUN,
    TES_POMPA_AKTIF,               
    TES_POMPA_CANCEL_CONFIRM,      
    MENU_TES_SOLENOID,
    MENU_INPUT_DEBIT_SEL,
    INPUT_DEBIT_ML,
    INPUT_DEBIT_DETIK,
    INPUT_DEBIT_HASIL,

  MENU_RINGKASAN,

  SUB_MENU_D,
    KALK_WATER_LEVEL_BENTUK,
      KALK_WL_TABUNG_VOL, KALK_WL_TABUNG_JARI, KALK_WL_TABUNG_HASIL,
      KALK_WL_BALOK_VOL, KALK_WL_BALOK_PANJANG, KALK_WL_BALOK_LEBAR, KALK_WL_BALOK_HASIL,
    KALK_INPUT_MENU,
      KALK_PPM_EC, KALK_PPM_DIL, KALK_PPM_VOL,
      KALK_PPM_AWAL, KALK_PPM_TARGET, KALK_PPM_HASIL,
      KALK_PH_VOL, KALK_PH_ASAM, KALK_PH_KONSEN,
      KALK_PH_PPM,
      KALK_PH_AWAL, KALK_PH_TARGET, KALK_PH_HASIL,

  MENU_PENGATURAN_SISTEM,
    MENU_ZN_HUB,
      MENU_LIHAT_KPKI_ZN,
        MENU_EDIT_KPKI_ZN,
      UJI_ZN_PILIH_KANAL,
      UJI_ZN_INPUT_VOLUME,   // volume tandon uji (L)
      UJI_ZN_INPUT_DURASI,   // lama rekam maksimum uji reaksi (menit)
      UJI_ZN_LOGGING,
      UJI_ZN_KONFIRMASI,
      MENU_ZN_REKAMAN,       //daftar rekaman kurva Uji Reaksi terakhir
  MENU_PENGATURAN_PERSAMAAN,
    PENGATURAN_AB_PILIH_DIL,
    PENGATURAN_AB_LIST,        
    PENGATURAN_PH_PILIH_ASAM,
    PENGATURAN_PH_PILIH_FASE,
    PENGATURAN_PH_LIST,
    PENGATURAN_PH_INPUT,
    PENGATURAN_RESET_CONFIRM,
    PERSAMAAN_KONFIRMASI,      // jadwal cek aktif: konfirmasi sebelum persamaan nutrisi diubah
    ZN_KONFIRMASI_JADWAL,      // jadwal aktif (mode CL PI ZN): konfirmasi sebelum Kp/Ki ZN Aktif diubah
    RESET_SISTEM_CONFIRM,
    MENU_KUNCI_KANAL,          // status & buka kunci kanal AB / pH
    RESOLUSI_PERINGATAN,       // 1 dtk pompa halus > 1/2 batasan
    KUNCI_KANAL_KONFIRMASI,

  MENU_KREDIT
};
MenuState currentState = MENU_UTAMA;

bool statusOverlayAktif = false;

// ================
// DATA MENU UTAMA 
// ================
const int TOTAL_MENU_UTAMA_ROWS = 11;
const char* menuUtamaOptions[TOTAL_MENU_UTAMA_ROWS] = {
  "Mulai Dosing",
  "Status Dosing",
  "Parameter Aktif",
  "Parameter Baru",
  "Parameter Dosing",
  "PPM/pH Tanaman",
  "Mode Kalibrasi",
  "Uji Debit Pompa",
  "Mode Kalkulator",
  "Pengaturan Sistem",
  "Kredit"
};
int selMenuUtama    = 0;
int scrollMenuUtama = 0;

// ==============================
// DATA MENU A – PARAMETER DOSING 
// ==============================
const int TOTAL_MENU_A_ROWS = 11;
const char* menuAOptions[TOTAL_MENU_A_ROWS] = {
  "Jenis Larutan",
  "Volume Tandon",
  "Target PPM",
  "Target pH",
  "Waktu Dosing",
  "Debit Pompa",
  "Lama Bilas Sensor",
  "Jeda Stabilisasi",
  "Batasan PPM/pH",
  "Jenis Dosing",
  "Jeda Homogenisasi"
};
int selMenuA=0, scrollMenuA=0;

// ==========================
// DATA MENU "PPM/pH TANAMAN" (khusus Sayuran Daun)
// ==========================
// Data kebutuhan pH & PPM per tanaman (sesuai tabel HidroponikPedia.com)
struct TanamanData { const char* nama; float ph; int ppm; };

const TanamanData dataSayuranDaun[17] PROGMEM = {
  {"Artichoke",6.5f,560},        {"Asparagus",6.0f,980},
  {"Bawang Pre",6.5f,980},       {"Bayam",6.0f,1260},
  {"Brokoli",6.0f,1960},         {"Brussel Kecambah",6.5f,1750},
  {"Endive",5.5f,1400},          {"Kailan",5.5f,1050},
  {"Kangkung",5.5f,1050},        {"Kubis",6.5f,1750},
  {"Kubis Bunga",6.5f,1750},     {"Pakcoy",7.0f,1050},
  {"Sawi Manis",5.5f,1050},      {"Sawi Pahit",6.0f,840},
  {"Seledri",6.5f,1260},         {"Selada",6.0f,560},
  {"Silverbeet",6.0f,1260}
};

// Bibit/Persemaian: nama & pH sama dgn Sayuran Daun, PPM seluruhnya 400
const TanamanData dataBibit[17] PROGMEM = {
  {"Artichoke",6.5f,400},        {"Asparagus",6.0f,400},
  {"Bawang Pre",6.5f,400},       {"Bayam",6.0f,400},
  {"Brokoli",6.0f,400},          {"Brussel Kecambah",6.5f,400},
  {"Endive",5.5f,400},           {"Kailan",5.5f,400},
  {"Kangkung",5.5f,400},         {"Kubis",6.5f,400},
  {"Kubis Bunga",6.5f,400},      {"Pakcoy",7.0f,400},
  {"Sawi Manis",5.5f,400},       {"Sawi Pahit",6.0f,400},
  {"Seledri",6.5f,400},          {"Selada",6.0f,400},
  {"Silverbeet",6.0f,400}
};

// Dua kategori: 0 = Bibit/Persemaian, 1 = Tanaman Dewasa (Sayuran Daun)
const TanamanData* const kategoriTanamanData[2] = { dataBibit, dataSayuranDaun };
const int kategoriTanamanTotal[2] = {17, 17};
const char* tanamanKategoriJudul[2] = { "BIBIT/PERSEMAIAN", "SAYURAN DAUN" };

// Menu pilih fase tanaman
const int TOTAL_PPM_PH_TANAMAN_ROWS = 2;
const char* ppmPhTanamanOptions[TOTAL_PPM_PH_TANAMAN_ROWS] = {
  "Bibit/Persemaian","Tanaman Dewasa"
};
int selPPMPHTanaman=0, scrollPPMPHTanaman=0;

int selTanamanKategori=0;
int selTanamanItem=0, scrollTanamanItem=0;

const char* jenisOptions[2] = {"AB Mix","pH Down"};
int selJenis = 0;

const char* dilOptions[2] = {"5 ml/L","10 ml/L"};
int selDil=0;

// EC Bebas (0.20-4.00, format wajib 3 digit "X.XX")
const float EC_BEBAS_MIN = 0.20f;
const float EC_BEBAS_MAKS = 4.00f;
String ecBebasDigits = "";
String kalkEC = "";   // EC bebas utk Kalkulator PPM
int ecBebasTujuan = 0;   // 0 = Parameter Dosing (terpilihEC/Baru), 1 = Kalkulator (kalkEC)

const char* acidOptions[2] = {"Nitrat","Fosfat"};
int selAcid=0;
const int TOTAL_KONSEN_OPTIONS = 3;
int selConc=0, scrollConc=0;

// Konsentrasi Asam BEBAS (3-30%, bilangan bulat tanpa desimal)
const int KONSEN_BEBAS_MIN = 3;
const int KONSEN_BEBAS_MAKS = 30;
String konsenBebasDigits = "";
int konsenBebasTujuan = 0;   // 0 = Parameter Dosing, 1 = Kalkulator pH Down

// -- Titik acuan EC hasil pengambilan data empiris (hanya 2, bukan lagi 8) --
// Index 0 = 5 mL/L, diukur di EC 2.0 ; Index 1 = 10 mL/L, diukur di EC 2.5
const float EC_ANCHOR[2] = {2.0f, 2.5f};
const float regresiSlopeABDefault[2] = {176.66f, 105.51f};
float regresiSlopeAB[2];   // diisi dari regresiSlopeABDefault[] pada setup()
// Slope AB Mix untuk EC bebas: skala linear dari titik acuan tiap pengenceran
float hitungSlopeAB(float ec, int dil){
  if(dil<0 || dil>1 || ec<=0.0f) return 0.0f;
  return (regresiSlopeAB[dil]/EC_ANCHOR[dil]) * ec;
}

const float regresiIntercept = 112.0f;
int scrollKalkulasi=0;

float slopeABMixTerpilih = 0.0f;

// Persentase konsentrasi pH Down AKTIF (Parameter Dosing):
int terpilihConcPersen = 0;

// ========================
// DATA MENU B – KALIBRASI
// ========================
const int TOTAL_MENU_B_ROWS = 6;
const char* menuBOptions[TOTAL_MENU_B_ROWS] = {
  "Sensor TDS","Sensor pH","Sensor Water Level","Sensor Suhu","Tes Sensor TDS","Tes Sensor pH"
};
int selMenuB=0, scrollMenuB=0;
int rtcInput[6]={0,0,0,0,0,0};
// ── KALIBRASI AKTIF (dipakai pembacaan sensor) ──
// TDS: s.d. 5 jenis buffer (titik), pH: s.d. 3 jenis buffer. Nilai buffer < 0 = slot kosong.
// Tata letak array SAMA dgn versi lama (struct EEPROM utama tidak berubah -> kalibrasi lama tetap terbaca).
#define TDS_TITIK_KAL 5
#define PH_TITIK_KAL  3
int   calTDSBuffer[TDS_TITIK_KAL] = {0,0,0,0,0};   // PPM buffer tiap titik (-1 = kosong)
float tdsCVInput[TDS_TITIK_KAL]   = {0,0,0,0,0};   // CV (V, terkompensasi suhu) tiap titik
float tdsCVUrut[TDS_TITIK_KAL]  = {0,0,0,0,0};     // titik aktif terurut CV (runtime)
float tdsPPMUrut[TDS_TITIK_KAL] = {0,0,0,0,0};
int   tdsJumlahTitik = 0;
bool  tdsKalibrasiAktif = false;

float phBufferInput[PH_TITIK_KAL] = {4.01f, 6.86f, 9.18f};   // pH buffer tiap titik (-1 = kosong)
float phVoltUrut[PH_TITIK_KAL]    = {0,0,0};
float phBufferUrut[PH_TITIK_KAL]  = {0,0,0};
int   phJumlahTitik = 0;
bool  phKalibrasi3TitikAktif = false;

// ── DRAFT KALIBRASI (TES_119) ──
// Sampel dikelompokkan per NILAI buffer: buffer sama boleh dipakai berkali-kali (dirata-rata, makin akurat) dan
// urutan pengambilan bebas. Kalibrasi baru AKTIF saat jumlah sampel minimum tercapai (TDS 5, pH 3) dan minimal
// 2 jenis buffer berbeda. Draft tersimpan di EEPROM -> bisa dilanjutkan kapan saja (tak perlu ulang dari awal).
#define KAL_TDS_SAMPEL_MIN 5
#define KAL_PH_SAMPEL_MIN  3
struct KalibDraft {
  uint16_t magic;
  int16_t  tdsPPM[TDS_TITIK_KAL];  float tdsSumCV[TDS_TITIK_KAL];  uint8_t tdsN[TDS_TITIK_KAL];
  int16_t  phX100[PH_TITIK_KAL];   float phSumV[PH_TITIK_KAL];     uint8_t phN[PH_TITIK_KAL];
  float    phSumSuhu[PH_TITIK_KAL];                                  // jumlah suhu per buffer pH (acuan ATC)
  // Sampel TERAKHIR per sensor (untuk "Hapus Sampel Terakhir"); idx -1 = tidak ada
  int8_t   tdsUndoIdx;  float tdsUndoCV;
  int8_t   phUndoIdx;   float phUndoV;  float phUndoSuhu;
};
#define KALIB_DRAFT_MAGIC 0xCA20u   // format TES_120 (draft TES_119 diganti draft baru dari kalibrasi aktif)
KalibDraft kalDraft;
byte kalSensorSel = 0;        // submenu kalibrasi: 0 = TDS, 1 = pH
int  selKalMenu = 0;          // pilihan di submenu kalibrasi
int  scrollKalMenu = 0;       // gulir submenu kalibrasi (4 item)
int  selKalLihat = 0;         // kursor buffer di layar Lihat/Hapus Data
int  scrollKalLihat = 0;      // layar Lihat Data
int  selKalUlang = 1;         // konfirmasi (default Tidak)
#define AKSI_KAL_HAPUS_TERAKHIR 0
#define AKSI_KAL_HAPUS_BUFFER   1
#define AKSI_KAL_MULAI_ULANG    2
#define AKSI_KAL_HAPUS_KALIBRASI 3   // TES_129: hapus kalibrasi aktif + draft -> kembali ke rumus bawaan
const int TOTAL_KAL_MENU = 5;        // Ambil/Tambah, Lihat/Hapus, Hapus Akhir, Mulai Ulang Draft, Hapus Data Kalibrasi
byte aksiKal = AKSI_KAL_MULAI_ULANG;

// ── DATA "TES SENSOR" (Mode Kalibrasi) ──
const float SUHU_KOREKSI_MAKS = 5.0f;   // TES_127: koreksi kalibrasi suhu maks +/-5 C (DS18B20 akurasi +/-0,5 C)
int   tesSensorLamaBilas  = 0;   // lama pembilasan chamber oleh D1 (detik)
int   tesSensorJedaStabil = 0;   // jeda stabilisasi setelah D1 OFF (detik)
byte  tesSensorFase       = 0;   // 0=bilas, 1=stabilisasi, 2=ukur
int   tesSensorDetik      = 0;
int   tesSensorHasilPPM   = 0;
float tesSensorHasilPH    = 0.0f;

// ==============================
// DATA MENU C – UJI DEBIT POMPA
// ==============================
const int TOTAL_UJI_OPTIONS = 8;
const char* ujiOptions[TOTAL_UJI_OPTIONS] = {
  "Pompa D1","Pompa D2","Pompa D3",
  "Pompa P1","Pompa P2","Pompa P3",
  "Katup Solenoid","Input Debit Pompa"
};

const int TOTAL_PUMP_OPTIONS = 6;
const char* pumpOptions[TOTAL_PUMP_OPTIONS] = {
  "Pompa D1","Pompa D2","Pompa D3",
  "Pompa P1","Pompa P2","Pompa P3"
};
const char* pumpNamaRingkas[TOTAL_PUMP_OPTIONS] = {
  "Pompa D1","Pompa D2","Pompa D3",
  "Pompa P1","Pompa P2","Pompa P3"
};
int selPump=0, scrollPump=0, inputPumpDelay=0;

const int pumpRelayPin[TOTAL_PUMP_OPTIONS] = {
  PIN_RELAY_D1, PIN_RELAY_D2, PIN_RELAY_D3,
  PIN_RELAY_P1, PIN_RELAY_P2, PIN_RELAY_P3
};

bool solenoidTerbuka = false;

// Variabel proses tes pompa non-blocking
int  tesPompaDetikBerjalan = 0;
int  tesPompaDetikTarget   = 0;
int  tesPompaSelCancel     = 1;
bool tesPompaBerjalan      = false;   // true selama relay pompa uji menyala (tetap jalan di latar belakang)
bool ujiD1Aktif = false;   // true selama Uji Reaksi menyalakan D1 (sirkulasi sampel ke sensor)
int  tesPompaIdx           = 0;       // indeks pompa yang sedang diuji
unsigned long tesPompaMulaiMs = 0;

// Variabel sub menu "Input Debit Pompa"
int   selInputDebit    = 0;
int   scrollInputDebit = 0;
long  inputDebitMl     = 0;   // long: input s.d. 99999 mL (int AVR hanya s.d. 32767)
int   inputDebitDetik  = 0;
float inputDebitHasil  = 0.0f;

// =================
// DATA DEBIT POMPA 
// =================
int terpilihDebitPump[TOTAL_PUMP_OPTIONS] = {0,0,0,0,0,0};
const float DEBIT_MAKS_LMNT = 10.0f;     // batas debit sesuai spesifikasi pompa terpasang (L/menit)
// TES_138: debit disimpan dlm mL/menit (L/menit x 1000; dulu x 100). Resolusi 0.01 L/mnt membuat pompa peristaltik
// kecil (mis. P1 0.034 L/mnt -> 0.03 = -12%, 0.014 -> 0.01 = -29%) didosis jauh dari persamaan.
#define DEBIT_SKALA 1000.0f
const int   DEBIT_MAKS_SIMPAN = 10000;
int selDebitPump=0, scrollDebitPump=0;

// =========================
// DATA MENU D – KALKULATOR
// =========================
const int TOTAL_MENU_D_ROWS = 2;
const char* menuDOptions[TOTAL_MENU_D_ROWS] = {
  "Tinggi Water Level","PPM/pH dari Input"
};
int selMenuD=0, scrollMenuD=0;
const char* kalkInputOptions[2] = {"PPM","pH"};
int selKalkInput=0;
int kalkSelEC=0, kalkScrollEC=0, kalkSelDil=0, kalkVolume=0;
int kalkPPMAwal=0, kalkPPMTarget=0;
float kalkHasilTambah=0.0f, kalkHasilTotal=0.0f;
int scrollKalkHasil=0;

// Slope PPM dari Input AKTIF 
float kalkSlopeTerpilih = 0.0f;

// ── DATA KALKULATOR "TINGGI WATER LEVEL" ──
int selKalkWLBentuk=0;             
int kalkWLVolume=0;                 
int kalkWLJariJari=0;              
int kalkWLPanjang=0;                
int kalkWLLebar=0;                  
float kalkWLHasil=0.0f;             


// ── DATA KALKULATOR pH DOWN ──

const int TOTAL_TABEL_PH = 12;

// TES_139: tabel slope pH Down (penurunan pH per 1 mL asam 10% dlm 10 L) dari data empiris pengambilan langsung
// (PPM 100-1300; kolom PPM = rata-rata PPM tiap rentang, fase atas 8-5.85, fase bawah 5.85-3.5). Baris > 1300 PPM
// (dan Fosfat atas 1100-1300) = PREDIKSI: kurva eksponensial (laju penurunan tetap) yg di-fit ke semua titik data
// (sensor TDS prototipe hanya stabil s/d ~1000 PPM). Baris 2200 & 2400 PPM di luar target maks (2100 PPM, rentang
// sensor industri): hanya ujung interpolasi (PPM terukur sedikit di atas 2100).
// Dulu (TES_115-138) tabel deret geometris tanpa dasar data: fase atas 200 PPM 0.532 (data 0.381).
// TES_139: tabel bawaan di PROGMEM (flash) -> hemat 384 byte RAM (dibaca dgn pgm_read_float).
const float tabelSlopeAtasDefault[TOTAL_TABEL_PH][2] PROGMEM = {
  {216.0f, 0.3810f},   // data empiris
  {407.0f, 0.3650f},   // data empiris
  {603.0f, 0.3140f},   // data empiris
  {818.0f, 0.2830f},   // data empiris
  {1040.0f, 0.2000f},   // data empiris
  {1200.0f, 0.1980f},   // data empiris
  {1400.0f, 0.1689f},   // prediksi (eksponensial, x0.862 / 200 PPM)
  {1600.0f, 0.1456f},   // prediksi (eksponensial, x0.862 / 200 PPM)
  {1800.0f, 0.1255f},   // prediksi (eksponensial, x0.862 / 200 PPM)
  {2000.0f, 0.1083f},   // prediksi (eksponensial, x0.862 / 200 PPM)
  {2200.0f, 0.0933f},   // prediksi (eksponensial, x0.862 / 200 PPM)
  {2400.0f, 0.0805f}    // prediksi (eksponensial, x0.862 / 200 PPM)
};
const float tabelSlopeBawahDefault[TOTAL_TABEL_PH][2] PROGMEM = {
  {220.0f, 1.4080f},   // data empiris
  {412.0f, 1.2630f},   // data empiris
  {621.0f, 1.2550f},   // data empiris
  {828.0f, 1.1330f},   // data empiris
  {1063.0f, 0.8320f},   // data empiris
  {1230.0f, 0.8200f},   // data empiris
  {1400.0f, 0.7489f},   // prediksi (eksponensial, x0.893 / 200 PPM)
  {1600.0f, 0.6687f},   // prediksi (eksponensial, x0.893 / 200 PPM)
  {1800.0f, 0.5970f},   // prediksi (eksponensial, x0.893 / 200 PPM)
  {2000.0f, 0.5331f},   // prediksi (eksponensial, x0.893 / 200 PPM)
  {2200.0f, 0.4760f},   // prediksi (eksponensial, x0.893 / 200 PPM)
  {2400.0f, 0.4250f}    // prediksi (eksponensial, x0.893 / 200 PPM)
};

const float tabelSlopeAtasFosfatDefault[TOTAL_TABEL_PH][2] PROGMEM = {
  {240.0f, 0.3810f},   // data empiris
  {382.0f, 0.3130f},   // data empiris
  {593.0f, 0.3125f},   // data empiris
  {808.0f, 0.2830f},   // data empiris
  {975.0f, 0.1880f},   // data empiris
  {1200.0f, 0.1799f},   // prediksi (eksponensial, x0.855 / 200 PPM)
  {1400.0f, 0.1538f},   // prediksi (eksponensial, x0.855 / 200 PPM)
  {1600.0f, 0.1315f},   // prediksi (eksponensial, x0.855 / 200 PPM)
  {1800.0f, 0.1124f},   // prediksi (eksponensial, x0.855 / 200 PPM)
  {2000.0f, 0.0961f},   // prediksi (eksponensial, x0.855 / 200 PPM)
  {2200.0f, 0.0821f},   // prediksi (eksponensial, x0.855 / 200 PPM)
  {2400.0f, 0.0702f}    // prediksi (eksponensial, x0.855 / 200 PPM)
};
const float tabelSlopeBawahFosfatDefault[TOTAL_TABEL_PH][2] PROGMEM = {
  {250.0f, 1.2880f},   // data empiris
  {382.0f, 1.2500f},   // data empiris
  {599.0f, 1.1930f},   // data empiris
  {814.0f, 0.9640f},   // data empiris
  {982.0f, 0.7980f},   // data empiris
  {1256.0f, 0.5760f},   // data empiris
  {1400.0f, 0.5604f},   // prediksi (eksponensial, x0.851 / 200 PPM)
  {1600.0f, 0.4770f},   // prediksi (eksponensial, x0.851 / 200 PPM)
  {1800.0f, 0.4061f},   // prediksi (eksponensial, x0.851 / 200 PPM)
  {2000.0f, 0.3457f},   // prediksi (eksponensial, x0.851 / 200 PPM)
  {2200.0f, 0.2943f},   // prediksi (eksponensial, x0.851 / 200 PPM)
  {2400.0f, 0.2506f}    // prediksi (eksponensial, x0.851 / 200 PPM)
};

// TES_139: slope bawaan LAMA (TES_115-138) -> tabel EEPROM yg masih sama persis dgn ini (belum pernah diedit user)
// diganti otomatis dgn tabel data empiris di atas (sekali, penanda VERSI_DATA_139). Tabel yg sudah diedit dibiarkan.
const float tabelSlopeLamaPGM[4][TOTAL_TABEL_PH] PROGMEM = {
  {0.5320f, 0.4730f, 0.4207f, 0.3741f, 0.3326f, 0.2958f, 0.2630f, 0.2339f, 0.2080f, 0.1850f, 0.1645f, 0.1463f},
  {1.5194f, 1.3641f, 1.2247f, 1.0995f, 0.9872f, 0.8863f, 0.7957f, 0.7144f, 0.6414f, 0.5758f, 0.5170f, 0.4641f},
  {0.4648f, 0.4182f, 0.3762f, 0.3384f, 0.3044f, 0.2738f, 0.2463f, 0.2216f, 0.1993f, 0.1793f, 0.1613f, 0.1451f},
  {1.3740f, 1.2497f, 1.1365f, 1.0337f, 0.9401f, 0.8550f, 0.7776f, 0.7072f, 0.6414f, 0.5758f, 0.5170f, 0.4641f}
};

// Tabel AKTIF (dipakai seluruh perhitungan) 
float tabelSlopeAtas[TOTAL_TABEL_PH][2];
float tabelSlopeBawah[TOTAL_TABEL_PH][2];
float tabelSlopeAtasFosfat[TOTAL_TABEL_PH][2];
float tabelSlopeBawahFosfat[TOTAL_TABEL_PH][2];

const float batasFasePH = 5.85f;   // TES_140: = batas fase data empiris slope pH Down (dulu 5.9)

const char* konsenPHOptions[TOTAL_KONSEN_OPTIONS] = {"10%","20%","30%"};
const int   konsenPHValues[3]  = {10,20,30};

int   kalkPHVolume   = 0;
int   kalkPHAcid     = 0;
int   kalkPHKonsen   = 0;
int   scrollKalkPHKonsen = 0;
int   kalkPHPpm      = 0;
float kalkPHAwal     = 0.0f;
float kalkPHTarget   = 0.0f;
float kalkPHHasilAtas  = 0.0f;
float kalkPHHasilBawah = 0.0f;
float kalkPHHasilTotal = 0.0f;
int   scrollKalkPHHasil = 0;
int   kalkPHKonsenPersen = 10;



// ================================
// DATA MENU "PENGATURAN PERSAMAAN"
// ================================
int selPengaturan = 0;              

int selPengaturanABDil = 0;         
int selPengaturanABEC  = 0;        
int scrollPengaturanABEC = 0;       

int selPengaturanPHAsam = 0;       
int ppmAcuanTundaPH = 0;           // PPM acuan yg diketik, di-commit bersama slope
// Perubahan persamaan saat jadwal cek aktif -> ditahan dulu & dikonfirmasi (langsung berlaku ke jadwal berjalan)
#define AKSI_PERS_AB    0
#define AKSI_PERS_PH    1
#define AKSI_PERS_RESET 2
byte  aksiPersamaan = AKSI_PERS_AB;
float nilaiPersamaanTunda = 0.0f;
int   selPersamaanKonfirmasi = 1;   // default "Tidak"
int selPengaturanPHFase = 0;       
int selPengaturanPHIdx  = 0;        

int fasePengaturanPHInput = 0;

int selPengaturanResetConfirm = 1;
int selPengaturanSistem = 0;
int scrollPengaturanSistem = 0;

// Opsi Pengaturan Sistem (4 item). TES_121: "Tuning PI" dihapus — Kp/Ki CL Auto-Tune selalu dihitung dari persamaan.
const int TOTAL_PENGATURAN_SISTEM_ROWS = 5;
const char* pengaturanSistemOptions[TOTAL_PENGATURAN_SISTEM_ROWS] = {
  "Persamaan Nutrisi","Waktu RTC","Kp/Ki PI ZN","Kunci Kanal","Reset Sistem"
};
int selKunciKanal = 0, selKunciKonf = 1;
int selResetSistemConfirm = 1;  

int scrollPengaturanOffset = 0;
unsigned long tsScrollPengaturan = 0;

// ============================
// VARIABEL PARAMETER TERSIMPAN
// ============================
String inputBuffer = "";

int    terpilihVolume      = 0;
int    terpilihPPM         = 0;
float  terpilihPH          = 0.0f;
String terpilihWaktuDosing = "--:--";
String terpilihEC          = "-";
String terpilihDilution    = "";
String terpilihAcid        = "-";
String terpilihConc        = "";

const int TOTAL_SUMMARY_ROWS = 36;   // 0-14 parameter, 15-16 batas, 17 jenis, 18 makro, 19-20 resolusi, 21.. Kp/Ki PI (maks 10), jeda J0-J4 (TES_144: dulu 33)
int scrollSummary = 0;
int scrollSummaryBaru = 0;

// ======================================
// DATA SUB-MENU "BATASAN PPM/pH" (TES_116)
// ======================================
// Cek harian pada Waktu Dosing. Kanal didosing hanya bila hasil cek keluar dari batasan:
//   AB Mix  : PPM < terpilihPPM - batasPPM   -> dosing AB hingga target
//   pH Down : pH  > terpilihPH  + batasPH    -> dosing pH Down hingga target
// PPM di atas batas / pH di bawah batas TIDAK bisa dikoreksi (aktuator satu arah) -> hanya peringatan.
// Batas minimum = toleransi kontrol PI (30 PPM / 0.25 pH) agar hasil dosing selalu jatuh di dalam batasan.
const int BATAS_PPM_MIN = 30,  BATAS_PPM_MAKS = 500;
// TES_140: target PPM 100-2100 (rentang sensor TDS versi industri s/d 2100 PPM; dulu 100-2500)
const int TARGET_PPM_MIN = 100, TARGET_PPM_MAKS = 2100;
bool targetPPMValid(int v){ return v>=TARGET_PPM_MIN && v<=TARGET_PPM_MAKS; }
const int BATAS_PH_MIN_X100 = 25, BATAS_PH_MAKS_X100 = 100;   // pH disimpan x100 (0.25 - 1.00)
int  batasPPM = 0;              // +/- PPM (Parameter Aktif)
int  batasPHx100 = 0;           // +/- pH x100 (Parameter Aktif)
bool batasanSudahDiatur = false;
int  batasPPMTunda = 0;         // nilai PPM yg diketik, di-commit bersama pH setelah layar pH di-OK
bool batasPPMValid(int v){ return v>=BATAS_PPM_MIN && v<=BATAS_PPM_MAKS; }
bool batasPHValid(int v){  return v>=BATAS_PH_MIN_X100 && v<=BATAS_PH_MAKS_X100; }
// Batas PPM harus < target agar batas bawah (target - batas) > 0; bila tidak, AB Mix tak akan pernah didosis.
bool batasPPMSesuaiTarget(int batas, int target){ return target<=0 || batas < target; }
// Batas PPM maksimum yang boleh diinput untuk target tertentu
int batasPPMMaksUntuk(int target){ int m = BATAS_PPM_MAKS; if(target>0 && target-1 < m) m = target-1; return m; }

// Jadwal cek berikutnya (tanggal); jam = Waktu Dosing
int dosingTargetHari  = 1;
int dosingTargetBulan = 1;
int dosingTargetTahun = 2026;

// ==========================================
// DATA MODE DOSING OTOMATIS – "MULAI DOSING"
// ==========================================
// dosingMode (nilai internal tetap demi EEPROM): 1 = Open Loop D85%+P15%, 3 = Closed Loop Auto-Tune (Kp/Ki dari model),
//                                                4 = Closed Loop PI ZN (Kp/Ki dari uji reaksi Ziegler-Nichols)
const int TOTAL_JENIS_DOSING_OPTIONS = 3;
const char* jenisDosingOptions[TOTAL_JENIS_DOSING_OPTIONS] = {
  "Open Loop","CL Auto-Tune","CL PI ZN"
};
int selJenisDosing = 0;

int  dosingMode    = 1;
// Besar MAKRO dosing (D2/D3 AB Mix & P1 pH Down tahap pertama), % dari kebutuhan menurut persamaan.
// Open Loop: sisa (100-makro)% oleh tahap mikro P2/P3 & P1. Closed Loop: sisa dikoreksi PI.
const int PERSEN_MAKRO_MIN = 50, PERSEN_MAKRO_MAKS = 90, PERSEN_MAKRO_DEFAULT = 85;
int  persenMakro     = PERSEN_MAKRO_DEFAULT;   // Parameter Aktif
int  persenMakroBaru = PERSEN_MAKRO_DEFAULT;   // Parameter Baru (draft)
int  modeJenisTunda  = 1;                      // jenis yg dipilih, di-commit bersama persen makro
bool persenMakroValid(int v){ return v>=PERSEN_MAKRO_MIN && v<=PERSEN_MAKRO_MAKS; }

int terpilihLamaCekSensor = 0;      // dipakai sbg LAMA PEMBILASAN chamber oleh pompa D1 (detik)
int terpilihJedaStabilisasi = 0;    // jeda stabilisasi (D1 OFF) sebelum jendela median (detik)

// ── KONTROL PI (MODE CLOSED-LOOP) ──
// TES_122: toleransi diperketat. Bacaan per detik berfluktuasi ±30 PPM / ±0.25 pH, tetapi keputusan PI memakai MEDIAN
// 20 detik (fluktuasi median jauh lebih kecil), dan dosis < 0.5 dtk pompa tetap ditolak deadband -> tak overshoot.
#define PI_TOL_PPM          15      // toleransi berhenti kanal PPM (± PPM)
#define PI_TOL_PH           0.10f   // toleransi berhenti kanal pH (± pH)
#define PI_PULSA_MIN_DETIK  1       // pulsa minimum pompa peristaltik (detik) = resolusi timer dosing (tick 1 dtk)
#define PI_DEADBAND_DETIK   0.5f    // pulsa hitungan PI < 0.5 dtk = di bawah resolusi pompa -> anggap sudah tercapai (cegah overshoot)
#define PI_MAKS_ITERASI     30      // batas pengaman iterasi PI per kanal (cegah dosing tanpa henti)
// TES_121: batas pulsa per iterasi TIDAK lagi tetap 30 dtk, tetapi = waktu pompa yang menurut persamaan cukup untuk
// menutup SELURUH error saat ini (lihat piBatasPulsa*), dibatasi juga PUMP_MAKS_DETIK pompa terkait.
const float PI_FRAKSI_KOREKSI = 0.5f;   // Kp = 0.5/g : satu pulsa menutup 50% error menurut model
const float PI_RASIO_KI       = 0.1f;   // Ki = 0.1·Kp : akar karakteristik riil (g·Ki <= 0.0625) -> tanpa overshoot
// Struktur kontroler PI diskrit (output = detik pulsa pompa)
struct PIController { float kp, ki, integral; };
PIController piPPM = {0.05f, 0.01f, 0.0f};  // kanal PPM: mengatur pulsa P2 & P3 (AB Mix); Kp dihitung otomatis dari slope+debit aktual
PIController piPH  = {20.0f, 2.0f, 0.0f};   // kanal pH : mengatur pulsa P1 (pH Down); piPH.kp diisi dinamis dari piPHKpAtas/piPHKpBawah
float piPHKpAtas  = 20.0f;   // Kp saat bacaan pH >= batasFasePH (fase atas kurva pH Down) - dihitung otomatis
float piPHKpBawah = 20.0f;   // Kp saat bacaan pH <  batasFasePH (fase bawah kurva pH Down) - dihitung otomatis
float piPulsaDetikAB = 0.0f, piPulsaDetikPH = 0.0f;
int  piIterasiPPM = 0, piIterasiPH = 0;
// ── CEK RESPON (TES_122): tiap 3 pulsa PI (iterasi 3, 6, 9, ...) kenaikan TERUKUR dibanding jumlah prediksi
// persamaan 3 pulsa tsb. Terukur < 30% prediksi -> sensor macet / chamber tak tersirkulasi (D1) / stok habis /
// pompa macet -> kanal dihentikan + alarm (cegah dosis berulang ke larutan yang sebenarnya sudah cukup).
#define RESPON_JENDELA        3        // jumlah pulsa per evaluasi
const float RESPON_FRAKSI_MIN = 0.30f; // terukur minimal 30% prediksi
const float RESPON_PRED_MIN_PPM = 45.0f;   // prediksi 3 pulsa terlalu kecil (< 3x toleransi) -> tak dinilai (derau sensor)
const float RESPON_PRED_MIN_PH  = 0.30f;
struct JendelaRespon { float acuan; float prediksi; byte n; float vol; };   // vol: volume pH Down yg benar2 keluar (TES_130)
JendelaRespon resAB = {0,0,0,0}, resPH = {0,0,0,0};
// Catatan PI siklus berjalan (disimpan di 4 bit atas RekamCek.err): masalah kanal untuk Status Dosing
#define PI_FLAG_AB_RESPON  0x01
#define PI_FLAG_PH_RESPON  0x02
#define PI_FLAG_AB_ITERASI 0x04
#define PI_FLAG_PH_ITERASI 0x08
uint8_t piFlagSiklus = 0;
bool makroABDilewati = false;    // siklus ini: makro D2/D3 dilewati krn terlalu singkat (< MAKRO_MIN_DETIK)
// ── KUNCI KANAL (TES_123): kanal yang tidak merespon dikunci -> TIDAK didosing di siklus berikutnya (tetap diukur)
// sampai user membuka kunci di Pengaturan Sistem > Kunci Kanal (setelah sensor/pompa/stok dicek). Disimpan di EEPROM.
#define KUNCI_AB 0x01
#define KUNCI_PH 0x02
uint8_t kanalTerkunci = 0;
// Cek respon OPEN LOOP (TES_123): makro -> cek 2 dibandingkan prediksi makro (OL hanya punya 2 dosis per kanal)
float olAcuanAB = 0, olVolAwalAB = 0, olAcuanPH = 0, olVolAwalPH = 0;
float volAwalMikroAB = 0;       // volume AB tercatat saat mikro mulai (cek respon mikro Open Loop)
float olAcuanMikroAB = 0;       // TES_130: PPM terukur sebelum mikro Open Loop (acuan cek respon P2-P3)
bool  cekMakroABTertunda = false, cekMakroPHTertunda = false;   // TES_130: respon makro CL dinilai di cek PI pertama
// TES_138: makro yang sudah dipompa tapi responnya belum dinilai DISIMPAN (EEPROM). Daya padam saat homogenisasi ->
// siklus ulang dulu menilai respon makro itu pada cek pertama; dulu sensor macet + daya padam = makro kedua (overdosis).
#define MP_AB       0x01
#define MP_PH       0x02
#define MP_AB_ULANG 0x04   // sudah diukur ulang sekali (respon < 30%) -> ukur berikutnya < 30% = kunci kanal
#define MP_PH_ULANG 0x08
uint8_t  makroPulih = 0;
int16_t  makroPulihAcuanPPM = 0, makroPulihPredPPM = 0, makroPulihAcuanPHx100 = 0;
uint16_t makroPulihVolPHx10 = 0;
bool     siklusDariPulih = false;      // siklus dimulai dari DAYA PULIH (makroPulih dipertahankan)
bool     ulangPulihAB = false, ulangPulihPH = false;   // ukur ulang cek pertama sesudah homogenisasi
// TES_137: ukur ulang respon (pengadukan lambat / jeda homogenisasi kurang) sebelum memutuskan kunci / lanjut
bool  ulangResponAB = false, ulangResponPH = false;   // cek ini < 70% -> homogenisasi ulang lalu ukur lagi
bool  sedangUlangAB = false, sedangUlangPH = false;   // cek yang berjalan adalah ukur ulang
bool  responLambatSiklus = false;                     // ada kanal yang dihentikan krn respon lambat
float naikPertamaAB = 0.0f, frPertamaPH = 0.0f, phPertamaUlang = 0.0f;

int8_t  piFasePHTerakhir = -1;   // 1 = fase atas, 0 = fase bawah, -1 = belum ada (reset integral pH saat fase berganti)
// TES_122: makro D2/D3 lebih singkat dari ini -> dilewati, seluruh kebutuhan langsung ke P2/P3 (mikro / PI)
#define MAKRO_MIN_DETIK 3

// ============================================================================
// === MODE DOSING 4 — "ZN MANUAL Kp/Ki" (arsitektur 7 poin) ================
// ============================================================================
// PI aktif yang dipakai state machine dosing ketika dosingMode==4.
// Sengaja dipisah dari piPPM/piPH (mode 3, Model-Based) sesuai kesepakatan.
PIController piPPM_ZN = {0.0f, 0.0f, 0.0f};
float piPHKpAtas_ZN = 0.0f, piPHKiAtas_ZN = 0.0f;
float piPHKpBawah_ZN = 0.0f, piPHKiBawah_ZN = 0.0f;

const float FRAKSI_AMAN_ZN = 0.5f;   // batas atas fraksi penutupan error / iterasi

// -- Poin 1: signature konfigurasi yang mempengaruhi Kp/Ki --
struct SignatureKpKi {
  float   volume;
  uint8_t jenisAsam;        // 0 = Nitrat, 1 = Fosfat
  float   konsentrasiAsam;  // %
  uint8_t jenisABMixDil;    // 0 = 5 mL/L, 1 = 10 mL/L
  float   targetEC;
  float   debitP1;          // mL/detik, pompa peristaltik kanal pH
  float   debitP2;          // mL/detik, pompa peristaltik kanal PPM/AB Mix
};
const float EPS_VOL_ZN=0.5f, EPS_KONS_ZN=0.5f, EPS_EC_ZN=0.01f, EPS_DEBIT_ZN=0.05f;

// TES_141: VOLUME tidak dibandingkan (Kp/Ki diskalakan otomatis V_param/V_uji). Item lain beda = Kp/Ki dari
// konfigurasi lama -> peringatan uji ulang saat Mulai Dosing / aktivasi Parameter Baru (bisa dilanjutkan).
// Hasil: item PERTAMA yang berbeda (teks <= 14 karakter), NULL = sama.
const __FlashStringHelper* signatureBeda(const SignatureKpKi &a, const SignatureKpKi &b){
  if(a.jenisAsam!=b.jenisAsam)                               return F("Jenis asam");
  if(fabs(a.konsentrasiAsam-b.konsentrasiAsam)>=EPS_KONS_ZN) return F("Kons. asam");
  if(a.jenisABMixDil!=b.jenisABMixDil)                       return F("Pengenceran AB");
  if(fabs(a.targetEC-b.targetEC)>=EPS_EC_ZN)                 return F("EC AB Mix");
  if(fabs(a.debitP1-b.debitP1)>=EPS_DEBIT_ZN)                return F("Debit P1");
  if(fabs(a.debitP2-b.debitP2)>=EPS_DEBIT_ZN)                return F("Debit P2/P3");
  return NULL;
}
bool signatureSama(const SignatureKpKi &a, const SignatureKpKi &b){ return signatureBeda(a,b)==NULL; }

// -- Poin 2 (direvisi): cukup 1 slot aktif, sama seperti Parameter Dosing lain --
struct ProfilKpKiZN {
  bool terisi=false;
  SignatureKpKi sig={0,0,0,0,0,0,0};
  float kpPPM=0, kiPPM=0;
  float kpPHAtas=0, kiPHAtas=0;
  float kpPHBawah=0, kiPHBawah=0;
};
ProfilKpKiZN profilZNAktif;
void sanitasiProfilZN(ProfilKpKiZN &p);   // TES_148
float faktorSkalaPPMZNPada(int kanal, bool baru, int tgt);   // TES_148
// TES_141: acuan uji per profil (0 = Aktif, 1 = Baru). Kp/Ki tersimpan berlaku utk VOLUME acuan (sig.volume = volume
// tandon uji) dan, kanal pH, PPM tandon saat uji. Saat dipakai diskalakan ke Parameter konteks:
//   Kp,Ki x V_param/V_uji (gain proses ~ 1/V)  dan  kanal pH x slope(PPM uji)/slope(PPM target) (gain pH Down ~ slope).
int16_t znPPMUji[2][2] = {{0,0},{0,0}};   // [profil][0 = pH atas, 1 = pH bawah]; 0 = tak diketahui (tanpa skala PPM)
int     ujiVolumeL = 0;                    // volume tandon uji terakhir (isi awal input); 0 = belum pernah
int     hasilUjiPPMAcuan = 0;              // PPM acuan hasil uji terakhir (kanal pH: PPM terukur saat bilas)
uint8_t znKanalLama[2] = {0,0};            // TES_144: [profil] bit kanal (1 PPM, 2 pH atas, 4 pH bawah) hasil uji konfigurasi lama
const int UJI_VOL_MIN = 10, UJI_VOL_MAKS = 3000;
const int ZN_TOL_PPM_SKALA = 30;           // |PPM uji - target| <= toleransi sensor TDS (+/-30 PPM) -> tanpa skala PPM
const float ZN_RASIO_KI_AKAR_RIIL = 0.25f; // Ki <= 0.25.g.Kp^2 (b <= a^2/4, a = g.Kp, b = g.Ki): akar PI per iterasi riil

int selZNHub=0;
int halKpKiZN=0;          // 0=PPM, 1=pH Atas, 2=pH Bawah — poin 3
int znEditTahap=0;        // 0=input Kp, 1=input Ki (pada layar edit manual)
float znEditKpSementara=0;
int ujiKanalDipilih=0;    // 0=PPM(AB Mix), 1=pH Atas, 2=pH Bawah

// TES_119: lama rekam diatur user (TES_138: 5-240 menit) & pulsa dihitung otomatis dari gain model agar respon cukup besar
// tanpa mengganggu larutan. Kurva disimpan 20 titik dgn "desimasi": bila penuh, separuh titik dibuang & interval
// sampel digandakan -> awal kurva (bagian curam, penentu D & tau) tetap rapat, rekaman panjang tetap muat di RAM.
// Rumus tetap metode kurva reaksi Ziegler-Nichols (PI): Kp = 0.9*tau/D / K , Ki = 0.27*tau/D^2 / K.
#define UJI_MAKS_SAMPEL     20
#define UJI_INTERVAL_AWAL_MS 4000UL
#define UJI_INTERVAL_MAKS_MS 60000UL   // TES_138: interval sampel maks 1 mnt (rekaman s/d 4 jam tetap terpantau tiap menit)
#define UJI_REKAM_MIN_MENIT 5
#define UJI_REKAM_MAKS_MENIT 240   // TES_138: = batas jeda homogenisasi (4 jam); uji berhenti sendiri saat stabil
#define JEDA_MIN_MENIT 1
#define JEDA_MAKS_MENIT 240        // TES_138: J0..J4 1-240 menit (dulu 210)
#define UJI_PULSA_MIN_DETIK 2.0f
#define UJI_PULSA_MAKS_DETIK 600.0f   // TES_138: dulu 30 dtk -> tandon besar (>~1500 L) selalu "Respon tidak terdeteksi"
#define UJI_STABIL_JEDA_MIN_DETIK 60.0f   // TES_138: jendela stabil minimum & masa tunggu minimum sesudah pulsa
#define UJI_STABIL_FRAKSI_WAKTU   0.30f   // TES_138: jendela stabil = maks(60 dtk, 30% waktu berjalan)
#define UJI_STABIL_RELATIF        0.03f   // TES_138: perubahan dalam jendela < 3% respon
#define UJI_STABIL_KONFIRMASI     5       // TES_138: syarat stabil harus terpenuhi 5 sampel berturut-turut (tahan derau)
#define UJI_STABIL_MAKS_RELATIF   0.21f   // TES_138: perubahan dalam jendela juga wajib < 21% respon (kenaikan lambat awal != datar)
                                          // TES_143: 15% -> 21% (> ambang datar derau pada respon = 10 x derau)
const float UJI_RESPON_MIN_PPM = 5.0f;        // respon < ini = tidak terdeteksi
const float UJI_RESPON_MIN_PH  = 0.05f;
const float UJI_STABIL_AMB_PPM = 2.0f;        // ambang perubahan absolut dalam jendela (rata-rata 3 sampel)
const float UJI_STABIL_AMB_PH  = 0.015f;
const float UJI_TARGET_RESPON_PPM = 50.0f;    // kenaikan PPM yang dituju oleh pulsa uji (minimum, sensor tenang)
const float UJI_TARGET_RESPON_PH  = 0.20f;    // penurunan pH yang dituju oleh pulsa uji (minimum, sensor tenang)
// TES_142: penyesuaian derau sensor
#define UJI_DERAU_DETIK   60                  // lama ukur derau sesudah bilas (D1 tetap mengalir)
#define UJI_SUB_MS        500UL               // sub-sampel tiap 0,5 dtk (1 bacaan = rata-rata 10 ADC, ~0,1 dtk)
#define UJI_BLOK_DERAU_MS 4000UL              // blok derau = interval sampel awal kurva (4 dtk)
const float UJI_TARGET_RESPON_PPM_MAKS = 250.0f;   // TES_143: respon tuju maksimum (sensor prototipe; dulu 150 PPM / 0.60 pH)
const float UJI_TARGET_RESPON_PH_MAKS  = 1.00f;
const float UJI_SNR_TUJU = 25.0f;   // respon tuju >= 25 x derau sampel -> ambang datar ~8% respon
const float UJI_SNR_MIN  = 10.0f;   // TES_143: < 10 x derau (dulu 14x): ambang datar (2.04 x derau) > 20% respon -> plateau tak terbedakan
const float UJI_K_DATAR_DERAU   = 2.04f;  // 2.5 x sd selisih 2 rata-rata 3 sampel (= 0.816 x derau sampel)
const float UJI_K_DETEKSI_DERAU = 3.0f;   // respon < 3 x derau = tidak terdeteksi
const float UJI_PH_DATA_MIN = 3.5f;       // TES_144: batas bawah data slope pH Down (fase bawah 5.85-3.5)

float ujiWaktu[UJI_MAKS_SAMPEL];
float ujiNilai[UJI_MAKS_SAMPEL];
// TES_145: salinan rekaman kurva Uji Reaksi terakhir per kanal (0 = PPM AB Mix, 1 = pH fase atas, 2 = pH fase bawah).
// Nilai ABSOLUT (nilai awal + respon) x10 untuk PPM / x1000 untuk pH (int16 hemat RAM: 3 x 21 x 4 = 252 byte).
// Titik ke-0 = nilai awal (t = 0 dtk), titik berikutnya = sampel kurva. Salinan EEPROM: eepromSimpanZNRek / eepromMuatZNRek.
#define UJI_REK_TITIK (UJI_MAKS_SAMPEL + 1)
uint16_t znRekWaktu[3][UJI_REK_TITIK];     // detik sejak awal pulsa
int16_t  znRekNilai[3][UJI_REK_TITIK];
byte     znRekN[3] = {0,0,0};              // jumlah titik tersimpan per kanal (0 = belum pernah diuji)
int      znRekKanal = -1;                  // kanal yang sedang ditampilkan (-1 = belum ada rekaman)
int      znRekScroll = 0;                  // indeks titik pertama yang tampil
int znRekKanalPertama(){ for(int c=0;c<3;c++) if(znRekN[c]>0) return c; return -1; }
int znRekKanalBerikut(int k){ for(int st=1;st<=3;st++){ int c=(k+st)%3; if(znRekN[c]>0) return c; } return k; }   // AB Mix -> Atas -> Bawah -> AB Mix
int   ujiJumlahSampel=0;
float ujiC0=0;
float ujiDurasiPulsa=5.0f;
int   ujiDurasiMaksMenit=60;   // TES_138: bawaan 60 mnt (uji berhenti sendiri saat stabil)
bool  ujiPulsaAktif=false;     // TES_138: pulsa uji non-blocking (pompa P2+P3 / P1 sedang menyala)
// TES_138: penilaian stabil memakai data mentah (tak terpengaruh penjarangan buffer kurva):
//   ujiRaw  = 3 sampel terakhir (rata-rata -> nilai sekarang & plateau),
//   ujiSt*  = 10 titik acuan (rata-rata 3 sampel) yang dicatat tiap waktu bertambah 5% -> selalu ada acuan ~30% waktu lalu
#define UJI_TITIK_ACUAN 10
float ujiRaw[3]; byte ujiRawN=0, ujiRawIdx=0;
uint16_t ujiStT[UJI_TITIK_ACUAN]; float ujiStY[UJI_TITIK_ACUAN];   // waktu (detik, <= 14400+) hemat RAM
byte ujiStN=0, ujiStIdx=0; float ujiStTerakhirT=0.0f;
byte  ujiStabilBerturut=0;
byte  ujiFase=0;               // TES_138: 0 = bilas chamber (D1) sebelum nilai awal, 1 = pulsa & rekam kurva
unsigned long ujiBilasMulaiMs=0; int ujiBilasDetik=30;
bool  ujiTdsUntukPH=false;     // TES_139: kanal pH: TDS dinyalakan saat bilas utk mengukur PPM tandon (ukuran pulsa pH)
float ujiPPMTerukur=-1.0f;
int   ujiRekamMinMenit=UJI_REKAM_MIN_MENIT;   // TES_138: minimum = pulsa + 3 mnt (rekaman lebih pendek pasti gagal)
int   ujiPulsaMaksDetik=0;                    // TES_142: pulsa terpanjang (respon tuju maks)
unsigned long ujiDetikTampil=0xFFFFFFFFUL;
unsigned long ujiIntervalMs = UJI_INTERVAL_AWAL_MS;
unsigned long ujiTimerAwal=0, ujiTimerSampelTerakhir=0;
float hasilUjiTau,hasilUjiD,hasilUjiK,hasilUjiKp,hasilUjiKi;
// TES_142: derau & rata-rata sub-sampel
float ujiDerauSd = 0.0f;          // derau 1 sampel kurva (sd blok 4 dtk, tren dibuang); 0 = belum/tenang
byte  ujiInvalidBerturut = 0;      // TES_144: bacaan sensor tidak valid berturut-turut selama uji
float ujiResponTuju = 0.0f;       // respon yang dituju pulsa (prediksi model)
unsigned long ujiDerauMulaiMs = 0, ujiSubTerakhirMs = 0, ujiBlokMulaiMs = 0, ujiSubPertamaMs = 0, ujiSubAkhirMs = 0;
float ujiSubSum = 0.0f; int ujiSubN = 0;     // akumulasi sub-sampel (blok derau / interval rekam)
float ujiRegY0 = 0.0f, ujiRegSx = 0.0f, ujiRegSy = 0.0f, ujiRegSxx = 0.0f, ujiRegSxy = 0.0f, ujiRegSyy = 0.0f; int ujiRegN = 0;
float ujiRegXAkhir = 0.0f;


const char* znHubOptions[3] = {"Uji Reaksi Baru","Rekaman Terakhir","Lihat Kp/Ki Aktif"};   // TES_145: + Rekaman Terakhir
const char* ujiKanalOptions[3] = {"PPM (AB Mix)","pH Fase Atas","pH Fase Bawah"};
bool piPPMSelesai = false, piPHSelesai = false;

int dosingJedaHomogenisasi[6] = {0,0,0,0,0,0};   // [0..3] = J1..J4, [4] = J0 (TES_134), [5] cadangan
#define IDX_JEDA_J0 4                              // TES_134: slot array utk J0 (sesudah isi tandon / daya pulih)
int jedaJ0Menit(){ int v = dosingJedaHomogenisasi[IDX_JEDA_J0]; return (v>=JEDA_MIN_MENIT && v<=JEDA_MAKS_MENIT) ? v : 0; }
int dosingJedaInputIdx = 0;

int scrollJedaOffset = 0;
unsigned long tsScrollJeda = 0;

// ===============
// PARAMETER BARU 
// ===============
bool dosingSedangMenunggu = false; // true selama menunggu jadwal cek harian berikutnya
bool jadwalCekAktif = false;       // true selama cek harian ter-arm (Mulai Dosing s.d. dibatalkan); dipakai lanjut otomatis setelah daya pulih
int finalStatusDetikBerjalan = 0;
const int FINAL_STATUS_AUTO_LANJUT_DETIK = 15;

// ── Siklus berjalan ──
unsigned long dosingMulaiMs   = 0;              // millis() saat pengisian tandon selesai
float dosingTotalABmL = 0.0f, dosingTotalPHmL = 0.0f;   // akumulasi estimasi volume siklus BERJALAN
int  rekapDetikBerjalan       = 0;
#define REKAP_AUTO_LANJUT_DETIK 5               // rekap otomatis beralih ke Status Dosing setelah 5 detik
byte statusHalaman = 0;                         // overlay Status Dosing. Saat siklus berjalan: 0 = status utama, 1 = jadwal & dosing terakhir.
                                                // TES_146 - saat TIDAK ada siklus (menunggu cek): 0 = tampilan menunggu, 1 = hasil cek terakhir, 2 = jadwal & dosing terakhir

// ── Hasil cek harian terakhir + rekap dosing terakhir (Status Dosing; EEPROM terpisah, tahan mati listrik) ──
// Status dalam/luar batas TIDAK disimpan: selalu dinilai ulang terhadap target & batasan AKTIF saat ini, sehingga
// setelah Parameter Baru dipakai, nilai PPM/pH terakhir langsung dinilai dengan target baru.
#define CEK_PPM_RENDAH 0x01   // PPM < target - batas
#define CEK_PPM_TINGGI 0x02   // PPM > target + batas (tak bisa diturunkan otomatis)
#define CEK_PH_RENDAH  0x04   // pH  < target - batas (tak bisa dinaikkan otomatis)
#define CEK_PH_TINGGI  0x08   // pH  > target + batas
#define CEK_ERR_TIDAK  0      // siklus tuntas
#define CEK_ERR_TANDON 1      // pengisian tandon melewati batas waktu -> cek hari itu dilewati
#define CEK_ERR_SENSOR 2      // sensor TDS/pH tidak valid -> cek hari itu dilewati
#define CEK_ERR_LAMBAT 4      // TES_137: respon dosis lambat (pengadukan) -> dosis kanal dihentikan, dilanjut cek berikutnya
struct RekamCek {
  uint16_t magic;
  int      ppm;        // -1 = belum ada
  float    ph;         // <0 = belum ada
  uint8_t  err;        // CEK_ERR_*
  uint8_t  didosing;   // 1 = siklus itu mendosis
  uint8_t  tgl, bln, jam, mnt;  uint16_t thn;     // waktu cek (tgl 0 = RTC tidak valid)
  // Rekap siklus DOSING terakhir (tak tertimpa oleh cek tanpa dosis)
  uint8_t  dTgl, dBln, dJam, dMnt; uint16_t dThn; // waktu dosing selesai (dTgl 0 = belum pernah)
  int      dDurasiMenit;
  float    dABmL, dPHmL;
};
#define REKAM_CEK_MAGIC 0xC17Cu   // format TES_118 (format TES_116/117 otomatis diabaikan)
RekamCek cekTerakhir = {REKAM_CEK_MAGIC, -1, -1.0f, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0.0f, 0.0f};

// ── Data tambahan TES_118 (EEPROM terpisah; struct utama tidak diubah agar kalibrasi tetap aman) ──
bool draftBaruAda = false;      // true = Parameter Baru sudah disalin dari Aktif & sedang diedit (hanya saat jadwal menunggu)
ProfilKpKiZN profilZNBaru;      // profil Kp/Ki PI ZN untuk konfigurasi Parameter Baru
struct KonfigEkstra {
  uint16_t magic;
  uint8_t  draftBaruAda;
  uint8_t  cadangan1;            // dulu piManual (Tuning PI, dihapus TES_121) — dipertahankan agar tata letak tetap
  ProfilKpKiZN profilZNBaru;
  uint8_t  persenMakro;          // TES_121 (data lama: byte tak valid -> default 85%)
  uint8_t  persenMakroBaru;
  uint8_t  kanalTerkunci;        // TES_123 (data lama: byte > 3 -> 0)
  uint8_t  versiJeda;            // TES_134: VERSI_JEDA_J0 = slot J0 sudah dipakai (data lama -> J0 diisi dari J1)
  // TES_138 (9 byte, total 64 = penuh): makro sudah dipompa tapi responnya belum dinilai saat daya padam
  uint8_t  makroPulih;           // bit MP_*
  int16_t  makroPulihAcuanPPM;   // PPM sebelum makro AB
  int16_t  makroPulihPredPPM;    // kenaikan PPM prediksi makro AB (volume yg benar2 keluar)
  int16_t  makroPulihAcuanPHx100;// pH sebelum makro pH Down (x100)
  uint16_t makroPulihVolPHx10;   // mL pH Down makro yg keluar (x10)
};
#define VERSI_JEDA_J0 0x34
#define VERSI_DATA_138 0x35      // TES_138: J0 + kolom makroPulih valid
#define VERSI_DATA_139 0x36      // TES_139: + tabel slope pH Down data empiris (migrasi tabel bawaan lama)
bool jedaJ0SudahAda = false;     // TES_134: diisi saat muat EEPROM ekstra
uint8_t versiDataEkstra = 0xFF;  // TES_138: versi data saat dimuat (migrasi)
#define KONFIG_EKSTRA_MAGIC 0xE118u
const unsigned long BUZZER_ALARM_DURASI_MS = 30UL*1000UL;   // alarm putus-putus 30 dtk (RTC rusak / hasil di luar batas)
int isiTandonDetikBerjalan = 0;
byte tandonPenuhBerturut = 0;                    // detik berturut-turut sensor level HIGH (debounce)
const byte TANDON_PENUH_KONFIRMASI_DETIK = 3;    // sensor harus bertahan 3 dtk agar dianggap penuh
String ecTunda = "";                             // EC yg diketik, baru di-commit setelah pengenceran dipilih
const long TANDON_TIMEOUT_DETIK = 6L*3600L;   // 6 jam: cukup utk tandon 3000 L dgn debit isi rendah
// Pengaman uji Water Level: batasi durasi agar solenoid tak terbuka tanpa batas
unsigned long waterLevelTestMulai = 0;
bool          waterLevelTestTimeout = false;
const unsigned long WL_TEST_TIMEOUT_MS = 15UL*60UL*1000UL; // 15 menit

//  BUZZER 
unsigned long buzzerMulaiMs  = 0;
unsigned long buzzerDurasiMs = 0;
bool          buzzerBunyi    = false;
bool          wlBuzzerSudah  = false;
bool          wlBuzzerMilikWL = false;   // buzzer yg sedang bunyi dinyalakan oleh menu Water Level (boleh dimatikan menu itu)
bool          wlIsiAktif     = false;    // menu Water Level: pengisian tandon dimulai user (B), bukan otomatis
unsigned long solenoidTesMulai = 0;      // Tes Katup Solenoid: waktu katup dibuka
byte          solenoidTesPenuhDetik = 0; // TES_128: detik berturut-turut sensor level HIGH saat tes katup
bool          solenoidTesPenuh = false;  // TES_128: katup tes ditutup otomatis karena tandon penuh
const unsigned long SOLENOID_TES_MAKS_MS = 15UL*60UL*1000UL;   // katup tes otomatis menutup setelah 15 menit   
const unsigned long BUZZER_DURASI_MS = 10UL*1000UL; 
const unsigned long BUZZER_PUTUS_MS = 500UL;               // 0,5 dtk ON / 0,5 dtk OFF
bool          buzzerPutus    = false;
bool          buzzerFaseOn   = false;
unsigned long buzzerToggleMs = 0;

String terpilihECBaru        = "-";
String terpilihDilutionBaru  = "";
float  slopeABMixTerpilihBaru = 0.0f;
String terpilihAcidBaru      = "-";
String terpilihConcBaru      = "";
int    terpilihConcPersenBaru = 0;   // 0 = belum dipilih (wajib)
int    selJenisBaru = 0;

int    terpilihVolumeBaru = 0;
int    terpilihPPMBaru    = 0;
float  terpilihPHBaru     = 0.0f;
String terpilihWaktuDosingBaru = "--:--";
int    terpilihDebitPumpBaru[TOTAL_PUMP_OPTIONS] = {0,0,0,0,0,0};

int  batasPPMBaru    = 0;
int  batasPHx100Baru = 0;
bool batasanBaruSudahDiatur = false;

int  dosingModeBaru       = -1; // -1 = belum dipilih
int  terpilihLamaCekSensorBaru = 0;
int  terpilihJedaStabilisasiBaru = 0;
bool jenisDosingBaruSudahDiatur = false;

int  dosingJedaHomogenisasiBaru[6] = {0,0,0,0,0,0};
bool jedaBaruSudahDiatur = false;

// Penanda kelengkapan Parameter Aktif 
bool jenisDosingSudahDiatur = false;
bool jedaSudahDiatur        = false;

int selMulaiDosingKonfirmasi = 1; // default "Tidak"

int selCancelDosing = 1;

bool cancelConfirmAktif = false;
unsigned long cancelConfirmMulaiMs = 0;
const unsigned long CANCEL_CONFIRM_TIMEOUT_MS = 30000UL;   // overlay "Cancel Dosing?" tertutup sendiri (= Tidak) setelah 30 dtk

int   dosingCekDetikBerjalan = 0;
byte  cekSensorFase = 0;   // 0=bilas (D1 ON), 1=stabilisasi (D1 OFF), 2=ukur (jendela median)
long  dosingPPMSum   = 0;
int   dosingPPMCount = 0;
float dosingPHSum    = 0.0f;
int   dosingPHCount  = 0;
bool  dosingPPMTerkunci = false;
bool  dosingPHTerkunci  = false;
int   dosingPPMHasilCek = 0;
float dosingPHHasilCek  = 0.0f;

MenuState dosingTujuanSetelahCekPPM = DOSING_MACRO_AB;
MenuState dosingTujuanSetelahCekPH  = DOSING_MACRO_PH;
MenuState dosingTujuanSetelahHomogen = DOSING_CEK_PPM2;

float dosingABMixTahap1  = 0.0f;
float dosingABMixTahap2  = 0.0f;float dosingPHDownTahap1 = 0.0f;
float dosingPHDownTahap2 = 0.0f;bool dosingLewatiAB = false;
bool dosingLewatiPH = false;

bool dosingSudahAdaDosis = false;

int   statusPPMTerakhir = -1;
float statusPHTerakhir  = -1.0f;

unsigned long dosingFaseTimer = 0;

String dosingPumpLabel        = "";
int    dosingPumpRelayA       = -1;
int    dosingPumpIdxA = -1, dosingPumpIdxB = -1;
float  abVolA = 0.0f, abVolB = 0.0f;   // TES_124: volume aktual AB sisi A & B siklus ini (penyeimbang A:B)   // indeks pompa yg sedang dipakai fase pompa (untuk rekap volume)
float  piGainPHTerakhir = 0.0f;                    // gain efektif pH (pH/dtk) iterasi PI terakhir (info; prediksi respon kini prediksiTurunPH)
int    dosingPumpRelayB       = -1;
int    dosingPump1DetikTarget = 0;
int    dosingPump2DetikTarget = 0;
int    dosingPumpDetikTarget  = 0;
int    dosingPumpDetikBerjalan= 0;
bool   dosingPump1Selesai     = false;
bool   dosingPump2Selesai     = false;

int dosingHomogenDetikBerjalan = 0;
int dosingJedaIsiDetikBerjalan = 0;
const int JEDA_ISI_DETIK = 5;
bool isiAdaAirBaru = false;   // TES_133: solenoid sempat terbuka di siklus ini (air baru masuk) -> homogenisasi dulu
// TES_133: air baru dari pengisian belum tercampur -> cek PPM pertama (dasar dosis makro) bisa salah (mis. sampel dekat
// saluran masuk terbaca rendah -> makro berlebih, tak bisa dikoreksi). Bila ada air masuk: tunggu Jeda Homogenisasi J0.
long jedaIsiTargetDetik(){
  long j = isiAdaAirBaru ? (long)jedaJ0Menit()*60L : 0L;   // TES_134: J0 (dulu J1)
  return (j > (long)JEDA_ISI_DETIK) ? j : (long)JEDA_ISI_DETIK;
}
const int JEDA_STABIL_SENSOR_DETIK = 5;   // stabilisasi 5 dtk tiap kali relay sensor TDS ON/OFF
bool tdsRelayAktif = false;               // status daya modul TDS (relay ke-9)
byte cekAkhirTahap = 0;                   // Cek Akhir: 0=ukur PPM (TDS ON), 1=ukur pH (TDS OFF)
byte tesSensorJenis = 0;                  // Tes Sensor: 0=TDS, 1=pH

// ====================
// FORWARD DECLARATIONS
// ====================
void tampilkanMenuUtama();
void tampilkanSubMenuA();
void tampilkanMenuJenisLarutan();
void tampilkanMenuPengenceran();
void tampilkanMenuJenisAsam();
void tampilkanMenuKonsentrasi();
void tampilkanInputVolume();
void tampilkanInputPPM();
void tampilkanInputPH();
void tampilkanInputWaktuDosing();
void tampilkanMenuPilihDebitPompa();
void tampilkanInputNilaiDebit();
void tampilkanHasilKalkulasiPPM();
void tampilkanHasilTargetPH();
void tampilkanMenuPPMPHTanaman();
void tampilkanMenuTanamanList();
void tampilkanSubMenuB();
void tampilkanInputTDSBuffer(int bufferIndex);
void tampilkanKalibrasiTDSLayar(int bufferIndex);
void tampilkanInputpHBuffer(int bufferIndex);
void tampilkanKalibrasiPHLayar(int bufferIndex);
void tampilkanInputRTC(int idx);
void tampilkanRTCHasil();
void tampilkanWaterLevel();
void tampilkanKalkSuhuInput();
void tampilkanMenuPilihPompa();
void tampilkanInputDelayPompa();
void tampilkanTesPompaAktif();
void tampilkanCancelTesPompa();
void tampilkanTesSolenoid();
void tampilkanMenuInputDebitPompa();
void tampilkanInputDebitMl();
void tampilkanInputDebitDetik();
void tampilkanInputDebitHasil();
void tampilkanMenuRingkasan();
void tampilkanMenuRingkasanBaru();
String ringkasanBarisKeBaru(int idx);
bool parameterLengkap();
bool parameterBaruLengkap();
bool semuaDebitTerisiBaru();
void salinParameterBaruKeAktif();
void resetStatusParameterBaru();
void mulaiDosingSekarang();
void tampilkanMulaiDosingKonfirmasi();
bool cekWaktuDosingTercapai();
void mulaiSiklusCekHarian();
void lanjutkanSiklusBerikutnya();
void jadwalkanCekBerikutnya();
void selesaikanSiklusCek(bool adaDosis);
void prosesFinalStatus();
void cekPemicuDosingLatarBelakang();
void tampilkanSubMenuD();
void tampilkanKalkWLBentuk();
void tampilkanKalkWLTabungVol();
void tampilkanKalkWLTabungJari();
void tampilkanKalkWLTabungHasil();
void tampilkanKalkWLBalokVol();
void tampilkanKalkWLBalokPanjang();
void tampilkanKalkWLBalokLebar();
void tampilkanKalkWLBalokHasil();
void hitungKalkWLTabung();
void hitungKalkWLBalok();
void tampilkanKalkInputMenu();
void tampilkanKalkMenuDil();
void tampilkanKalkInputVolume();
void tampilkanKalkInputPPMAwal();
void tampilkanKalkInputPPMTarget();
void tampilkanKalkHasilPPM();
void tampilkanKalkPHInputVolume();
void tampilkanKalkPHMenuAsam();
void tampilkanKalkPHMenuKonsen();
void tampilkanKalkPHInputPPM();
void tampilkanKalkPHInputAwal();
void tampilkanKalkPHInputTarget();
void tampilkanKalkPHHasil();
float hitungPHDownVolumeKalk(float phAwal, float phTarget, int ppmAcuan, int acidIdx, int konsenPersen, int volume);
float hitungDurasiPompaDetikKalk(float volumeML, float debitLmnt);
float interpSlopePH(const float tabel[][2], float ppm);
int cariIndexDil(String dil);
void hitungKalkPH();
String formatPPMSensor();
String formatPHSensor();
void updateSuhuAsync();
void isiBufferPH();
int  getMedianPH();
void relayOn(int pin);
void relayOff(int pin);
void bukaSolenoid();
void tutupSolenoid();
void matikanSemuaPompa();

void tampilkanInputBatasPPM();
void tampilkanInputBatasPH();
bool isTahunKabisat(int tahun);
int  getMaksHariBulan(int bulan, int tahun);
void tambahHari(int &d, int &m, int &y, int n);

void tampilkanMenuJenisDosing();
void tampilkanInputECBebas();
void prosesInputECBebas(char key);
struct SignatureKpKi;
bool signatureSama(const SignatureKpKi &a, const SignatureKpKi &b);
const __FlashStringHelper* signatureBeda(const SignatureKpKi &a, const SignatureKpKi &b);   // TES_141
SignatureKpKi ambilSignatureAktif();
SignatureKpKi ambilSignatureKonteks(bool baru);
bool validasiKpKiZNSebelumDosing();
bool profilZNLengkap(const ProfilKpKiZN &p, float phTarget);
void simpanKanalZN(bool baru, int kanal, float kp, float ki);
void muatPIZNDariProfilAktif();
extern bool znTundaDariUji;
bool validasiKpKiZNParameterBaru();   // validasi Kp/Ki ZN thd draft Parameter Baru SEBELUM draft di-commit ke Aktif
void tampilkanMenuZNHub();
void tampilkanRekamanZN();
void simpanRekamanZN();
void tampilkanKpKiZN();
void tampilkanEditKpKiZN();
void prosesEditKpKiZN(char key);
void tampilkanUjiZNPilihKanal();
float bacaSensorTerpilihRataRata();
void  setelPompaUji(bool nyala);
bool  ujiSiapkanPulsa();
int   ujiLamaBilasDetik();
bool  ujiBentrokJadwal();
void  tampilkanBentrokJadwalUji();
bool  ujiMulaiPulsa();
void  hentikanUjiReaksi();
float ujiNilaiHalus(int i);
float ujiNilaiSekarang();
bool  ujiKurvaDatar(float t);
void  tampilkanUjiLoggingBaris1();
float hitungGainModel(int kanal, bool baru);
float hitungGainModelPPM(int kanal, bool baru, int ppmAcuan);
void  mulaiUjiStepResponse();
float cariWaktuPadaFraksi(float fraksiTarget,float deltaTotal);
void  selesaiUjiStepResponse(bool stabil);
void  tickUjiStepResponse();
void  tampilkanHasilUjiZN();
void  prosesKonfirmasiUjiZN(char key);
void tampilkanDosingInputLamaCek();
void tampilkanDosingInputStabilisasi();
void terapkanJenisDosing(int mode, int persen);
float piHitungPulsa(float error, PIController &pi, float uMaks);
void  piReset();
void  batasiKpZNDenganGainModel(PIController &pi, int kanal);
void  hitungKonstantaPIOtomatis();
void  siapkanFasePompaPiAB();
void  siapkanFasePompaPiPH();
int   jedaUntukState(MenuState s);
// Prototipe eksplisit fungsi berparameter MenuState: tanpa ini, Arduino IDE membuat prototipe otomatis di atas
// deklarasi enum MenuState -> error "'MenuState' has not been declared".
void konfirmasiSuksesInti(String msg, MenuState nextState, bool catatanBaru);
void konfirmasiSukses(String msg, MenuState nextState);
void konfirmasiParam(String msg, MenuState nextState);
void handleRTCField(char key, int idx, int minV, int maxV, MenuState nextState, int nextIdx);
bool isiNilaiAwalInput(MenuState s, MenuState asal);
bool rtcSiapUntukMulai();
void infoKanalTerkunciSaatMulai();
void mintaSimpanKanalZN(int kanal, float kp, float ki, MenuState kembali, float volRef, int ppmRef);
void  prosesTesSensor();
void  tampilkanTesSensorInputBilas();
void  tampilkanTesSensorInputStabil();
void  tampilkanTesSensorProses();
void  tampilkanTesSensorHasil();
int   bacaAdcTDS();
int   bacaAdcPH();
float adcKeVolt(long adc);
float adcKeVoltTDS(long adc);
int   tdsAdcMinValid();
int   phAdcMinValid();
int   phAdcMaxValid();
void tampilkanDosingInputJeda(int idx);
void updateDosingProses();
void prosesTungguWaktu();
void prosesIsiTandon();
void prosesJedaIsi();
void tampilkanDosingJedaIsi();
void mulaiCekPPM(MenuState tujuan);
void mulaiCekUlangRespon(MenuState homogen, MenuState tujuan);   // TES_137
void catatMakroPulih();
float batasVolSisiAB(int idxA, int idxB);
bool nilaiMakroPulihAB(int ppmSekarang);
bool nilaiMakroPulihPH(float phSekarang);
void prosesCekPPM();
void mulaiCekPH(MenuState tujuan);
void prosesCekPH();
void siapkanFasePompaAB();
void siapkanFasePompaPH();
void prosesFasePompa();
void prosesHomogenisasi();
void mulaiCekAkhir();
void prosesCekAkhir();
void prosesLewatiTahapAB();   
void prosesLewatiTahapPH();   
float hitungDurasiPompaDetik(float volumeML, int pumpIdx);
float hitungPHDownVolume(float phAwal, float phTarget, int ppmAcuan);
float prediksiTurunPH(float ph, float volML, int ppmAcuan);
float prediksiTurunPHK(bool baru, float ph, float volML, int ppmAcuan);
bool  phTanpaRespon(float phAcuan, float phSekarang, float volKeluar, int ppmAcuan);
void tampilkanDosingTungguWaktu(bool tercapai);
void tampilkanStatusMenunggu();   // TES_147
void tampilkanDosingIsiTandon();
void tampilkanDosingTandonTimeout();
void tampilkanDosingSensorInvalid();
void hentikanDosingSensorInvalid();
void pantauLCD(unsigned long now);
void gambarUlangLayarAktif();
bool sensorTdsPhAktif();
bool sensorTdsPerluAktif();
bool sensorPhPerluAktif();
void primingBufferSensor();
void primingBufferSensorSelektif(bool bacaTds, bool bacaPh);
void bacaSensorSelektif(bool bacaTds, bool bacaPh);
void nyalakanRelayTDS();
void matikanRelayTDS();
void tampilkanDosingCekPPM();
void tampilkanDosingCekPH();
void tampilkanDosingPompaAktif();
void tampilkanDosingHomogenisasi();
void tampilkanDosingCekAkhir();
void tampilkanCancelDosing();
void tampilkanStatusDosing();
bool statusModeMenunggu();   // TES_146
void tampilkanRekapDosing();
void prosesRekapDosing();
void komitParameterBaruLaluMulai();
void eepromSimpanRekamCek();
float kpMaksModel(int kanal, bool baru);
const __FlashStringHelper* teksPIFlag(uint8_t pf);
const __FlashStringHelper* teksCatatan(uint8_t pf);
void tampilkanPeringatanResolusi();
void mulaiPeringatanResolusi(bool baru);
float resolusiPH1dtk(bool baru);
float resolusiPPM1dtk(bool baru);
bool resolusiPHKasar(bool baru);
bool resolusiPPMKasar(bool baru);
bool perluPeringatanResolusi(bool baru);
int ppmAcuanPHDown();
void tampilkanMenuKunciKanal();
void tampilkanKunciKanalKonfirmasi();
int piCekKe(int iterasi, float pulsa, bool selesai);
void mulaiJedaPulih();
void tampilkanJedaPulih();
ProfilKpKiZN& profilZNKonteks(bool baru);
int jumlahBarisPI(bool baru);
void ringkasanBarisLanjut(char* buf, size_t n, int idx, bool baru);
void tampilkanInputPersenMakro();
void tampilkanZNKonfirmasiJadwal();
void mulaiBuzzerPutus(unsigned long durasiMs);
void terapkanResetPersamaan();
void tampilkanUlangFaseDosing();
bool statusFaseDosingBerjalan(MenuState s);

void tampilkanErrorLengkapi(bool baru);
const __FlashStringHelper* itemBelumLengkap(bool baru);
bool batasPPMSesuaiTarget(int batas, int target);

void tampilkanMenuKredit();

// ==== EEPROM: definisi TIPE (wajib sebelum fungsi apa pun) ====
#define EEPROM_MAGIC 0x4E46u
#define EEPROM_VERSI 9   // dinaikkan: tabel EC 8x2 diganti regresiSlopeAB[2]
#define EEPROM_ALAMAT 0
#define EEPROM_ALAMAT_SUHUKALPH (EEPROM_ALAMAT + (int)sizeof(KonfigTersimpan) + 8)  // suhu kalibrasi pH disimpan terpisah (tak ubah struct/versi)
#define EEPROM_ALAMAT_MIGRASI   (EEPROM_ALAMAT_SUHUKALPH + 8)   // penanda migrasi sekali (1 byte, di luar struct)
#define EEPROM_ALAMAT_KOMIT     (EEPROM_ALAMAT_MIGRASI + 1)     // TES_148: penanda commit Parameter Baru sedang ditulis (1 byte)
#define KOMIT_BERJALAN          0xA5
#define EEPROM_ALAMAT_REKAMCEK  (EEPROM_ALAMAT_MIGRASI + 8)     // hasil cek harian terakhir (struct RekamCek, di luar struct utama)
#define EEPROM_ALAMAT_EKSTRA    (EEPROM_ALAMAT_REKAMCEK + 64)   // KonfigEkstra TES_118 (draft Baru, PI manual, profil ZN Baru)
#define MIGRASI_GAIN_PER_SISI   0x5A
struct KonfigTersimpan {
  uint16_t magic;
  uint8_t  versi;
  // -- Parameter Dosing Aktif --
  int   terpilihVolume;
  int   terpilihPPM;
  float terpilihPH;
  char  waktuDosing[6];
  char  ec[16];
  char  dilution[16];
  char  acid[16];
  char  conc[16];
  float slopeABMixTerpilih;
  int   terpilihConcPersen;
  int   terpilihDebitPump[TOTAL_PUMP_OPTIONS];
  // -- Batasan / Jenis / Jeda --
  // TES_116: slot 2 int ini dulunya Loop Dosing (hari) & Hari Ke. Tata letak struct SENGAJA tidak diubah agar
  // EEPROM_VERSI tetap -> kalibrasi sensor, persamaan & Kp/Ki tidak terhapus saat upgrade. Nilai lama (1-14)
  // otomatis tidak lolos batasPPMValid() -> Batasan dianggap belum diatur.
  int   batasPPM;
  int   batasPHx100;
  int   dosingMode;
  int   selJenisDosing;
  int   terpilihLamaCekSensor;
  int   terpilihJedaStabilisasi;
  int   dosingJedaHomogenisasi[6];
  bool  batasanSudahDiatur;
  bool  jenisDosingSudahDiatur;
  bool  jedaSudahDiatur;
  // -- Kalibrasi Sensor --
  int   calTDSBuffer[5];
  float tdsCVInput[5];
  bool  tdsKalibrasiAktif;
  float calPhVolt1, calPhVolt2, calPhVolt3;
  float phBufferInput[3];
  bool  phKalibrasi3TitikAktif;
  float phSlope, phOffset;
  // -- Tabel slope kustom dari menu "Pengaturan Persamaan" (agar tak hilang saat mati listrik) --
  float regresiSlopeAB[2];
  float tabelSlopeAtas[TOTAL_TABEL_PH][2];
  float tabelSlopeBawah[TOTAL_TABEL_PH][2];
  float tabelSlopeAtasFosfat[TOTAL_TABEL_PH][2];
  float tabelSlopeBawahFosfat[TOTAL_TABEL_PH][2];
  float suhuOffset;
  bool  suhuInputNegatif;
  // -- Konstanta kontrol PI (Closed-Loop) --
  float piKpPPM, piKiPPM, piKpPH, piKiPH;
  float piKpPHAtas, piKpPHBawah;
  // -- Hasil masukan tanggal/waktu RTC terakhir --
  int   rtcTahun, rtcBulan, rtcTgl, rtcJam, rtcMenit, rtcDetik;
  // -- Parameter Baru (draft) yang belum di-commit --
  char  ecBaru[16];
  char  dilutionBaru[16];
  char  acidBaru[16];
  char  concBaru[16];
  char  waktuDosingBaru[6];
  float slopeABMixTerpilihBaru;
  int   terpilihConcPersenBaru;
  int   selJenisBaru;
  int   terpilihVolumeBaru;
  int   terpilihPPMBaru;
  float terpilihPHBaru;
  int   terpilihDebitPumpBaru[TOTAL_PUMP_OPTIONS];
  int   batasPPMBaru;
  int   batasPHx100Baru;
  bool  batasanBaruSudahDiatur;
  int   dosingModeBaru;
  int   terpilihLamaCekSensorBaru;
  int   terpilihJedaStabilisasiBaru;
  bool  jenisDosingBaruSudahDiatur;
  int   dosingJedaHomogenisasiBaru[6];
  bool  jedaBaruSudahDiatur;
  // -- Status siklus dosing (untuk resume setelah daya mati) --
  bool  dosingSedangMenunggu;
  bool  jadwalCekAktif;
  int   dosingTargetHari;
  int   dosingTargetBulan;
  int   dosingTargetTahun;

  // -- TAMBAHAN v7: Riwayat Kp/Ki ZN Manual (poin 2, multi-profil) --
  ProfilKpKiZN profilZNAktif;
};
struct { int tahun=0, bulan=0, tgl=0, jam=0, menit=0, detik=0; } rtcCache;   // cache RTC 24 byte, gantikan eepromBayangan penuh
bool eepromSiap = false;
bool eepromBaruDibuat = false;   // true bila EEPROM kosong/beda versi saat boot (mis. setelah Reset Pabrik)
void eepromKemas(KonfigTersimpan &k);
void eepromTerapkan(const KonfigTersimpan &k);
void eepromMuat();
void eepromAutoSimpan();
void eepromSimpanRTC();

void muatUlangKonstantaAktif();
String scrollingLine(String text, int offset);
void tampilkanMenuPengaturanSistem();
void tampilkanResetSistemConfirm();
void resetPabrik();
void tampilkanMenuPengaturanPersamaan();
void tampilkanPengaturanABPilihDil();
void tampilkanPengaturanABList();
void tampilkanPengaturanPHPilihAsam();
void tampilkanPengaturanPHPilihFase();
void tampilkanPengaturanPHList();
void tampilkanPengaturanPHInput();
void tampilkanPengaturanResetConfirm();

float bacaCVTDS();
void  hitungKalibrasiTDS();
float interpolasiTDS(float cv);
float interpolasiPH(float volt);

// ==========
// HELPER LCD
// ==========
void lcdPrint20(int col, int row, const char* text) {
  char buf[21];
  int i = 0;
  for (; i < 20 && text[i]; i++) buf[i] = text[i];
  for (; i < 20; i++) buf[i] = ' ';
  buf[20] = '\0';
  lcd.setCursor(col, row); lcd.print(buf);
}
void lcdPrint20(int col, int row, const __FlashStringHelper* text) {
  PGM_P p = reinterpret_cast<PGM_P>(text);
  char buf[21];
  int i = 0;
  for (; i < 20; i++) { char c = (char)pgm_read_byte(p + i); if (!c) break; buf[i] = c; }
  for (; i < 20; i++) buf[i] = ' ';
  buf[20] = '\0';
  lcd.setCursor(col, row); lcd.print(buf);
}
void lcdPrint20(int col, int row, const String& text) { lcdPrint20(col, row, text.c_str()); }

void lcdPrintCenter(int row, const char* text) {
  int len = (int)strlen(text); if (len > 20) len = 20;
  int pad = (20 - len) / 2; if (pad < 0) pad = 0;
  char buf[21];
  int i = 0;
  for (; i < pad; i++) buf[i] = ' ';
  for (int j = 0; j < len; j++) buf[i++] = text[j];
  for (; i < 20; i++) buf[i] = ' ';
  buf[20] = '\0';
  lcd.setCursor(0, row); lcd.print(buf);
}
void lcdPrintCenter(int row, const __FlashStringHelper* text) {
  PGM_P p = reinterpret_cast<PGM_P>(text);
  int len = (int)strlen_P(p); if (len > 20) len = 20;
  int pad = (20 - len) / 2; if (pad < 0) pad = 0;
  char buf[21];
  int i = 0;
  for (; i < pad; i++) buf[i] = ' ';
  for (int j = 0; j < len; j++) buf[i++] = (char)pgm_read_byte(p + j);
  for (; i < 20; i++) buf[i] = ' ';
  buf[20] = '\0';
  lcd.setCursor(0, row); lcd.print(buf);
}
void lcdPrintCenter(int row, const String& text) { lcdPrintCenter(row, text.c_str()); }

void lcdPrintLR(int row, const char* left, const char* right) {
  int ll = (int)strlen(left), rl = (int)strlen(right);
  int spaces = 20 - (ll + rl); if (spaces < 1) spaces = 1;
  char buf[21];
  int i = 0;
  for (int j = 0; j < ll && i < 20; j++) buf[i++] = left[j];
  for (int j = 0; j < spaces && i < 20; j++) buf[i++] = ' ';
  for (int j = 0; j < rl && i < 20; j++) buf[i++] = right[j];
  for (; i < 20; i++) buf[i] = ' ';
  buf[20] = '\0';
  lcd.setCursor(0, row); lcd.print(buf);
}
void lcdPrintLR(int row, const String& left, const String& right) { lcdPrintLR(row, left.c_str(), right.c_str()); }

// ── Helper tampilan TANPA String (TES_119) — dipakai layar yang diperbarui tiap detik agar heap tidak terfragmentasi
// saat alat menyala berminggu-minggu. Format di flash (PSTR), float lewat dtostrf.
#include <stdarg.h>
void lcdCenterP(int row, PGM_P fmt, ...){
  char b[21]; va_list ap; va_start(ap, fmt); vsnprintf_P(b, sizeof(b), fmt, ap); va_end(ap);
  lcdPrintCenter(row, b);
}
char* fmtF(char *buf, float v, int dec){ dtostrf(v, 1, dec, buf); return buf; }   // buf min. 10 char
char* fmtVolRingkas(char* b, size_t n, const char* label, float ml);
void lcdBar(int row, long jalan, long total){
  int pj = (total>0) ? (int)((jalan*20L)/total) : 0;
  if(pj>20) pj=20; if(pj<0) pj=0;
  char bar[21];
  for(int b=0;b<20;b++) bar[b] = (b<pj) ? (char)255 : ' ';
  bar[20]='\0';
  lcd.setCursor(0,row); lcd.print(bar);
}

// Efek scroll pada satu baris teks yang melebihi 20 kolom LCD
String scrollingLine(String text, int offset) {
  if((int)text.length() <= 20) return text;
  String extended = text + "   " + text;
  int wrapLen = text.length() + 3;
  int off = offset % wrapLen;
  return extended.substring(off, off+20);
}

// ===================
// KONFIRMASI & ERROR
// ===================
// Jeda tampilan (pengganti delay): buzzer tetap dilayani & pompa uji yang waktunya habis tetap dimatikan tepat waktu
void layaniLatarBelakang(){
  wdt_reset();          // jeda tampilan tidak boleh memicu watchdog
  updateBuzzer();
  if(tesPompaBerjalan && (millis()-tesPompaMulaiMs)/1000UL >= (unsigned long)tesPompaDetikTarget)
    digitalWrite(pumpRelayPin[tesPompaIdx], RELAY_AKTIF_LOW ? HIGH : LOW);   // UI tes pompa diselesaikan oleh loop()
}
void jedaAman(unsigned long ms){
  unsigned long t0 = millis();
  while(millis() - t0 < ms) layaniLatarBelakang();
}

// Parameter dosing sedang disimpan ke draft Parameter BARU (jadwal cek aktif menunggu)?
bool simpanKeParamBaru(){ return dosingSedangMenunggu; }
// Nilai yang DITAMPILKAN di layar input: draft Baru bila sudah ada; bila belum, nilai Aktif (yang akan disalin ke draft)
bool tampilBaru(){ return dosingSedangMenunggu && draftBaruAda; }

void konfirmasiSuksesInti(String msg, MenuState nextState, bool catatanBaru) {
  lcd.clear(); lcd.setCursor(0,1); lcd.print(msg);
  if(catatanBaru){ lcd.setCursor(0,2); lcd.print(F("(Parameter Baru)")); }
  jedaAman(catatanBaru ? 2000 : 1500);
  currentState=nextState; lcd.clear();
  switch(nextState) {
    case MENU_UTAMA:           tampilkanMenuUtama(); break;
    case SUB_MENU_A:           tampilkanSubMenuA(); break;
    case SUB_MENU_B:           tampilkanSubMenuB(); break;
    case SUB_MENU_C:           tampilkanMenuPilihPompa(); break;   // TES_128: kursor tetap di pompa yang baru diuji
    case MENU_DEBIT_POMPA_SEL: tampilkanMenuPilihDebitPompa(); break;
    case SUB_MENU_D:           tampilkanSubMenuD(); break;
    case MENU_JENIS_LARUTAN:   tampilkanMenuJenisLarutan(); break;
    case MENU_PENGATURAN_SISTEM: tampilkanMenuPengaturanSistem(); break;
    case MENU_ZN_HUB:          tampilkanMenuZNHub(); break;
    case MENU_TANAMAN_LIST:    tampilkanMenuTanamanList(); break;
    case MENU_LIHAT_KPKI_ZN:   tampilkanKpKiZN(); break;
    default: tampilkanMenuUtama(); break;
  }
}
void konfirmasiSukses(String msg, MenuState nextState){ konfirmasiSuksesInti(msg, nextState, false); }
// Untuk input Parameter Dosing: saat jadwal menunggu, beri keterangan bahwa nilai masuk ke Parameter Baru
void konfirmasiParam(String msg, MenuState nextState){ konfirmasiSuksesInti(msg, nextState, simpanKeParamBaru()); }

void jedaInfo(unsigned int ms){
  unsigned long t0 = millis();
  while(millis() - t0 < ms){
    layaniLatarBelakang();
    if(customKeypad.getKey() != NO_KEY) break;
  }
}

// Seluruh tampilan error input salah 
void tampilkanError(String msg) {
  lcd.clear();
  lcdPrintCenter(1, F("INPUT SALAH"));
  if(msg.length()>0) lcdPrintCenter(2, msg);   
  jedaAman(1500);
  lcd.clear();
}

// Tampilan khusus saat parameter dosing belum lengkap: sebutkan item PERTAMA yang belum terisi (urutan menu)
void tampilkanErrorLengkapi(bool baru) {
  lcd.clear();
  lcdPrintCenter(0, baru ? F("PARAMETER BARU") : F("LENGKAPI PARAMETER"));
  lcdPrintCenter(1, baru ? F("BELUM LENGKAP:") : F("DOSING:"));
  const __FlashStringHelper* item = itemBelumLengkap(baru);
  if(item) lcdPrintCenter(2, item); else lcdPrint20(0,2,F(""));
  lcdPrintCenter(3, F("(Parameter Dosing)"));
  jedaInfo(3000);
  lcd.clear();
}

// ===============================================
// HELPER ADC — ADS1115 (16-bit) / ADC internal (10-bit)
// ===============================================
// Semua pembacaan analog pH & TDS lewat ADS1115 bila terdeteksi (referensi internal
// ADS stabil, tidak ikut goyang dgn rail 5V). Bila ADS1115 tidak terpasang, otomatis
// jatuh kembali ke ADC internal board.
int bacaAdcTDS(){ return analogRead(PIN_TDS); }   // TDS SELALU native Arduino ADC (bukan ADS1115) — lihat catatan crosstalk channel ADS
int bacaAdcPH(){  if(adsSiap){ int r=(int)ads.readADC_SingleEnded(ADS_CH_PH);  return r<0?0:r; } return analogRead(PIN_PH); }
// Cadangan kanal suhu analog (A2). DS18B20 adalah sensor digital OneWire sehingga
// TIDAK perlu lewat ADS1115; kanal ini hanya dipakai bila diganti sensor suhu analog.
// Konversi hitungan ADC -> Volt (ADS1115 GAIN_TWOTHIRDS: 6,144 V / 32767)
float adcKeVolt(long adc){ return adsSiap ? (adc*6.144f/32767.0f) : (adc*TDS_VREF/1024.0f); }  // KHUSUS pH
float adcKeVoltTDS(long adc){ return adc*TDS_VREF/1024.0f; }   // KHUSUS TDS — selalu native 10-bit (TDS tidak lagi lewat ADS1115)
// Batas validitas diskalakan mengikuti resolusi ADC yg aktif
int tdsAdcMinValid(){ return TDS_ADC_MIN_VALID; }   // TDS selalu native -> tidak perlu skala ADS lagi
// TES_148: skala tegangan sebenarnya (ADS 6.144 V/32767 vs ADC 5 V/1024 = 26.04x); dulu 32x -> batas atas 6.09 V tak pernah tercapai
#define ADS_SKALA_V (32767.0f/6.144f*5.0f/1024.0f)
int phAdcMinValid(){  return adsSiap ? (int)(PH_ADC_MIN_VALID*ADS_SKALA_V) : PH_ADC_MIN_VALID; }
int phAdcMaxValid(){  return adsSiap ? (int)(PH_ADC_MAX_VALID*ADS_SKALA_V) : PH_ADC_MAX_VALID; }

// ===================
// FUNGSI SENSOR – TDS
// ===================
void isiBufferTDS() {
  tdsBuffer[tdsBufferIdx]=bacaAdcTDS();
  tdsBufferIdx=(tdsBufferIdx+1)%TDS_SCOUNT;
}
int getMedianTDS() {
  for(int i=0;i<TDS_SCOUNT;i++) tdsBufferTemp[i]=tdsBuffer[i];
  for(int i=0;i<TDS_SCOUNT-1;i++)
    for(int j=i+1;j<TDS_SCOUNT;j++)
      if(tdsBufferTemp[j]<tdsBufferTemp[i]){int t=tdsBufferTemp[i];tdsBufferTemp[i]=tdsBufferTemp[j];tdsBufferTemp[j]=t;}
  return tdsBufferTemp[TDS_SCOUNT/2];
}
int hitungPPM(float suhu) {
  isiBufferTDS();
  int medianAdc = getMedianTDS();
  tdsSensorValid = (medianAdc >= tdsAdcMinValid());
  float v=adcKeVoltTDS(medianAdc), cv=v/(1.0f+0.02f*(suhu-25.0f));
  if(tdsKalibrasiAktif){
    return (int)round(interpolasiTDS(cv));
  }
  return (int)((133.42f*cv*cv*cv-255.86f*cv*cv+857.39f*cv)*0.5f);
}

float bacaCVTDS() {
  long s=0; int raw[10];
  for(int i=0;i<10;i++){raw[i]=bacaAdcTDS();s+=raw[i];delay(10);}
  // Validitas pakai median 10 sampel (lebih tahan glitch dp 1 sampel terakhir)
  for(int i=0;i<9;i++) for(int j=i+1;j<10;j++) if(raw[j]<raw[i]){int t=raw[i];raw[i]=raw[j];raw[j]=t;}
  int medianAdc=(raw[4]+raw[5])/2;
  tdsSensorValid = (medianAdc >= tdsAdcMinValid());
  float v=((float)s/10.0f)*TDS_VREF/1024.0f;   // TES_138: rata-rata float (dulu s/10 integer -> resolusi sub-hitungan ADC hilang)
  return v/(1.0f+0.02f*(suhularutan-25.0f));
}

// Interpolasi linier bertingkat atas n titik terurut x; DI LUAR rentang diekstrapolasi dgn segmen ujung
// (dulu dijepit ke titik ujung -> bacaan di atas buffer tertinggi 'macet' di nilai buffer itu).
float interpolasiTitik(const float *x, const float *y, int n, float v){
  if(n<=0) return 0.0f;
  if(n==1) return y[0];
  int i;
  if(v <= x[0]) i = 0;
  else if(v >= x[n-1]) i = n-2;
  else { i = 0; while(i < n-2 && v > x[i+1]) i++; }
  float dx = x[i+1]-x[i];
  if(fabs(dx) < 0.0001f) return y[i];
  return y[i] + (v-x[i])*(y[i+1]-y[i])/dx;
}

// Bangun tabel terurut dari titik aktif (slot buffer >= 0). Aktif bila >= 2 titik.
void hitungKalibrasiTDS() {
  int n=0;
  for(int i=0;i<TDS_TITIK_KAL;i++) if(calTDSBuffer[i]>=0){ tdsCVUrut[n]=tdsCVInput[i]; tdsPPMUrut[n]=(float)calTDSBuffer[i]; n++; }
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(tdsCVUrut[j]<tdsCVUrut[i]){
    float t=tdsCVUrut[i]; tdsCVUrut[i]=tdsCVUrut[j]; tdsCVUrut[j]=t;
    t=tdsPPMUrut[i]; tdsPPMUrut[i]=tdsPPMUrut[j]; tdsPPMUrut[j]=t;
  }
  tdsJumlahTitik = n;
  tdsKalibrasiAktif = (n>=2);
}

float interpolasiTDS(float cv) {
  float p = interpolasiTitik(tdsCVUrut, tdsPPMUrut, tdsJumlahTitik, cv);
  return (p < 0.0f) ? 0.0f : p;
}

// ===================
// FUNGSI SENSOR – pH
// ===================
float bacaVoltPH() {
  long s=0; int raw[10];
  for(int i=0;i<10;i++){raw[i]=bacaAdcPH();s+=raw[i];delay(10);}
  // Validitas pakai median 10 sampel (lebih tahan glitch dp 1 sampel terakhir)
  for(int i=0;i<9;i++) for(int j=i+1;j<10;j++) if(raw[j]<raw[i]){int t=raw[i];raw[i]=raw[j];raw[j]=t;}
  int medianAdc=(raw[4]+raw[5])/2;
  phSensorValid = (medianAdc >= phAdcMinValid()) && (medianAdc <= phAdcMaxValid());
  float rata = (float)s/10.0f;   // TES_138: rata-rata float (dulu s/10 integer)
  return adsSiap ? (rata*6.144f/32767.0f) : (rata*TDS_VREF/1024.0f);
}

void isiBufferPH() {
  phBuffer[phBufferIdx]=bacaAdcPH();
  phBufferIdx=(phBufferIdx+1)%PH_SCOUNT;
}
int getMedianPH() {
  for(int i=0;i<PH_SCOUNT;i++) phBufferTemp[i]=phBuffer[i];
  for(int i=0;i<PH_SCOUNT-1;i++)
    for(int j=i+1;j<PH_SCOUNT;j++)
      if(phBufferTemp[j]<phBufferTemp[i]){int t=phBufferTemp[i];phBufferTemp[i]=phBufferTemp[j];phBufferTemp[j]=t;}
  return phBufferTemp[PH_SCOUNT/2];
}

// -- Kalibrasi pH: sampling & median jendela bergulir 15 dtk (mirip median saat dosing) --
void mulaiKalPhBuffer(){
  if(tdsRelayAktif){                        // TDS harus mati saat kalibrasi pH
    matikanRelayTDS();
    lcd.clear();
    lcdPrintCenter(1, F("STABIL SENSOR pH"));
    lcdPrintCenter(2, String(F("Tunggu "))+String(JEDA_STABIL_SENSOR_DETIK)+F(" detik.."));
    jedaAman((unsigned long)JEDA_STABIL_SENSOR_DETIK*1000UL);
    lcd.clear();
  }
  primingBufferSensorSelektif(false, true);
  kalPhRollIdx = 0; kalPhRollCount = 0;
  suhuKalPhSum = 0.0f; suhuKalPhCount = 0;
  tKalPhBufMulai = millis();
  kalPhTungguTampil = false;
}
void sampelKalPh(){
  kalPhRoll[kalPhRollIdx] = bacaAdcPH();
  kalPhRollIdx = (kalPhRollIdx + 1) % KAL_PH_ROLL_N;
  if(kalPhRollCount < KAL_PH_ROLL_N) kalPhRollCount++;
  suhuKalPhSum += suhularutan; suhuKalPhCount++;
  if(suhuKalPhCount >= 1200){ suhuKalPhSum *= 0.5f; suhuKalPhCount /= 2; }   // TES_148: int 16-bit tak meluap (layar dibiarkan berjam-jam)
}
int getMedianKalPh(){
  int n = kalPhRollCount;
  if(n <= 0) return bacaAdcPH();
  static int tmp[KAL_PH_ROLL_N];
  for(int i=0;i<n;i++) tmp[i]=kalPhRoll[i];
  for(int i=0;i<n-1;i++)
    for(int j=i+1;j<n;j++)
      if(tmp[j]<tmp[i]){int t=tmp[i];tmp[i]=tmp[j];tmp[j]=t;}
  return tmp[n/2];
}
// -- Kalibrasi TDS: sampling & median jendela bergulir 15 dtk (pola sama seperti pH) --
void mulaiKalTdsBuffer(){
  nyalakanRelayTDS();                       // daya modul TDS ON utk kalibrasi
  lcd.clear();
  lcdPrintCenter(1, F("STABILISASI DAYA"));
  lcdPrintCenter(2, String(F("Tunggu "))+String(JEDA_STABIL_SENSOR_DETIK)+F(" detik.."));
  jedaAman((unsigned long)JEDA_STABIL_SENSOR_DETIK*1000UL);
  lcd.clear();
  primingBufferSensorSelektif(true, false);
  kalTdsRollIdx = 0; kalTdsRollCount = 0;
  tKalTdsBufMulai = millis();
  kalTdsTungguTampil = false;
}
void sampelKalTds(){
  kalTdsRoll[kalTdsRollIdx] = bacaAdcTDS();
  kalTdsRollIdx = (kalTdsRollIdx + 1) % KAL_TDS_ROLL_N;
  if(kalTdsRollCount < KAL_TDS_ROLL_N) kalTdsRollCount++;
}
int getMedianKalTds(){
  int n = kalTdsRollCount;
  if(n <= 0) return bacaAdcTDS();
  static int tmp[KAL_TDS_ROLL_N];
  for(int i=0;i<n;i++) tmp[i]=kalTdsRoll[i];
  for(int i=0;i<n-1;i++)
    for(int j=i+1;j<n;j++)
      if(tmp[j]<tmp[i]){int t=tmp[i];tmp[i]=tmp[j];tmp[j]=t;}
  return tmp[n/2];
}
void tampilkanPesanKalTunggu(){
  lcd.clear();
  lcdPrintCenter(1, F("TUNGGU STABIL"));
  lcdPrintCenter(2, F("baca stabil dulu"));
  jedaInfo(1000);
}
// -- Kalkulator sensor & dosing: median 15 detik terakhir ppmTerbaca/phTerbaca (sampel @1 dtk) --
void resetPpmRoll15(){ ppmRoll15Idx=0; ppmRoll15Count=0; }
void sampelPpmRoll15(){
  ppmRoll15[ppmRoll15Idx]=ppmTerbaca;
  ppmRoll15Idx=(ppmRoll15Idx+1)%ROLL_MED_N;
  if(ppmRoll15Count<ROLL_MED_N) ppmRoll15Count++;
}
int medianPpmRoll15(){
  int n=ppmRoll15Count;
  if(n<=0) return ppmTerbaca;
  static int tmp[ROLL_MED_N];
  for(int i=0;i<n;i++) tmp[i]=ppmRoll15[i];
  for(int i=0;i<n-1;i++)
    for(int j=i+1;j<n;j++)
      if(tmp[j]<tmp[i]){int t=tmp[i];tmp[i]=tmp[j];tmp[j]=t;}
  return tmp[n/2];
}
void resetPhRoll15(){ phRoll15Idx=0; phRoll15Count=0; }
void sampelPhRoll15(){
  phRoll15[phRoll15Idx]=phTerbaca;
  phRoll15Idx=(phRoll15Idx+1)%ROLL_MED_N;
  if(phRoll15Count<ROLL_MED_N) phRoll15Count++;
}
float medianPhRoll15(){
  int n=phRoll15Count;
  if(n<=0) return phTerbaca;
  static float tmp[ROLL_MED_N];
  for(int i=0;i<n;i++) tmp[i]=phRoll15[i];
  for(int i=0;i<n-1;i++)
    for(int j=i+1;j<n;j++)
      if(tmp[j]<tmp[i]){float t=tmp[i];tmp[i]=tmp[j];tmp[j]=t;}
  return tmp[n/2];
}
// ── Kompensasi suhu pH (kemiringan Nernst / ATC) ──
const bool  PH_ATC_AKTIF        = true;   
const float PH_SUHU_KALIBRASI_C = 25.0f;  
float terapkanKompensasiSuhuPH(float phKal) {
  if(!PH_ATC_AKTIF) return phKal;
  float tK = suhularutan + 273.15f;
  if(tK < 250.0f) tK = PH_SUHU_KALIBRASI_C + 273.15f;   
  float tKalRef = suhuKalibrasiPH + 273.15f;
  if(tKalRef < 250.0f || tKalRef > 350.0f) tKalRef = PH_SUHU_KALIBRASI_C + 273.15f;
  float faktor = tKalRef / tK;                           
  return 7.0f + (phKal - 7.0f) * faktor;
}

float hitungPH() {
  isiBufferPH();
  int medianAdc = getMedianPH();
  phSensorValid = (medianAdc >= phAdcMinValid()) && (medianAdc <= phAdcMaxValid());
  float v = adcKeVolt(medianAdc);
  float hasil = phKalibrasi3TitikAktif ? interpolasiPH(v) : (phSlope*v+phOffset);
  hasil = terapkanKompensasiSuhuPH(hasil);   // ATC kemiringan Nernst berdasar suhu larutan
  if(!(hasil >= PH_VALID_MIN && hasil <= PH_VALID_MAKS)) phSensorValid = false;   // TES_148: dulu 14.00 (terpotong) dianggap valid -> pH Down maks
  return constrain(hasil,0.0f,14.0f);
}

void hitungKalibrasiPH() {
  float vv[PH_TITIK_KAL] = {calPhVolt1, calPhVolt2, calPhVolt3};
  int n=0;
  for(int i=0;i<PH_TITIK_KAL;i++) if(phBufferInput[i]>=0.0f){ phVoltUrut[n]=vv[i]; phBufferUrut[n]=phBufferInput[i]; n++; }
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(phVoltUrut[j]<phVoltUrut[i]){
    float t=phVoltUrut[i]; phVoltUrut[i]=phVoltUrut[j]; phVoltUrut[j]=t;
    t=phBufferUrut[i]; phBufferUrut[i]=phBufferUrut[j]; phBufferUrut[j]=t;
  }
  phJumlahTitik = n;
  phKalibrasi3TitikAktif = (n>=2);
}

float interpolasiPH(float volt) {
  return interpolasiTitik(phVoltUrut, phBufferUrut, phJumlahTitik, volt);
}

// ── Draft kalibrasi: simpan / muat / tambah sampel / evaluasi aktivasi ──
#define EEPROM_ALAMAT_DRAFTKAL (EEPROM_ALAMAT_EKSTRA + 64)
// TES_141: acuan uji Kp/Ki ZN (PPM uji kanal pH per profil + volume uji terakhir) - blok kecil terpisah sesudah draft
// kalibrasi (struct lama tidak diubah -> alamat blok lain tidak bergeser, tak perlu migrasi tata letak)
struct KonfigZNUji141 {      // format TES_141-143 (dibaca utk migrasi)
  uint16_t magic;
  int16_t  ppmUji[2][2];
  uint16_t volUjiTerakhir;
};
struct KonfigZNUji {
  uint16_t magic;
  int16_t  ppmUji[2][2];     // = znPPMUji
  uint16_t volUjiTerakhir;   // = ujiVolumeL
  uint8_t  kanalLama[2];     // TES_144: = znKanalLama
};
#define KONFIG_ZNUJI_MAGIC_141 0x2141u
#define KONFIG_ZNUJI_MAGIC 0x2144u
#define EEPROM_ALAMAT_ZNUJI (EEPROM_ALAMAT_DRAFTKAL + 128)
static_assert(sizeof(KalibDraft) <= 128, "KalibDraft melewati blok ZN uji");
#ifdef __AVR__   // TES_144: ukuran tata letak AVR (tanpa padding)
static_assert(sizeof(KonfigEkstra) <= 64, "KonfigEkstra melewati blok draft kalibrasi");   // TES_144
static_assert(sizeof(RekamCek) <= 64, "RekamCek melewati blok ekstra");
#endif
void kalDraftKosongkan(byte sensor){
  if(sensor==0){ for(int i=0;i<TDS_TITIK_KAL;i++){ kalDraft.tdsPPM[i]=-1; kalDraft.tdsSumCV[i]=0.0f; kalDraft.tdsN[i]=0; }
                 kalDraft.tdsUndoIdx=-1; }
  else { for(int i=0;i<PH_TITIK_KAL;i++){ kalDraft.phX100[i]=-1; kalDraft.phSumV[i]=0.0f; kalDraft.phN[i]=0; kalDraft.phSumSuhu[i]=0.0f; }
         kalDraft.phUndoIdx=-1; }
}
void kalDraftSimpan(){ kalDraft.magic = KALIB_DRAFT_MAGIC; EEPROM.put(EEPROM_ALAMAT_DRAFTKAL, kalDraft); }
// Dipanggil setelah kalibrasi aktif dimuat. Draft belum ada -> diisi dari kalibrasi aktif (n=1 per titik) agar
// "Tambah Sampel" langsung menyempurnakan kalibrasi yang sudah ada.
void kalDraftMuat(){
  EEPROM.get(EEPROM_ALAMAT_DRAFTKAL, kalDraft);
  if(kalDraft.magic == KALIB_DRAFT_MAGIC) return;
  kalDraftKosongkan(0); kalDraftKosongkan(1);
  if(tdsKalibrasiAktif){
    int g=0;
    for(int i=0;i<TDS_TITIK_KAL && g<TDS_TITIK_KAL;i++) if(calTDSBuffer[i]>=0){
      kalDraft.tdsPPM[g]=calTDSBuffer[i]; kalDraft.tdsSumCV[g]=tdsCVInput[i]; kalDraft.tdsN[g]=1; g++; }
  }
  if(phKalibrasi3TitikAktif){
    float vv[PH_TITIK_KAL] = {calPhVolt1, calPhVolt2, calPhVolt3};
    int g=0;
    for(int i=0;i<PH_TITIK_KAL;i++) if(phBufferInput[i]>=0.0f){
      kalDraft.phX100[g]=(int16_t)(phBufferInput[i]*100.0f+0.5f); kalDraft.phSumV[g]=vv[i]; kalDraft.phN[g]=1;
      kalDraft.phSumSuhu[g]=suhuKalibrasiPH; g++; }
  }
  kalDraftSimpan();
}
int kalJumlahSampel(byte sensor){
  int t=0;
  if(sensor==0){ for(int i=0;i<TDS_TITIK_KAL;i++) t+=kalDraft.tdsN[i]; }
  else         { for(int i=0;i<PH_TITIK_KAL;i++)  t+=kalDraft.phN[i]; }
  return t;
}
int kalJumlahJenis(byte sensor){
  int t=0;
  if(sensor==0){ for(int i=0;i<TDS_TITIK_KAL;i++) if(kalDraft.tdsN[i]>0) t++; }
  else         { for(int i=0;i<PH_TITIK_KAL;i++)  if(kalDraft.phN[i]>0)  t++; }
  return t;
}
// Tambah 1 sampel ke kelompok buffer yang sama (atau kelompok baru). false bila jenis buffer sudah penuh.
bool kalTambahSampelTDS(int ppm, float cv){
  int idx=-1, kosong=-1;
  for(int i=0;i<TDS_TITIK_KAL;i++){
    if(kalDraft.tdsN[i]>0 && kalDraft.tdsPPM[i]==ppm){ idx=i; break; }
    if(kalDraft.tdsN[i]==0 && kosong<0) kosong=i;
  }
  if(idx<0){ if(kosong<0) return false; idx=kosong; kalDraft.tdsPPM[idx]=ppm; kalDraft.tdsSumCV[idx]=0.0f; kalDraft.tdsN[idx]=0; }
  if(kalDraft.tdsN[idx] < 250){ kalDraft.tdsSumCV[idx]+=cv; kalDraft.tdsN[idx]++;
                                 kalDraft.tdsUndoIdx=(int8_t)idx; kalDraft.tdsUndoCV=cv; }
  kalDraftSimpan();
  return true;
}
bool kalTambahSampelPH(int phX100, float volt, float suhu){
  int idx=-1, kosong=-1;
  for(int i=0;i<PH_TITIK_KAL;i++){
    if(kalDraft.phN[i]>0 && kalDraft.phX100[i]==phX100){ idx=i; break; }
    if(kalDraft.phN[i]==0 && kosong<0) kosong=i;
  }
  if(idx<0){ if(kosong<0) return false; idx=kosong; kalDraft.phX100[idx]=phX100; kalDraft.phSumV[idx]=0.0f; kalDraft.phN[idx]=0; }
  if(kalDraft.phN[idx] < 250){ kalDraft.phSumV[idx]+=volt; kalDraft.phN[idx]++; kalDraft.phSumSuhu[idx]+=suhu;
                                kalDraft.phUndoIdx=(int8_t)idx; kalDraft.phUndoV=volt; kalDraft.phUndoSuhu=suhu; }
  kalDraftSimpan();
  return true;
}
// Hapus SATU sampel terakhir (salah ketik nilai / probe belum bersih). Hanya 1 langkah ke belakang.
bool kalHapusTerakhir(byte sensor){
  if(sensor==0){
    int i = kalDraft.tdsUndoIdx;
    if(i<0 || i>=TDS_TITIK_KAL || kalDraft.tdsN[i]==0) return false;
    kalDraft.tdsSumCV[i] -= kalDraft.tdsUndoCV; kalDraft.tdsN[i]--;
    if(kalDraft.tdsN[i]==0){ kalDraft.tdsPPM[i]=-1; kalDraft.tdsSumCV[i]=0.0f; }
    kalDraft.tdsUndoIdx = -1;
  } else {
    int i = kalDraft.phUndoIdx;
    if(i<0 || i>=PH_TITIK_KAL || kalDraft.phN[i]==0) return false;
    kalDraft.phSumV[i] -= kalDraft.phUndoV; kalDraft.phSumSuhu[i] -= kalDraft.phUndoSuhu; kalDraft.phN[i]--;
    if(kalDraft.phN[i]==0){ kalDraft.phX100[i]=-1; kalDraft.phSumV[i]=0.0f; kalDraft.phSumSuhu[i]=0.0f; }
    kalDraft.phUndoIdx = -1;
  }
  kalDraftSimpan();
  return true;
}
// Hapus SELURUH sampel satu jenis buffer (mis. nilai buffer salah diketik sejak awal)
void kalHapusKelompok(byte sensor, int i){
  if(sensor==0){
    kalDraft.tdsPPM[i]=-1; kalDraft.tdsSumCV[i]=0.0f; kalDraft.tdsN[i]=0;
    if(kalDraft.tdsUndoIdx==i) kalDraft.tdsUndoIdx=-1;
  } else {
    kalDraft.phX100[i]=-1; kalDraft.phSumV[i]=0.0f; kalDraft.phN[i]=0; kalDraft.phSumSuhu[i]=0.0f;
    if(kalDraft.phUndoIdx==i) kalDraft.phUndoIdx=-1;
  }
  kalDraftSimpan();
}
// TES_129: Hapus Data Kalibrasi — kalibrasi AKTIF + seluruh draft sensor ini dihapus -> rumus bawaan.
// (Mulai Ulang Draft hanya mengosongkan draft; kalibrasi aktif tetap dipakai.)
void kalHapusKalibrasi(byte sensor){
  if(sensor==0){
    for(int i=0;i<TDS_TITIK_KAL;i++){ calTDSBuffer[i]=-1; tdsCVInput[i]=0.0f; }
    hitungKalibrasiTDS();                       // 0 titik -> tdsKalibrasiAktif = false
  } else {
    for(int i=0;i<PH_TITIK_KAL;i++) phBufferInput[i]=-1.0f;
    calPhVolt1=0.0f; calPhVolt2=0.0f; calPhVolt3=0.0f;
    hitungKalibrasiPH();                        // 0 titik -> phKalibrasi3TitikAktif = false
    phSlope = PH_SLOPE_BAWAAN; phOffset = PH_OFFSET_BAWAAN;   // rumus bawaan pH
    suhuKalibrasiPH = PH_SUHU_KALIBRASI_C;      // acuan ATC kembali 25 C
    EEPROM.put(EEPROM_ALAMAT_SUHUKALPH, suhuKalibrasiPH);
  }
  kalDraftKosongkan(sensor);
  kalDraftSimpan();
  eepromAutoSimpan();
}
// Hasil evaluasi aktivasi
#define KAL_BELUM_CUKUP 0
#define KAL_AKTIF       1
#define KAL_BUTUH_JENIS 2
#define KAL_TIDAK_WAJAR 3
// Kelompok diurutkan menurut tegangan; nilai buffer harus berubah searah (monoton) & tegangan antar buffer cukup
// berbeda. Bila lolos -> ditulis ke kalibrasi AKTIF (nilai rata-rata tiap kelompok).
byte kalEvaluasi(byte sensor){
  int n = (sensor==0) ? TDS_TITIK_KAL : PH_TITIK_KAL;
  if(kalJumlahSampel(sensor) < ((sensor==0)?KAL_TDS_SAMPEL_MIN:KAL_PH_SAMPEL_MIN)) return KAL_BELUM_CUKUP;
  if(kalJumlahJenis(sensor) < 2) return KAL_BUTUH_JENIS;
  float x[TDS_TITIK_KAL], y[TDS_TITIK_KAL]; int g=0;
  for(int i=0;i<n;i++){
    if(sensor==0){ if(kalDraft.tdsN[i]>0){ x[g]=kalDraft.tdsSumCV[i]/kalDraft.tdsN[i]; y[g]=kalDraft.tdsPPM[i]; g++; } }
    else         { if(kalDraft.phN[i]>0){  x[g]=kalDraft.phSumV[i]/kalDraft.phN[i];   y[g]=kalDraft.phX100[i]/100.0f; g++; } }
  }
  for(int i=0;i<g-1;i++) for(int j=i+1;j<g;j++) if(x[j]<x[i]){ float t=x[i]; x[i]=x[j]; x[j]=t; t=y[i]; y[i]=y[j]; y[j]=t; }
  const float dxMin = (sensor==0) ? 0.005f : 0.010f;   // beda tegangan minimum antar buffer (V)
  int arah = 0;
  for(int i=0;i<g-1;i++){
    if(x[i+1]-x[i] < dxMin) return KAL_TIDAK_WAJAR;
    int a = (y[i+1]>y[i]) ? 1 : ((y[i+1]<y[i]) ? -1 : 0);
    if(a==0 || (arah!=0 && a!=arah)) return KAL_TIDAK_WAJAR;
    arah = a;
  }
  if(sensor==0 && arah<0) return KAL_TIDAK_WAJAR;      // TDS: PPM wajib naik bersama tegangan
  if(sensor==0){
    for(int i=0;i<TDS_TITIK_KAL;i++){ if(i<g){ calTDSBuffer[i]=(int)y[i]; tdsCVInput[i]=x[i]; } else { calTDSBuffer[i]=-1; tdsCVInput[i]=0.0f; } }
    hitungKalibrasiTDS();
  } else {
    float vv[PH_TITIK_KAL]={0,0,0};
    for(int i=0;i<PH_TITIK_KAL;i++){ if(i<g){ phBufferInput[i]=y[i]; vv[i]=x[i]; } else phBufferInput[i]=-1.0f; }
    calPhVolt1=vv[0]; calPhVolt2=vv[1]; calPhVolt3=vv[2];
    float sS=0.0f; int nS=0;
    for(int i=0;i<PH_TITIK_KAL;i++){ sS+=kalDraft.phSumSuhu[i]; nS+=kalDraft.phN[i]; }
    if(nS>0){ suhuKalibrasiPH = sS / nS; EEPROM.put(EEPROM_ALAMAT_SUHUKALPH, suhuKalibrasiPH); }
    hitungKalibrasiPH();
  }
  eepromAutoSimpan();
  return KAL_AKTIF;
}

// ==========================
// FUNGSI SENSOR SUHU DS18B20 
// ==========================
// Sensor suhu dianggap ERROR bila 3x berturut-turut tak terbaca / di luar rentang wajar. Saat error, suhu larutan
// memakai cadangan 25.0 C (= tanpa kompensasi) agar kompensasi TDS & ATC pH tidak memakai nilai basi diam-diam.
const float SUHU_CADANGAN_C = 25.0f;
bool suhuSensorValid = true;
byte suhuGagalBerturut = 0;
void updateSuhuAsync() {
  unsigned long now = millis();
  if(!suhuKonversiSedangBerjalan) {
    sensorSuhu.requestTemperatures();
    tsSuhuMulai = now;
    suhuKonversiSedangBerjalan = true;
  } else if(now - tsSuhuMulai >= INTERVAL_SUHU_KONVERSI) {
    float t = sensorSuhu.getTempCByIndex(0);
    if(t != DEVICE_DISCONNECTED_C && t > -20.0f && t < 85.0f) {   // 85.0 = nilai reset DS18B20 (bukan bacaan sah)
      suhuRawTerakhir = t;
      suhularutan = t + suhuOffset;
      suhuGagalBerturut = 0;
      suhuSensorValid = true;
    } else {
      if(suhuGagalBerturut < 255) suhuGagalBerturut++;
      if(suhuGagalBerturut >= 3){ suhuSensorValid = false; suhularutan = SUHU_CADANGAN_C; }
    }
    suhuKonversiSedangBerjalan = false;
  }
}
// Angka dengan jumlah desimal menyesuaikan besarnya (muat di LCD, tetap terbaca utk nilai sangat kecil/besar)
String angkaRingkas(float v){
  float a = fabs(v);
  int dec = (a<0.01f)?5 : (a<1.0f)?4 : (a<10.0f)?3 : (a<100.0f)?2 : 1;
  return String(v, dec);
}
// Teks suhu untuk LCD: "ERR" bila sensor suhu error
String formatSuhu(){ return suhuSensorValid ? String(suhularutan,1) : String(F("ERR")); }
char* teksSuhu(char *b){ if(suhuSensorValid) dtostrf(suhularutan,1,1,b); else strcpy(b,"ERR"); return b; }

void bacaSensorSelektif(bool bacaTds, bool bacaPh) {
  if(bacaTds) ppmTerbaca=hitungPPM(suhularutan);
  if(bacaPh)  phTerbaca=hitungPH();
}
void bacaSemua() {
  bacaSensorSelektif(true, true);
}

String formatPPMSensor() {
  if(!tdsSensorValid) return "-";
  return String(ppmTerbaca);
}
String formatPHSensor() {
  if(!phSensorValid) return "-";
  return String(phTerbaca,2);
}
// Versi char (tanpa String) untuk layar tiap detik
char* teksPPM(char *b){ if(!tdsSensorValid){ strcpy(b,"-"); } else itoa(ppmTerbaca,b,10); return b; }
char* teksPH(char *b){  if(!phSensorValid){  strcpy(b,"-"); } else dtostrf(phTerbaca,1,2,b); return b; }

// =================
// FUNGSI RTC DS3231
// =================
bool rtcPerluSet = false;   // true bila DS3231 kehilangan daya (baterai habis): waktu chip tak bisa dipercaya
bool rtcWaktuValid() {
  if(!rtcSiap || rtcPerluSet) return false;   // RTC kehilangan daya -> dianggap tidak valid sampai waktu diset ulang
  DateTime n = rtc.now();
  // Validasi rentang wajar tanggal/waktu dari chip
  return n.year() >= 2000 && n.year() <= 2099
      && n.month() >= 1 && n.month() <= 12
      && n.day()   >= 1 && n.day()   <= 31;
}
// Format HH:MM valid (tolak "--:--", digit non-numerik, jam/menit di luar rentang)
bool waktuDosingFormatValid(const String& w) {
  if(w.length() != 5) return false;
  if(w.charAt(2) != ':') return false;
  for(int i=0;i<5;i++){
    if(i==2) continue;
    if(w.charAt(i) < '0' || w.charAt(i) > '9') return false;
  }
  int jam = w.substring(0,2).toInt();
  int men = w.substring(3,5).toInt();
  return (jam >= 0 && jam <= 23 && men >= 0 && men <= 59);
}
String getWaktuStr() {
  if(!rtcWaktuValid()) return String(F("--:--:--"));
  DateTime n=rtc.now(); char b[9];
  snprintf(b,sizeof(b),"%02d:%02d:%02d",n.hour(),n.minute(),n.second()); return String(b);   // TES_144: snprintf (bacaan I2C rusak)
}
String getTanggalStr() {
  if(!rtcWaktuValid()) return String(F("--/--/--"));
  DateTime n=rtc.now(); char b[9];
  snprintf(b,sizeof(b),"%02d/%02d/%02d",n.day(),n.month(),n.year()%100); return String(b);
}
String getTanggalLengkapStr() {
  if(!rtcWaktuValid()) return String(F("--/--/----"));
  DateTime n=rtc.now(); char b[11];
  snprintf(b,sizeof(b),"%02d/%02d/%04d",n.day(),n.month(),n.year()); return String(b);
}
void terapkanWaktuRTC() {
  // DS3231 memakai I2C (tanpa WP/CH 3-wire).
  // Tulis waktu baru lalu verifikasi baca-balik (termasuk detik).
  if(!rtcSiap){
    lcd.clear();
    lcdPrintCenter(1, F("RTC TIDAK SIAP"));
    lcdPrintCenter(2, F("Cek kabel I2C"));
    jedaInfo(2500);
    eepromSimpanRTC();
    return;
  }
  DateTime baru(rtcInput[0], rtcInput[1], rtcInput[2],
                rtcInput[3], rtcInput[4], rtcInput[5]);
  bool lamaValid = rtcWaktuValid();                       // TES_144: waktu sebelum dikoreksi (besar koreksi)
  uint32_t tLama = lamaValid ? rtc.now().unixtime() : 0UL;
  rtc.adjust(baru);          // adjust() juga menghapus flag "kehilangan daya" (OSF) pada DS3231
  rtcPerluSet = false;
  // Verifikasi: baca balik. Bila gagal (mis. kabel/baterai), beri tahu.
  DateTime cek = rtc.now();
  if(rtc.lostPower()) rtcPerluSet = true;
  // TES_127: bandingkan selisih detik (<= 2 dtk) — detik bisa berganti di antara tulis & baca balik
  long selisih = (long)(cek.unixtime() - baru.unixtime());
  bool setGagal = (!rtcWaktuValid() || cek.year() < 2000 || cek.month() == 0 || cek.day() == 0 || selisih < 0 || selisih > 2);
  if(setGagal){
    lcd.clear();
    lcdPrintCenter(1, F("RTC GAGAL SIMPAN"));
    lcdPrintCenter(2, F("Cek kabel/baterai"));
    jedaInfo(2500);
  }
  // TES_144: koreksi JAM kecil (<= 12 jam) -> target cek (tanggal + Waktu Dosing) DIPERTAHANKAN: RTC lambat -> cek yang
  // terlewat langsung dijalankan; RTC cepat -> cek yang sudah jalan lebih awal tidak diulang di hari yang sama. (Dulu
  // selalu dijadwal ulang dari jam baru: cek hari itu bisa terlewat / berjalan 2x.) Koreksi TANGGAL besar (mis. RTC
  // salah tahun) -> dijadwal ulang agar tidak menunggu bertahun-tahun / langsung jalan. Set gagal -> jadwal tak diubah.
  if(dosingSedangMenunggu && !setGagal){
    int32_t delta = lamaValid ? (int32_t)(baru.unixtime() - tLama) : 0x7FFFFFFFL;
    bool jadwalUlang = (!lamaValid || delta > 12L*3600L || delta < -12L*3600L);
    if(jadwalUlang) jadwalkanCekBerikutnya();
    lcd.clear();
    lcdPrintCenter(1, jadwalUlang ? F("Jadwal disesuaikan") : F("Jadwal cek tetap"));
    char b[21];
    snprintf_P(b,sizeof(b),PSTR("Cek: %02d/%02d %s"),dosingTargetHari,dosingTargetBulan,terpilihWaktuDosing.c_str());
    lcdPrintCenter(2, b);
    jedaInfo(2500);
    lcd.clear();
  }
  eepromSimpanRTC();
}

// ======================================================
// VALIDASI TANGGAL RTC – JUMLAH HARI PER BULAN & KABISAT
// ======================================================
bool isTahunKabisat(int tahun) {

  if(tahun % 4 != 0)   return false;
  if(tahun % 100 == 0) return (tahun % 400 == 0);
  return true;
}
int getMaksHariBulan(int bulan, int tahun) {
  switch(bulan) {
    case 1:  return 31;
    case 2:  return isTahunKabisat(tahun) ? 29 : 28;
    case 3:  return 31;
    case 4:  return 30;
    case 5:  return 31;
    case 6:  return 30;
    case 7:  return 31;
    case 8:  return 31;
    case 9:  return 30;
    case 10: return 31;
    case 11: return 30;
    case 12: return 31;
    default: return 31;
  }
}

void tambahHari(int &d, int &m, int &y, int n) {
  for(int i=0;i<n;i++){
    d++;
    int maxH = getMaksHariBulan(m, y);
    if(d > maxH){
      d = 1;
      m++;
      if(m > 12){ m = 1; y++; }
    }
  }
}

// Cari index opsi EC/Dilution dari nilai string-nya (-1 jika tidak ketemu).
int cariIndexDil(String dil){
  for(int i=0;i<2;i++) if(dil==dilOptions[i]) return i;
  return -1;
}

// ==========================
// MUAT ULANG KONSTANTA AKTIF
// ==========================
void muatUlangKonstantaAktif() {
  for(int i=0;i<2;i++) regresiSlopeAB[i]=regresiSlopeABDefault[i];
  for(int i=0;i<TOTAL_TABEL_PH;i++){
    tabelSlopeAtas[i][0]=pgm_read_float(&tabelSlopeAtasDefault[i][0]);
    tabelSlopeAtas[i][1]=pgm_read_float(&tabelSlopeAtasDefault[i][1]);
    tabelSlopeBawah[i][0]=pgm_read_float(&tabelSlopeBawahDefault[i][0]);
    tabelSlopeBawah[i][1]=pgm_read_float(&tabelSlopeBawahDefault[i][1]);
    tabelSlopeAtasFosfat[i][0]=pgm_read_float(&tabelSlopeAtasFosfatDefault[i][0]);
    tabelSlopeAtasFosfat[i][1]=pgm_read_float(&tabelSlopeAtasFosfatDefault[i][1]);
    tabelSlopeBawahFosfat[i][0]=pgm_read_float(&tabelSlopeBawahFosfatDefault[i][0]);
    tabelSlopeBawahFosfat[i][1]=pgm_read_float(&tabelSlopeBawahFosfatDefault[i][1]);
  }
  int idxDilA=cariIndexDil(terpilihDilution);
  if(terpilihEC!="-" && idxDilA>=0) slopeABMixTerpilih     = hitungSlopeAB(terpilihEC.toFloat(), idxDilA);
  int idxDilB=cariIndexDil(terpilihDilutionBaru);
  if(terpilihECBaru!="-" && idxDilB>=0) slopeABMixTerpilihBaru = hitungSlopeAB(terpilihECBaru.toFloat(), idxDilB);
  kalkSlopeTerpilih = hitungSlopeAB(kalkEC.toFloat(), kalkSelDil);
}

void hitungKalkPPM() {
  float slope = kalkSlopeTerpilih;
  if(slope <= 0.0f){ kalkHasilTotal=0.0f; kalkHasilTambah=0.0f; return; }  // guard bagi-nol slope
  if(kalkVolume>0){
    kalkHasilTotal=((float)(kalkPPMTarget-(int)regresiIntercept)/slope)*(float)kalkVolume;
    kalkHasilTambah=(kalkPPMAwal>=kalkPPMTarget)?0.0f:
                    ((float)(kalkPPMTarget-kalkPPMAwal)/slope)*(float)kalkVolume;
  } else {
    kalkHasilTotal=(float)(kalkPPMTarget-(int)regresiIntercept)/slope;
    kalkHasilTambah=(kalkPPMAwal>=kalkPPMTarget)?0.0f:
                    (float)(kalkPPMTarget-kalkPPMAwal)/slope;
  }
  if(kalkHasilTotal<0.0f) kalkHasilTotal=0.0f;  
}


// =================
// KALKULASI pH DOWN
// =================
float interpSlopePH(const float tabel[][2], float ppm) {
  if(ppm <= tabel[0][0]) return tabel[0][1];
  if(ppm >= tabel[TOTAL_TABEL_PH-1][0]) return tabel[TOTAL_TABEL_PH-1][1];
  for(int i=0;i<TOTAL_TABEL_PH-1;i++){
    float p1=tabel[i][0], p2=tabel[i+1][0];
    if(ppm>=p1 && ppm<=p2){
      float s1=tabel[i][1], s2=tabel[i+1][1];
      if(p2 - p1 < 0.01f) return s1; 
      float t = (ppm - p1) / (p2 - p1);
      return s1 + t*(s2-s1);
    }
  }
  return tabel[TOTAL_TABEL_PH-1][1];
}

// ============================================================================
// CL AUTO-TUNE — Kp & Ki DIHITUNG DARI PERSAMAAN (tanpa tuning manual)
// ============================================================================
// Proses per iterasi (integrator diskrit): y[k+1] = y[k] + g·u[k], u = detik pulsa pompa, g = kenaikan per detik pulsa.
//   PPM : g = q_sisi(mL/dtk) × slope AB (PPM per mL/L) / V tandon(L)                       [konstan thd PPM]
//   pH  : g_ef = Δe / t_model, t_model = volume pH Down dari persamaan (rentang pH sekarang -> target, pada PPM
//         TERUKUR, fase atas/bawah 5.85 otomatis) / q_P1  -> Kp ikut rentang pH & PPM saat itu (gain scheduling)
//   Kp = 0.5 / g  -> satu pulsa menutup 50% error (konservatif, aktuator satu arah tak boleh overshoot)
//   Ki = 0.1 · Kp -> karakteristik z² − (2−gKp)z + (1−gKp+gKi): dgn gKp=0.5, gKi=0.05 kedua akar riil (0.86 & 0.64)
//                    -> respons tanpa osilasi/overshoot; Ki mengoreksi bila persamaan sedikit meleset.
float debitMLdtk(int idx){ return (float)terpilihDebitPump[idx]/DEBIT_SKALA*1000.0f/60.0f; }
float gainModelPPM(){
  float q = 0.5f*(debitMLdtk(4)+debitMLdtk(5));          // per SISI (A = B), konsisten dgn makro
  if(q<=0.0f || slopeABMixTerpilih<=0.0f || terpilihVolume<=0) return 0.0f;
  return q * slopeABMixTerpilih / (float)terpilihVolume;    // PPM per detik pulsa
}
// Waktu pompa P1 (detik) yang menurut persamaan pH Down diperlukan dari phSekarang ke phTujuan pada PPM tertentu
float waktuModelPH(float phSekarang, float phTujuan, int ppm){
  float q = debitMLdtk(3);
  if(q<=0.0f) return 0.0f;
  return hitungPHDownVolume(phSekarang, phTujuan, ppm) / q;
}
// Gain efektif pH (pH per detik pulsa) sepanjang rentang phSekarang -> target (melintasi fase 5.85 bila perlu)
float gainEfektifPH(float phSekarang, int ppm){
  float e = phSekarang - terpilihPH;
  float t = waktuModelPH(phSekarang, terpilihPH, ppm);
  return (e>0.0f && t>0.0f) ? (e/t) : 0.0f;
}
// Nilai acuan (untuk tampilan & batas aman ZN) pada PPM target: Kp/Ki PPM, Kp pH fase atas/bawah.
// Saat dosing, kanal pH dihitung ulang tiap iterasi dari rentang pH & PPM terukur (lihat prosesCekPH).
void hitungKonstantaPIOtomatis(){
  float g = gainModelPPM();
  if(g>0.0f){ piPPM.kp = PI_FRAKSI_KOREKSI/g; piPPM.ki = PI_RASIO_KI*piPPM.kp; }
  float qPH = debitMLdtk(3);
  int acidIdx = (terpilihAcid=="Fosfat") ? 1 : 0;
  const float (*tabelAtasP)[2]  = (acidIdx==0) ? tabelSlopeAtas  : tabelSlopeAtasFosfat;
  const float (*tabelBawahP)[2] = (acidIdx==0) ? tabelSlopeBawah : tabelSlopeBawahFosfat;
  float skalaVolume = (terpilihConcPersen>0 && terpilihVolume>0)
    ? (((float)terpilihVolume/10.0f) * (10.0f/(float)terpilihConcPersen)) : 0.0f;
  if(qPH>0.0f && skalaVolume>0.0f){
    float sA = interpSlopePH(tabelAtasP,  (float)terpilihPPM);
    float sB = interpSlopePH(tabelBawahP, (float)terpilihPPM);
    if(sA>0.0f) piPHKpAtas  = PI_FRAKSI_KOREKSI / (qPH*sA/skalaVolume);
    if(sB>0.0f) piPHKpBawah = PI_FRAKSI_KOREKSI / (qPH*sB/skalaVolume);
    piPH.ki = PI_RASIO_KI * min(piPHKpAtas, piPHKpBawah);
  }
}

void hitungKalkPH() {
  kalkPHHasilAtas  = 0.0f;
  kalkPHHasilBawah = 0.0f;

  const float (*tabelAtasDipakai)[2]  = (kalkPHAcid==0) ? tabelSlopeAtas  : tabelSlopeAtasFosfat;
  const float (*tabelBawahDipakai)[2] = (kalkPHAcid==0) ? tabelSlopeBawah : tabelSlopeBawahFosfat;

  int konsenPersen = kalkPHKonsenPersen;
  if(konsenPersen <= 0){ kalkPHHasilTotal = 0.0f; return; }   // guard bagi-nol konsentrasi
  float skalaVolume = ((float)kalkPHVolume/10.0f) * (10.0f/(float)konsenPersen);

  if(kalkPHAwal > batasFasePH){
    float batasAtas = (kalkPHTarget > batasFasePH) ? kalkPHTarget : batasFasePH;
    float dPhAtas = kalkPHAwal - batasAtas;
    if(dPhAtas > 0.0f){
      float slopeAtas = interpSlopePH(tabelAtasDipakai, (float)kalkPHPpm);
      if(slopeAtas > 0.0f) kalkPHHasilAtas = (dPhAtas/slopeAtas) * skalaVolume;
    }
  }
  if(kalkPHTarget < batasFasePH){
    float batasBawah = (kalkPHAwal < batasFasePH) ? kalkPHAwal : batasFasePH;
    float dPhBawah = batasBawah - kalkPHTarget;
    if(dPhBawah > 0.0f){
      float slopeBawah = interpSlopePH(tabelBawahDipakai, (float)kalkPHPpm);
      if(slopeBawah > 0.0f) kalkPHHasilBawah = (dPhBawah/slopeBawah) * skalaVolume;
    }
  }
  kalkPHHasilTotal = kalkPHHasilAtas + kalkPHHasilBawah;
}


// ======================================
// FUNGSI RELAY – POMPA & KATUP SOLENOID
// ======================================
// TES_152: relay yang BERGANTI keadaan diberi jarak min RELAY_JEDA_MS dari pergantian relay sebelumnya -> arus
// kumparan relay + lonjakan beban (pompa/katup/buzzer) tidak terjadi bersamaan (cegah tegangan 5V turun -> restart).
#define RELAY_JEDA_MS 150UL
unsigned long relayGantiMs = 0;
// Pompa dosis D2/D3/P1/P2/P3 (lama nyala = volume dosis) TIDAK diberi jeda; jeda hanya utk D1, katup, buzzer, daya TDS.
bool relayBerjeda(int pin){ return pin==PIN_RELAY_D1 || pin==PIN_RELAY_SOLENOID || pin==PIN_RELAY_BUZZER || pin==PIN_RELAY_TDS; }
void relayTulis(int pin, uint8_t level){
  if(digitalRead(pin) != level && relayBerjeda(pin)){
    unsigned long d = millis() - relayGantiMs;
    if(d < RELAY_JEDA_MS){ wdt_reset(); delay(RELAY_JEDA_MS - d); }
    digitalWrite(pin, level);
    relayGantiMs = millis();
  } else {
    if(digitalRead(pin) != level) relayGantiMs = millis();   // pompa dosis berganti -> relay berjeda berikutnya menunggu
    digitalWrite(pin, level);
  }
}
void relayOn(int pin) {
  relayTulis(pin, RELAY_AKTIF_LOW ? LOW : HIGH);
  delay(5);   
}
void relayOff(int pin) {
  relayTulis(pin, RELAY_AKTIF_LOW ? HIGH : LOW);
  delay(5);   
}
// -- Daya modul sensor TDS: ON hanya saat benar-benar mengukur TDS --
void nyalakanRelayTDS(){ if(!tdsRelayAktif){ relayOn(PIN_RELAY_TDS); tdsRelayAktif = true; } }
void matikanRelayTDS(){ if(tdsRelayAktif){ relayOff(PIN_RELAY_TDS); tdsRelayAktif = false; } }
// TES_149: lonjakan listrik (EMI) tiap kali katup solenoid on/off bisa me-reset Arduino / mengacaukan LCD I2C.
// (1) katup tak boleh dibuka lagi < KATUP_JEDA_BUKA_MS sesudah berganti keadaan (sensor water level bergetar/gelombang
//     dulu membuat katup on-off berulang -> lonjakan berulang); MENUTUP selalu langsung (aman).
// (2) re-init LCD ditunda LCD_REINIT_TUNDA_MS sesudah katup berganti (dulu langsung, saat lonjakan masih terjadi).
unsigned long katupGantiMs = 0;
bool katupPernahGanti = false;
unsigned long lcdReinitMs = 0;
bool bolehBukaSolenoid(){ return !katupPernahGanti || (millis() - katupGantiMs >= KATUP_JEDA_BUKA_MS); }
void bukaSolenoid() {
  if(solenoidTerbuka) return;
  if(!bolehBukaSolenoid()) return;   // dicoba lagi oleh pemanggil berikutnya (tiap detik)
  relayOn(PIN_RELAY_SOLENOID);
  solenoidTerbuka = true;
  katupGantiMs = millis(); katupPernahGanti = true;
  lcdPerluReinit = true; lcdReinitMs = katupGantiMs;
}
void tutupSolenoid() {
  relayOff(PIN_RELAY_SOLENOID);
  if(solenoidTerbuka){ katupGantiMs = millis(); katupPernahGanti = true; lcdPerluReinit = true; lcdReinitMs = katupGantiMs; }
  solenoidTerbuka = false;
}

// FUNGSI BUZZER (indikator, non-blocking, relay IN8 aktif) 
void mulaiBuzzer(unsigned long durasiMs){
  relayOn(PIN_RELAY_BUZZER);
  buzzerPutus    = false;
  buzzerBunyi    = true;
  buzzerMulaiMs  = millis();
  buzzerDurasiMs = durasiMs;
}
// Buzzer putus-putus (0,5 dtk ON/OFF) - pembeda ALARM (RTC rusak / hasil cek di luar batas) dari bunyi selesai biasa
void mulaiBuzzerPutus(unsigned long durasiMs){
  relayOn(PIN_RELAY_BUZZER);
  buzzerPutus    = true;
  buzzerFaseOn   = true;
  buzzerToggleMs = millis();
  buzzerBunyi    = true;
  buzzerMulaiMs  = millis();
  buzzerDurasiMs = durasiMs;
}
void hentikanBuzzer(){
  relayOff(PIN_RELAY_BUZZER);
  buzzerBunyi = false;
  buzzerPutus = false;
}
void updateBuzzer(){
  if(!buzzerBunyi) return;
  unsigned long now = millis();
  if(now - buzzerMulaiMs >= buzzerDurasiMs){ hentikanBuzzer(); return; }
  if(buzzerPutus && (now - buzzerToggleMs >= BUZZER_PUTUS_MS)){
    buzzerToggleMs = now;
    buzzerFaseOn = !buzzerFaseOn;
    if(buzzerFaseOn) relayOn(PIN_RELAY_BUZZER); else relayOff(PIN_RELAY_BUZZER);
  }
}
void matikanSemuaPompa() {
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++) relayOff(pumpRelayPin[i]);
  tesPompaBerjalan = false;   // semua relay pompa mati -> tes pompa (bila ada) otomatis berakhir
  ujiD1Aktif = false;         // D1 ikut mati (pompa indeks 0) -> uji reaksi tak perlu mematikannya lagi
  ujiPulsaAktif = false;      // TES_138: pompa pulsa uji (P1/P2/P3) ikut mati
}

// ============================================
// FUNGSI MODE DOSING OTOMATIS – "MULAI DOSING"
// ============================================
bool statusFaseDosingBerjalan(MenuState s) {
  return (s==DOSING_TUNGGU_WAKTU || s==DOSING_ISI_TANDON  || s==DOSING_JEDA_ISI ||
          s==DOSING_CEK_PPM1     || s==DOSING_MACRO_AB    || s==DOSING_HOMOGEN1 ||
          s==DOSING_CEK_PPM2     || s==DOSING_MICRO_AB    || s==DOSING_HOMOGEN2 || s==DOSING_CEK_PPM3 ||
          s==DOSING_CEK_PH1      || s==DOSING_MACRO_PH    || s==DOSING_HOMOGEN3 ||
          s==DOSING_CEK_PH2      || s==DOSING_MICRO_PH    || s==DOSING_HOMOGEN4 ||
          s==DOSING_PI_CEK_PPM   || s==DOSING_PI_POMPA_AB || s==DOSING_PI_HOMOGEN_AB ||
          s==DOSING_PI_CEK_PH    || s==DOSING_PI_POMPA_PH || s==DOSING_PI_HOMOGEN_PH ||
          s==DOSING_CEK_AKHIR    || s==DOSING_JEDA_PULIH);
}

// Batas aman durasi penyalaan tiap pompa saat eksekusi dosing (detik).
const long PUMP_MAKS_DETIK[TOTAL_PUMP_OPTIONS] = {600L,1200L,1200L,1200L,10800L,10800L};

float hitungDurasiPompaDetik(float volumeML, int pumpIdx) {
  float debit = terpilihDebitPump[pumpIdx] / DEBIT_SKALA;   // L/menit
  if(debit <= 0.0f) return 0.0f;
  return (volumeML * 0.06f) / debit;
}

// Durasi Pompa Generik
float hitungDurasiPompaDetikKalk(float volumeML, float debitLmnt) {
  if(debitLmnt <= 0.0f) return 0.0f;
  return (volumeML * 0.06f) / debitLmnt;
}

float hitungPHDownVolume(float phAwal, float phTarget, int ppmAcuan) {
  int acidIdx = (terpilihAcid=="Fosfat") ? 1 : 0;

  const float (*tabelAtasP)[2]  = (acidIdx==0) ? tabelSlopeAtas  : tabelSlopeAtasFosfat;
  const float (*tabelBawahP)[2] = (acidIdx==0) ? tabelSlopeBawah : tabelSlopeBawahFosfat;

  int konsenPersen = terpilihConcPersen;
  if(konsenPersen <= 0) return 0.0f;   
  float skalaVolume = ((float)terpilihVolume/10.0f) * (10.0f/(float)konsenPersen);

  float hasilAtas=0.0f, hasilBawah=0.0f;
  if(phAwal > batasFasePH){
    float batasAtas = (phTarget > batasFasePH) ? phTarget : batasFasePH;
    float dPhAtas = phAwal - batasAtas;
    if(dPhAtas > 0.0f){
      float slopeAtas = interpSlopePH(tabelAtasP, (float)ppmAcuan);
      if(slopeAtas > 0.0f) hasilAtas = (dPhAtas/slopeAtas) * skalaVolume;
    }
  }
  if(phTarget < batasFasePH){
    float batasBawah = (phAwal < batasFasePH) ? phAwal : batasFasePH;
    float dPhBawah = batasBawah - phTarget;
    if(dPhBawah > 0.0f){
      float slopeBawah = interpSlopePH(tabelBawahP, (float)ppmAcuan);
      if(slopeBawah > 0.0f) hasilBawah = (dPhBawah/slopeBawah) * skalaVolume;
    }
  }
  return hasilAtas + hasilBawah;
}

// TES_130: penurunan pH menurut persamaan pH Down bila 'volML' masuk pada pH 'ph' (kebalikan hitungPHDownVolume).
// Model BERFASE: di atas 5.85 memakai slope fase atas sampai 5.85, sisanya slope fase bawah (~3x lebih kuat).
// Dipakai cek respon pH. Dulu prediksi = gain RATA-RATA (pH sekarang -> target) x detik: bila pH masih di fase atas
// dan target di fase bawah, prediksi pulsa jauh terlalu besar (bisa 2x) -> kanal pH terkunci palsu saat stok sedikit
// lebih lemah dari model.
float prediksiTurunPHK(bool baru, float ph, float volML, int ppmAcuan){
  int conc = baru ? terpilihConcPersenBaru : terpilihConcPersen;
  int vol  = baru ? terpilihVolumeBaru : terpilihVolume;
  if(volML <= 0.0f || conc <= 0 || vol <= 0) return 0.0f;
  bool fosfat = ((baru ? terpilihAcidBaru : terpilihAcid)=="Fosfat");
  float skala = ((float)vol/10.0f) * (10.0f/(float)conc);
  float sA = interpSlopePH(fosfat ? tabelSlopeAtasFosfat  : tabelSlopeAtas,  (float)ppmAcuan);
  float sB = interpSlopePH(fosfat ? tabelSlopeBawahFosfat : tabelSlopeBawah, (float)ppmAcuan);
  float turun = 0.0f;
  if(ph > batasFasePH && sA > 0.0f){
    float vAtas = (ph - batasFasePH) / sA * skala;     // volume sampai batas fase 5.85
    if(volML <= vAtas) return volML * sA / skala;
    turun = ph - batasFasePH; volML -= vAtas;
  }
  if(sB > 0.0f) turun += volML * sB / skala;
  return turun;
}
float prediksiTurunPH(float ph, float volML, int ppmAcuan){ return prediksiTurunPHK(false, ph, volML, ppmAcuan); }

// TES_130: respon pH dinilai di RUANG VOLUME: penurunan pH terukur dikonversi ke volume pH Down yang menurut
// persamaan menghasilkan penurunan itu (dari pH acuan, berfase), lalu dibandingkan dgn volume yang benar-benar keluar.
// Stok x0.5 -> rasio 0.5 di fase mana pun. (Rasio penurunan pH saja bisa turun di bawah 0.3 walau stok hanya ~30%
// lebih lemah, bila pH awal sedikit di atas 5.85 & target jauh di fase bawah -> kunci palsu.)
// true = TIDAK ada respon berarti (prediksi cukup besar & volume setara < 30% volume keluar).
bool phTanpaRespon(float phAcuan, float phSekarang, float volKeluar, int ppmAcuan){
  float pred = prediksiTurunPH(phAcuan, volKeluar, ppmAcuan);
  if(pred < RESPON_PRED_MIN_PH || volKeluar <= 0.0f) return false;     // dosis terlalu kecil utk dinilai (derau)
  float volSetara = hitungPHDownVolume(phAcuan, phSekarang, ppmAcuan);   // 0 bila pH tidak turun
  return volSetara < RESPON_FRAKSI_MIN * volKeluar;
}

// ── TES_137: penilaian respon dgn ukur ulang ──
#define RESPON_CEK_ULANG   0.90f   // respon < 90% prediksi -> ukur ulang sesudah homogenisasi tambahan
                                   // TES_139: dulu 70% -> s/d 30% makro masih belum tercampur dianggap 'OK', dosis berikut
                                   // (mikro OL / PI) dihitung dari bacaan itu -> overdosis pH Down (aduk ~= jeda: pH 4.8)
#define RESPON_NAIK_LAMBAT 0.07f   // di antara 2 ukuran (tanpa dosis) berubah >= 7% prediksi -> pengadukan lambat
                                   // TES_139: dulu 15% -> aduk 15 mnt + jeda 2 mnt (naik 14.5%) lolos sbg stok lemah; 7% + ambang absolut (30 PPM / 0.08 pH) = tahan derau
#define RESPON_OK     0
#define RESPON_TUNDA  1            // ukur ulang dulu
#define RESPON_KUNCI  2            // tetap < 30% sesudah ukur ulang -> kerusakan
#define RESPON_LAMBAT 3            // respon baru tampak sesudah ukur ulang -> hentikan kanal ini utk siklus ini
byte nilaiResponAB(float naik, float pred){
  if(pred < RESPON_PRED_MIN_PPM) return RESPON_OK;   // dosis terlalu kecil utk dinilai (derau)
  float fr = naik / pred;
  if(!sedangUlangAB){
    if(fr < RESPON_CEK_ULANG){ ulangResponAB = true; naikPertamaAB = naik; return RESPON_TUNDA; }
    return RESPON_OK;
  }
  sedangUlangAB = false;
  float d = naik - naikPertamaAB;
  // TES_139: kenaikan susulan diperiksa DULU: bacaan masih bergerak = sensor & pompa bekerja, pengadukan yg lambat
  // (dulu < 30% langsung dikunci walau bacaan masih naik -> kunci palsu pada aduk 30 mnt + jeda 2 mnt)
  if(d >= RESPON_NAIK_LAMBAT*pred && d >= 2.0f*(float)PI_TOL_PPM) return RESPON_LAMBAT;
  if(fr < RESPON_FRAKSI_MIN) return RESPON_KUNCI;
  // TES_139: ambang absolut 45 -> 30 PPM (>= 2x toleransi, di atas derau). Pengadukan sangat lambat (tau >> jeda) menaikkan
  // bacaan sedikit demi sedikit; di bawah ambang -> dianggap 'stok lemah' -> dosis lanjut di atas bacaan belum tercampur
  // -> overdosis. Salah 'lambat' hanya menunda kanal ke cek besok (aman), salah 'lanjut' tak bisa dikoreksi.
  return RESPON_OK;
}
// pH dinilai di ruang volume (model berfase 5.85): volume setara penurunan terukur / volume pH Down yg keluar
byte nilaiResponPH(float acuan, float sekarang, float vol, int ppm, float predMin){
  if(predMin < RESPON_PRED_MIN_PH || vol <= 0.0f) return RESPON_OK;
  float fr = hitungPHDownVolume(acuan, sekarang, ppm) / vol;
  if(!sedangUlangPH){
    if(fr < RESPON_CEK_ULANG){ ulangResponPH = true; frPertamaPH = fr; phPertamaUlang = sekarang; return RESPON_TUNDA; }
    return RESPON_OK;
  }
  sedangUlangPH = false;
  if(fr - frPertamaPH >= RESPON_NAIK_LAMBAT && (phPertamaUlang - sekarang) >= 0.08f) return RESPON_LAMBAT;   // TES_139: susulan dulu
  if(fr < RESPON_FRAKSI_MIN) return RESPON_KUNCI;
  // TES_139: ambang susulan 0.15 -> 0.08 pH (di atas derau). Tabel data empiris: pH Down per mL lebih lemah -> kenaikan
  // per jeda lebih kecil; aduk 15 mnt + jeda 2 mnt dulu lolos sbg 'stok lemah' -> pH jatuh ke ~3.5 sesudah tercampur.
  return RESPON_OK;
}

// ── TES_138: makro tertunda (daya padam sebelum responnya dinilai) ──
// TES_139: dicatat saat pompa makro MULAI, tiap 15 dtk selama memompa (volume yg sudah keluar), dan saat selesai.
// Daya padam di tengah pompa makro -> siklus ulang menilai dosis yg sempat keluar (dulu hanya sesudah pompa selesai).
// Tiap 15 dtk (bukan tiap detik): umur EEPROM (100 rb tulis/sel) -> pompa makro 10 mnt/hari ~ 40 tulis/hari.
void catatMakroPulih(){
  if(currentState==DOSING_MACRO_AB && terpilihVolume>0){
    float pred = slopeABMixTerpilih * (dosingTotalABmL - olVolAwalAB) / (float)terpilihVolume;
    makroPulih = (makroPulih & ~MP_AB_ULANG) | MP_AB;
    makroPulihAcuanPPM = (int16_t)olAcuanAB;
    makroPulihPredPPM  = (int16_t)min(pred, 30000.0f);
  } else if(currentState==DOSING_MACRO_PH){
    float vol = dosingTotalPHmL - olVolAwalPH;
    makroPulih = (makroPulih & ~MP_PH_ULANG) | MP_PH;
    makroPulihAcuanPHx100 = (int16_t)(olAcuanPH*100.0f + 0.5f);
    makroPulihVolPHx10    = (uint16_t)min(vol*10.0f + 0.5f, 65000.0f);
  }
}
// Cek pertama siklus (sesudah daya pulih): makro sebelum padam terbaca? true = kanal AB dilewati siklus ini.
//   >= 70% prediksi : sensor & pompa normal -> lanjut biasa (bacaan sudah memuat makro itu)
//   30-70%          : pengadukan belum merata -> AB tak didosis siklus ini (catatan respon lambat), cek besok
//   < 30%           : homogenisasi J1 lalu ukur ulang sekali; masih < 30% -> kanal dikunci (sensor/pompa rusak)
bool nilaiMakroPulihAB(int ppmSekarang){
  if(!(makroPulih & MP_AB)) return false;
  float pred = (float)makroPulihPredPPM;
  if(pred < RESPON_PRED_MIN_PPM){ makroPulih &= ~(MP_AB|MP_AB_ULANG); return false; }   // dosis sempat keluar terlalu kecil utk dinilai
  float fr = (pred > 0.0f) ? ((float)ppmSekarang - (float)makroPulihAcuanPPM)/pred : 1.0f;
  if(fr >= RESPON_CEK_ULANG){ makroPulih &= ~(MP_AB|MP_AB_ULANG); return false; }
  if(fr >= RESPON_FRAKSI_MIN){ makroPulih &= ~(MP_AB|MP_AB_ULANG); responLambatSiklus = true; return true; }
  if(!(makroPulih & MP_AB_ULANG)){ makroPulih |= MP_AB_ULANG; ulangPulihAB = true; return true; }
  makroPulih &= ~(MP_AB|MP_AB_ULANG);
  piFlagSiklus |= PI_FLAG_AB_RESPON; kanalTerkunci |= KUNCI_AB; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
  return true;
}
bool nilaiMakroPulihPH(float phSekarang){
  if(!(makroPulih & MP_PH)) return false;
  float vol = (float)makroPulihVolPHx10/10.0f, acuan = (float)makroPulihAcuanPHx100/100.0f;
  float fr = (vol > 0.0f) ? hitungPHDownVolume(acuan, phSekarang, ppmAcuanPHDown())/vol : 1.0f;
  if(prediksiTurunPH(acuan, vol, ppmAcuanPHDown()) < RESPON_PRED_MIN_PH) fr = 1.0f;   // dosis terlalu kecil utk dinilai
  if(fr >= RESPON_CEK_ULANG){ makroPulih &= ~(MP_PH|MP_PH_ULANG); return false; }
  if(fr >= RESPON_FRAKSI_MIN){ makroPulih &= ~(MP_PH|MP_PH_ULANG); responLambatSiklus = true; return true; }
  if(!(makroPulih & MP_PH_ULANG)){ makroPulih |= MP_PH_ULANG; ulangPulihPH = true; return true; }
  makroPulih &= ~(MP_PH|MP_PH_ULANG);
  piFlagSiklus |= PI_FLAG_PH_RESPON; kanalTerkunci |= KUNCI_PH; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
  return true;
}

// Kalkulasi pH Down Generik
float hitungPHDownVolumeKalk(float phAwal, float phTarget, int ppmAcuan, int acidIdx, int konsenPersen, int volume) {
  if(konsenPersen <= 0) return 0.0f;   // guard bagi-nol konsentrasi
  const float (*tabelAtasP)[2]  = (acidIdx==0) ? tabelSlopeAtas  : tabelSlopeAtasFosfat;
  const float (*tabelBawahP)[2] = (acidIdx==0) ? tabelSlopeBawah : tabelSlopeBawahFosfat;

  float skalaVolume = ((float)volume/10.0f) * (10.0f/(float)konsenPersen);

  float hasilAtas=0.0f, hasilBawah=0.0f;
  if(phAwal > batasFasePH){
    float batasAtas = (phTarget > batasFasePH) ? phTarget : batasFasePH;
    float dPhAtas = phAwal - batasAtas;
    if(dPhAtas > 0.0f){
      float slopeAtas = interpSlopePH(tabelAtasP, (float)ppmAcuan);
      if(slopeAtas > 0.0f) hasilAtas = (dPhAtas/slopeAtas) * skalaVolume;
    }
  }
  if(phTarget < batasFasePH){
    float batasBawah = (phAwal < batasFasePH) ? phAwal : batasFasePH;
    float dPhBawah = batasBawah - phTarget;
    if(dPhBawah > 0.0f){
      float slopeBawah = interpSlopePH(tabelBawahP, (float)ppmAcuan);
      if(slopeBawah > 0.0f) hasilBawah = (dPhBawah/slopeBawah) * skalaVolume;
    }
  }
  return hasilAtas + hasilBawah;
}

//==============
// PROSES DOSING
//==============

// ── Fase A: Menunggu Waktu Dosing ──
bool cekWaktuDosingTercapai() {
  if(!rtcWaktuValid()) return false; // RTC tidak valid 
  if(!waktuDosingFormatValid(terpilihWaktuDosing)) return false;
  DateTime n = rtc.now();
  int jamT = terpilihWaktuDosing.substring(0,2).toInt();
  int menT = terpilihWaktuDosing.substring(3,5).toInt();
  DateTime target(dosingTargetTahun, dosingTargetBulan, dosingTargetHari, jamT, menT, 0);
  return n.unixtime() >= target.unixtime();
}

// ── Pembanding hasil cek terhadap Batasan PPM/pH (pH dibandingkan dalam satuan x100 agar bebas galat float) ──
int  phKeX100(float v){ return (int)(v*100.0f + 0.5f); }
bool ppmDiBawahBatas(int p){  return p < terpilihPPM - batasPPM; }
bool ppmDiAtasBatas(int p){   return p > terpilihPPM + batasPPM; }
bool phDiBawahBatas(float v){ return phKeX100(v) < phKeX100(terpilihPH) - batasPHx100; }
bool phDiAtasBatas(float v){  return phKeX100(v) > phKeX100(terpilihPH) + batasPHx100; }

// Bersihkan seluruh variabel kerja satu siklus (dipakai saat siklus mulai, siklus lanjut, dan Mulai Dosing)
void resetVariabelSiklus(){
  dosingPPMTerkunci=false; dosingPHTerkunci=false;
  piReset();                                   // integral, iterasi PI & akumulasi volume siklus berjalan
  dosingABMixTahap1=0.0f; dosingABMixTahap2=0.0f;
  dosingPHDownTahap1=0.0f; dosingPHDownTahap2=0.0f;
  dosingLewatiAB=false; dosingLewatiPH=false;
  dosingSudahAdaDosis=false;
}

// Jadwal cek berikutnya = kemunculan Waktu Dosing (HH:MM) berikutnya SETELAH saat ini:
// hari ini bila jamnya belum lewat, selain itu besok. Siklus yang sangat lama (melewati jam cek) cukup
// melompati satu hari -> tidak ada lagi bentrok jadwal seperti pada Loop Dosing.
void jadwalkanCekBerikutnya(){
  if(!rtcWaktuValid() || !waktuDosingFormatValid(terpilihWaktuDosing)){
    tambahHari(dosingTargetHari, dosingTargetBulan, dosingTargetTahun, 1);   // tanpa RTC: majukan 1 hari dari jadwal lama
    return;
  }
  DateTime n = rtc.now();
  int jamT = terpilihWaktuDosing.substring(0,2).toInt();
  int menT = terpilihWaktuDosing.substring(3,5).toInt();
  dosingTargetHari  = n.day();
  dosingTargetBulan = n.month();
  dosingTargetTahun = n.year();
  long nowSec    = (long)n.hour()*3600L + (long)n.minute()*60L + (long)n.second();
  long targetSec = (long)jamT*3600L + (long)menT*60L;
  if(nowSec >= targetSec) tambahHari(dosingTargetHari, dosingTargetBulan, dosingTargetTahun, 1);
}

// Mulai satu siklus cek harian: isi tandon -> cek PPM -> (dosing AB bila perlu) -> cek pH -> (dosing pH bila perlu)
// Dipakai saat jadwal tercapai DAN saat daya pulih di tengah siklus (gerbang batasan mencegah dosis ganda).
void mulaiSiklusCekHarian() {
  // Dipicu saat user sedang di menu lain -> beri tahu bahwa menu ditutup otomatis
  bool dariMenuLain = !(currentState==DOSING_TUNGGU_WAKTU || currentState==DOSING_TUNDA_KALIBRASI ||
                        currentState==DOSING_FINAL_STATUS || currentState==DOSING_REKAP ||
                        statusFaseDosingBerjalan(currentState));
  matikanSemuaPompa();             // hentikan tes pompa / uji reaksi / bilas tes sensor yg mungkin berjalan
  if(dariMenuLain && !statusOverlayAktif && !cancelConfirmAktif){
    lcd.clear();
    lcdPrintCenter(1, F("WAKTU CEK HARIAN"));
    lcdPrintCenter(2, F("Menu ditutup"));
    jedaInfo(2000);
  }
  inputBuffer = ""; ecBebasDigits = ""; ecTunda = "";   // buang sisa input menu yang terpotong
  resetTawaranTiba();              // TES_135
  mulaiBuzzer(BUZZER_DURASI_MS);   // waktu cek tercapai
  statusPPMTerakhir=-1; statusPHTerakhir=-1.0f;   // bacaan siklus berjalan (hasil cek terakhir tersimpan di cekTerakhir)
  resetVariabelSiklus();
  if(!siklusDariPulih) makroPulih = 0;   // TES_138: siklus biasa -> tak ada makro tertunda dari siklus lain
  siklusDariPulih = false; ulangPulihAB = false; ulangPulihPH = false;
  tandonPenuhBerturut=0;
  dosingSedangMenunggu = false;
  jadwalCekAktif = true;
  isiTandonDetikBerjalan = 0;
  isiAdaAirBaru = false;
  eepromAutoSimpan();   // persist: siklus sedang berjalan (untuk lanjut otomatis bila daya mati)
  currentState = DOSING_ISI_TANDON;
  if(cancelConfirmAktif){ lcd.clear(); tampilkanCancelDosing(); }        // overlay batal tetap di depan
  else if(!statusOverlayAktif){ lcd.clear(); tampilkanDosingIsiTandon(); }
}

// Status dalam/luar batas hasil cek terakhir, dinilai terhadap target & batasan AKTIF saat ini.
// (Nilai PPM/pH tetap hasil cek terakhir; bila target berubah lewat Parameter Baru, penilaian ikut target baru.)
byte hitungFlagCek(){
  if(!batasanSudahDiatur || terpilihPPM<=0 || terpilihPH<=0.0f) return 0;
  byte flag = 0;
  if(cekTerakhir.ppm >= 0){
    if(ppmDiBawahBatas(cekTerakhir.ppm))     flag |= CEK_PPM_RENDAH;
    else if(ppmDiAtasBatas(cekTerakhir.ppm)) flag |= CEK_PPM_TINGGI;
  }
  if(cekTerakhir.ph >= 0.0f){
    if(phDiBawahBatas(cekTerakhir.ph))       flag |= CEK_PH_RENDAH;
    else if(phDiAtasBatas(cekTerakhir.ph))   flag |= CEK_PH_TINGGI;
  }
  return flag;
}
// Status cek terakhir perlu perhatian user (di luar batas / cek gagal)?
uint8_t kodeErrCek(){ uint8_t k = cekTerakhir.err & 0x0F; return (k<=CEK_ERR_SENSOR || k==CEK_ERR_LAMBAT) ? k : CEK_ERR_TIDAK; }   // CEK_ERR_* (4 bit bawah); kode lain (rekaman TES_131/132) = tuntas
uint8_t piFlagCek(){  return (cekTerakhir.err >> 4) & 0x0F; }   // PI_FLAG_* (4 bit atas, TES_122; data lama = 0)
bool cekPerluPerhatian(){ return kodeErrCek()!=CEK_ERR_TIDAK || piFlagCek()!=0 || hitungFlagCek()!=0 || kanalTerkunci!=0; }

// Catat hasil siklus (berhasil / gagal) ke rekam cek terakhir & EEPROM, lalu jadwalkan cek berikutnya.
void catatHasilSiklus(bool adaDosis, uint8_t kodeErr){
  cekTerakhir.magic    = REKAM_CEK_MAGIC;
  cekTerakhir.ppm      = statusPPMTerakhir;
  cekTerakhir.ph       = statusPHTerakhir;
  cekTerakhir.err      = (uint8_t)((kodeErr & 0x0F) | ((piFlagSiklus & 0x0F) << 4));
  cekTerakhir.didosing = adaDosis ? 1 : 0;
  bool rtcOk = rtcWaktuValid();
  DateTime n = rtcOk ? rtc.now() : DateTime(2000,1,1,0,0,0);
  if(rtcOk){
    cekTerakhir.tgl=n.day(); cekTerakhir.bln=n.month(); cekTerakhir.thn=n.year();
    cekTerakhir.jam=n.hour(); cekTerakhir.mnt=n.minute();
  } else cekTerakhir.tgl=0;   // waktu cek tidak diketahui
  if(adaDosis){   // rekap "DOSING TERAKHIR" hanya diperbarui oleh siklus yang benar-benar mendosis
    if(rtcOk){ cekTerakhir.dTgl=n.day(); cekTerakhir.dBln=n.month(); cekTerakhir.dThn=n.year();
               cekTerakhir.dJam=n.hour(); cekTerakhir.dMnt=n.minute(); }
    else       { cekTerakhir.dTgl=0; }
    cekTerakhir.dDurasiMenit = (int)((millis() - dosingMulaiMs) / 60000UL);
    cekTerakhir.dABmL = dosingTotalABmL;
    cekTerakhir.dPHmL = dosingTotalPHmL;
  }
  eepromSimpanRekamCek();
  jadwalkanCekBerikutnya();
  dosingSedangMenunggu = true;   // jadwal berikutnya ter-arm sejak siklus selesai (layar rekap/status hanya tampilan)
  eepromAutoSimpan();
}

// Dipanggil SEKALI saat siklus cek selesai: tanpa dosis (kedua kanal dalam batas / tak bisa dikoreksi) atau
// setelah cek akhir. Mencatat hasil cek terakhir, menjadwalkan cek berikutnya, lalu menampilkan rekap/status.
void selesaikanSiklusCek(bool adaDosis){
  makroPulih = 0;   // TES_138
  relayOff(PIN_RELAY_D1);
  matikanRelayTDS();
  catatHasilSiklus(adaDosis, responLambatSiklus ? CEK_ERR_LAMBAT : CEK_ERR_TIDAK);   // TES_137
  byte flag = hitungFlagCek();
  uint8_t pf = piFlagCek();
  if(flag || pf || kanalTerkunci || responLambatSiklus) mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);   // luar batas / PI / kunci / lambat
  else                            mulaiBuzzer(BUZZER_DURASI_MS);
  if(!statusOverlayAktif && !cancelConfirmAktif){
    lcd.clear();
    lcdPrintCenter(1, adaDosis ? F("DOSING SELESAI") : F("CEK SELESAI"));
    if(pf & (PI_FLAG_AB_RESPON|PI_FLAG_PH_RESPON)){
      lcdPrintCenter(2, teksPIFlag(pf));
      lcdPrintCenter(3, F("Kanal dikunci!"));
    } else if(kanalTerkunci){
      lcdPrintCenter(2, teksCatatan(0));
      lcdPrintCenter(3, F("Buka di Pengaturan"));
    }
    else if(responLambatSiklus){   // TES_137: kanal dihentikan krn respon baru tampak sesudah homogenisasi ulang
      lcdPrintCenter(2, F("Respon dosis lambat"));
      lcdPrintCenter(3, F("Cek jeda/sirkulasi"));
    }
    else if(pf)        lcdPrintCenter(2, teksPIFlag(pf));
    else if(flag)      lcdPrintCenter(2, F("Di luar batas"));
    else if(!adaDosis) lcdPrintCenter(2, F("Sesuai batas"));
    jedaInfo(3000);
  }
  statusOverlayAktif = false; statusHalaman = 0;
  cancelConfirmAktif = false;    // siklus sudah selesai -> overlay batal (bila terbuka) tak berlaku lagi
  lcd.clear();
  if(adaDosis){
    currentState = DOSING_REKAP;
    rekapDetikBerjalan = 0;
    tampilkanRekapDosing();
  } else {
    currentState = DOSING_FINAL_STATUS;
    finalStatusDetikBerjalan = 0;
    tampilkanStatusDosing();
  }
}

// Siklus HARI INI gagal (tandon tak penuh dalam 6 jam / sensor tidak valid): semua aktuator dimatikan,
// kegagalan dicatat di Status Dosing, alarm berbunyi, dan jadwal BESOK tetap berjalan (tidak dimatikan).
void gagalkanSiklusHariIni(uint8_t kodeErr){
  matikanSemuaPompa();
  relayOff(PIN_RELAY_D1);
  tutupSolenoid();
  matikanRelayTDS();
  catatHasilSiklus(dosingSudahAdaDosis, kodeErr);
  resetVariabelSiklus();
  mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
  cancelConfirmAktif = false; statusOverlayAktif = false; statusHalaman = 0;   // pesan gagal harus terlihat
  currentState = (kodeErr==CEK_ERR_TANDON) ? DOSING_TANDON_TIMEOUT : DOSING_SENSOR_INVALID;
  lcd.clear();
  if(kodeErr==CEK_ERR_TANDON) tampilkanDosingTandonTimeout(); else tampilkanDosingSensorInvalid();
}

// Dipanggil dari DOSING_FINAL_STATUS untuk kembali ke layar menunggu cek berikutnya
void lanjutkanSiklusBerikutnya() {
  matikanSemuaPompa();
  tutupSolenoid();
  resetVariabelSiklus();
  dosingSedangMenunggu = true;
  eepromAutoSimpan();   // persist status "menunggu jadwal berikutnya"
  currentState=DOSING_TUNGGU_WAKTU;
  lcd.clear();
  tampilkanDosingTungguWaktu(false);
}

// Dipanggil tiap detik selama layar DOSING_FINAL_STATUS tampil.
// Bila hasil cek terakhir di luar batasan, layar TIDAK berpindah otomatis (tetap tampil sampai tombol ditekan)
// agar peringatan tidak terlewat. Jadwal cek berikutnya tetap berjalan di latar belakang.
void prosesFinalStatus() {
  if(cekPerluPerhatian()) return;
  finalStatusDetikBerjalan++;
  if(finalStatusDetikBerjalan >= FINAL_STATUS_AUTO_LANJUT_DETIK){
    lanjutkanSiklusBerikutnya();
  }
}

// ── Jeda homogenisasi setelah daya pulih di tengah siklus (TES_122) ──
int jedaPulihDetik = 0;
void mulaiJedaPulih(){
  matikanSemuaPompa(); tutupSolenoid(); matikanRelayTDS();
  siklusDariPulih = true;   // TES_138: makroPulih (bila ada) dinilai di cek pertama siklus ulang
  jedaPulihDetik = 0;
  currentState = DOSING_JEDA_PULIH;
  lcd.clear(); tampilkanJedaPulih();
}
void tampilkanJedaPulih(){
  int jeda = jedaJ0Menit();   // TES_134: J0 (dulu J1)
  lcdPrintCenter(0, F("DAYA PULIH"));
  lcdPrintCenter(1, F("Homogenisasi Larutan"));
  lcdCenterP(2, PSTR("%d/%d menit"), jedaPulihDetik/60, jeda);
  lcdBar(3, jedaPulihDetik, (long)jeda*60L);
}
void prosesJedaPulih(){
  jedaPulihDetik++;
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanJedaPulih();
  if(jedaPulihDetik >= jedaJ0Menit()*60) mulaiSiklusCekHarian();   // TES_134: J0
}

void prosesTungguWaktu() {
  bool tercapai = cekWaktuDosingTercapai();
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingTungguWaktu(tercapai);
  if(tercapai){
    mulaiSiklusCekHarian();
  }
}

// ── TES_135: JAM CEK TIBA SAAT DI MENU LAIN -> TAWARAN TUNDA 5 MENIT ──
const unsigned long TIBA_BUZZER_MS = 5000UL;     // buzzer saat tawaran muncul: 5 dtk
const unsigned long TIBA_JAWAB_MS  = 300000UL;   // TES_136: tawaran menunggu jawaban maks 5 mnt, lalu dosing
const unsigned long TIBA_TUNDA_MS  = 300000UL;   // tunda (B / Batal pasang sensor): 5 menit
const unsigned long TUNDA_PASANG_MS = 300000UL;  // TES_136: layar "Pasang ke chamber" maks 5 mnt, lalu dosing
const unsigned long IDLE_MENU_MS   = 300000UL;   // menu tanpa tombol > 5 mnt -> layar dosing / status
extern bool inputIsiAwal;        // didefinisikan di bagian input (isi awal nilai lama)
bool promptTibaAktif = false;
const __FlashStringHelper* ujiBatalTundaJudul = NULL;   // TES_148: uji dibatalkan saat prompt TIBA -> info ditampilkan sesudahnya
unsigned long promptTibaMulaiMs = 0;
byte tibaFase = 0;               // 0 = belum ditawarkan, 1 = ditunda (B / Batal pasang sensor) -> tawarkan lagi 5 mnt kemudian
unsigned long tibaAcuanMs = 0;   // awal hitungan 5 menit tunda
MenuState stateSebelumTunda = MENU_KAL_SENSOR;   // TES_136: menu kalibrasi yg ditinggalkan saat layar pasang sensor
String inputSebelumTunda = "";
unsigned long tundaMulaiMs = 0;
extern MenuState stateIsiAwalTerakhir;
unsigned long tombolTerakhirMs = 0;
// Layar menu/uji milik user (bukan layar proses/hasil dosing)
bool diMenuLain(){
  return !(statusFaseDosingBerjalan(currentState) || currentState==DOSING_TUNDA_KALIBRASI ||
           currentState==DOSING_FINAL_STATUS || currentState==DOSING_REKAP ||
           currentState==DOSING_TANDON_TIMEOUT || currentState==DOSING_SENSOR_INVALID);
}
void resetTawaranTiba(){ tibaFase = 0; promptTibaAktif = false; lcdTertahan = false; ujiBatalTundaJudul = NULL; }
void tampilkanPromptTiba(){
  bool t = lcdTertahan; lcdTertahan = false;
  lcd.clear();
  lcdPrintCenter(0, F("TIBA WAKTU DOSING"));
  lcdPrintCenter(1, F("Tunda dosing 5"));
  lcdPrintCenter(2, F("menit berikutnya?"));
  lcdPrintCenter(3, F("<-   |   OK"));
  lcdTertahan = t;
}
void bukaPromptTiba(){
  promptTibaAktif = true; promptTibaMulaiMs = millis();
  mulaiBuzzer(TIBA_BUZZER_MS);
  lcdTertahan = false; tampilkanPromptTiba(); lcdTertahan = true;   // menu di belakang tetap berjalan, tak digambar
}
void tutupPromptTiba(){
  promptTibaAktif = false; lcdTertahan = false;
  lcd.clear(); gambarUlangLayarAktif();   // kembali ke layar menu yang sedang dibuka
  if(ujiBatalTundaJudul){
    const __FlashStringHelper* j = ujiBatalTundaJudul; ujiBatalTundaJudul = NULL;
    if(currentState==MENU_ZN_HUB){
      lcd.clear(); lcdPrintCenter(0, F("UJI REAKSI")); lcdPrintCenter(1, j); lcdPrintCenter(2, F("Uji dibatalkan"));
      mulaiBuzzerPutus(3000UL); jedaInfo(3500); lcd.clear(); tampilkanMenuZNHub();
    }
  }
}
// Mulai dosing dari menu lain. Menu kalibrasi sensor: probe mungkin masih di larutan buffer -> layar
// "Pasang ke chamber, B=Mulai" dulu (perilaku lama), selain itu siklus langsung dimulai.
void mulaiDosingDariMenuLain(){
  if(sedangKalibrasiBuffer()) masukTundaKalibrasi();
  else                        mulaiSiklusCekHarian();
}
// Dipanggil tiap detik saat jam cek sudah tiba & user berada di menu lain
void prosesTibaDiMenuLain(){
  unsigned long now = millis();
  if(promptTibaAktif){
    if(now - promptTibaMulaiMs >= TIBA_JAWAB_MS){   // TES_136: 5 mnt tak dijawab -> masuk dosing
      promptTibaAktif = false; lcdTertahan = false;
      mulaiDosingDariMenuLain();
    }
    return;
  }
  if(tibaFase==0){ bukaPromptTiba(); return; }
  if(now - tibaAcuanMs >= TIBA_TUNDA_MS) bukaPromptTiba();   // tunda habis & masih di menu lain -> tanya lagi
}
// Tombol saat layar tawaran tampil: B = tunda 5 mnt, * = dosing sekarang, tombol lain diabaikan
void prosesTombolPromptTiba(char key){
  if(key=='B'){ hentikanBuzzer(); tibaFase = 1; tibaAcuanMs = millis(); tutupPromptTiba(); }
  else if(key=='*'){ hentikanBuzzer(); promptTibaAktif = false; lcdTertahan = false; mulaiDosingDariMenuLain(); }
}
// Uji / kalibrasi yang sedang berjalan -> layar tidak ditinggalkan otomatis saat menganggur
bool menuSedangProses(){
  return tesPompaBerjalan || currentState==TES_POMPA_AKTIF || currentState==TES_POMPA_CANCEL_CONFIRM ||
         currentState==TES_SENSOR_PROSES || (currentState==MENU_TES_SOLENOID && solenoidTerbuka) ||
         currentState==UJI_ZN_LOGGING || currentState==UJI_ZN_KONFIRMASI || sedangKalibrasiBuffer() ||
         (currentState==ZN_KONFIRMASI_JADWAL && znTundaDariUji) ||   // TES_144: hasil uji menunggu jawaban
         (currentState==MENU_WATER_LEVEL && (wlIsiAktif || solenoidTerbuka));   // TES_148: isi tandon manual berjalan
}
// Menu dibiarkan > 5 mnt tanpa tombol -> layar tunggu dosing (jadwal aktif) atau Status Dosing (jadwal tak aktif)
void cekMenuMenganggur(){
  if(menuSedangProses()){ tombolTerakhirMs = millis(); return; }   // TES_136: hitungan 5 mnt dimulai sesudah proses selesai
  if(promptTibaAktif || tibaFase!=0 || !diMenuLain()) return;   // tawaran tunda berjalan -> ikut aturan tawaran
  if(millis() - tombolTerakhirMs < IDLE_MENU_MS) return;
  tombolTerakhirMs = millis();
  if(!dosingSedangMenunggu && currentState==MENU_UTAMA && statusOverlayAktif) return;   // sudah di Status Dosing
  inputBuffer = ""; ecBebasDigits = ""; ecTunda = ""; inputIsiAwal = false;
  cancelConfirmAktif = false;
  if(dosingSedangMenunggu){
    statusOverlayAktif = false;
    currentState = DOSING_TUNGGU_WAKTU;
    lcd.clear(); tampilkanDosingTungguWaktu(false);
  } else {
    currentState = MENU_UTAMA; selMenuUtama = 1; scrollMenuUtama = 0;
    statusOverlayAktif = true; statusHalaman = 0;
    lcd.clear(); tampilkanStatusDosing();
  }
}

void cekPemicuDosingLatarBelakang() {
  // RTC tak valid/hilang saat jadwal sedang menunggu = cek TIDAK akan pernah jalan -> bunyikan alarm
  // (1x setelah 60 dtk, lalu diulang tiap 10 menit selama masih rusak).
  static unsigned long rtcRusakDetik = 0;
  if(dosingSedangMenunggu && !rtcWaktuValid()){
    rtcRusakDetik++;
    if(rtcRusakDetik==60 || (rtcRusakDetik>60 && (rtcRusakDetik-60)%600UL==0)) mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
  } else rtcRusakDetik = 0;
  // Layar pasang sensor (kalibrasi): B = mulai, * = kembali ke kalibrasi; TES_136: maks 5 mnt lalu dosing otomatis
  if(currentState==DOSING_TUNDA_KALIBRASI){
    if(millis() - tundaMulaiMs >= TUNDA_PASANG_MS) mulaiSiklusCekHarian();
    return;
  }
  // Jadwal berjalan di LATAR BELAKANG: user boleh berada di menu mana pun (uji debit, kalkulator, pengaturan, dll).
  // Saat jam cek tiba, layar otomatis pindah ke proses dosing & terkunci sampai siklus selesai.
  if(dosingSedangMenunggu && currentState!=DOSING_TUNGGU_WAKTU){
    if(cekWaktuDosingTercapai()){
      if(diMenuLain()) prosesTibaDiMenuLain();   // TES_135: tawarkan tunda 5 mnt (kalibrasi: lewat layar pasang sensor)
      else             mulaiSiklusCekHarian();   // layar rekap / status akhir / gagal -> langsung
    } else if(tibaFase!=0 || promptTibaAktif){ if(promptTibaAktif) tutupPromptTiba(); resetTawaranTiba(); }   // jadwal bergeser / RTC tak valid
  } else if(!dosingSedangMenunggu && (tibaFase!=0 || promptTibaAktif)){
    if(promptTibaAktif) tutupPromptTiba();
    resetTawaranTiba();
  }
}

// Kalibrasi buffer TDS/pH: probe sedang di larutan buffer, BUKAN di chamber tandon -> cek tidak boleh langsung jalan
bool sedangKalibrasiBuffer(){
  // Seluruh menu kalibrasi TDS/pH: chamber mungkin masih berisi larutan buffer
  // TES_126: kalibrasi suhu ikut ditunda -> probe suhu mungkin masih di air acuan (kompensasi suhu TDS/pH salah)
  return currentState==MENU_KAL_SENSOR || currentState==MENU_KAL_TDS_SAMPEL || currentState==MENU_KAL_PH_SAMPEL ||
         currentState==MENU_KAL_LIHAT  || currentState==MENU_KAL_ULANG_KONF || currentState==MENU_SUHU_KALIBRASI;
}
// TES_136: *=Batal di layar pasang sensor -> kembali ke menu kalibrasi semula; tawaran muncul lagi 5 mnt kemudian
void kembaliDariTundaKalibrasi(){
  hentikanBuzzer();
  MenuState s = stateSebelumTunda;
  if(!(s==MENU_KAL_SENSOR || s==MENU_KAL_TDS_SAMPEL || s==MENU_KAL_PH_SAMPEL || s==MENU_KAL_LIHAT ||
       s==MENU_KAL_ULANG_KONF || s==MENU_SUHU_KALIBRASI)) s = MENU_KAL_SENSOR;
  currentState = s;
  stateIsiAwalTerakhir = s;               // isian yang sudah diketik tidak ditimpa nilai lama
  inputBuffer = inputSebelumTunda; inputIsiAwal = false;
  tibaFase = 1; tibaAcuanMs = millis();   // tawarkan lagi 5 menit kemudian (bila masih di menu)
  lcd.clear();
  if(s==MENU_KAL_TDS_SAMPEL){ mulaiKalTdsBuffer(); tampilkanInputTDSBuffer(0); }   // pengambilan sampel diulang dari awal
  else if(s==MENU_KAL_PH_SAMPEL){ mulaiKalPhBuffer(); tampilkanInputpHBuffer(0); }
  else gambarUlangLayarAktif();
}
void tampilkanTundaKalibrasi(){
  lcdPrintCenter(0, F("WAKTU CEK HARIAN"));
  lcdPrintCenter(1, F("Kalibrasi dihentikan"));
  lcdPrintCenter(2, F("Pasang ke chamber,"));   // sensor dilepas saat kalibrasi -> pasang kembali dulu
  lcdPrintCenter(3, F("<-   |  OK"));
}
void masukTundaKalibrasi(){
  resetTawaranTiba();   // TES_135
  stateSebelumTunda = currentState;   // TES_136: menu kalibrasi yang ditinggalkan (*=Batal kembali ke sini)
  inputSebelumTunda = inputBuffer;
  tundaMulaiMs = millis();
  matikanSemuaPompa();
  matikanRelayTDS();
  inputBuffer = "";
  statusOverlayAktif = false;
  mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
  currentState = DOSING_TUNDA_KALIBRASI;
  lcd.clear();
  tampilkanTundaKalibrasi();
}
// Overlay "Cancel Dosing?" (dipakai di fase dosing, layar tunggu [D], dan layar tunda kalibrasi [*])
void bukaCancelDosing(){
  selCancelDosing = 1;              // default "Tidak"
  cancelConfirmAktif = true;
  cancelConfirmMulaiMs = millis();
  lcd.clear();
  tampilkanCancelDosing();
}
void tutupCancelDosing(){
  cancelConfirmAktif = false;
  lcd.clear();
  tampilkanUlangFaseDosing();
}

// TES_127: RTC dicek PALING AWAL (sebelum peringatan resolusi & sebelum draft Baru di-commit) -> bila RTC tak valid,
// draft Parameter Baru & jadwal yang sedang menunggu tidak tersentuh.
bool rtcSiapUntukMulai(){
  if(rtcWaktuValid()) return true;
  lcd.clear();
  lcdPrintCenter(1, F("RTC TIDAK VALID"));
  lcdPrintCenter(2, F("Set waktu dulu"));
  lcdPrintCenter(3, F("(Pengaturan Sistem)"));
  jedaInfo(2500);
  lcd.clear();
  currentState=MENU_UTAMA;
  tampilkanMenuUtama();
  return false;
}
// TES_127: kanal yang masih terkunci tidak akan didosing -> beri tahu saat jadwal dimulai (info, tidak memblokir)
void infoKanalTerkunciSaatMulai(){
  if(!kanalTerkunci) return;
  lcd.clear();
  lcdPrintCenter(0, teksCatatan(0));
  lcdPrintCenter(1, F("Kanal hanya diukur,"));
  lcdPrintCenter(2, F("tidak didosing"));
  lcdPrintCenter(3, F("(Pengaturan Sistem)"));
  mulaiBuzzer(1000UL);
  jedaInfo(3000);
}
// Memulai jadwal cek harian memakai Parameter Aktif yang sedang tersimpan
void mulaiDosingSekarang(){
  // RTC harus valid; tanpa waktu chip yang andal, jadwal tidak boleh di-arm
  if(!rtcWaktuValid()){
    lcd.clear();
    lcdPrintCenter(1, F("RTC TIDAK VALID"));
    lcdPrintCenter(2, F("Set waktu dulu"));
    jedaInfo(2500);
    lcd.clear();
    currentState=MENU_UTAMA;
    tampilkanMenuUtama();
    return;
  }
  if(!waktuDosingFormatValid(terpilihWaktuDosing)){
    lcd.clear();
    lcdPrintCenter(1, F("WAKTU DOSING"));
    lcdPrintCenter(2, F("TIDAK VALID"));
    jedaInfo(2500);
    lcd.clear();
    currentState=MENU_UTAMA;
    tampilkanMenuUtama();
    return;
  }
  infoKanalTerkunciSaatMulai();
  matikanSemuaPompa();
  tutupSolenoid();
  statusPPMTerakhir=-1; statusPHTerakhir=-1.0f;
  resetVariabelSiklus();
  // TES_136: cek hari ini sudah tiba tetapi sedang ditunda (user di menu, mis. commit Parameter Baru) -> tetap dijalankan
  bool cekTertunda = dosingSedangMenunggu && (tibaFase!=0 || promptTibaAktif) && cekWaktuDosingTercapai();
  resetTawaranTiba();
  if(!cekTertunda) jadwalkanCekBerikutnya();     // cek pertama: hari ini bila Waktu Dosing belum lewat, selain itu besok
  dosingSedangMenunggu = true;
  jadwalCekAktif = true;        // jadwal kini ter-arm (untuk lanjut otomatis setelah daya pulih)
  eepromAutoSimpan();
  currentState=DOSING_TUNGGU_WAKTU;
  lcd.clear();
  tampilkanDosingTungguWaktu(false);
}

// Commit draft Parameter Baru -> Aktif, lalu mulai ulang jadwal cek harian.
void komitParameterBaruLaluMulai(){
  if(!rtcSiapUntukMulai()) return;   // TES_127: RTC rusak -> draft Baru TIDAK di-commit (jadwal lama tetap menunggu)
  EEPROM.update(EEPROM_ALAMAT_KOMIT, KOMIT_BERJALAN);   // TES_148: daya putus di tengah penulisan -> terdeteksi saat boot
  salinParameterBaruKeAktif();
  resetStatusParameterBaru();
  eepromAutoSimpan();
  EEPROM.update(EEPROM_ALAMAT_KOMIT, 0xFF);
  if(validasiKpKiZNSebelumDosing()){
    mulaiDosingSekarang();
  } else {
    currentState=MENU_UTAMA; lcd.clear(); tampilkanMenuUtama();
  }
}

void tampilkanMulaiDosingKonfirmasi(){
  lcdPrintCenter(0,F("Atur dosing baru?"));
  lcdPrint20(0,1, String(selMulaiDosingKonfirmasi==0?"> ":"  ")+F("Ya"));
  lcdPrint20(0,2, String(selMulaiDosingKonfirmasi==1?"> ":"  ")+F("Tidak"));
  lcdPrint20(0,3,F(""));
}


// ── Fase B: Mengisi Tandon (Sensor Water Level + Solenoid) ──
void prosesIsiTandon() {
  bool sensorAtas = (digitalRead(PIN_WATER_LEVEL) == HIGH);
  // Katup menutup segera saat sensor tersentuh; "penuh" baru sah bila bertahan TANDON_PENUH_KONFIRMASI_DETIK
  // (cegah percikan/gelombang saat pengisian memicu dosing dengan volume air yang kurang).
  if(sensorAtas){ if(tandonPenuhBerturut<255) tandonPenuhBerturut++; } else tandonPenuhBerturut=0;
  bool penuh = (tandonPenuhBerturut >= TANDON_PENUH_KONFIRMASI_DETIK);
  if(!sensorAtas) { if(!solenoidTerbuka) bukaSolenoid(); isiAdaAirBaru = true; }
  else            { if(solenoidTerbuka)  tutupSolenoid();  }

  if(!penuh){
    isiTandonDetikBerjalan++;
    if(isiTandonDetikBerjalan >= TANDON_TIMEOUT_DETIK){
      gagalkanSiklusHariIni(CEK_ERR_TANDON);   // > 6 jam belum penuh: lewati hari ini, jadwal besok tetap jalan
      return;
    }
  }

  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingIsiTandon();
  if(penuh){
    tutupSolenoid();
    mulaiBuzzer(BUZZER_DURASI_MS); // volume pengisian tandon tercapai -> buzzer 10 detik
    dosingMulaiMs = millis();      // mulai hitung durasi siklus dosing
    dosingJedaIsiDetikBerjalan = 0;
    currentState = DOSING_JEDA_ISI; // jeda stabilisasi (5 dtk; ada air baru -> Jeda Homogenisasi 1) sebelum fase cek
    if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingJedaIsi(); }
  }
}

// ── Jeda stabilisasi setelah tandon penuh, sebelum fase Cek PPM (TES_133: ada air baru -> Jeda Homogenisasi 1) ──
void prosesJedaIsi() {
  dosingJedaIsiDetikBerjalan++;
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingJedaIsi();
  if((long)dosingJedaIsiDetikBerjalan >= jedaIsiTargetDetik()){
    mulaiCekPPM(DOSING_MACRO_AB);
    currentState = DOSING_CEK_PPM1;
    if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingCekPPM(); }
  }
}

// ── Fase Cek PPM: BILAS (D1 ON) -> STABILISASI (D1 OFF) -> UKUR (median 20 dtk) ──
void mulaiCekPPM(MenuState tujuan) {
  dosingTujuanSetelahCekPPM = tujuan;
  nyalakanRelayTDS();                 // daya modul TDS ON -> stabilisasi 5 dtk dulu (fase 0)
  dosingPPMTerkunci = false;
  cekSensorFase = 0;
  dosingCekDetikBerjalan = 0;
  dosingPPMSum = 0;
  dosingPPMCount = 0;
  resetPpmRoll15();
}
void prosesCekPPM() {
  // fase 0 = stabilisasi daya sensor: bacaan belum dipercaya, jangan cek validitas
  if(cekSensorFase>0 && !tdsSensorValid){ hentikanDosingSensorInvalid(); return; }
  if(!dosingPPMTerkunci){
    dosingCekDetikBerjalan++;
    if(cekSensorFase==0){                       // FASE 0: STABILISASI DAYA SENSOR TDS (5 dtk)
      if(dosingCekDetikBerjalan >= JEDA_STABIL_SENSOR_DETIK){
        primingBufferSensorSelektif(true,false);   // TES_138: buffer median diisi bacaan SESUDAH stabil (dulu berisi bacaan detik ke-1 -> modul TDS lambat = sensor invalid palsu)
        relayOn(PIN_RELAY_D1);                // baru mulai membilas chamber dgn sampel baru
        cekSensorFase = 1;
        dosingCekDetikBerjalan = 0;
      }
    } else if(cekSensorFase==1){              // FASE 1: MEMBILAS (D1 ON)
      if(dosingCekDetikBerjalan >= terpilihLamaCekSensor){
        relayOff(PIN_RELAY_D1);               // D1 OFF -> larutan mulai diam
        cekSensorFase = 2;
        dosingCekDetikBerjalan = 0;
      }
    } else if(cekSensorFase==2){              // FASE 2: STABILISASI (semua pompa OFF)
      if(dosingCekDetikBerjalan >= terpilihJedaStabilisasi){
        cekSensorFase = 3;
        dosingCekDetikBerjalan = 0;
        resetPpmRoll15();                     // jendela median hanya berisi data fase ukur
      }
    } else {                                  // FASE 3: UKUR (median ROLL_MED_N detik)
      sampelPpmRoll15();   // rekam bacaan tiap detik, median 20 detik terakhir dipakai saat terkunci
      if(dosingCekDetikBerjalan >= ROLL_MED_N){
        dosingPPMTerkunci = true;
        dosingPPMHasilCek = medianPpmRoll15();
        statusPPMTerakhir = dosingPPMHasilCek;

        if(dosingTujuanSetelahCekPPM==DOSING_MACRO_AB){
          // GERBANG BATASAN (cek pertama siklus): AB Mix HANYA didosing bila PPM < target - batas.
          // Di dalam batas / di atas batas -> lewati (PPM tinggi tak bisa diturunkan; dicatat saat siklus selesai).
          dosingLewatiAB = !ppmDiBawahBatas(dosingPPMHasilCek);
          if(kanalTerkunci & KUNCI_AB) dosingLewatiAB = true;   // kanal terkunci: diukur saja, tidak didosing
          if(nilaiMakroPulihAB(dosingPPMHasilCek)) dosingLewatiAB = true;   // TES_138: makro sebelum daya padam belum terbaca
        } else {
          // Cek lanjutan (mikro / PI) setelah kanal terpicu: dosing terus menuju target (tengah batasan)
          dosingLewatiAB = (dosingPPMHasilCek >= terpilihPPM);
          // Cek respon OPEN LOOP: kenaikan setelah makro vs prediksi makro (volume yg benar2 keluar)
          if(dosingTujuanSetelahCekPPM==DOSING_MICRO_AB && terpilihVolume>0){
            float pred = slopeABMixTerpilih * (dosingTotalABmL - olVolAwalAB) / (float)terpilihVolume;
            float naik = (float)dosingPPMHasilCek - olAcuanAB;
            byte r = nilaiResponAB(naik, pred);          // TES_137
            if(r!=RESPON_TUNDA) makroPulih &= ~(MP_AB|MP_AB_ULANG);   // TES_138: makro sudah dinilai
            if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_AB_RESPON; kanalTerkunci |= KUNCI_AB; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); dosingLewatiAB = true; }   // mikro tidak dijalankan
            else if(r==RESPON_LAMBAT){ responLambatSiklus = true; dosingLewatiAB = true; }   // mikro ditunda ke cek berikut
          }
          // TES_130: cek respon MIKRO Open Loop (pompa P2-P3, berbeda dgn makro D2-D3) pada Cek PPM 3.
          // Dulu P2-P3 rusak / selang lepas tak terdeteksi (hanya tampak 'di luar batas' atau malah lolos).
          if(dosingTujuanSetelahCekPPM==DOSING_CEK_PPM3 && terpilihVolume>0){
            float pred = slopeABMixTerpilih * (dosingTotalABmL - volAwalMikroAB) / (float)terpilihVolume;
            float naik = (float)dosingPPMHasilCek - olAcuanMikroAB;
            byte r = nilaiResponAB(naik, pred);          // TES_137
            if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_AB_RESPON; kanalTerkunci |= KUNCI_AB; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); }
            else if(r==RESPON_LAMBAT) responLambatSiklus = true;
          }
        }

        // TES_122: makro D2/D3 < MAKRO_MIN_DETIK (debit diafragma besar, kekurangan kecil) -> dosis minimum 1 dtk
        // D2/D3 bisa melewati batas atas. Makro dilewati; seluruh kebutuhan langsung ke P2/P3 (mikro / PI).
        if(dosingTujuanSetelahCekPPM==DOSING_MACRO_AB && !dosingLewatiAB){
          float sisa = (float)(terpilihPPM - dosingPPMHasilCek);
          float vMakro = (sisa>0.0f && slopeABMixTerpilih>0.0f) ? (sisa/slopeABMixTerpilih*(float)terpilihVolume*((float)persenMakro/100.0f)) : 0.0f;
          float q2 = debitMLdtk(1), q3 = debitMLdtk(2);
          if(q2>0.0f && q3>0.0f && (vMakro/q2 < (float)MAKRO_MIN_DETIK || vMakro/q3 < (float)MAKRO_MIN_DETIK)){
            makroABDilewati = true;
            dosingTujuanSetelahCekPPM = (dosingMode==3 || dosingMode==4) ? DOSING_PI_POMPA_AB : DOSING_MICRO_AB;
          }
        }
        if(dosingTujuanSetelahCekPPM==DOSING_PI_POMPA_AB){
          // CLOSED-LOOP: error -> pulsa PI untuk pompa peristaltik P2 & P3
          float errorPPM = (float)(terpilihPPM - dosingPPMHasilCek);
          // TES_130: cek respon MAKRO Closed Loop (D2-D3) di cek PI pertama, sama dgn Open Loop. Dulu sensor TDS macet
          // / D2-D3 rusak baru ketahuan setelah 3 pulsa PI tambahan -> AB overdosis jauh di atas batas.
          bool hentiLambat = false;                  // TES_137
          if(cekMakroABTertunda){
            cekMakroABTertunda = false;
            if(terpilihVolume>0){
              float pred = slopeABMixTerpilih * (dosingTotalABmL - olVolAwalAB) / (float)terpilihVolume;
              float naik = (float)dosingPPMHasilCek - olAcuanAB;
              byte r = nilaiResponAB(naik, pred);
              if(r!=RESPON_TUNDA) makroPulih &= ~(MP_AB|MP_AB_ULANG);   // TES_138: makro sudah dinilai
              if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_AB_RESPON; kanalTerkunci |= KUNCI_AB; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); }
              else if(r==RESPON_TUNDA) cekMakroABTertunda = true;   // dinilai lagi di ukur ulang
              else if(r==RESPON_LAMBAT){ responLambatSiklus = true; hentiLambat = true; }
            }
          }
          // Cek respon tiap 3 pulsa: kenaikan terukur sejak sebelum pulsa pertama jendela vs jumlah prediksi 3 pulsa
          if(resAB.n >= RESPON_JENDELA && !ulangResponAB){
            float naik = (float)dosingPPMHasilCek - resAB.acuan;
            byte r = nilaiResponAB(naik, resAB.prediksi);
            if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_AB_RESPON; kanalTerkunci |= KUNCI_AB; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); }
            else if(r==RESPON_LAMBAT){ responLambatSiklus = true; hentiLambat = true; }
            if(r!=RESPON_TUNDA){ resAB.n = 0; resAB.prediksi = 0.0f; }   // jendela dipertahankan utk ukur ulang
          }
          if((piFlagSiklus & PI_FLAG_AB_RESPON) || hentiLambat){
            piPPMSelesai = true;              // tidak ada respon / respon lambat -> kanal AB dihentikan siklus ini
          } else if(ulangResponAB){
            // TES_137: respon < 70% -> tanpa pulsa; homogenisasi ulang lalu ukur lagi (lihat akhir cek)
          } else if(errorPPM <= (float)PI_TOL_PPM || piIterasiPPM >= PI_MAKS_ITERASI){
            piPPMSelesai = true;              // target tercapai / batas iterasi
            if(errorPPM > (float)PI_TOL_PPM) piFlagSiklus |= PI_FLAG_AB_ITERASI;
          } else {
            if(dosingMode==3) hitungKonstantaPIOtomatis();          // Auto-Tune: Kp/Ki dari persamaan & debit aktif
            PIController piPPMAktif = (dosingMode==4) ? piPPM_ZN : piPPM;   // salinan: batas aman tak menimpa nilai tersimpan
            if(dosingMode==4) batasiKpZNDenganGainModel(piPPMAktif, 0);
            // Batas pulsa: waktu model utk menutup seluruh error (nominal per sisi), maks. batas aman P2/P3
            float gP = gainModelPPM();
            float uMaks = (gP>0.0f) ? (errorPPM/gP) : (float)PUMP_MAKS_DETIK[4];
            if(uMaks > (float)PUMP_MAKS_DETIK[4]) uMaks = (float)PUMP_MAKS_DETIK[4];
            piPulsaDetikAB = piHitungPulsa(errorPPM, piPPMAktif, uMaks);
            if(dosingMode==4) piPPM_ZN.integral = piPPMAktif.integral; else piPPM.integral = piPPMAktif.integral;
            if(piPulsaDetikAB <= 0.0f) piPPMSelesai = true;   // sisa error < resolusi 1 dtk pompa -> selesai tanpa pulsa
            else {
              piIterasiPPM++;
              if(resAB.n==0) resAB.acuan = (float)dosingPPMHasilCek;   // awal jendela respon baru
            }
          }
        } else if(dosingTujuanSetelahCekPPM==DOSING_CEK_PPM3){
          // Cek PPM 3 (Open Loop, setelah mikro): hanya pengukuran -> statusPPMTerakhir = acuan pH Down
        } else if(ulangResponAB){
          // TES_137: Open Loop makro < 70% -> mikro ditahan, ukur ulang dulu
        } else if(!dosingLewatiAB){
          float slope = slopeABMixTerpilih;
          float sisaPPM = (float)(terpilihPPM - dosingPPMHasilCek);
          float totalAB = (sisaPPM>0.0f && slope>0.0f) ? ((sisaPPM/slope)*(float)terpilihVolume) : 0.0f;
          if(dosingTujuanSetelahCekPPM==DOSING_MACRO_AB)
            dosingABMixTahap1 = min(totalAB * ((float)persenMakro/100.0f), batasVolSisiAB(1,2));   // makro = persen makro Parameter Aktif
          else
            dosingABMixTahap2 = min(totalAB, batasVolSisiAB(4,5));      // DOSING_MICRO_AB (open-loop)
          // TES_138: volume tampil = volume yg benar2 dipompa (dulu dibatasi batas aman pompa baru saat pompa mulai)
        } else {
          dosingABMixTahap1 = 0.0f;
          dosingABMixTahap2 = 0.0f;
        }
        sedangUlangAB = false;   // TES_137: ukur ulang (bila ada) sudah dinilai
        dosingFaseTimer = millis();
      }
    }
  }
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPPM();
  if(dosingPPMTerkunci && (millis()-dosingFaseTimer >= 3000UL)){
    if(ulangResponAB){ mulaiCekUlangResponAB(); return; }   // TES_137
    if(ulangPulihAB){ ulangPulihAB = false; mulaiCekUlangRespon(DOSING_HOMOGEN1, DOSING_CEK_PPM1); return; }   // TES_138
    if(dosingTujuanSetelahCekPPM==DOSING_PI_POMPA_AB){
      if(piPPMSelesai){
        // PI kanal AB selesai -> lanjut ke cek pH tahap makro
        mulaiCekPH(DOSING_MACRO_PH);
        currentState = DOSING_CEK_PH1;
        if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingCekPH(); }
      } else {
        siapkanFasePompaPiAB();
      }
    }
    else if(dosingLewatiAB || dosingTujuanSetelahCekPPM==DOSING_CEK_PPM3) prosesLewatiTahapAB();
    else                    siapkanFasePompaAB();
  }
}

// Melewati tahap dosing AB Mix (PPM dalam/di atas batasan, atau target tercapai) -> lanjut cek pH
void prosesLewatiTahapAB() {
  mulaiCekPH(DOSING_MACRO_PH);
  currentState = DOSING_CEK_PH1;
  if((!statusOverlayAktif && !cancelConfirmAktif)){
    lcd.clear();
    tampilkanDosingCekPH();
  }
}

// ── Fase Cek pH: BILAS (D1 ON) -> STABILISASI (D1 OFF) -> UKUR (median 20 dtk) ──
void mulaiCekPH(MenuState tujuan) {
  dosingTujuanSetelahCekPH = tujuan;
  matikanRelayTDS();                  // TDS OFF -> stabilisasi 5 dtk dulu (fase 0), bebas interferensi
  dosingPHTerkunci = false;
  cekSensorFase = 0;
  dosingCekDetikBerjalan = 0;
  dosingPHSum = 0.0f;
  dosingPHCount = 0;
  resetPhRoll15();
}
void prosesCekPH() {
  // fase 0 = stabilisasi setelah daya TDS dimatikan: bacaan belum dipercaya
  if(cekSensorFase>0 && !phSensorValid){ hentikanDosingSensorInvalid(); return; }
  if(!dosingPHTerkunci){
    dosingCekDetikBerjalan++;
    if(cekSensorFase==0){                       // FASE 0: STABILISASI SENSOR (daya TDS baru OFF)
      if(dosingCekDetikBerjalan >= JEDA_STABIL_SENSOR_DETIK){
        primingBufferSensorSelektif(false,true);   // TES_138: buffer pH diisi bacaan sesudah stabil
        relayOn(PIN_RELAY_D1);                // baru mulai membilas chamber
        cekSensorFase = 1;
        dosingCekDetikBerjalan = 0;
      }
    } else if(cekSensorFase==1){              // FASE 1: MEMBILAS (D1 ON)
      if(dosingCekDetikBerjalan >= terpilihLamaCekSensor){
        relayOff(PIN_RELAY_D1);
        cekSensorFase = 2;
        dosingCekDetikBerjalan = 0;
      }
    } else if(cekSensorFase==2){              // FASE 2: STABILISASI (D1 OFF)
      if(dosingCekDetikBerjalan >= terpilihJedaStabilisasi){
        cekSensorFase = 3;
        dosingCekDetikBerjalan = 0;
        resetPhRoll15();
      }
    } else {                                  // FASE 3: UKUR (median ROLL_MED_N detik)
      sampelPhRoll15();   // rekam bacaan tiap detik, median 20 detik terakhir dipakai saat terkunci
      if(dosingCekDetikBerjalan >= ROLL_MED_N){
        dosingPHTerkunci = true;
        dosingPHHasilCek = medianPhRoll15();
        statusPHTerakhir = dosingPHHasilCek;

        if(dosingTujuanSetelahCekPH==DOSING_MACRO_PH){
          // GERBANG BATASAN (cek pH pertama siklus, sesudah tahap AB): pH Down HANYA didosing bila pH > target + batas.
          // Di dalam batas / di bawah batas -> lewati (pH rendah tak bisa dinaikkan; dicatat saat siklus selesai).
          dosingLewatiPH = !phDiAtasBatas(dosingPHHasilCek);
          if(kanalTerkunci & KUNCI_PH) dosingLewatiPH = true;   // kanal terkunci: diukur saja
          if(nilaiMakroPulihPH(dosingPHHasilCek)) dosingLewatiPH = true;   // TES_138
          // TES_139: tahap AB baru saja dinilai LAMBAT (pengadukan tak sempat merata dlm jeda) -> larutan belum homogen:
          // bacaan pH & PPM acuan pH Down tidak mewakili tandon -> pH Down TIDAK didosis siklus ini (diukur saja, cek besok).
          // Dulu pH Down tetap didosis di atas bacaan itu: aduk 15 mnt + jeda 2 mnt -> pH jatuh ke ~3.5 sesudah tercampur.
          if(responLambatSiklus) dosingLewatiPH = true;
        } else {
          // Cek lanjutan (mikro / PI) setelah kanal terpicu: dosing terus menuju target (tengah batasan)
          dosingLewatiPH = (dosingPHHasilCek <= terpilihPH);
          if(dosingTujuanSetelahCekPH==DOSING_MICRO_PH){   // cek respon OPEN LOOP pH setelah makro
            { float vol = dosingTotalPHmL - olVolAwalPH; int pa = ppmAcuanPHDown();   // TES_130/137
              byte r = nilaiResponPH(olAcuanPH, dosingPHHasilCek, vol, pa, prediksiTurunPH(olAcuanPH, vol, pa));
              if(r!=RESPON_TUNDA) makroPulih &= ~(MP_PH|MP_PH_ULANG);   // TES_138: makro sudah dinilai
              if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_PH_RESPON; kanalTerkunci |= KUNCI_PH; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); dosingLewatiPH = true; }
              else if(r==RESPON_LAMBAT){ responLambatSiklus = true; dosingLewatiPH = true; } }
          }
        }

        if(dosingTujuanSetelahCekPH==DOSING_PI_POMPA_PH){
          // CLOSED-LOOP pH: error positif = pH masih di atas target -> perlu pH Down
          float errorPH = dosingPHHasilCek - terpilihPH;
          // TES_130: cek respon MAKRO pH Closed Loop di cek PI pertama (sensor pH macet -> dulu 3 pulsa PI tambahan,
          // pH larutan jatuh jauh di bawah batas).
          bool hentiLambat = false;                  // TES_137
          if(cekMakroPHTertunda){
            cekMakroPHTertunda = false;
            float vol = dosingTotalPHmL - olVolAwalPH; int pa = ppmAcuanPHDown();
            byte r = nilaiResponPH(olAcuanPH, dosingPHHasilCek, vol, pa, prediksiTurunPH(olAcuanPH, vol, pa));
            if(r!=RESPON_TUNDA) makroPulih &= ~(MP_PH|MP_PH_ULANG);   // TES_138: makro sudah dinilai
            if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_PH_RESPON; kanalTerkunci |= KUNCI_PH; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); }
            else if(r==RESPON_TUNDA) cekMakroPHTertunda = true;   // dinilai lagi di ukur ulang
            else if(r==RESPON_LAMBAT){ responLambatSiklus = true; hentiLambat = true; }
          }
          // Cek respon tiap 3 pulsa (penurunan pH terukur vs jumlah prediksi)
          if(resPH.n >= RESPON_JENDELA && !ulangResponPH){
            // TES_130: dinilai di ruang volume; resPH.prediksi = syarat dosis cukup besar
            byte r = nilaiResponPH(resPH.acuan, dosingPHHasilCek, resPH.vol, ppmAcuanPHDown(), resPH.prediksi);
            if(r==RESPON_KUNCI){ piFlagSiklus |= PI_FLAG_PH_RESPON; kanalTerkunci |= KUNCI_PH; mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS); }
            else if(r==RESPON_LAMBAT){ responLambatSiklus = true; hentiLambat = true; }
            if(r!=RESPON_TUNDA){ resPH.n = 0; resPH.prediksi = 0.0f; resPH.vol = 0.0f; }   // dipertahankan utk ukur ulang
          }
          if((piFlagSiklus & PI_FLAG_PH_RESPON) || hentiLambat){
            piPHSelesai = true;
          } else if(ulangResponPH){
            // TES_137: respon < 70% -> tanpa pulsa; homogenisasi ulang lalu ukur lagi
          } else if(errorPH <= PI_TOL_PH || piIterasiPH >= PI_MAKS_ITERASI){
            piPHSelesai = true;
            if(errorPH > PI_TOL_PH) piFlagSiklus |= PI_FLAG_PH_ITERASI;
          } else {
            // Gain efektif dari persamaan pH Down pada rentang pH sekarang -> target & PPM TERUKUR siklus ini
            int   ppmAcuanPI = ppmAcuanPHDown();
            float gEf   = gainEfektifPH(dosingPHHasilCek, ppmAcuanPI);
            float tModel= waktuModelPH(dosingPHHasilCek, terpilihPH, ppmAcuanPI);   // detik P1 utk seluruh error
            PIController piPHAktif;
            // TES_122: fase pH (atas/bawah 5.85) berganti -> integral fase lama tak relevan dgn Kp/Ki fase baru -> reset
            int8_t fasePH = (dosingPHHasilCek >= batasFasePH) ? 1 : 0;
            if(piFasePHTerakhir>=0 && fasePH!=piFasePHTerakhir) piPH.integral = 0.0f;
            piFasePHTerakhir = fasePH;
            piGainPHTerakhir = gEf;
            piPHAktif.integral = piPH.integral;   // integral pH dilacak di piPH (di-reset piReset() tiap siklus)
            if(dosingMode==4){
              bool fasaAtas = (dosingPHHasilCek >= batasFasePH);
              piPHAktif.kp = fasaAtas ? piPHKpAtas_ZN : piPHKpBawah_ZN;
              piPHAktif.ki = fasaAtas ? piPHKiAtas_ZN : piPHKiBawah_ZN;
              float fz = faktorSkalaVolZN(false) * faktorSkalaPPMZNPada(fasaAtas ? 1 : 2, false, ppmAcuanPI);   // TES_148: skala ke PPM TERUKUR
              piPHAktif.kp *= fz; piPHAktif.ki *= fz;
              batasAmanKpKiZN(piPHAktif.kp, piPHAktif.ki, gEf);   // batas aman ZN ikut rentang pH & PPM
            } else {
              // CL Auto-Tune: Kp = 0.5/g_ef, Ki = 0.1·Kp (dihitung ulang tiap iterasi; piPH.kp TIDAK ditimpa -> EEPROM tenang)
              piPHAktif.kp = (gEf>0.0f) ? (PI_FRAKSI_KOREKSI/gEf) : 0.0f;
              piPHAktif.ki = PI_RASIO_KI * piPHAktif.kp;
            }
            float uMaks = (tModel>0.0f) ? tModel : (float)PUMP_MAKS_DETIK[3];
            if(uMaks > (float)PUMP_MAKS_DETIK[3]) uMaks = (float)PUMP_MAKS_DETIK[3];
            piPulsaDetikPH = piHitungPulsa(errorPH, piPHAktif, uMaks);
            piPH.integral = piPHAktif.integral;   // simpan balik integral yg sudah bertambah
            if(piPulsaDetikPH <= 0.0f) piPHSelesai = true;    // sisa error < resolusi 1 dtk pompa -> selesai tanpa pulsa
            else {
              piIterasiPH++;
              if(resPH.n==0) resPH.acuan = dosingPHHasilCek;   // awal jendela respon baru
            }
          }
        } else if(ulangResponPH){
          // TES_137: Open Loop makro pH < 70% -> mikro ditahan, ukur ulang dulu
        } else if(!dosingLewatiPH){
          int ppmAcuan = ppmAcuanPHDown();
          float total = hitungPHDownVolume(dosingPHHasilCek, terpilihPH, ppmAcuan);
          float vMaksP1 = (debitMLdtk(3)>0.0f) ? (float)PUMP_MAKS_DETIK[3]*debitMLdtk(3) : 1e9f;   // TES_138: = batas saat pompa
          if(dosingTujuanSetelahCekPH==DOSING_MACRO_PH)
            dosingPHDownTahap1 = min(total * ((float)persenMakro/100.0f), vMaksP1);
          else
            dosingPHDownTahap2 = min(total, vMaksP1);       // DOSING_MICRO_PH (open-loop)
        } else {
          dosingPHDownTahap1 = 0.0f;
          dosingPHDownTahap2 = 0.0f;
        }
        sedangUlangPH = false;   // TES_137
        dosingFaseTimer = millis();
      }
    }
  }
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPH();
  if(dosingPHTerkunci && (millis()-dosingFaseTimer >= 3000UL)){
    if(ulangResponPH){ mulaiCekUlangResponPH(); return; }   // TES_137
    if(ulangPulihPH){ ulangPulihPH = false; mulaiCekUlangRespon(DOSING_HOMOGEN3, DOSING_CEK_PH1); return; }   // TES_138
    if(dosingTujuanSetelahCekPH==DOSING_PI_POMPA_PH){
      if(piPHSelesai){
        // PI kanal pH selesai -> cek akhir (TES_138: siklus tanpa dosis sama sekali -> selesai tanpa dosing,
        // dulu tercatat "DOSING SELESAI" 0 mL & menimpa catatan dosing terakhir)
        prosesLewatiTahapPH();
      } else {
        siapkanFasePompaPiPH();
      }
    }
    else if(dosingLewatiPH) prosesLewatiTahapPH();
    else                    siapkanFasePompaPH();
  }
}

// Melewati tahap dosing pH Down (pH dalam/di bawah batasan, atau target tercapai).
// Bila siklus ini sama sekali tidak mendosis -> selesai langsung (cek akhir hanya mengulang bacaan yang sama).
void prosesLewatiTahapPH() {
  if(!dosingSudahAdaDosis){ selesaikanSiklusCek(false); return; }
  mulaiCekAkhir();
  currentState = DOSING_CEK_AKHIR;
  if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingCekAkhir(); }
}

// PPM acuan persamaan pH Down = PPM larutan SAAT pH Down didosis = PPM terukur terakhir siklus ini.
// Closed Loop: hasil cek PI terakhir (sudah mendekati target, tanpa pulsa sesudahnya).
// Open Loop: hasil Cek PPM 3 setelah mikro AB (TES_124); tanpa dosis AB = hasil cek PPM pertama.
int ppmAcuanPHDown(){
  return (statusPPMTerakhir>=0) ? statusPPMTerakhir : ppmTerbaca;
}

// TES_138: batas aman volume per sisi pasangan pompa AB (sama dgn yg diterapkan saat pompa mulai)
float batasVolSisiAB(int idxA, int idxB){
  float qA = debitMLdtk(idxA), qB = debitMLdtk(idxB);
  if(qA<=0.0f || qB<=0.0f) return 1e9f;
  return min((float)PUMP_MAKS_DETIK[idxA]*qA, (float)PUMP_MAKS_DETIK[idxB]*qB);
}
// ── Menyiapkan & memulai fase pompa AB Mix ──
void siapkanFasePompaAB() {
  bool adaSebelum = dosingSudahAdaDosis;
  float volumeTotal;
  int idxA, idxB;
  MenuState nextState;
  if(dosingTujuanSetelahCekPPM==DOSING_MACRO_AB){
    volumeTotal = dosingABMixTahap1;
    idxA=1; idxB=2; dosingPumpLabel="Pompa D2-D3 Aktif:"; // fase makro SELALU diafragma D2&D3
    nextState=DOSING_MACRO_AB;
    olAcuanAB = (float)dosingPPMHasilCek; olVolAwalAB = dosingTotalABmL;   // acuan cek respon makro (OL & CL)
    cekMakroABTertunda = (dosingMode==3 || dosingMode==4);                    // CL: dinilai di cek PI pertama
  } else {
    // DOSING_MICRO_AB (open-loop, tahap sisa (100 - makro)%)
    volumeTotal = dosingABMixTahap2;
    idxA=4; idxB=5; dosingPumpLabel="Pompa P2-P3 Aktif:"; nextState=DOSING_MICRO_AB;
    volAwalMikroAB = dosingTotalABmL;
    olAcuanMikroAB = (float)dosingPPMHasilCek;   // TES_130: acuan cek respon mikro (P2-P3) di Cek PPM 3
  }
  {
    // Batas aman diterapkan pada VOLUME per sisi (bukan durasi tiap pompa sendiri-sendiri)
    // -> bila terpotong, A & B tetap sama banyak (rasio 1:1 terjaga).
    float qA = debitMLdtk(idxA), qB = debitMLdtk(idxB);
    if(qA>0.0f && qB>0.0f){
      float vMaks = min((float)PUMP_MAKS_DETIK[idxA]*qA, (float)PUMP_MAKS_DETIK[idxB]*qB);
      if(volumeTotal > vMaks) volumeTotal = vMaks;
    }
    // TES_124: target tiap sisi = rata-rata volume A/B yg SUDAH keluar + dosis ini -> selisih pembulatan 1 dtk
    // (debit A != B) dikompensasi pada dosis berikutnya, A & B tetap seimbang. (Kekurangan/kelebihan rata-rata
    // tidak dikompensasi di sini karena sudah terbaca sensor & masuk ke perhitungan dosis.)
    // Dosis < 0,5 dtk TIDAK dipaksa 1 dtk (aktuator satu arah: dosis minimum bisa melewati batas).
    float abTargetSisi = 0.5f*(abVolA + abVolB) + volumeTotal;
    long tA = (qA>0.0f) ? (long)round(fmax(0.0f, abTargetSisi - abVolA)/qA) : 0;
    long tB = (qB>0.0f) ? (long)round(fmax(0.0f, abTargetSisi - abVolB)/qB) : 0;
    if(tA > PUMP_MAKS_DETIK[idxA]) tA = PUMP_MAKS_DETIK[idxA];  // pengaman pembulatan
    if(tB > PUMP_MAKS_DETIK[idxB]) tB = PUMP_MAKS_DETIK[idxB];
    dosingPump1DetikTarget = (int)tA;
    dosingPump2DetikTarget = (int)tB;
  }
  dosingPumpDetikTarget  = max(dosingPump1DetikTarget, dosingPump2DetikTarget);
  dosingSudahAdaDosis = adaSebelum || (dosingPumpDetikTarget>0);   // hanya bila benar-benar ada yg dipompa
  dosingPumpRelayA = pumpRelayPin[idxA];
  dosingPumpRelayB = pumpRelayPin[idxB];
  dosingPumpIdxA = idxA; dosingPumpIdxB = idxB;
  dosingPump1Selesai = (dosingPump1DetikTarget<=0);
  dosingPump2Selesai = (dosingPump2DetikTarget<=0);
  dosingPumpDetikBerjalan = 0;
  if(!dosingPump1Selesai) relayOn(dosingPumpRelayA);
  if(!dosingPump2Selesai) relayOn(dosingPumpRelayB);
  currentState = nextState;
  if(currentState==DOSING_MACRO_AB) catatMakroPulih();   // TES_139: makro mulai -> acuan tersimpan
  if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingPompaAktif(); }
}

// ── Menyiapkan & memulai fase pompa pH Down ──
void siapkanFasePompaPH() {
  bool adaSebelum = dosingSudahAdaDosis;
  float volumeTotal;
  String labelPompa;
  MenuState nextState;
  if(dosingTujuanSetelahCekPH==DOSING_MACRO_PH){
    volumeTotal = dosingPHDownTahap1; labelPompa="Pompa P1 Aktif:"; nextState=DOSING_MACRO_PH;
    olAcuanPH = dosingPHHasilCek; olVolAwalPH = dosingTotalPHmL;
    cekMakroPHTertunda = (dosingMode==3 || dosingMode==4);
    // (TES_130: prediksi respon makro pH memakai prediksiTurunPH berfase, bukan gain rata-rata)
  } else {
    // DOSING_MICRO_PH (open-loop, tahap sisa (100 - makro)%)
    volumeTotal = dosingPHDownTahap2; labelPompa="Pompa P1 Aktif:"; nextState=DOSING_MICRO_PH;
  }
  {
    float qP = debitMLdtk(3);
    if(qP>0.0f && volumeTotal > (float)PUMP_MAKS_DETIK[3]*qP) volumeTotal = (float)PUMP_MAKS_DETIK[3]*qP;
  }
  dosingPumpLabel = labelPompa;
  {
    long tP = (long)round(hitungDurasiPompaDetik(volumeTotal, 3));
    // TES_124: dosis < 0,5 dtk = 0 (tidak dipaksa 1 dtk). Di bawah pH 5.85, 1 dtk P1 bisa > 0,3 pH -> lewat batas.
    if(tP > PUMP_MAKS_DETIK[3]) tP = PUMP_MAKS_DETIK[3];  // batas aman P1 (pH down)
    dosingPump1DetikTarget = (int)tP;
  }
  dosingPump2DetikTarget = 0;
  dosingPumpDetikTarget  = dosingPump1DetikTarget;
  dosingSudahAdaDosis = adaSebelum || (dosingPumpDetikTarget>0);
  dosingPumpRelayA = pumpRelayPin[3];
  dosingPumpRelayB = -1;
  dosingPumpIdxA = 3; dosingPumpIdxB = -1;
  dosingPump1Selesai = (dosingPump1DetikTarget<=0);
  dosingPump2Selesai = true;
  dosingPumpDetikBerjalan = 0;
  if(!dosingPump1Selesai) relayOn(dosingPumpRelayA);
  currentState = nextState;
  if(currentState==DOSING_MACRO_PH) catatMakroPulih();   // TES_139
  if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingPompaAktif(); }
}

// ── Proses berjalannya pompa dosing ─────────────────────────
void prosesFasePompa() {
  // TES_122: rekap volume = volume yang BENAR-BENAR keluar (detik nyala x debit), dihitung tiap detik pompa menyala.
  // AB Mix dicatat per sisi (rata-rata A & B, keduanya sama banyak); pH Down = P1.
  {
    bool fasePH = (currentState==DOSING_MACRO_PH || currentState==DOSING_MICRO_PH || currentState==DOSING_PI_POMPA_PH);
    float vA = (!dosingPump1Selesai && dosingPumpIdxA>=0) ? debitMLdtk(dosingPumpIdxA) : 0.0f;
    float vB = (!dosingPump2Selesai && dosingPumpIdxB>=0 && dosingPumpRelayB>=0) ? debitMLdtk(dosingPumpIdxB) : 0.0f;
    if(fasePH) dosingTotalPHmL += vA;
    else     { dosingTotalABmL += 0.5f*(vA + vB); abVolA += vA; abVolB += vB; }
  }
  dosingPumpDetikBerjalan++;
  if((currentState==DOSING_MACRO_AB || currentState==DOSING_MACRO_PH) && (dosingPumpDetikBerjalan%15)==0) catatMakroPulih();   // TES_139
  if(!dosingPump1Selesai && dosingPumpDetikBerjalan>=dosingPump1DetikTarget){
    relayOff(dosingPumpRelayA); dosingPump1Selesai=true;
  }
  if(!dosingPump2Selesai && dosingPumpRelayB>=0 && dosingPumpDetikBerjalan>=dosingPump2DetikTarget){
    relayOff(dosingPumpRelayB); dosingPump2Selesai=true;
  }
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingPompaAktif();
  bool pompaSelesai = dosingPump1Selesai && (dosingPumpRelayB<0 || dosingPump2Selesai);
  if(pompaSelesai){
    MenuState homogenState, tujuanSetelah;
    catatMakroPulih();   // TES_138: makro selesai dipompa -> simpan acuan & prediksi (dinilai walau daya padam)
    if(currentState==DOSING_MACRO_AB){
      homogenState=DOSING_HOMOGEN1;
      if(dosingMode==3||dosingMode==4) tujuanSetelah = DOSING_PI_CEK_PPM;   // Closed-Loop (Model/ZN): masuk loop PI AB
      else                   tujuanSetelah = DOSING_CEK_PPM2;     // Open Loop: lanjut mikro
    }
    else if(currentState==DOSING_MICRO_AB){
      homogenState=DOSING_HOMOGEN2; tujuanSetelah=DOSING_CEK_PPM3;   // TES_124: cek PPM akhir AB dulu (acuan pH Down)
    }
    else if(currentState==DOSING_PI_POMPA_AB){
      homogenState=DOSING_PI_HOMOGEN_AB; tujuanSetelah=DOSING_PI_CEK_PPM;
    }
    else if(currentState==DOSING_MACRO_PH){
      if(dosingMode==3||dosingMode==4){ homogenState=DOSING_HOMOGEN3; tujuanSetelah=DOSING_PI_CEK_PH; }
      else             { homogenState=DOSING_HOMOGEN3; tujuanSetelah=DOSING_CEK_PH2;  }
    }
    else if(currentState==DOSING_MICRO_PH){
      homogenState=DOSING_HOMOGEN4; tujuanSetelah=DOSING_CEK_AKHIR;
    }
    else { // DOSING_PI_POMPA_PH
      homogenState=DOSING_PI_HOMOGEN_PH; tujuanSetelah=DOSING_PI_CEK_PH;
    }
    dosingHomogenDetikBerjalan = 0;
    dosingTujuanSetelahHomogen = tujuanSetelah;
    currentState = homogenState;
    // TES_124: dosis < 0,5 dtk -> tidak ada yang dipompa -> jeda homogenisasi tak perlu ditunggu
    if(dosingPumpDetikTarget<=0) dosingHomogenDetikBerjalan = jedaUntukState(homogenState)*60 - 1;
    if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingHomogenisasi(); }
  }
}

// ── Proses homogenisasi ──
// TES_137: respon < 70% -> homogenisasi ulang (jeda tahap yg sama) lalu cek yang sama diulang, tanpa dosis
void mulaiCekUlangRespon(MenuState homogen, MenuState tujuan){
  dosingHomogenDetikBerjalan = 0;
  dosingTujuanSetelahHomogen = tujuan;
  currentState = homogen;
  if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingHomogenisasi(); }
}
void mulaiCekUlangResponAB(){
  ulangResponAB = false; sedangUlangAB = true;
  if(dosingTujuanSetelahCekPPM==DOSING_MICRO_AB)      mulaiCekUlangRespon(DOSING_HOMOGEN1, DOSING_CEK_PPM2);   // OL sesudah makro
  else if(dosingTujuanSetelahCekPPM==DOSING_CEK_PPM3) mulaiCekUlangRespon(DOSING_HOMOGEN2, DOSING_CEK_PPM3);   // OL sesudah mikro
  else mulaiCekUlangRespon(cekMakroABTertunda ? DOSING_HOMOGEN1 : DOSING_PI_HOMOGEN_AB, DOSING_PI_CEK_PPM);    // CL
}
void mulaiCekUlangResponPH(){
  ulangResponPH = false; sedangUlangPH = true;
  if(dosingTujuanSetelahCekPH==DOSING_MICRO_PH) mulaiCekUlangRespon(DOSING_HOMOGEN3, DOSING_CEK_PH2);          // OL sesudah makro
  else mulaiCekUlangRespon(cekMakroPHTertunda ? DOSING_HOMOGEN3 : DOSING_PI_HOMOGEN_PH, DOSING_PI_CEK_PH);    // CL
}
void prosesHomogenisasi() {
  dosingHomogenDetikBerjalan++;
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingHomogenisasi();
  int jedaMenit = jedaUntukState(currentState);
  if(dosingHomogenDetikBerjalan >= jedaMenit*60){
    MenuState tujuan = dosingTujuanSetelahHomogen;
    if((!statusOverlayAktif && !cancelConfirmAktif)) lcd.clear();
    if(tujuan==DOSING_CEK_PPM1){            // TES_138: ukur ulang cek pertama (makro sebelum daya padam)
      mulaiCekPPM(DOSING_MACRO_AB);
      currentState=DOSING_CEK_PPM1;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPPM();
    } else if(tujuan==DOSING_CEK_PPM2){
      mulaiCekPPM(DOSING_MICRO_AB);
      currentState=DOSING_CEK_PPM2;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPPM();
    } else if(tujuan==DOSING_PI_CEK_PPM){
      mulaiCekPPM(DOSING_PI_POMPA_AB);      // cek berikutnya di dalam loop PI AB
      currentState=DOSING_PI_CEK_PPM;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPPM();
    } else if(tujuan==DOSING_CEK_PPM3){
      mulaiCekPPM(DOSING_CEK_PPM3);         // ukur saja (bilas D1 -> stabil -> median), tanpa dosis
      currentState=DOSING_CEK_PPM3;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPPM();
    } else if(tujuan==DOSING_CEK_PH1){
      mulaiCekPH(DOSING_MACRO_PH);
      currentState=DOSING_CEK_PH1;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPH();
    } else if(tujuan==DOSING_CEK_PH2){
      mulaiCekPH(DOSING_MICRO_PH);
      currentState=DOSING_CEK_PH2;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPH();
    } else if(tujuan==DOSING_PI_CEK_PH){
      mulaiCekPH(DOSING_PI_POMPA_PH);       // cek berikutnya di dalam loop PI pH
      currentState=DOSING_PI_CEK_PH;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekPH();
    } else if(tujuan==DOSING_CEK_AKHIR){
      mulaiCekAkhir();
      currentState=DOSING_CEK_AKHIR;
      if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekAkhir();
    }
  }
}

// ── Fase Pengecekan Akhir PPM & pH: BILAS -> STABILISASI -> UKUR ──
void mulaiCekAkhir() {
  nyalakanRelayTDS();                 // TAHAP 1: ukur PPM (daya TDS ON) -> stabilisasi 5 dtk dulu
  cekAkhirTahap = 0;
  dosingPPMTerkunci=false; dosingPHTerkunci=false;
  cekSensorFase=0;
  dosingCekDetikBerjalan=0;
  dosingPPMSum=0; dosingPPMCount=0;
  dosingPHSum=0.0f; dosingPHCount=0;
  resetPpmRoll15(); resetPhRoll15();
}

// Cek akhir (hanya bila siklus mendosis): ukur PPM lalu pH, kemudian selesaikanSiklusCek(true)
void prosesCekAkhir() {
  // Validitas dicek per tahap; fase 0 (stabilisasi daya sensor) dikecualikan
  if(cekSensorFase>0){
    if(cekAkhirTahap==0 && !tdsSensorValid){ hentikanDosingSensorInvalid(); return; }
    if(cekAkhirTahap==1 && !phSensorValid){  hentikanDosingSensorInvalid(); return; }
  }
  dosingCekDetikBerjalan++;
  if(cekSensorFase==0){                         // STABILISASI DAYA SENSOR (relay TDS baru ON/OFF)
    if(dosingCekDetikBerjalan >= JEDA_STABIL_SENSOR_DETIK){
      primingBufferSensorSelektif(cekAkhirTahap==0, cekAkhirTahap==1);   // TES_138: buffer diisi bacaan sesudah stabil
      relayOn(PIN_RELAY_D1);
      cekSensorFase=1; dosingCekDetikBerjalan=0;
    }
  } else if(cekSensorFase==1){                  // MEMBILAS (D1 ON)
    if(dosingCekDetikBerjalan >= terpilihLamaCekSensor){
      relayOff(PIN_RELAY_D1);
      cekSensorFase=2; dosingCekDetikBerjalan=0;
    }
  } else if(cekSensorFase==2){                  // STABILISASI LARUTAN (D1 OFF)
    if(dosingCekDetikBerjalan >= terpilihJedaStabilisasi){
      cekSensorFase=3; dosingCekDetikBerjalan=0;
      if(cekAkhirTahap==0) resetPpmRoll15(); else resetPhRoll15();
    }
  } else {                                      // UKUR (median 20 detik terakhir)
    if(cekAkhirTahap==0){
      sampelPpmRoll15();
      if(dosingCekDetikBerjalan >= ROLL_MED_N && !dosingPPMTerkunci){
        dosingPPMTerkunci = true;
        statusPPMTerakhir = medianPpmRoll15();
        // TAHAP 1 (PPM) selesai -> daya TDS OFF, lanjut TAHAP 2 (pH) dgn durasi input yang sama
        matikanRelayTDS();
        cekAkhirTahap = 1;
        cekSensorFase = 0;
        dosingCekDetikBerjalan = 0;
        resetPhRoll15();
      }
    } else {
      sampelPhRoll15();
      if(dosingCekDetikBerjalan >= ROLL_MED_N && !dosingPHTerkunci){
        dosingPHTerkunci = true;
        statusPHTerakhir = medianPhRoll15();
      }
    }
  }
  if((!statusOverlayAktif && !cancelConfirmAktif)) tampilkanDosingCekAkhir();
  if(dosingPPMTerkunci && dosingPHTerkunci){
    selesaikanSiklusCek(true);   // catat hasil cek, jadwalkan cek berikutnya, tampilkan rekap
  }
}

// ============================================================
// KONTROL PI (MODE CLOSED-LOOP) — anti-windup model "clamping"
// ============================================================
// Output PI = detik pulsa pompa peristaltik. Integral HANYA diakumulasi bila
// output tidak sedang jenuh (ter-clamp) -> mencegah integral windup.
// Batas aman Kp ZN SAAT BERJALAN: gain model berubah bila PPM target / volume / debit berubah setelah uji reaksi
// (mis. slope pH turun ~3x antara 200 dan 2000 PPM) -> Kp lama bisa melewati batas anti-overshoot.
// kanal: 0 = PPM, 1 = pH fase atas, 2 = pH fase bawah (pemetaan sama dgn Uji Reaksi).
float kpMaksModel(int kanal, bool baru){   // batas aman Kp (anti overshoot) = FRAKSI_AMAN_ZN / gain model; 0 bila gain tak valid
  float g = hitungGainModel(kanal, baru);
  return (g>0.0f) ? (FRAKSI_AMAN_ZN/g) : 0.0f;
}
void batasiKpZNDenganGainModel(PIController &pi, int kanal){   // proses dosing selalu memakai Parameter Aktif
  float f = faktorSkalaZN(kanal, false); pi.kp *= f; pi.ki *= f;   // TES_141: skala acuan uji -> Parameter Aktif
  batasAmanKpKiZN(pi.kp, pi.ki, hitungGainModel(kanal, false));
}
// uMaks = batas pulsa iterasi ini (detik) = waktu model utk menutup SELURUH error, dibatasi batas aman pompa.
// Melewati uMaks berarti PI meminta dosis lebih besar dari yang menurut persamaan dibutuhkan -> pasti overshoot.
float piHitungPulsa(float error, PIController &pi, float uMaks){
  if(error <= 0.0f) return 0.0f;            // aktuator satu arah: hanya bisa menambah
  // TES_130: kanal dianggap SELESAI bila menurut MODEL sisa error < kenaikan 1 dtk pompa (uMaks = waktu model utk
  // menutup seluruh error < 1 dtk) -> pulsa minimum 1 dtk tak dipaksa (cegah overshoot, aktuator satu arah).
  // Dulu syaratnya Kp·e < 0,5 dtk: utk Kp Auto-Tune (0,5/g) hasilnya sama, tetapi utk Kp PI ZN yang lebih kecil
  // kanal berhenti padahal error masih beberapa kali kenaikan 1 dtk (bisa di luar batasan) & integral tak sempat
  // bekerja. Kini Kp ZN kecil tetap memberi pulsa minimum 1 dtk selama model butuh >= 1 dtk.
  if(uMaks < 2.0f*PI_DEADBAND_DETIK) return 0.0f;
  float uTanpaBatas = pi.kp*error + pi.ki*pi.integral;
  if(!(uTanpaBatas > 0.0f)) return 0.0f;   // TES_144: Kp = Ki = 0 (kanal ZN kosong) / NaN -> selesai tanpa dosis (dulu dipaksa 1 dtk)
  float u = constrain(uTanpaBatas, 0.0f, uMaks);
  if(u == uTanpaBatas) pi.integral += error;   // tidak jenuh -> integral boleh bertambah
  if(u < (float)PI_PULSA_MIN_DETIK) u = (float)PI_PULSA_MIN_DETIK; // pulsa minimum pompa (<= uMaks krn uMaks >= 1 dtk)
  // TES_138: pompa ber-resolusi 1 dtk -> dibulatkan DI SINI & tak boleh melewati uMaks (dulu round() di pemanggil:
  // 1,54 dtk -> 2 dtk = melewati waktu model utk seluruh error -> overshoot pada tandon kecil / gain besar).
  float uBulat = (float)round(u);
  if(uBulat > uMaks) uBulat = floor(uMaks);
  if(uBulat < (float)PI_PULSA_MIN_DETIK) uBulat = (float)PI_PULSA_MIN_DETIK;
  return uBulat;
}
void piReset(){
  piPPM.integral = 0.0f; piPH.integral = 0.0f; piPPM_ZN.integral = 0.0f;   // integral ZN juga wajib direset tiap siklus
  piIterasiPPM = 0; piIterasiPH = 0;
  piPPMSelesai = false; piPHSelesai = false;
  resAB.n = 0; resAB.prediksi = 0.0f; resPH.n = 0; resPH.prediksi = 0.0f; resPH.vol = 0.0f;
  abVolA = 0.0f; abVolB = 0.0f;
  piFlagSiklus = 0; piFasePHTerakhir = -1; makroABDilewati = false;
  cekMakroABTertunda = false; cekMakroPHTertunda = false;
  ulangResponAB = ulangResponPH = sedangUlangAB = sedangUlangPH = false; responLambatSiklus = false;   // TES_137
  piPulsaDetikAB = 0.0f; piPulsaDetikPH = 0.0f;
  dosingTotalABmL = 0.0f; dosingTotalPHmL = 0.0f;   // reset akumulasi volume utk siklus baru
}

// ── Pulsa PI untuk AB Mix: peristaltik P2 & P3 (mode Closed-Loop) ──
void siapkanFasePompaPiAB(){
  long tP = (long)round(piPulsaDetikAB);
  if(tP < PI_PULSA_MIN_DETIK) tP = PI_PULSA_MIN_DETIK;
  // Volume per sisi = pulsa nominal x debit rata-rata P2/P3; tiap pompa dijalankan selama yg diperlukan utk
  // volume itu -> A : B tetap sama walau debit P2 dan P3 berbeda (konsisten dgn makro).
  float qP2 = debitMLdtk(4), qP3 = debitMLdtk(5);   // mL/dtk
  float vSisi = (float)tP * 0.5f*(qP2+qP3);
  // Batas aman pompa diterapkan pada VOLUME per sisi (bukan per pompa) -> bila terpotong, A & B tetap sama
  float vMaks = min((float)PUMP_MAKS_DETIK[4]*qP2, (float)PUMP_MAKS_DETIK[5]*qP3);
  if(vSisi > vMaks) vSisi = vMaks;
  // Target per sisi (lihat siapkanFasePompaAB): selisih pembulatan A/B dikompensasi pulsa berikutnya
  float abTargetSisi = 0.5f*(abVolA + abVolB) + vSisi;
  long tP2 = (qP2>0.0f) ? (long)round(fmax(0.0f, abTargetSisi - abVolA)/qP2) : tP;
  long tP3 = (qP3>0.0f) ? (long)round(fmax(0.0f, abTargetSisi - abVolB)/qP3) : tP;
  if(tP2 < PI_PULSA_MIN_DETIK && tP3 < PI_PULSA_MIN_DETIK){ tP2 = PI_PULSA_MIN_DETIK; tP3 = PI_PULSA_MIN_DETIK; }
  if(tP2 > PUMP_MAKS_DETIK[4]) tP2 = PUMP_MAKS_DETIK[4];
  if(tP3 > PUMP_MAKS_DETIK[5]) tP3 = PUMP_MAKS_DETIK[5];
  dosingPumpLabel = "P2-P3 " + String(piIterasiPPM) + "/" + String(PI_MAKS_ITERASI) + " Aktif";
  // Prediksi kenaikan PPM pulsa ini (dari volume yg benar-benar dijalankan) -> jendela cek respon
  if(terpilihVolume>0) resAB.prediksi += slopeABMixTerpilih * (0.5f*((float)tP2*qP2 + (float)tP3*qP3)) / (float)terpilihVolume;
  resAB.n++;
  dosingPump1DetikTarget = (int)tP2;
  dosingPump2DetikTarget = (int)tP3;   // P2 & P3 pulsa bersamaan (A & B sekaligus)
  dosingPumpDetikTarget  = (int)max(tP2,tP3);
  dosingPumpRelayA = pumpRelayPin[4];
  dosingPumpRelayB = pumpRelayPin[5];
  dosingPumpIdxA = 4; dosingPumpIdxB = 5;
  // TES_129: sisi yang target detiknya 0 (sudah unggul dari penyeimbangan A/B) tidak dinyalakan sama sekali
  // (dulu tetap menyala ~1 dtk sebelum dimatikan pada detik pertama -> A/B tak seimbang & tak sesuai prediksi)
  dosingPump1Selesai = (tP2<=0);
  dosingPump2Selesai = (tP3<=0);
  dosingPumpDetikBerjalan = 0;
  if(!dosingPump1Selesai) relayOn(dosingPumpRelayA);
  if(!dosingPump2Selesai) relayOn(dosingPumpRelayB);
  dosingSudahAdaDosis = true;
  currentState = DOSING_PI_POMPA_AB;
  if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingPompaAktif(); }
}

// ── Pulsa PI untuk pH Down: peristaltik P1 (mode Closed-Loop) ──
void siapkanFasePompaPiPH(){
  long tP = (long)round(piPulsaDetikPH);
  if(tP < PI_PULSA_MIN_DETIK) tP = PI_PULSA_MIN_DETIK;
  if(tP > PUMP_MAKS_DETIK[3]) tP = PUMP_MAKS_DETIK[3];   // batas aman P1
  dosingPumpLabel = "P1 " + String(piIterasiPH) + "/" + String(PI_MAKS_ITERASI) + " Aktif";
  // TES_130: prediksi penurunan pH pulsa ini dgn model berfase dari pH terukur iterasi ini (dulu gain rata-rata x detik)
  resPH.prediksi += prediksiTurunPH(dosingPHHasilCek, (float)tP * debitMLdtk(3), ppmAcuanPHDown());
  resPH.vol      += (float)tP * debitMLdtk(3);
  resPH.n++;
  dosingPump1DetikTarget = (int)tP;
  dosingPump2DetikTarget = 0;
  dosingPumpDetikTarget  = (int)tP;
  dosingPumpRelayA = pumpRelayPin[3];
  dosingPumpRelayB = -1;
  dosingPumpIdxA = 3; dosingPumpIdxB = -1;
  dosingPump1Selesai = false;
  dosingPump2Selesai = true;
  dosingPumpDetikBerjalan = 0;
  relayOn(dosingPumpRelayA);
  dosingSudahAdaDosis = true;
  currentState = DOSING_PI_POMPA_PH;
  if((!statusOverlayAktif && !cancelConfirmAktif)){ lcd.clear(); tampilkanDosingPompaAktif(); }
}

// Slot jeda homogenisasi untuk tiap state (PI memakai slot micro: 1 & 3)
int jedaUntukState(MenuState s){
  if     (s==DOSING_HOMOGEN1)                          return dosingJedaHomogenisasi[0];
  else if(s==DOSING_HOMOGEN2 || s==DOSING_PI_HOMOGEN_AB) return dosingJedaHomogenisasi[1];
  else if(s==DOSING_HOMOGEN3)                          return dosingJedaHomogenisasi[2];
  else if(s==DOSING_HOMOGEN4 || s==DOSING_PI_HOMOGEN_PH) return dosingJedaHomogenisasi[3];
  return 0;
}

// Menerapkan pilihan jenis dosing ke Parameter Aktif / Baru sesuai konteks
// (TES_118: ganti jenis tidak lagi mereset Jeda Homogenisasi — semua jenis memakai 4 slot jeda yang sama)
void terapkanJenisDosing(int mode, int persen){
  siapkanDraftBaru();
  if(dosingSedangMenunggu){
    dosingModeBaru = mode;
    persenMakroBaru = persen;
    jenisDosingBaruSudahDiatur = true;
  } else {
    dosingMode = mode;
    persenMakro = persen;
    jenisDosingSudahDiatur = true;
  }
}

// TES_128: volume ringkas "AB 950 mL" / "AB 1.25 L" / "AB 12.5 L" / "AB 125 L" (maks 9 karakter per sisi) -> dua sisi
// selalu muat di 20 kolom (dulu "AB 13636 mL" + "pH 450 mL" = 21 kolom -> satuan kanan terpotong pada tandon besar)
char* fmtVolRingkas(char* b, size_t n, const char* label, float ml){
  char f[10];
  if(ml < 0.0f) ml = 0.0f;
  if(ml < 999.5f)          snprintf_P(b,n,PSTR("%s %ld mL"), label, (long)round(ml));
  else if(ml < 9995.0f)    snprintf_P(b,n,PSTR("%s %s L"),  label, fmtF(f, ml/1000.0f, 2));
  else if(ml < 99950.0f)   snprintf_P(b,n,PSTR("%s %s L"),  label, fmtF(f, ml/1000.0f, 1));
  else                     snprintf_P(b,n,PSTR("%s %ld L"), label, (long)round(ml/1000.0f));
  return b;
}
// ── Layar rekap dosing terakhir ──
void tampilkanRekapDosing(){
  char b[21];
  lcdPrintCenter(0, F("DOSING TERAKHIR:"));
  if(cekTerakhir.dTgl==0) snprintf_P(b,sizeof(b),PSTR("--:-- --/--/----"));
  else snprintf_P(b,sizeof(b),PSTR("%02d:%02d %02d/%02d/%04d"),cekTerakhir.dJam,cekTerakhir.dMnt,cekTerakhir.dTgl,cekTerakhir.dBln,cekTerakhir.dThn);
  lcdPrintCenter(1, b);                                                // dari RTC saat dosing selesai
  lcdPrintCenter(2, String(F("WAKTU: ")) + String(cekTerakhir.dDurasiMenit) + F(" MENIT"));
  // Baris 4: total estimasi volume siklus dosing terakhir (kiri: AB Mix, kanan: pH Down)
  // TES_122: volume yg benar-benar keluar (detik nyala x debit). AB = per sisi (A = B), pH = pH Down P1.
  { char kiri[12], kanan[12]; fmtVolRingkas(kiri,sizeof(kiri),"AB",cekTerakhir.dABmL); fmtVolRingkas(kanan,sizeof(kanan),"pH",cekTerakhir.dPHmL);
    lcdPrintLR(3, kiri, kanan); }
}
// Dipanggil tiap 1 detik: rekap tampil 5 detik lalu beralih ke Status Dosing
void prosesRekapDosing(){
  rekapDetikBerjalan++;
  if(rekapDetikBerjalan >= REKAP_AUTO_LANJUT_DETIK){
    currentState = DOSING_FINAL_STATUS;
    finalStatusDetikBerjalan = 0;
    lcd.clear();
    tampilkanStatusDosing();
  }
}

// ── Dispatcher utama, dipanggil tiap 1 detik dari loop() ────
void updateDosingProses() {
  switch(currentState){
    case DOSING_TUNGGU_WAKTU: prosesTungguWaktu(); break;
    case DOSING_ISI_TANDON:   prosesIsiTandon();   break;
    case DOSING_JEDA_ISI:     prosesJedaIsi();     break;
    case DOSING_CEK_PPM1:
    case DOSING_CEK_PPM2:
    case DOSING_CEK_PPM3:
    case DOSING_PI_CEK_PPM:   prosesCekPPM();      break;
    case DOSING_MACRO_AB:
    case DOSING_MICRO_AB:
    case DOSING_PI_POMPA_AB:
    case DOSING_MACRO_PH:
    case DOSING_MICRO_PH:
    case DOSING_PI_POMPA_PH:  prosesFasePompa();   break;
    case DOSING_HOMOGEN1:
    case DOSING_HOMOGEN2:
    case DOSING_HOMOGEN3:
    case DOSING_HOMOGEN4:
    case DOSING_PI_HOMOGEN_AB:
    case DOSING_PI_HOMOGEN_PH: prosesHomogenisasi();break;
    case DOSING_CEK_PH1:
    case DOSING_CEK_PH2:
    case DOSING_PI_CEK_PH:    prosesCekPH();       break;
    case DOSING_CEK_AKHIR:    prosesCekAkhir();    break;
    case DOSING_REKAP:        prosesRekapDosing();  break;
    case DOSING_FINAL_STATUS: prosesFinalStatus(); break;
    case DOSING_JEDA_PULIH:   prosesJedaPulih();   break;
    default: break;
  }
}

void tampilkanUlangFaseDosing() {
  switch(currentState){
    case DOSING_TUNGGU_WAKTU: tampilkanDosingTungguWaktu(false); break;
    case DOSING_ISI_TANDON:   tampilkanDosingIsiTandon();  break;
    case DOSING_JEDA_ISI:     tampilkanDosingJedaIsi();    break;   // TES_126: dulu kosong s.d. 1 dtk
    case DOSING_CEK_PPM1:
    case DOSING_CEK_PPM2:
    case DOSING_CEK_PPM3:
    case DOSING_PI_CEK_PPM:   tampilkanDosingCekPPM();     break;
    case DOSING_MACRO_AB:
    case DOSING_MICRO_AB:
    case DOSING_PI_POMPA_AB:
    case DOSING_MACRO_PH:
    case DOSING_MICRO_PH:
    case DOSING_PI_POMPA_PH:  tampilkanDosingPompaAktif(); break;
    case DOSING_HOMOGEN1:
    case DOSING_HOMOGEN2:
    case DOSING_HOMOGEN3:
    case DOSING_HOMOGEN4:
    case DOSING_PI_HOMOGEN_AB:
    case DOSING_PI_HOMOGEN_PH: tampilkanDosingHomogenisasi();break;
    case DOSING_CEK_PH1:
    case DOSING_CEK_PH2:
    case DOSING_PI_CEK_PH:    tampilkanDosingCekPH();      break;
    case DOSING_CEK_AKHIR:    tampilkanDosingCekAkhir();   break;
    case DOSING_TUNDA_KALIBRASI: tampilkanTundaKalibrasi(); break;
    case DOSING_JEDA_PULIH:   tampilkanJedaPulih();        break;
    default: break;
  }
}

// ========
// UTILITAS
// ========
bool semuaDebitTerisi() {
  // D1 (index 0) TIDAK dipakai dosing (hanya sirkulasi cek sensor) -> debit tak diinput
  for(int i=1;i<TOTAL_PUMP_OPTIONS;i++) if(terpilihDebitPump[i]==0) return false;
  return true;
}
bool semuaDebitTerisiBaru() {
  for(int i=1;i<TOTAL_PUMP_OPTIONS;i++) if(terpilihDebitPumpBaru[i]==0) return false;
  return true;
}
// Jumlah slot Jeda Homogenisasi: J0..J4 (TES_134: J0 ditambahkan) untuk semua jenis dosing.
// Urutan tampilan/input u = 0..4 -> J0..J4; indeks array: J0 = [4], J1..J4 = [0..3] (tata letak EEPROM tetap).
const int JUMLAH_SLOT_JEDA = 5;
int slotJeda(int u){ return (u==0) ? IDX_JEDA_J0 : u-1; }

int maksScrollRingkasan(bool baru){
  bool jedaDiatur = baru ? jedaBaruSudahDiatur : jedaSudahDiatur;
  int nJeda = jedaDiatur ? JUMLAH_SLOT_JEDA : 1;
  int total = 21 + jumlahBarisPI(baru) + nJeda;   // 21 baris tetap (0-20) + baris Kp/Ki PI + baris jeda
  int maks  = total - 3;   // 3 baris data terlihat sekaligus
  return (maks < 0) ? 0 : maks;
}
// Pastikan semua slot jeda yg relevan dgn mode benar-benar terisi 
bool jedaSlotLengkap(int* arr){
  for(int i=0;i<JUMLAH_SLOT_JEDA;i++) if(arr[slotJeda(i)] <= 0) return false;
  return true;
}
// Layar: batasan PPM tidak sesuai dgn target (dipakai saat input batasan & saat target PPM diubah)
void tampilkanBatasanTidakSesuai(int target){
  lcd.clear();
  lcdPrintCenter(0, F("BATASAN TIDAK SESUAI"));
  lcdPrintCenter(1, String(F("Target PPM: ")) + String(target));
  lcdPrintCenter(2, String(F("Batas maks: +/-")) + String(target-1));
  lcdPrintCenter(3, F("Atur ulang Batasan"));
  jedaInfo(3000);
  lcd.clear();
}
// Target PPM baru saja berubah (input / PPM-pH Tanaman): bila batasan yang ada kini >= target -> batasan dinyatakan
// belum diatur (wajib input ulang) dan user diberi tahu. Konteks Aktif/Baru mengikuti tempat penyimpanan.
void cekBatasanSetelahTargetBerubah(){
  bool baru = simpanKeParamBaru();
  bool &flag = baru ? batasanBaruSudahDiatur : batasanSudahDiatur;
  int  b = baru ? batasPPMBaru : batasPPM;
  int  t = baru ? terpilihPPMBaru : terpilihPPM;
  if(flag && !batasPPMSesuaiTarget(b,t)){ flag = false; tampilkanBatasanTidakSesuai(t); }
}
// Item PERTAMA (urutan menu Parameter Dosing) yang belum terisi / tidak valid; NULL bila lengkap.
// Dipakai parameterLengkap()/parameterBaruLengkap() dan layar "LENGKAPI PARAMETER" agar pesan selalu konsisten.
const __FlashStringHelper* itemBelumLengkap(bool baru){
  const String& ec    = baru ? terpilihECBaru       : terpilihEC;
  const String& dil   = baru ? terpilihDilutionBaru : terpilihDilution;
  float  slope = baru ? slopeABMixTerpilihBaru : slopeABMixTerpilih;
  const String& acid  = baru ? terpilihAcidBaru     : terpilihAcid;
  int    conc  = baru ? terpilihConcPersenBaru : terpilihConcPersen;
  int    vol   = baru ? terpilihVolumeBaru   : terpilihVolume;
  int    ppm   = baru ? terpilihPPMBaru      : terpilihPPM;
  float  ph    = baru ? terpilihPHBaru       : terpilihPH;
  int*   deb   = baru ? terpilihDebitPumpBaru : terpilihDebitPump;
  if(ec=="-" || dil.length()==0 || slope<=0.0f)          return F("AB Mix (EC/Encer)");
  if(acid=="-" || conc<=0)                               return F("pH Down (Asam/Kons)");
  if(vol<=0)                                             return F("Volume Tandon");
  if(!targetPPMValid(ppm))                               return F("Target PPM");   // TES_140: > 2100 (data lama) -> isi ulang
  if(ph<=0.0f)                                           return F("Target pH");
  if(!waktuDosingFormatValid(baru ? terpilihWaktuDosingBaru : terpilihWaktuDosing)) return F("Waktu Dosing");
  if(deb[1]==0) return F("Debit Pompa D2");
  if(deb[2]==0) return F("Debit Pompa D3");
  if(deb[3]==0) return F("Debit Pompa P1");
  if(deb[4]==0) return F("Debit Pompa P2");
  if(deb[5]==0) return F("Debit Pompa P3");
  if((baru ? terpilihLamaCekSensorBaru : terpilihLamaCekSensor)<=0)       return F("Lama Bilas Sensor");
  if((baru ? terpilihJedaStabilisasiBaru : terpilihJedaStabilisasi)<=0)   return F("Jeda Stabilisasi");
  int bPPM = baru ? batasPPMBaru : batasPPM;
  int bPH  = baru ? batasPHx100Baru : batasPHx100;
  if(!(baru ? batasanBaruSudahDiatur : batasanSudahDiatur) || !batasPPMValid(bPPM) || !batasPHValid(bPH))
                                                         return F("Batasan PPM/pH");
  if(!batasPPMSesuaiTarget(bPPM, ppm))                   return F("Batasan (atur ulang)");
  if(!(baru ? jenisDosingBaruSudahDiatur : jenisDosingSudahDiatur) || (baru ? dosingModeBaru : dosingMode)<0)
                                                         return F("Jenis Dosing");
  if(!(baru ? jedaBaruSudahDiatur : jedaSudahDiatur) || !jedaSlotLengkap(baru ? dosingJedaHomogenisasiBaru : dosingJedaHomogenisasi))
                                                         return F("Jeda Homogenisasi");
  return NULL;
}
bool parameterLengkap()     { return itemBelumLengkap(false) == NULL; }
bool parameterBaruLengkap() { return itemBelumLengkap(true)  == NULL; }

// ── Perubahan Persamaan Nutrisi ──
// Persamaan berlaku global (Aktif & Baru). Saat jadwal cek aktif, perubahan langsung dipakai oleh jadwal yang
// sedang berjalan -> minta konfirmasi dulu; saat jadwal tidak aktif -> langsung diterapkan.
void tampilkanPersamaanKonfirmasi(){
  lcdPrintCenter(0, F("JADWAL CEK AKTIF"));
  lcdPrintCenter(1, F("Terapkan ke jadwal?"));
  lcdPrint20(0,2, String(selPersamaanKonfirmasi==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,3, String(selPersamaanKonfirmasi==1?"> ":"  ") + F("Tidak"));
}
void mintaAksiPersamaan(byte aksi, float nilai){
  aksiPersamaan = aksi;
  nilaiPersamaanTunda = nilai;
  if(dosingSedangMenunggu || jadwalCekAktif){
    selPersamaanKonfirmasi = 1;
    currentState = PERSAMAAN_KONFIRMASI;
    lcd.clear();
    tampilkanPersamaanKonfirmasi();
  } else {
    selesaiAksiPersamaan(true);
  }
}
void selesaiAksiPersamaan(bool terapkan){
  bool jadwalAktif = (dosingSedangMenunggu || jadwalCekAktif);
  if(terapkan){
    if(aksiPersamaan==AKSI_PERS_AB){
      regresiSlopeAB[selPengaturanABDil]=nilaiPersamaanTunda;
      bool editSelAktif = (terpilihEC!="-" && terpilihDilution==dilOptions[selPengaturanABDil]);
      bool editSelBaru  = (terpilihECBaru!="-" && terpilihDilutionBaru==dilOptions[selPengaturanABDil]);
      if(editSelAktif) slopeABMixTerpilih     = hitungSlopeAB(terpilihEC.toFloat(), selPengaturanABDil);
      if(editSelBaru)  slopeABMixTerpilihBaru = hitungSlopeAB(terpilihECBaru.toFloat(), selPengaturanABDil);
      kalkSlopeTerpilih = hitungSlopeAB(kalkEC.toFloat(), kalkSelDil);
      hitungKonstantaPIOtomatis();
    } else if(aksiPersamaan==AKSI_PERS_PH){
      float (*tabelEdit)[2];
      if(selPengaturanPHAsam==0) tabelEdit = (selPengaturanPHFase==0)? tabelSlopeAtas : tabelSlopeBawah;
      else                       tabelEdit = (selPengaturanPHFase==0)? tabelSlopeAtasFosfat : tabelSlopeBawahFosfat;
      tabelEdit[selPengaturanPHIdx][0]=(float)ppmAcuanTundaPH;   // PPM acuan & slope di-commit BERSAMAAN
      tabelEdit[selPengaturanPHIdx][1]=nilaiPersamaanTunda;
      hitungKonstantaPIOtomatis();   // slope pH berubah -> Kp otomatis ikut
    } else {
      terapkanResetPersamaan();
    }
  }
  lcd.clear();
  if(!terapkan)                         lcdPrintCenter(1, F("Tidak Disimpan"));
  else if(aksiPersamaan==AKSI_PERS_RESET) lcdPrintCenter(1, F("Reset Berhasil!"));
  else                                  lcdPrintCenter(1, F("Konstanta Disimpan!"));
  if(terapkan && jadwalAktif) lcdPrintCenter(2, F("(Jadwal ikut ubah)"));
  jedaAman(1500);
  lcd.clear();
  if(aksiPersamaan==AKSI_PERS_AB)      { currentState=PENGATURAN_AB_LIST;        tampilkanPengaturanABList(); }
  else if(aksiPersamaan==AKSI_PERS_PH) { currentState=PENGATURAN_PH_LIST;        tampilkanPengaturanPHList(); }
  else                                 { currentState=MENU_PENGATURAN_PERSAMAAN; tampilkanMenuPengaturanPersamaan(); }
}

// Terapkan RESET tabel persamaan ke default
void terapkanResetPersamaan(){
  muatUlangKonstantaAktif(); // terapkan default (juga menyegarkan cache slope)
  hitungKonstantaPIOtomatis();
}
// Menyalin seluruh Parameter Baru (draft) menjadi Parameter Aktif
void salinParameterBaruKeAktif(){
  terpilihEC          = terpilihECBaru;
  terpilihDilution     = terpilihDilutionBaru;
  slopeABMixTerpilih   = slopeABMixTerpilihBaru;
  terpilihAcid         = terpilihAcidBaru;
  terpilihConc         = terpilihConcBaru;
  terpilihConcPersen   = terpilihConcPersenBaru;

  terpilihVolume       = terpilihVolumeBaru;
  terpilihPPM          = terpilihPPMBaru;
  terpilihPH           = terpilihPHBaru;
  terpilihWaktuDosing  = terpilihWaktuDosingBaru;
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++) terpilihDebitPump[i]=terpilihDebitPumpBaru[i];

  batasPPM               = batasPPMBaru;
  batasPHx100            = batasPHx100Baru;
  batasanSudahDiatur     = batasanBaruSudahDiatur;

  dosingMode             = dosingModeBaru;
  persenMakro            = persenMakroBaru;
  terpilihLamaCekSensor  = terpilihLamaCekSensorBaru;
  terpilihJedaStabilisasi = terpilihJedaStabilisasiBaru;
  jenisDosingSudahDiatur = jenisDosingBaruSudahDiatur;

  for(int i=0;i<6;i++) dosingJedaHomogenisasi[i]=dosingJedaHomogenisasiBaru[i];
  jedaSudahDiatur = jedaBaruSudahDiatur;

  // Profil Kp/Ki PI ZN milik Parameter Baru ikut menjadi profil aktif
  if(profilZNBaru.terisi){ profilZNAktif = profilZNBaru; znPPMUji[0][0]=znPPMUji[1][0]; znPPMUji[0][1]=znPPMUji[1][1]; znKanalLama[0]=znKanalLama[1]; muatPIZNDariProfilAktif(); }

  hitungKonstantaPIOtomatis();   // Parameter Aktif baru diterapkan -> Kp/Ki Auto-Tune dihitung ulang dari persamaan & debit
}

// Edit PERTAMA saat jadwal menunggu: Parameter Baru = salinan Parameter Aktif, sehingga cukup mengubah item
// yang perlu saja (tidak wajib mengisi ulang semua). Tidak berbuat apa pun bila jadwal tidak menunggu / draft sudah ada.
void siapkanDraftBaru(){
  if(!dosingSedangMenunggu || draftBaruAda) return;
  terpilihECBaru          = terpilihEC;
  terpilihDilutionBaru    = terpilihDilution;
  slopeABMixTerpilihBaru  = slopeABMixTerpilih;
  terpilihAcidBaru        = terpilihAcid;
  terpilihConcBaru        = terpilihConc;
  terpilihConcPersenBaru  = terpilihConcPersen;
  terpilihVolumeBaru      = terpilihVolume;
  terpilihPPMBaru         = terpilihPPM;
  terpilihPHBaru          = terpilihPH;
  terpilihWaktuDosingBaru = terpilihWaktuDosing;
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++) terpilihDebitPumpBaru[i]=terpilihDebitPump[i];
  batasPPMBaru            = batasPPM;
  batasPHx100Baru         = batasPHx100;
  batasanBaruSudahDiatur  = batasanSudahDiatur;
  dosingModeBaru          = dosingMode;
  persenMakroBaru         = persenMakro;
  terpilihLamaCekSensorBaru   = terpilihLamaCekSensor;
  terpilihJedaStabilisasiBaru = terpilihJedaStabilisasi;
  jenisDosingBaruSudahDiatur  = jenisDosingSudahDiatur;
  for(int i=0;i<6;i++) dosingJedaHomogenisasiBaru[i]=dosingJedaHomogenisasi[i];
  jedaBaruSudahDiatur     = jedaSudahDiatur;
  profilZNBaru            = profilZNAktif;
  znPPMUji[1][0] = znPPMUji[0][0]; znPPMUji[1][1] = znPPMUji[0][1];   // TES_141
  znKanalLama[1] = znKanalLama[0];   // TES_144
  draftBaruAda = true;
}
// Mengosongkan kembali Parameter Baru setelah dipakai (siap diisi utk siklus berikutnya)
void resetStatusParameterBaru(){
  terpilihECBaru="-"; terpilihDilutionBaru=""; terpilihAcidBaru="-"; terpilihConcBaru="";
  terpilihConcPersenBaru=0; slopeABMixTerpilihBaru=0.0f; selJenisBaru=0;
  terpilihVolumeBaru=0; terpilihPPMBaru=0; terpilihPHBaru=0.0f; terpilihWaktuDosingBaru="--:--";
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++) terpilihDebitPumpBaru[i]=0;
  batasPPMBaru=0; batasPHx100Baru=0; batasanBaruSudahDiatur=false;
  dosingModeBaru=-1; persenMakroBaru=PERSEN_MAKRO_DEFAULT; terpilihLamaCekSensorBaru=0; terpilihJedaStabilisasiBaru=0;
  jenisDosingBaruSudahDiatur=false;
  for(int i=0;i<6;i++) dosingJedaHomogenisasiBaru[i]=0;
  jedaBaruSudahDiatur=false;
  profilZNBaru = ProfilKpKiZN();   // kosong (terisi=false)
  znPPMUji[1][0] = 0; znPPMUji[1][1] = 0; znKanalLama[1] = 0;
  draftBaruAda = false;
}
// TES_138: nilai tersimpan mL/menit. Ringkasan "Pompa D2: " + teks <= 20 kolom -> maks 10 karakter:
// desimal ke-3 = 0 -> 2 desimal ("1.50 L/mnt"); desimal ke-3 != 0 -> 3 desimal tanpa spasi ("0.034L/mnt"); >= 10 -> "10.0 L/mnt"
String formatDebit(int mlMenit) {
  if(mlMenit==0) return "-";
  char buf[16];
  if(mlMenit >= 10000)      snprintf_P(buf, sizeof(buf), PSTR("%d.%d L/mnt"), mlMenit/1000, (mlMenit%1000)/100);
  else if(mlMenit%10 == 0)  snprintf_P(buf, sizeof(buf), PSTR("%d.%02d L/mnt"), mlMenit/1000, (mlMenit%1000)/10);
  else                      snprintf_P(buf, sizeof(buf), PSTR("%d.%03dL/mnt"), mlMenit/1000, mlMenit%1000);
  return String(buf);
}
float bufferToFloat() { return inputBuffer.toFloat(); }


void naik(int &sel, int &scroll, int total) {
  if(sel>0){sel--;if(sel<scroll)scroll--;}
}
void turun(int &sel, int &scroll, int total) {
  if(sel<total-1){sel++;if(sel-scroll>2)scroll++;}
}

// ==============
// UTILITAS INPUT
// ==============
void handleNumericInput(char key) {
  if(key>='0'&&key<='9'){if(inputBuffer.length()<8)inputBuffer+=key;}
}
void handleTitikDesimal() {
  if(inputBuffer.indexOf('.')>=0) return;
  if(inputBuffer.length()==0) return;
  if(inputBuffer.length()>=7) return;
  inputBuffer+='.';
}
void handleRTCField(char key, int idx, int minV, int maxV, MenuState nextState, int nextIdx) {
  if(key=='D') {
    inputBuffer = "";
    tampilkanInputRTC(idx);
  }
  else if(key=='*') {
    inputBuffer = "";   // TES_144
    currentState = MENU_PENGATURAN_SISTEM;
    lcd.clear();
    tampilkanMenuPengaturanSistem();
  }
  else if(key=='B') {
    if(inputBuffer.length() == 0) return;
    int v = inputBuffer.toInt();
    if(idx == 0) {
      // Batasi tahun 2000-2099 (konsisten validasi RTC & tampilan); TES_127: < 4 digit tak lagi diam saja
      if(inputBuffer.length() != 4 || v < 2000 || v > 2099){
        lcd.clear();
        lcdPrintCenter(1, F("Tahun: 2000-2099"));
        jedaAman(1500);
        inputBuffer = "";
        lcd.clear();
        tampilkanInputRTC(idx);
        return;
      }
      rtcInput[idx] = v;
      inputBuffer = "";
      currentState = nextState;
      lcd.clear();
      tampilkanInputRTC(nextIdx);
    } else {
      if(v >= minV && v <= maxV) {
        rtcInput[idx] = v;
        inputBuffer = "";
        currentState = nextState;
        lcd.clear();
        tampilkanInputRTC(nextIdx);
      } else {
        tampilkanError(String(F("Batas: ")) + String(minV) + F("-") + String(maxV));
        inputBuffer = "";
        tampilkanInputRTC(idx);
      }
    }
  }
  else if(key>='0' && key<='9') {
    int maxLen = (idx == 0) ? 4 : 2;
    if(inputBuffer.length() < maxLen) {
      inputBuffer += key;
      tampilkanInputRTC(idx);
    }
  }
}

// =====
// SETUP
// =====
void eepromKemas(KonfigTersimpan &k){
  memset(&k, 0, sizeof(k));
  k.magic = EEPROM_MAGIC; k.versi = EEPROM_VERSI;
  k.terpilihVolume = terpilihVolume;
  k.terpilihPPM    = terpilihPPM;
  k.terpilihPH     = terpilihPH;
  terpilihWaktuDosing.toCharArray(k.waktuDosing, sizeof(k.waktuDosing));
  terpilihEC.toCharArray(k.ec, sizeof(k.ec));
  terpilihDilution.toCharArray(k.dilution, sizeof(k.dilution));
  terpilihAcid.toCharArray(k.acid, sizeof(k.acid));
  terpilihConc.toCharArray(k.conc, sizeof(k.conc));
  k.slopeABMixTerpilih = slopeABMixTerpilih;
  k.terpilihConcPersen = terpilihConcPersen;
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++) k.terpilihDebitPump[i]=terpilihDebitPump[i];
  k.batasPPM               = batasPPM;
  k.batasPHx100            = batasPHx100;
  k.dosingMode             = dosingMode;
  k.selJenisDosing         = selJenisDosing;
  k.terpilihLamaCekSensor  = terpilihLamaCekSensor;
  k.terpilihJedaStabilisasi = terpilihJedaStabilisasi;
  for(int i=0;i<6;i++) k.dosingJedaHomogenisasi[i]=dosingJedaHomogenisasi[i];
  k.batasanSudahDiatur     = batasanSudahDiatur;
  k.jenisDosingSudahDiatur = jenisDosingSudahDiatur;
  k.jedaSudahDiatur        = jedaSudahDiatur;
  for(int i=0;i<TDS_TITIK_KAL;i++){ k.calTDSBuffer[i]=calTDSBuffer[i]; k.tdsCVInput[i]=tdsCVInput[i]; }
  k.tdsKalibrasiAktif = tdsKalibrasiAktif;
  k.calPhVolt1 = calPhVolt1; k.calPhVolt2 = calPhVolt2; k.calPhVolt3 = calPhVolt3;
  for(int i=0;i<3;i++) k.phBufferInput[i]=phBufferInput[i];
  k.phKalibrasi3TitikAktif = phKalibrasi3TitikAktif;
  k.phSlope = phSlope; k.phOffset = phOffset;
  for(int i=0;i<2;i++) k.regresiSlopeAB[i]=regresiSlopeAB[i];
  k.profilZNAktif = profilZNAktif;   // Kp/Ki ZN Manual (1 slot aktif)
  for(int i=0;i<TOTAL_TABEL_PH;i++){
    k.tabelSlopeAtas[i][0]=tabelSlopeAtas[i][0];             k.tabelSlopeAtas[i][1]=tabelSlopeAtas[i][1];
    k.tabelSlopeBawah[i][0]=tabelSlopeBawah[i][0];           k.tabelSlopeBawah[i][1]=tabelSlopeBawah[i][1];
    k.tabelSlopeAtasFosfat[i][0]=tabelSlopeAtasFosfat[i][0]; k.tabelSlopeAtasFosfat[i][1]=tabelSlopeAtasFosfat[i][1];
    k.tabelSlopeBawahFosfat[i][0]=tabelSlopeBawahFosfat[i][0];k.tabelSlopeBawahFosfat[i][1]=tabelSlopeBawahFosfat[i][1];
  }
  k.suhuOffset = suhuOffset; k.suhuInputNegatif = suhuInputNegatif;
  k.piKpPPM = piPPM.kp; k.piKiPPM = piPPM.ki; k.piKpPH = piPH.kp; k.piKiPH = piPH.ki;
  k.piKpPHAtas = piPHKpAtas; k.piKpPHBawah = piPHKpBawah;
  k.rtcTahun=rtcCache.tahun; k.rtcBulan=rtcCache.bulan;
  k.rtcTgl=rtcCache.tgl;     k.rtcJam=rtcCache.jam;
  k.rtcMenit=rtcCache.menit; k.rtcDetik=rtcCache.detik;
  // -- Parameter Baru (draft) --
  terpilihECBaru.toCharArray(k.ecBaru, sizeof(k.ecBaru));
  terpilihDilutionBaru.toCharArray(k.dilutionBaru, sizeof(k.dilutionBaru));
  terpilihAcidBaru.toCharArray(k.acidBaru, sizeof(k.acidBaru));
  terpilihConcBaru.toCharArray(k.concBaru, sizeof(k.concBaru));
  terpilihWaktuDosingBaru.toCharArray(k.waktuDosingBaru, sizeof(k.waktuDosingBaru));
  k.slopeABMixTerpilihBaru = slopeABMixTerpilihBaru;
  k.terpilihConcPersenBaru = terpilihConcPersenBaru;
  k.selJenisBaru = selJenisBaru;
  k.terpilihVolumeBaru = terpilihVolumeBaru;
  k.terpilihPPMBaru = terpilihPPMBaru;
  k.terpilihPHBaru = terpilihPHBaru;
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++) k.terpilihDebitPumpBaru[i]=terpilihDebitPumpBaru[i];
  k.batasPPMBaru = batasPPMBaru;
  k.batasPHx100Baru = batasPHx100Baru;
  k.batasanBaruSudahDiatur = batasanBaruSudahDiatur;
  k.dosingModeBaru = dosingModeBaru;
  k.terpilihLamaCekSensorBaru = terpilihLamaCekSensorBaru;
  k.terpilihJedaStabilisasiBaru = terpilihJedaStabilisasiBaru;
  k.jenisDosingBaruSudahDiatur = jenisDosingBaruSudahDiatur;
  for(int i=0;i<6;i++) k.dosingJedaHomogenisasiBaru[i]=dosingJedaHomogenisasiBaru[i];
  k.jedaBaruSudahDiatur = jedaBaruSudahDiatur;
  // -- Status siklus dosing --
  k.dosingSedangMenunggu = dosingSedangMenunggu;
  k.jadwalCekAktif = jadwalCekAktif;
  k.dosingTargetHari  = dosingTargetHari;
  k.dosingTargetBulan = dosingTargetBulan;
  k.dosingTargetTahun = dosingTargetTahun;
}

void eepromTerapkan(const KonfigTersimpan &k){
  terpilihVolume = k.terpilihVolume;
  terpilihPPM    = k.terpilihPPM;
  terpilihPH     = k.terpilihPH;
  terpilihWaktuDosing = String(k.waktuDosing);
  terpilihEC = String(k.ec);
  terpilihDilution = String(k.dilution);
  terpilihAcid = String(k.acid);
  terpilihConc = String(k.conc);
  slopeABMixTerpilih = k.slopeABMixTerpilih;
  terpilihConcPersen = k.terpilihConcPersen;
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++){ terpilihDebitPump[i]=k.terpilihDebitPump[i]; if(terpilihDebitPump[i]<0 || terpilihDebitPump[i]>DEBIT_MAKS_SIMPAN) terpilihDebitPump[i]=0; }   // luar spesifikasi -> wajib isi ulang
  batasPPM               = k.batasPPM;
  batasPHx100            = k.batasPHx100;
  dosingMode             = k.dosingMode;
  selJenisDosing         = k.selJenisDosing;
  terpilihLamaCekSensor  = k.terpilihLamaCekSensor;
  terpilihJedaStabilisasi = k.terpilihJedaStabilisasi;
  for(int i=0;i<6;i++) dosingJedaHomogenisasi[i]=k.dosingJedaHomogenisasi[i];
  // Migrasi dari TES_115: slot ini dulu berisi Loop Dosing (1-14 hari) -> tak lolos validasi -> dianggap belum diatur
  batasanSudahDiatur     = k.batasanSudahDiatur && batasPPMValid(batasPPM) && batasPHValid(batasPHx100);
  jenisDosingSudahDiatur = k.jenisDosingSudahDiatur;
  if(dosingMode!=1 && dosingMode!=3 && dosingMode!=4){ dosingMode=-1; jenisDosingSudahDiatur=false; }   // data lama/rusak -> wajib pilih ulang
  if(selJenisDosing<0 || selJenisDosing>=TOTAL_JENIS_DOSING_OPTIONS) selJenisDosing=0;
  jedaSudahDiatur        = k.jedaSudahDiatur;
  for(int i=0;i<TDS_TITIK_KAL;i++){ calTDSBuffer[i]=k.calTDSBuffer[i]; tdsCVInput[i]=k.tdsCVInput[i]; }
  tdsKalibrasiAktif = k.tdsKalibrasiAktif;
  calPhVolt1 = k.calPhVolt1; calPhVolt2 = k.calPhVolt2; calPhVolt3 = k.calPhVolt3;
  for(int i=0;i<3;i++) phBufferInput[i]=k.phBufferInput[i];
  phKalibrasi3TitikAktif = k.phKalibrasi3TitikAktif;
  phSlope = k.phSlope; phOffset = k.phOffset;
  for(int i=0;i<2;i++){ regresiSlopeAB[i]=k.regresiSlopeAB[i]; if(isnan(regresiSlopeAB[i]) || regresiSlopeAB[i]<=0.0f) regresiSlopeAB[i]=regresiSlopeABDefault[i]; }
  profilZNAktif = k.profilZNAktif;   // Kp/Ki ZN Manual (1 slot aktif)
  if(profilZNAktif.terisi) muatPIZNDariProfilAktif();   // pulihkan PI aktif mode ZN (variabel RAM) setelah boot / daya pulih
  for(int i=0;i<TOTAL_TABEL_PH;i++){
    tabelSlopeAtas[i][0]=k.tabelSlopeAtas[i][0];             tabelSlopeAtas[i][1]=k.tabelSlopeAtas[i][1];
    tabelSlopeBawah[i][0]=k.tabelSlopeBawah[i][0];           tabelSlopeBawah[i][1]=k.tabelSlopeBawah[i][1];
    tabelSlopeAtasFosfat[i][0]=k.tabelSlopeAtasFosfat[i][0]; tabelSlopeAtasFosfat[i][1]=k.tabelSlopeAtasFosfat[i][1];
    tabelSlopeBawahFosfat[i][0]=k.tabelSlopeBawahFosfat[i][0];tabelSlopeBawahFosfat[i][1]=k.tabelSlopeBawahFosfat[i][1];
  }
  kalkSlopeTerpilih = hitungSlopeAB(kalkEC.toFloat(), kalkSelDil);
  suhuOffset = k.suhuOffset; suhuInputNegatif = k.suhuInputNegatif;
  piPPM.kp = k.piKpPPM; piPPM.ki = k.piKiPPM; piPH.kp = k.piKpPH; piPH.ki = k.piKiPH;
  piPHKpAtas = k.piKpPHAtas; piPHKpBawah = k.piKpPHBawah;
  // Guard bila field PI dari EEPROM belum wajar (NaN / nol semua)
  if(isnan(piPPM.kp) || piPPM.kp<=0.0f){ piPPM.kp=0.05f; piPPM.ki=0.01f; }
  if(isnan(piPH.kp)  || piPH.kp <=0.0f){ piPH.kp=20.0f;  piPH.ki=2.0f;  }
  if(isnan(piPHKpAtas)  || piPHKpAtas  <=0.0f) piPHKpAtas  = 20.0f;
  if(isnan(piPHKpBawah) || piPHKpBawah <=0.0f) piPHKpBawah = 20.0f;
  // -- Parameter Baru (draft) --
  terpilihECBaru = String(k.ecBaru);
  terpilihDilutionBaru = String(k.dilutionBaru);
  terpilihAcidBaru = String(k.acidBaru);
  terpilihConcBaru = String(k.concBaru);
  terpilihWaktuDosingBaru = String(k.waktuDosingBaru);
  slopeABMixTerpilihBaru = k.slopeABMixTerpilihBaru;
  terpilihConcPersenBaru = k.terpilihConcPersenBaru;
  selJenisBaru = k.selJenisBaru;
  terpilihVolumeBaru = k.terpilihVolumeBaru;
  terpilihPPMBaru = k.terpilihPPMBaru;
  terpilihPHBaru = k.terpilihPHBaru;
  for(int i=0;i<TOTAL_PUMP_OPTIONS;i++){ terpilihDebitPumpBaru[i]=k.terpilihDebitPumpBaru[i]; if(terpilihDebitPumpBaru[i]<0 || terpilihDebitPumpBaru[i]>DEBIT_MAKS_SIMPAN) terpilihDebitPumpBaru[i]=0; }
  batasPPMBaru = k.batasPPMBaru;
  batasPHx100Baru = k.batasPHx100Baru;
  batasanBaruSudahDiatur = k.batasanBaruSudahDiatur && batasPPMValid(batasPPMBaru) && batasPHValid(batasPHx100Baru);
  dosingModeBaru = k.dosingModeBaru;
  terpilihLamaCekSensorBaru = k.terpilihLamaCekSensorBaru;
  terpilihJedaStabilisasiBaru = k.terpilihJedaStabilisasiBaru;
  jenisDosingBaruSudahDiatur = k.jenisDosingBaruSudahDiatur;
  if(dosingModeBaru!=1 && dosingModeBaru!=3 && dosingModeBaru!=4){ dosingModeBaru=-1; jenisDosingBaruSudahDiatur=false; }
  for(int i=0;i<6;i++) dosingJedaHomogenisasiBaru[i]=k.dosingJedaHomogenisasiBaru[i];
  jedaBaruSudahDiatur = k.jedaBaruSudahDiatur;
  // -- Status siklus dosing (resume) --
  dosingSedangMenunggu = k.dosingSedangMenunggu;
  jadwalCekAktif = k.jadwalCekAktif;
  dosingTargetHari  = k.dosingTargetHari;
  dosingTargetBulan = k.dosingTargetBulan;
  dosingTargetTahun = k.dosingTargetTahun;
  if(tdsKalibrasiAktif) hitungKalibrasiTDS(); // bangun ulang koefisien
  if(phKalibrasi3TitikAktif)  hitungKalibrasiPH();
}

// Buffer bersama utk kemas/muat EEPROM — static (bukan lokal-stack) supaya
// TIDAK ada alokasi struct besar berulang di stack tiap kali fungsi dipanggil
// (sumber risiko lag/korupsi runtime saat memori sudah tipis).
static KonfigTersimpan g_eepromScratch;

// Hasil cek harian terakhir: blok kecil terpisah (struct utama tidak diubah -> EEPROM_VERSI tetap)
void eepromMuatRekamCek(){
  RekamCek r;
  EEPROM.get(EEPROM_ALAMAT_REKAMCEK, r);
  if(r.magic==REKAM_CEK_MAGIC && !isnan(r.ph) && r.ph<=14.0f && r.ppm>=-1 && r.ppm<=5000) cekTerakhir = r;
}
void eepromSimpanRekamCek(){
  cekTerakhir.magic = REKAM_CEK_MAGIC;
  EEPROM.put(EEPROM_ALAMAT_REKAMCEK, cekTerakhir);   // put() hanya menulis byte yang berubah
}

void eepromMuatEkstra(){
  KonfigEkstra e;
  EEPROM.get(EEPROM_ALAMAT_EKSTRA, e);
  if(e.magic != KONFIG_EKSTRA_MAGIC) return;   // pertama kali / setelah reset pabrik -> nilai default
  draftBaruAda = (e.draftBaruAda==1);
  profilZNBaru = e.profilZNBaru;
  persenMakro     = persenMakroValid(e.persenMakro)     ? e.persenMakro     : PERSEN_MAKRO_DEFAULT;
  persenMakroBaru = persenMakroValid(e.persenMakroBaru) ? e.persenMakroBaru : PERSEN_MAKRO_DEFAULT;
  kanalTerkunci   = (e.kanalTerkunci <= (KUNCI_AB|KUNCI_PH)) ? e.kanalTerkunci : 0;
  jedaJ0SudahAda  = (e.versiJeda == VERSI_JEDA_J0 || e.versiJeda == VERSI_DATA_138 || e.versiJeda == VERSI_DATA_139);
  versiDataEkstra = e.versiJeda;
  if((e.versiJeda == VERSI_DATA_138 || e.versiJeda == VERSI_DATA_139) && e.makroPulih <= 0x0F){   // data lama: kolom ini belum ada (byte acak)
    makroPulih = e.makroPulih;
    makroPulihAcuanPPM = e.makroPulihAcuanPPM; makroPulihPredPPM = e.makroPulihPredPPM;
    makroPulihAcuanPHx100 = e.makroPulihAcuanPHx100; makroPulihVolPHx10 = e.makroPulihVolPHx10;
  } else makroPulih = 0;
}
void eepromSimpanEkstra(){
  KonfigEkstra e;
  memset(&e, 0, sizeof(e));
  e.magic = KONFIG_EKSTRA_MAGIC;
  e.draftBaruAda = draftBaruAda ? 1 : 0;
  e.profilZNBaru = profilZNBaru;
  e.persenMakro     = (uint8_t)persenMakro;
  e.persenMakroBaru = (uint8_t)persenMakroBaru;
  e.kanalTerkunci   = kanalTerkunci;
  e.versiJeda       = VERSI_DATA_139;
  e.makroPulih = makroPulih;
  e.makroPulihAcuanPPM = makroPulihAcuanPPM; e.makroPulihPredPPM = makroPulihPredPPM;
  e.makroPulihAcuanPHx100 = makroPulihAcuanPHx100; e.makroPulihVolPHx10 = makroPulihVolPHx10;
  EEPROM.put(EEPROM_ALAMAT_EKSTRA, e);         // put() hanya menulis byte yang berubah
}

// TES_141: data sebelum TES_141 (blok belum ada) -> uji dianggap di PPM target (= perilaku lama, tanpa skala PPM)
// TES_148: nilai Kp/Ki tersimpan rusak (NaN / negatif / sangat besar) -> 0 (kanal dianggap kosong, bukan dipakai diam-diam)
void sanitasiProfilZN(ProfilKpKiZN &p){
  float* v[6] = { &p.kpPPM, &p.kiPPM, &p.kpPHAtas, &p.kiPHAtas, &p.kpPHBawah, &p.kiPHBawah };
  for(int i=0;i<6;i++) if(!(*v[i] >= 0.0f && *v[i] < 1.0e6f)) *v[i] = 0.0f;
}
void eepromMuatZNUji(){
  sanitasiProfilZN(profilZNAktif); sanitasiProfilZN(profilZNBaru); muatPIZNDariProfilAktif();
  KonfigZNUji z;
  EEPROM.get(EEPROM_ALAMAT_ZNUJI, z);
  if(z.magic == KONFIG_ZNUJI_MAGIC || z.magic == KONFIG_ZNUJI_MAGIC_141){   // TES_141: tata letak awal sama
    for(int a=0;a<2;a++) for(int k=0;k<2;k++){ int v = z.ppmUji[a][k]; znPPMUji[a][k] = (v>=0 && v<=5000) ? (int16_t)v : 0; }
    ujiVolumeL = (z.volUjiTerakhir>=UJI_VOL_MIN && z.volUjiTerakhir<=UJI_VOL_MAKS) ? (int)z.volUjiTerakhir : 0;
    for(int a=0;a<2;a++) znKanalLama[a] = (z.magic == KONFIG_ZNUJI_MAGIC) ? (uint8_t)(z.kanalLama[a] & 7) : 0;
  } else {
    for(int k=0;k<2;k++){
      znPPMUji[0][k] = profilZNAktif.terisi ? (int16_t)terpilihPPM : 0;
      znPPMUji[1][k] = profilZNBaru.terisi  ? (int16_t)terpilihPPMBaru : 0;
    }
    ujiVolumeL = 0;
    znKanalLama[0] = 0; znKanalLama[1] = 0;   // TES_144
  }
}
void eepromSimpanZNUji(){
  KonfigZNUji z;
  memset(&z, 0, sizeof(z));
  z.magic = KONFIG_ZNUJI_MAGIC;
  for(int a=0;a<2;a++) for(int k=0;k<2;k++) z.ppmUji[a][k] = znPPMUji[a][k];
  z.volUjiTerakhir = (uint16_t)ujiVolumeL;
  z.kanalLama[0] = znKanalLama[0]; z.kanalLama[1] = znKanalLama[1];   // TES_144
  EEPROM.put(EEPROM_ALAMAT_ZNUJI, z);   // put() hanya menulis byte yang berubah
}

// ── TES_145: rekaman kurva Uji Reaksi TERAKHIR per kanal (blok terpisah sesudah blok ZN uji -> struct lain & EEPROM_VERSI tetap) ──
// Tata letak: [0..1] magic | [2..4] jumlah titik kanal 0..2 | [8..] 3 blok kanal: waktu uint16[UJI_REK_TITIK] + nilai int16[UJI_REK_TITIK]
#define KONFIG_ZNREK_MAGIC   0x2145u
#define EEPROM_ALAMAT_ZNREK  (EEPROM_ALAMAT_ZNUJI + 32)           // KonfigZNUji (<= 32 byte) ada tepat sebelumnya
#define ZNREK_BLOK_BYTE      (UJI_REK_TITIK*4)
#define ZNREK_ALAMAT_N(k)    (EEPROM_ALAMAT_ZNREK + 2 + (k))
#define ZNREK_ALAMAT_BLOK(k) (EEPROM_ALAMAT_ZNREK + 8 + (k)*ZNREK_BLOK_BYTE)
#define ZNREK_UKURAN         (8 + 3*ZNREK_BLOK_BYTE)
#ifdef __AVR__
static_assert(sizeof(KonfigZNUji) <= 32, "KonfigZNUji melewati blok rekaman ZN");
#ifdef E2END
static_assert(EEPROM_ALAMAT_ZNREK + ZNREK_UKURAN <= E2END + 1, "Blok rekaman ZN melewati ukuran EEPROM");
#endif
#endif
void eepromMuatZNRek(){
  for(int k=0;k<3;k++) znRekN[k] = 0;
  uint16_t m; EEPROM.get(EEPROM_ALAMAT_ZNREK, m);
  if(m != KONFIG_ZNREK_MAGIC) return;   // belum pernah ada rekaman / sesudah Reset Sistem
  for(int k=0;k<3;k++){
    byte n = EEPROM.read(ZNREK_ALAMAT_N(k));
    if(n < 1 || n > UJI_REK_TITIK) continue;   // 0 / 0xFF / tak wajar = tidak ada
    int a = ZNREK_ALAMAT_BLOK(k);
    for(int i=0;i<UJI_REK_TITIK;i++){
      EEPROM.get(a + i*2, znRekWaktu[k][i]);
      EEPROM.get(a + UJI_REK_TITIK*2 + i*2, znRekNilai[k][i]);
    }
    znRekN[k] = n;
  }
}
// Simpan rekaman kanal k dari RAM. put()/update() hanya menulis byte yang berubah. Jumlah titik ditulis terakhir
// (dan dikosongkan dulu) -> bila daya mati di tengah penulisan, rekaman setengah jadi tidak dianggap sah.
void eepromSimpanZNRek(int k){
  if(k < 0 || k > 2) return;
  uint16_t m; EEPROM.get(EEPROM_ALAMAT_ZNREK, m);
  if(m != KONFIG_ZNREK_MAGIC){   // blok baru: kosongkan semua jumlah titik, lalu tulis magic
    for(int c=0;c<3;c++) EEPROM.update(ZNREK_ALAMAT_N(c), 0);
    m = KONFIG_ZNREK_MAGIC; EEPROM.put(EEPROM_ALAMAT_ZNREK, m);
  }
  EEPROM.update(ZNREK_ALAMAT_N(k), 0);
  int a = ZNREK_ALAMAT_BLOK(k);
  for(int i=0;i<UJI_REK_TITIK;i++){
    uint16_t w = (i < znRekN[k]) ? znRekWaktu[k][i] : 0;
    int16_t  v = (i < znRekN[k]) ? znRekNilai[k][i] : 0;
    EEPROM.put(a + i*2, w);
    EEPROM.put(a + UJI_REK_TITIK*2 + i*2, v);
  }
  EEPROM.update(ZNREK_ALAMAT_N(k), znRekN[k]);
}

void eepromMuat(){
  eepromMuatRekamCek();
  eepromMuatEkstra();
  EEPROM.get(EEPROM_ALAMAT, g_eepromScratch);
  if(g_eepromScratch.magic != EEPROM_MAGIC || g_eepromScratch.versi != EEPROM_VERSI){
    eepromBaruDibuat = true;
    eepromKemas(g_eepromScratch);
    EEPROM.put(EEPROM_ALAMAT, g_eepromScratch);
    kalDraftMuat();
    eepromMuatZNUji();
    eepromMuatZNRek();   // TES_145
    eepromSiap = true; return;
  }
  rtcCache.tahun=g_eepromScratch.rtcTahun; rtcCache.bulan=g_eepromScratch.rtcBulan;
  rtcCache.tgl=g_eepromScratch.rtcTgl;     rtcCache.jam=g_eepromScratch.rtcJam;
  rtcCache.menit=g_eepromScratch.rtcMenit; rtcCache.detik=g_eepromScratch.rtcDetik;
  eepromTerapkan(g_eepromScratch);
  // TES_138: slope AB dihitung ulang dari EC + pengenceran + regresi tersimpan (dulu nilai cache dipercaya apa adanya:
  // regresi direset/korup -> slope basi; NaN lolos cek '<= 0' -> dosis AB 0/NaN tanpa peringatan)
  { int iA=cariIndexDil(terpilihDilution), iB=cariIndexDil(terpilihDilutionBaru);
    slopeABMixTerpilih     = (terpilihEC!="-"     && iA>=0) ? hitungSlopeAB(terpilihEC.toFloat(), iA)     : 0.0f;
    slopeABMixTerpilihBaru = (terpilihECBaru!="-" && iB>=0) ? hitungSlopeAB(terpilihECBaru.toFloat(), iB) : 0.0f;
    if(!(slopeABMixTerpilih > 0.0f))     slopeABMixTerpilih = 0.0f;
    if(!(slopeABMixTerpilihBaru > 0.0f)) slopeABMixTerpilihBaru = 0.0f;
    hitungKonstantaPIOtomatis(); }
  // TES_138: data sebelum TES_138 menyimpan debit x100 (0.01 L/mnt) -> x10 = mL/menit (sekali; penanda VERSI_DATA_138)
  bool migrasiDebit = (versiDataEkstra != VERSI_DATA_138 && versiDataEkstra != VERSI_DATA_139);
  if(migrasiDebit){
    for(int i=0;i<TOTAL_PUMP_OPTIONS;i++){
      terpilihDebitPump[i]     = (terpilihDebitPump[i]>=0     && terpilihDebitPump[i]<=1000)     ? terpilihDebitPump[i]*10     : 0;
      terpilihDebitPumpBaru[i] = (terpilihDebitPumpBaru[i]>=0 && terpilihDebitPumpBaru[i]<=1000) ? terpilihDebitPumpBaru[i]*10 : 0;
    }
    hitungKonstantaPIOtomatis();
  }
  // TES_139: tabel slope pH Down yg masih = bawaan lama (belum diedit) -> tabel data empiris baru (sekali)
  bool migrasiTabelPH = false;
  if(versiDataEkstra != VERSI_DATA_139){
    float (*tb[4])[2] = { tabelSlopeAtas, tabelSlopeBawah, tabelSlopeAtasFosfat, tabelSlopeBawahFosfat };
    const float (*df[4])[2] = { tabelSlopeAtasDefault, tabelSlopeBawahDefault, tabelSlopeAtasFosfatDefault, tabelSlopeBawahFosfatDefault };
    for(int t=0;t<4;t++){
      bool samaLama = true;
      for(int i=0;i<TOTAL_TABEL_PH && samaLama;i++){
        if(fabs(tb[t][i][0] - 200.0f*(float)(i+1)) > 0.5f) samaLama = false;
        if(fabs(tb[t][i][1] - pgm_read_float(&tabelSlopeLamaPGM[t][i])) > 0.00005f) samaLama = false;
      }
      if(samaLama){
        for(int i=0;i<TOTAL_TABEL_PH;i++){ tb[t][i][0]=pgm_read_float(&df[t][i][0]); tb[t][i][1]=pgm_read_float(&df[t][i][1]); }
        migrasiTabelPH = true;
      }
    }
    if(migrasiTabelPH) hitungKonstantaPIOtomatis();
  }
  // TES_134: data sebelum J0 ada -> J0 = J1 (perilaku lama: sesudah isi tandon / daya pulih menunggu J1)
  bool migrasiJ0 = false;
  if(!jedaJ0SudahAda){
    int* arr[2] = { dosingJedaHomogenisasi, dosingJedaHomogenisasiBaru };
    for(int a=0;a<2;a++){ int v = arr[a][0]; arr[a][IDX_JEDA_J0] = (v>=JEDA_MIN_MENIT && v<=JEDA_MAKS_MENIT) ? v : 0; }
    migrasiJ0 = true;
  }
  // Muat suhu kalibrasi pH dari alamat terpisah (acuan ATC Nernst). Guard bila EEPROM kosong/garbage.
  EEPROM.get(EEPROM_ALAMAT_SUHUKALPH, suhuKalibrasiPH);
  if(isnan(suhuKalibrasiPH) || suhuKalibrasiPH < -10.0f || suhuKalibrasiPH > 60.0f) suhuKalibrasiPH = 25.0f;
  // Migrasi SEKALI (TES_114): gain AB kini per sisi -> Kp/Ki otomatis tersimpan lama (separuh nilai benar) dihitung ulang.
  if(EEPROM.read(EEPROM_ALAMAT_MIGRASI) != MIGRASI_GAIN_PER_SISI){
    hitungKonstantaPIOtomatis();
    EEPROM.update(EEPROM_ALAMAT_MIGRASI, MIGRASI_GAIN_PER_SISI);
  }
  kalDraftMuat();   // setelah kalibrasi aktif & suhu kalibrasi pH dimuat (draft kosong diisi dari kalibrasi aktif)
  eepromMuatZNUji();   // TES_141: sesudah profil & target PPM dimuat
  eepromMuatZNRek();   // TES_145: rekaman kurva uji terakhir
  eepromSiap = true;
  if(migrasiJ0 || migrasiDebit || migrasiTabelPH || versiDataEkstra != VERSI_DATA_139) eepromAutoSimpan();   // TES_134/138/139: simpan hasil migrasi + penanda versi
}

void eepromAutoSimpan(){
  if(!eepromSiap) return;
  eepromKemas(g_eepromScratch);
  // Tidak perlu memcmp manual: EEPROM.put() bawaan Arduino sudah otomatis
  // hanya menulis byte yang benar-benar berubah (mekanisme update()-per-byte),
  // jadi menyimpan salinan "bayangan" penuh terpisah hanya utk pembanding
  // adalah pemborosan RAM (~500+ byte) yang tidak perlu.
  EEPROM.put(EEPROM_ALAMAT, g_eepromScratch);
  eepromSimpanEkstra();
  eepromSimpanZNUji();   // TES_141
}

void eepromSimpanRTC(){
  if(!eepromSiap) return;
  eepromKemas(g_eepromScratch);
  rtcCache.tahun=rtcInput[0]; rtcCache.bulan=rtcInput[1];
  rtcCache.tgl=rtcInput[2];   rtcCache.jam=rtcInput[3];
  rtcCache.menit=rtcInput[4]; rtcCache.detik=rtcInput[5];
  g_eepromScratch.rtcTahun=rtcCache.tahun; g_eepromScratch.rtcBulan=rtcCache.bulan;
  g_eepromScratch.rtcTgl=rtcCache.tgl;     g_eepromScratch.rtcJam=rtcCache.jam;
  g_eepromScratch.rtcMenit=rtcCache.menit; g_eepromScratch.rtcDetik=rtcCache.detik;
  EEPROM.put(EEPROM_ALAMAT, g_eepromScratch);
}

int ramMinimum = 32767;   // sisa RAM terendah sejak boot (indikator fragmentasi / kebocoran memori)
byte kreditHalaman = 0;
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup() {
  Wire.begin();
  Wire.setWireTimeout(25000, true); // batasi I2C agar tak menggantung selamanya saat bus macet
  lcd.init(); lcd.backlight();
  sensorSuhu.begin();
  sensorSuhu.setWaitForConversion(false);
  sensorSuhu.setResolution(12);
  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);

  const uint8_t RELAY_LEVEL_OFF = (RELAY_AKTIF_LOW ? HIGH : LOW);
  digitalWrite(PIN_RELAY_D1, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_D2, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_D3, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_P1, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_P2, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_P3, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_SOLENOID, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_BUZZER, RELAY_LEVEL_OFF);
  digitalWrite(PIN_RELAY_TDS, RELAY_LEVEL_OFF);
  pinMode(PIN_RELAY_D1, OUTPUT);
  pinMode(PIN_RELAY_D2, OUTPUT);
  pinMode(PIN_RELAY_D3, OUTPUT);
  pinMode(PIN_RELAY_P1, OUTPUT);
  pinMode(PIN_RELAY_P2, OUTPUT);
  pinMode(PIN_RELAY_P3, OUTPUT);
  pinMode(PIN_RELAY_SOLENOID, OUTPUT);
  pinMode(PIN_RELAY_BUZZER, OUTPUT);
  pinMode(PIN_RELAY_TDS, OUTPUT);
  relayOff(PIN_RELAY_BUZZER);
  matikanSemuaPompa();
  tutupSolenoid();

  // Inisialisasi RTC DS3231 (I2C). LCD juga di bus yang sama.
  rtcSiap = rtc.begin();
  if(!rtcSiap){
    // Tidak hard-stop: sistem tetap jalan, tapi fungsi berdasar waktu RTC dinonaktifkan
    // lewat rtcWaktuValid() sampai kabel/modul diperbaiki.
  }
  // Inisialisasi ADS1115 (ADC eksternal 16-bit, I2C 0x48). Berbagi bus dgn LCD & RTC.
  adsSiap = ads.begin(0x48);
  if(adsSiap) ads.setGain(GAIN_TWOTHIRDS);   // ±6,144 V: aman utk output modul sensor 0-5 V
  muatUlangKonstantaAktif();   // default tabel AB Mix & pH Down dulu
  eepromMuat();                // lalu timpa dgn parameter/kalibrasi/waktu tersimpan (jika ada)
  // RTC kehilangan daya (baterai habis / modul baru): TIDAK lagi disetel diam-diam ke waktu lama/compile.
  // Waktu dianggap tidak valid (jadwal ditahan + alarm) sampai user mengisi ulang Waktu RTC.
  if(rtcSiap){
    rtcPerluSet = false;
    // TES_146: DS3231 menahan akses I2C ~250 ms setelah VCC naik (tREC). Pembacaan terlalu dini bisa terbaca 0xFF
    // (lostPower palsu). Bila tampak tidak valid, tunggu lalu baca ulang; alarm hanya bila tetap tidak valid.
    if(rtc.lostPower() || !rtcWaktuValid()){
      delay(300);
      if(rtc.lostPower() || !rtcWaktuValid()) rtcPerluSet = true;
    }
  }
  // Sekali saat boot: daya TDS ON -> stabilisasi 5 dtk -> isi buffer TDS -> OFF -> stabilisasi 5 dtk
  nyalakanRelayTDS();
  delay((unsigned long)JEDA_STABIL_SENSOR_DETIK*1000UL);
  for(int i=0;i<TDS_SCOUNT;i++){tdsBuffer[i]=bacaAdcTDS();delay(20);}
  matikanRelayTDS();
  delay((unsigned long)JEDA_STABIL_SENSOR_DETIK*1000UL);
  for(int i=0;i<PH_SCOUNT;i++){phBuffer[i]=bacaAdcPH();delay(20);}

  sensorSuhu.requestTemperatures();
  tsSuhuMulai = millis();
  suhuKonversiSedangBerjalan = true;

  bacaSemua();
  lcd.clear();
  lcd.setCursor(3,0); lcd.print(F("SISTEM DOSING"));
  lcd.setCursor(3,1); lcd.print(F("HIDROPONIK v1.0"));
  delay(3000);
  lcd.clear();
  // Info penyebab restart & kondisi RTC
  if((mcusrSaatBoot & _BV(WDRF)) && !eepromBaruDibuat){
    lcdPrintCenter(1, F("PEMULIHAN OTOMATIS"));
    lcdPrintCenter(2, F("(program macet)"));
    delay(2500);
    lcd.clear();
  } else if(!(mcusrSaatBoot & _BV(PORF)) && (mcusrSaatBoot & (_BV(BORF) | _BV(EXTRF)))){
    // TES_149: penyebab restart utk diagnosa (mis. lonjakan katup solenoid): BOR = tegangan 5V turun, EXT = pin RESET
    lcdPrintCenter(0, F("TERJADI RESTART"));
    lcdPrintCenter(1, (mcusrSaatBoot & _BV(BORF)) ? F("Tegangan 5V turun") : F("Tombol/pin RESET"));
    lcdPrintCenter(2, F("Cek catu daya/relay"));
    lcdPrintCenter(3, F("& peredam solenoid"));
    delay(3500);
    lcd.clear();
  }
  if(rtcPerluSet){
    mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
    lcdPrintCenter(0, F("CEK KONDISI RTC"));
    lcdPrintCenter(1, F("Ganti baterai RTC /"));
    lcdPrintCenter(2, F("set ulang Waktu RTC"));
    lcdPrintCenter(3, F("(Pengaturan Sistem)"));
    unsigned long t0=millis(); while(millis()-t0<4000UL){ updateBuzzer(); if(customKeypad.getKey()!=NO_KEY) break; }
    lcd.clear();
  }
  wdt_enable(WDT_RUNTIME);   // mulai sini program wajib memanggil wdt_reset() < 8 dtk sekali
  // TES_148: daya putus saat commit Parameter Baru ditulis -> Parameter Aktif bisa setengah lama/baru -> jadwal dihentikan
  if(EEPROM.read(EEPROM_ALAMAT_KOMIT) == KOMIT_BERJALAN){
    EEPROM.update(EEPROM_ALAMAT_KOMIT, 0xFF);
    dosingSedangMenunggu = false; jadwalCekAktif = false;
    resetStatusParameterBaru(); eepromAutoSimpan();
    lcd.clear();
    lcdPrintCenter(0, F("SIMPAN TERPUTUS"));
    lcdPrintCenter(1, F("Cek Parameter Aktif"));
    lcdPrintCenter(2, F("lalu Mulai Dosing"));
    lcdPrintCenter(3, F("(jadwal dihentikan)"));
    mulaiBuzzerPutus(BUZZER_ALARM_DURASI_MS);
    jedaInfo(5000);
    lcd.clear();
  }
  // Resume otomatis setelah boot / daya pulih
  if((dosingSedangMenunggu || jadwalCekAktif) && !parameterLengkap()){
    // Mis. upgrade dari TES_115 (Batasan PPM/pH belum ada) -> jadwal lama dihentikan, user wajib atur Batasan dulu
    dosingSedangMenunggu = false;
    jadwalCekAktif = false;
    resetStatusParameterBaru();
    eepromAutoSimpan();
    // TES_126: item yang kurang ditampilkan langsung (dulu selalu "Batasan PPM/pH" walau penyebabnya item lain)
    const __FlashStringHelper* kurang = itemBelumLengkap(false);
    lcdPrintCenter(0, F("JADWAL DIHENTIKAN"));
    lcdPrintCenter(1, F("Lengkapi/atur ulang:"));
    lcdPrintCenter(2, kurang ? kurang : F("Parameter Dosing"));
    lcdPrintCenter(3, F("lalu Mulai Dosing"));
    jedaInfo(4000);
    lcd.clear();
    tampilkanMenuUtama();
  } else if(dosingSedangMenunggu){
    // Sedang MENUNGGU jadwal saat daya mati -> lanjutkan menunggu (target tersimpan).
    // Bila jam cek terlewat selama daya mati, cekWaktuDosingTercapai() langsung memicu cek (catch-up 1x).
    currentState = DOSING_TUNGGU_WAKTU;
    tampilkanDosingTungguWaktu(false);
  } else if(jadwalCekAktif){
    // Daya mati di TENGAH siklus -> siklus diulang dari awal (isi tandon -> cek). Aman dari dosis ganda:
    // gerbang Batasan hanya mendosis kanal yang masih di luar batas.
    lcdPrintCenter(0, F("DAYA PULIH"));
    lcdPrintCenter(1, F("Siklus cek diulang"));
    jedaInfo(2500);
    // TES_122: dosis yang mungkin baru masuk sebelum daya mati diberi waktu tercampur (jeda J1) dulu,
    // agar cek ulang tidak membaca larutan yang belum merata (cegah dosis ganda).
    mulaiJedaPulih();
  } else {
    tampilkanMenuUtama();
  }
}

// ============================================================
// TES SENSOR (Mode Kalibrasi) — bilas -> stabilisasi -> ukur (median 20 dtk)
// ============================================================
void tampilkanTesSensorInputBilas(){
  lcdPrintCenter(0, tesSensorJenis==0?F("TES SENSOR TDS"):F("TES SENSOR pH"));
  lcdPrintCenter(1,F("Lama Bilas(10-900s):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" detik"));
  lcdPrint20(0,3,F(""));
}
void tampilkanTesSensorInputStabil(){
  lcdPrintCenter(0, tesSensorJenis==0?F("TES SENSOR TDS"):F("TES SENSOR pH"));
  lcdPrintCenter(1,F("Jeda Stabil 10-900s:"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" detik"));
  lcdPrint20(0,3,F(""));
}
void tampilkanTesSensorProses(){
  bool modeTds = (tesSensorJenis==0);
  if(tesSensorFase==0)      lcdPrintCenter(0, modeTds?F("TDS: STABIL DAYA.."):F("pH: STABIL SENSOR.."));
  else if(tesSensorFase==1) lcdPrintCenter(0, modeTds?F("TDS: MEMBILAS.."):F("pH: MEMBILAS.."));
  else if(tesSensorFase==2) lcdPrintCenter(0, modeTds?F("TDS: STABILISASI.."):F("pH: STABILISASI.."));
  else                      lcdPrintCenter(0, modeTds?F("TDS: MENGUKUR.."):F("pH: MENGUKUR.."));
  // Bacaan live tiap detik, hanya sensor yang diuji
  char t[10];
  if(modeTds) lcdCenterP(1, PSTR("PPM: %s PPM"), teksPPM(t));
  else        lcdCenterP(1, PSTR("pH : %s"), teksPH(t));
  lcdCenterP(2, PSTR("Suhu: %s C"), teksSuhu(t));
  int target = (tesSensorFase==0)?JEDA_STABIL_SENSOR_DETIK:(tesSensorFase==1)?tesSensorLamaBilas:(tesSensorFase==2)?tesSensorJedaStabil:ROLL_MED_N;
  lcdCenterP(3, PSTR("%d/%d detik"), tesSensorDetik, target);
}
void tampilkanTesSensorHasil(){
  bool modeTds = (tesSensorJenis==0);
  lcdPrintCenter(0, modeTds?F("HASIL TES TDS"):F("HASIL TES pH"));
  if(modeTds) lcdPrintCenter(1, String(F("PPM median: "))+String(tesSensorHasilPPM));
  else        lcdPrintCenter(1, String(F("pH  median: "))+String(tesSensorHasilPH,2));
  lcdPrintCenter(2, String(F("Suhu: "))+formatSuhu()+F(" C"));
  lcdPrintCenter(3, F("<-"));
}
// Dipanggil tiap 1 detik dari loop() selama TES_SENSOR_PROSES
void prosesTesSensor(){
  tesSensorDetik++;
  if(tesSensorFase==0){                       // STABILISASI DAYA SENSOR (relay TDS baru ON/OFF)
    if(tesSensorDetik>=JEDA_STABIL_SENSOR_DETIK){
      relayOn(PIN_RELAY_D1);                  // baru mulai membilas chamber
      tesSensorFase=1; tesSensorDetik=0;
    }
  } else if(tesSensorFase==1){                // MEMBILAS: D1 ON
    if(tesSensorDetik>=tesSensorLamaBilas){
      relayOff(PIN_RELAY_D1);
      tesSensorFase=2; tesSensorDetik=0;
    }
  } else if(tesSensorFase==2){                // STABILISASI LARUTAN: D1 OFF
    if(tesSensorDetik>=tesSensorJedaStabil){
      tesSensorFase=3; tesSensorDetik=0;
      if(tesSensorJenis==0) resetPpmRoll15(); else resetPhRoll15();
    }
  } else {                                    // UKUR: median ROLL_MED_N detik terakhir
    if(tesSensorJenis==0) sampelPpmRoll15(); else sampelPhRoll15();
    if(tesSensorDetik>=ROLL_MED_N){
      if(tesSensorJenis==0) tesSensorHasilPPM = medianPpmRoll15();
      else                  tesSensorHasilPH  = medianPhRoll15();
      matikanRelayTDS();                      // selesai -> daya TDS OFF
      currentState = TES_SENSOR_HASIL;
      lcd.clear();
      tampilkanTesSensorHasil();
      return;
    }
  }
  tampilkanTesSensorProses();
}

// ============================================================================
// ISI AWAL LAYAR INPUT (TES_127): layar input parameter dibuka dengan NILAI YANG SEDANG DIPAKAI (konteks Aktif /
// Baru). B = pakai nilai itu, D = hapus, angka pertama yang diketik MENGGANTIKAN nilai lama (bukan ditambahkan).
// Input hasil ukur (debit mL/detik, buffer kalibrasi, suhu acuan), durasi tes pompa & kalkulator tetap kosong "_".
// ============================================================================
bool      inputIsiAwal = false;              // inputBuffer/ecBebasDigits masih berisi nilai lama
MenuState stateIsiAwalTerakhir = MENU_UTAMA; // state terakhir yang sudah diperiksa
int       jedaTunda[5] = {0,0,0,0,0};        // Jeda Homogenisasi J0..J4 (urutan input): di-commit BERSAMAAN di slot terakhir

// Kp/Ki ke teks <= 9 karakter, DIBULATKAN KE BAWAH (nilai tampil tak pernah melewati nilai tersimpan/batas aman).
// TES_138: desimal mengikuti besar nilai (>= 3 angka penting); dulu maks 5 desimal -> Ki < 0.00001 terisi "0".
void fmtIsiKp(char* b, float v){
  int dec = (v<0.001f)?8 : (v<0.01f)?7 : (v<0.1f)?6 : (v<10.0f)?5 : (v<100.0f)?4 : (v<1000.0f)?3 : 2;
  for(;;){
    float m = 1.0f; for(int i=0;i<dec;i++) m *= 10.0f;
    float t = floor(v*m + 0.001f)/m;
    dtostrf(t, 1, dec, b);
    int n = strlen(b);
    while(n>0 && b[n-1]=='0') b[--n] = '\0';     // buang nol di belakang koma
    if(n>0 && b[n-1]=='.') b[--n] = '\0';
    if(n<=9 || dec==0) break;
    dec--;
  }
}
// Isi inputBuffer dgn nilai yang sedang dipakai utk layar input s. true = ada nilai (layar perlu digambar ulang).
bool isiNilaiAwalInput(MenuState s, MenuState asal){
  bool baru = tampilBaru();   // konteks yang ditampilkan (sama dgn tujuan simpan: draft Baru dibuat dari Aktif)
  char b[12]; b[0] = '\0';
  switch(s){
    case MENU_VOL_TANDON:   { int v = baru ? terpilihVolumeBaru : terpilihVolume; if(v>0) itoa(v,b,10); } break;
    case MENU_TARGET_PPM:   { int v = baru ? terpilihPPMBaru : terpilihPPM;       if(v>0) itoa(v,b,10); } break;
    case MENU_TARGET_PH:    { float v = baru ? terpilihPHBaru : terpilihPH;      if(v>0.0f) dtostrf(v,4,2,b); } break;
    case MENU_WAKTU_DOSING: {
      const String& w = baru ? terpilihWaktuDosingBaru : terpilihWaktuDosing;
      if(waktuDosingFormatValid(w)){ b[0]=w.charAt(0); b[1]=w.charAt(1); b[2]=w.charAt(3); b[3]=w.charAt(4); b[4]='\0'; }
    } break;
    case MENU_DEBIT_POMPA_VAL: {
      int v = baru ? terpilihDebitPumpBaru[selDebitPump] : terpilihDebitPump[selDebitPump];
      if(v>0) snprintf_P(b,sizeof(b),PSTR("%d.%03d"),v/1000,v%1000);   // TES_138: 3 desimal
    } break;
    case DOSING_INPUT_LAMA_CEK:    { int v = baru ? terpilihLamaCekSensorBaru : terpilihLamaCekSensor;     if(v>=10 && v<=900) itoa(v,b,10); } break;
    case DOSING_INPUT_STABILISASI: { int v = baru ? terpilihJedaStabilisasiBaru : terpilihJedaStabilisasi; if(v>=10 && v<=900) itoa(v,b,10); } break;
    case DOSING_INPUT_JEDA:        { int v = jedaTunda[dosingJedaInputIdx]; if(v>=JEDA_MIN_MENIT && v<=JEDA_MAKS_MENIT) itoa(v,b,10); } break;
    case MENU_BATASAN_PPM: {
      // kembali dari layar pH (*) -> nilai yang baru saja diketik; selain itu batasan tersimpan
      int v = (asal==MENU_BATASAN_PH) ? batasPPMTunda
            : ((baru ? batasanBaruSudahDiatur : batasanSudahDiatur) ? (baru ? batasPPMBaru : batasPPM) : 0);
      if(batasPPMValid(v)) itoa(v,b,10);
    } break;
    case MENU_BATASAN_PH: {
      int v = (baru ? batasanBaruSudahDiatur : batasanSudahDiatur) ? (baru ? batasPHx100Baru : batasPHx100) : 0;
      if(batasPHValid(v)) snprintf_P(b,sizeof(b),PSTR("%03d"),v);
    } break;
    case MENU_PERSEN_MAKRO:   return inputBuffer.length()>0;   // sudah diisi saat memilih jenis dosing
    case UJI_ZN_INPUT_DURASI: return inputBuffer.length()>0;   // sudah diisi lama rekam terakhir
    case UJI_ZN_INPUT_VOLUME: { int v = (ujiVolumeL>0) ? ujiVolumeL : (baru ? terpilihVolumeBaru : terpilihVolume);   // TES_141
                                if(v>=UJI_VOL_MIN && v<=UJI_VOL_MAKS) itoa(v,b,10); } break;
    case MENU_EC_BEBAS: {
      String e = (asal==MENU_PENGENCERAN && ecTunda.length()>0) ? ecTunda : (baru ? terpilihECBaru : terpilihEC);
      float v = e.toFloat();
      if(v>=EC_BEBAS_MIN && v<=EC_BEBAS_MAKS){
        int x = (int)(v*100.0f + 0.5f);
        char d[4]; snprintf_P(d,sizeof(d),PSTR("%03d"),x);
        ecBebasDigits = d; return true;
      }
      return false;
    }
    case TES_SENSOR_INPUT_BILAS:  { int v = tesSensorLamaBilas>0 ? tesSensorLamaBilas : terpilihLamaCekSensor;    if(v>=10 && v<=900) itoa(v,b,10); } break;
    case TES_SENSOR_INPUT_STABIL: { int v = tesSensorJedaStabil>0 ? tesSensorJedaStabil : terpilihJedaStabilisasi; if(v>=10 && v<=900) itoa(v,b,10); } break;
    // Waktu RTC: hanya TANGGAL yang diisi awal. Jam/menit/detik wajib diketik: nilai awal akan basi selama menu
    // dibuka (B beruntun akan memundurkan jam beberapa detik tanpa disadari).
    case MENU_SET_RTC_TAHUN: if(rtcInput[0]>=2000 && rtcInput[0]<=2099) itoa(rtcInput[0],b,10); break;
    case MENU_SET_RTC_BULAN: if(rtcInput[1]>=1 && rtcInput[1]<=12) snprintf_P(b,sizeof(b),PSTR("%02d"),rtcInput[1]); break;
    case MENU_SET_RTC_TGL:   if(rtcInput[2]>=1 && rtcInput[2]<=31) snprintf_P(b,sizeof(b),PSTR("%02d"),rtcInput[2]); break;
    case MENU_EDIT_KPKI_ZN: {
      if(znEditTahap!=0) return false;
      // TES_141: isi awal = nilai EFEKTIF utk Parameter konteks (sudah diskalakan dari tandon uji & dibatasi) =
      // satuan yang sama dgn nilai yg diketik (edit manual berlaku utk volume & PPM target Parameter konteks)
      const ProfilKpKiZN &pz = profilZNKonteks(baru);
      float kp, ki; kpKiZNEfektif(halKpKiZN, baru, kp, ki);
      if(pz.terisi && kp>0.0f) fmtIsiKp(b, kp);
    } break;
    default: return false;
  }
  if(b[0]=='\0') return false;
  inputBuffer = b;
  return true;
}
// Dipanggil tiap putaran loop(): saat layar input baru dibuka -> isi nilai lama & gambar ulang
void cekIsiAwalInput(){
  if(currentState==stateIsiAwalTerakhir) return;
  MenuState asal = stateIsiAwalTerakhir;
  stateIsiAwalTerakhir = currentState;
  inputIsiAwal = false;
  if(statusOverlayAktif || cancelConfirmAktif) return;
  if(isiNilaiAwalInput(currentState, asal)){ inputIsiAwal = true; gambarUlangLayarAktif(); }
}
// Isi awal di dalam state yang sama (slot Jeda berikutnya, tahap Ki pada edit Kp/Ki)
void isiAwalLangsung(const char* teks){
  inputBuffer = teks; inputIsiAwal = (inputBuffer.length()>0);
}

// ==========
// LOOP UTAMA
// ==========
void loop() {
  wdt_reset();
  cekIsiAwalInput();
  updateSuhuAsync();
  tickUjiStepResponse();   // logging kurva reaksi non-blocking, aktif hanya saat UJI_ZN_LOGGING
  if(ujiPulsaAktif && currentState!=UJI_ZN_LOGGING) setelPompaUji(false);   // TES_138: jaring pengaman pompa pulsa uji
  updateBuzzer();   // matikan buzzer otomatis setelah durasi habis (non-blocking)

  unsigned long now=millis();
  pantauLCD(now);
  if(now-tsBacaTerakhir>=INTERVAL_SENSOR){
    tsBacaTerakhir=now;
    bool tdsPerlu = sensorTdsPerluAktif();
    bool phPerlu  = sensorPhPerluAktif();
    if(!tdsPerlu && tdsRelayAktif) matikanRelayTDS();   // jaring pengaman: TDS wajib mati saat tak dipakai
    if(ujiD1Aktif && currentState!=UJI_ZN_LOGGING){ relayOff(PIN_RELAY_D1); ujiD1Aktif=false; }   // jaring pengaman: D1 uji tak boleh menyala di luar logging
    bool sensorAktifKini = (tdsPerlu || phPerlu);
    if(sensorAktifKini && !sensorAktifSebelumnya) primingBufferSensorSelektif(tdsPerlu, phPerlu); // baru aktif -> segarkan buffer
    sensorAktifSebelumnya = sensorAktifKini;
    if(sensorAktifKini) bacaSensorSelektif(tdsPerlu, phPerlu);
    if(currentState==MENU_SUHU_KALIBRASI){ tampilkanKalkSuhuInput(); }
    // Proses tes pompa non-blocking
    if(tesPompaBerjalan){
      // Tes pompa berbasis millis(): tetap berjalan di latar belakang walau layar berpindah / cancel ditekan
      long dtk=(long)((millis()-tesPompaMulaiMs)/1000UL);
      if(dtk>tesPompaDetikTarget) dtk=tesPompaDetikTarget;
      tesPompaDetikBerjalan=(int)dtk;
      if(currentState==TES_POMPA_AKTIF) tampilkanTesPompaAktif();
      if(tesPompaDetikBerjalan>=tesPompaDetikTarget){
        relayOff(pumpRelayPin[tesPompaIdx]);
        tesPompaBerjalan=false;
        if(currentState==TES_POMPA_AKTIF){
          konfirmasiSukses(F("Tes Pompa Selesai!"), SUB_MENU_C);
        } else if(currentState==TES_POMPA_CANCEL_CONFIRM){
          // tes selesai saat layar masih di konfirmasi cancel -> kembali ke pilih pompa
          currentState=SUB_MENU_C; lcd.clear(); tampilkanMenuPilihPompa();
        }
      }
    }
    if(currentState==TES_SENSOR_PROSES) prosesTesSensor();   // Tes Sensor: bilas -> stabil -> ukur
    // TES_148: jaring pengaman katup: hanya boleh terbuka saat isi tandon dosing / tes katup / menu Water Level
    if(solenoidTerbuka && currentState!=DOSING_ISI_TANDON && currentState!=MENU_TES_SOLENOID && currentState!=MENU_WATER_LEVEL){ tutupSolenoid(); wlIsiAktif=false; }
    if(currentState==MENU_TES_SOLENOID){
      // Pengaman tes katup: tutup otomatis setelah 15 menit agar tandon tak meluap bila ditinggal
      if(solenoidTerbuka && (millis()-solenoidTesMulai >= SOLENOID_TES_MAKS_MS)){
        tutupSolenoid();
        mulaiBuzzer(BUZZER_DURASI_MS);
      }
      // TES_128: sensor water level tercapai (bertahan 3 dtk, sama dgn pengisian dosing) -> katup ditutup otomatis
      if(solenoidTerbuka && digitalRead(PIN_WATER_LEVEL)==HIGH){
        if(solenoidTesPenuhDetik < 255) solenoidTesPenuhDetik++;
        if(solenoidTesPenuhDetik >= TANDON_PENUH_KONFIRMASI_DETIK){ tutupSolenoid(); solenoidTesPenuh = true; mulaiBuzzer(BUZZER_DURASI_MS); }
      } else solenoidTesPenuhDetik = 0;
      tampilkanTesSolenoid();
    }
    updateDosingProses();
    cekPemicuDosingLatarBelakang();
    cekMenuMenganggur();   // TES_135
    // Overlay "Cancel Dosing?" dibiarkan tanpa jawaban -> anggap "Tidak" (proses dosing tetap berjalan normal)
    if(cancelConfirmAktif && (millis()-cancelConfirmMulaiMs >= CANCEL_CONFIRM_TIMEOUT_MS)) tutupCancelDosing();
    eepromAutoSimpan();   // simpan otomatis bila ada perubahan (1 slot, ditimpa)
    { int fr = freeRam(); if(fr < ramMinimum) ramMinimum = fr; }   // pantau sisa RAM terendah sejak boot
    // Overlay Status Dosing tetap menimpa tampilan fase asli 

    if(statusOverlayAktif && (statusFaseDosingBerjalan(currentState) || currentState==MENU_UTAMA)){
      tampilkanStatusDosing();
    }
    // Layar status akhir persisten pasca dosing selesai
    if(currentState==DOSING_FINAL_STATUS){
      tampilkanStatusDosing();
    }
  }
  if(now-tsKalTerakhir>=INTERVAL_KAL){
    tsKalTerakhir=now;
    if(currentState==MENU_KAL_TDS_SAMPEL) { sampelKalTds(); tampilkanKalibrasiTDSLayar(0); }
    if(currentState==MENU_KAL_PH_SAMPEL)  { sampelKalPh();  tampilkanKalibrasiPHLayar(0); }
    if(currentState==MENU_WATER_LEVEL) tampilkanWaterLevel();
  }
  if(now-tsRtcTerakhir>=INTERVAL_RTC){
    tsRtcTerakhir=now;
    if(currentState==MENU_SET_RTC_HASIL) tampilkanRTCHasil();
  }
  // Cek kesehatan RTC berkala (baterai lepas / kabel longgar saat runtime).
  // Tidak auto-adjust waktu diam-diam agar jadwal dosing tidak bergeser tanpa sepengetahuan user.
  // rtcWaktuValid() akan menolak jadwal bila waktu chip tidak wajar; user set ulang lewat menu RTC.
  if(now-tsRtcHealth>=INTERVAL_RTC_HEALTH){
    tsRtcHealth=now;
    // Probe ACK alamat DS3231 (0x68): tanpa rtc.begin() yg meng-init ulang TWI & (tergantung versi RTClib)
    // mengalokasikan objek I2C baru tiap menit. lostPower()/waktu invalid tetap ditahan rtcWaktuValid().
    Wire.beginTransmission(0x68);
    rtcSiap = (Wire.endTransmission()==0);
    if(rtcSiap && rtc.lostPower()) rtcPerluSet = true;   // osilator RTC sempat berhenti -> waktu tak bisa dipercaya
  }
  // Timer scroll tampilan ringkasan Jeda Homogenisasi 
  if(now-tsScrollJeda>=400UL){
    tsScrollJeda=now;
    if(currentState==DOSING_INPUT_JEDA) tampilkanDosingInputJeda(dosingJedaInputIdx);
  }

  char key=customKeypad.getKey();
  if(!key) return;
  tombolTerakhirMs = millis();   // TES_135: pemantau menu menganggur
  if(promptTibaAktif){ prosesTombolPromptTiba(key); return; }   // TES_135: layar TIBA WAKTU DOSING

  // Overlay Status Dosing: B = maju (halaman berikutnya -> layar proses/tunggu), * = mundur (halaman sebelumnya -> asal)
  // TES_146: saat menunggu cek berikutnya ada 3 halaman (menunggu, hasil cek terakhir, jadwal & dosis); saat siklus berjalan tetap 2.
  if(statusOverlayAktif){
    byte halMaks = statusModeMenunggu() ? 2 : 1;
    if(statusHalaman > halMaks) statusHalaman = halMaks;   // TES_148
    if(key=='B'){
      if(statusHalaman<halMaks){ statusHalaman++; lcd.clear(); tampilkanStatusDosing(); }
      else {
        statusHalaman=0;
        if(currentState==MENU_UTAMA && !dosingSedangMenunggu){ lcd.clear(); tampilkanStatusDosing(); }  // tak ada jadwal: kembali ke halaman pertama
        else {
          statusOverlayAktif=false; lcd.clear();
          if(currentState==MENU_UTAMA){ currentState=DOSING_TUNGGU_WAKTU; tampilkanDosingTungguWaktu(false); }
          else tampilkanUlangFaseDosing();
        }
      }
    }
    else if(key=='*'){
      if(statusHalaman>0){ statusHalaman--; lcd.clear(); tampilkanStatusDosing(); }
      else {
        statusOverlayAktif=false; lcd.clear();
        if(currentState==MENU_UTAMA) tampilkanMenuUtama();
        else tampilkanUlangFaseDosing();
      }
    }
    return;   // tombol lain diabaikan
  }

  // ── OVERLAY "Cancel Dosing? Ya/Tidak" ──
  if(cancelConfirmAktif){
    if     (key=='A'){if(selCancelDosing>0){selCancelDosing--;tampilkanCancelDosing();}}
    else if(key=='C'){if(selCancelDosing<1){selCancelDosing++;tampilkanCancelDosing();}}
    else if(key=='B'){
      if(selCancelDosing==0){
        // Ya, batalkan dosing: SEMUA aktuator langsung dimatikan
        matikanSemuaPompa();             // D1-D3, P1-P3
        tutupSolenoid();
        matikanRelayTDS();
        mulaiBuzzer(BUZZER_DURASI_MS);   // alarm: dosing dibatalkan manual
        dosingSedangMenunggu=false;
        jadwalCekAktif=false;   // pembatalan manual -> jadwal cek tidak lagi ter-arm
        resetStatusParameterBaru();   // draft Parameter Baru ikut dibuang (cegah draft usang terpakai nanti)
        piReset();
        makroPulih = 0; siklusDariPulih = false; ulangPulihAB = false; ulangPulihPH = false;   // TES_138
        eepromAutoSimpan();   
        statusPPMTerakhir=-1; statusPHTerakhir=-1.0f;
        dosingPPMTerkunci=false; dosingPHTerkunci=false;
        cancelConfirmAktif=false;
        currentState=MENU_UTAMA; lcd.clear(); tampilkanMenuUtama();
      } else {
        tutupCancelDosing();             // Tidak: proses (yang tetap berjalan selama overlay) ditampilkan lagi
      }
    }
    else if(key=='*'){
      tutupCancelDosing();
    }
    return;
  }

  // Isi awal: angka pertama menggantikan nilai lama; '#' tidak menghapus status isi awal; tombol lain mengakhirinya
  if(inputIsiAwal){
    if(key>='0' && key<='9'){ inputBuffer=""; if(currentState==MENU_EC_BEBAS) ecBebasDigits=""; }
    // TES_148: hanya tombol yang BERTINDAK mengakhiri isi awal. A/C/# di layar angka bulat tak berbuat apa-apa -> isi awal
    // tetap (dulu '#' lalu '5' pada "200" menjadi "2005"). '#' mengakhiri isi awal hanya di edit Kp/Ki ZN bila nilai
    // awal bilangan bulat ('12' '#' '5' -> '12.5').
    if(key>='0' && key<='9') inputIsiAwal=false;
    else if(key=='B' || key=='D' || key=='*') inputIsiAwal=false;
    else if(key=='#' && currentState==MENU_EDIT_KPKI_ZN && inputBuffer.length()>0 && inputBuffer.indexOf('.')<0) inputIsiAwal=false;
  }

  // ── MENU UTAMA ──
  if(currentState==MENU_UTAMA) {
    if     (key=='A'){naik(selMenuUtama,scrollMenuUtama,TOTAL_MENU_UTAMA_ROWS);tampilkanMenuUtama();}
    else if(key=='C'){turun(selMenuUtama,scrollMenuUtama,TOTAL_MENU_UTAMA_ROWS);tampilkanMenuUtama();}
    else if(key=='*'){currentState=MENU_UTAMA;tampilkanMenuUtama();}
    else if(key=='B'){
      lcd.clear(); inputBuffer="";
      if(selMenuUtama==0){ // Mulai Dosing
        if(!rtcSiapUntukMulai()) return;   // TES_127: cek RTC paling awal
        if(dosingSedangMenunggu){
          // Jadwal cek harian sedang menunggu jadwal berikutnya.
          if(draftBaruAda && parameterBaruLengkap()){
            // Ada draft Parameter Baru lengkap -> tawarkan pakai parameter baru
            selMulaiDosingKonfirmasi=1;
            currentState=MULAI_DOSING_KONFIRMASI;
            tampilkanMulaiDosingKonfirmasi();
          } else if(draftBaruAda){
            // Draft Parameter Baru ada tapi belum lengkap -> sebutkan item yg kurang; jadwal berjalan tak disentuh
            tampilkanErrorLengkapi(true);
            currentState=MENU_UTAMA;
            tampilkanMenuUtama();
          } else {
            // Belum ada perubahan parameter -> jadwal berjalan apa adanya
            lcd.clear();
            lcdPrintCenter(0, F("DOSING SEDANG"));
            lcdPrintCenter(1, F("MENUNGGU JADWAL"));
            lcdPrintCenter(2, F("Tidak ada perubahan"));
            lcdPrintCenter(3, F("Parameter Baru"));
            jedaInfo(2500);
            currentState=MENU_UTAMA;
            lcd.clear();
            tampilkanMenuUtama();
          }
        } else if(parameterLengkap()){
          if(validasiKpKiZNSebelumDosing()){
            if(perluPeringatanResolusi(false)) mulaiPeringatanResolusi(false);   // TES_125
            else                               mulaiDosingSekarang();
          } else {
            currentState=MENU_UTAMA; lcd.clear(); tampilkanMenuUtama();
          }
        } else {
          tampilkanErrorLengkapi(false);
          currentState=MENU_UTAMA;
          tampilkanMenuUtama();
        }
      }
      else if(selMenuUtama==1){ // Status Dosing dari Menu Utama -> langsung Hal.1 (status terpenting)
        statusOverlayAktif = true; statusHalaman = 0;
        tampilkanStatusDosing();
      }
      else if(selMenuUtama==2){currentState=MENU_RINGKASAN;scrollSummary=0;tampilkanMenuRingkasan();}
      else if(selMenuUtama==3){currentState=MENU_RINGKASAN_BARU;scrollSummaryBaru=0;tampilkanMenuRingkasanBaru();}
      else if(selMenuUtama==4){currentState=SUB_MENU_A;selMenuA=0;scrollMenuA=0;tampilkanSubMenuA();}
      else if(selMenuUtama==5){currentState=MENU_PPM_PH_TANAMAN;selPPMPHTanaman=0;scrollPPMPHTanaman=0;tampilkanMenuPPMPHTanaman();}
      else if(selMenuUtama==6){currentState=SUB_MENU_B;selMenuB=0;scrollMenuB=0;tampilkanSubMenuB();}
      else if(selMenuUtama==7){currentState=SUB_MENU_C;selPump=0;scrollPump=0;tampilkanMenuPilihPompa();}
      else if(selMenuUtama==8){currentState=SUB_MENU_D;selMenuD=0;scrollMenuD=0;tampilkanSubMenuD();}
      else if(selMenuUtama==9){ currentState=MENU_PENGATURAN_SISTEM; selPengaturanSistem=0; scrollPengaturanSistem=0; tampilkanMenuPengaturanSistem(); }   // TES_127: scroll ikut direset (dulu kursor tak terlihat)
      else if(selMenuUtama==10){ currentState=MENU_KREDIT; tampilkanMenuKredit(); }
    }
  }

  // ── KONFIRMASI "ATUR DOSING BARU?" (muncul saat Mulai Dosing & Parameter Baru lengkap) ──
  else if(currentState==MULAI_DOSING_KONFIRMASI){
    if     (key=='A'){if(selMulaiDosingKonfirmasi>0){selMulaiDosingKonfirmasi--;tampilkanMulaiDosingKonfirmasi();}}
    else if(key=='C'){if(selMulaiDosingKonfirmasi<1){selMulaiDosingKonfirmasi++;tampilkanMulaiDosingKonfirmasi();}}
    else if(key=='*'){currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
    else if(key=='B'){
      lcd.clear();
      if(selMulaiDosingKonfirmasi==0){
        // Ya: pakai Parameter Baru. Kp/Ki ZN tetap divalidasi dulu (keselamatan kontrol).
        if(!validasiKpKiZNParameterBaru()){
          // Mode CL PI ZN tapi Kp/Ki profil tidak cocok dgn draft Baru -> JANGAN commit;
          // draft Baru & jadwal tunggu yg sedang berjalan dibiarkan utuh.
          lcd.clear();
          currentState=MENU_UTAMA;
          tampilkanMenuUtama();
        } else {
          if(perluPeringatanResolusi(true)) mulaiPeringatanResolusi(true);   // TES_125: cek resolusi Param. Baru
          else                              komitParameterBaruLaluMulai();
        }
      } else {
        // Tidak: biarkan jadwal cek harian yang sedang menunggu tetap berjalan apa adanya.
        currentState=MENU_UTAMA;
        lcd.clear();
        tampilkanMenuUtama();
      }
    }
  }

  // ── RINGKASAN PARAMETER BARU ──
  else if(currentState==MENU_RINGKASAN_BARU){
    if     (key=='A'){if(scrollSummaryBaru>0){scrollSummaryBaru--;tampilkanMenuRingkasanBaru();}}
    else if(key=='C'){if(scrollSummaryBaru<maksScrollRingkasan(true)){scrollSummaryBaru++;tampilkanMenuRingkasanBaru();}}
    else{currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
  }

  // ── MENU KREDIT ──
  else if(currentState==MENU_KREDIT){
    // B = halaman Info Sistem (RAM, waktu nyala); tombol lain kembali
    if(key=='B' && kreditHalaman==0){ kreditHalaman=1; tampilkanMenuKredit(); }
    else { kreditHalaman=0; currentState=MENU_UTAMA; lcd.clear(); tampilkanMenuUtama(); }
  }

  // ── MENU "PENGATURAN PERSAMAAN" ──────
  else if(currentState==MENU_PENGATURAN_SISTEM){
    if     (key=='A'){naik(selPengaturanSistem,scrollPengaturanSistem,TOTAL_PENGATURAN_SISTEM_ROWS);tampilkanMenuPengaturanSistem();}
    else if(key=='C'){turun(selPengaturanSistem,scrollPengaturanSistem,TOTAL_PENGATURAN_SISTEM_ROWS);tampilkanMenuPengaturanSistem();}
    else if(key=='*'){currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
    else if(key=='B'){
      lcd.clear();
      if(selPengaturanSistem==0){
        selPengaturan=0;
        currentState=MENU_PENGATURAN_PERSAMAAN;
        tampilkanMenuPengaturanPersamaan();
      } else if(selPengaturanSistem==1){
        inputBuffer="";   // TES_144: sisa input lama tidak terbawa ke kolom tahun
        for(int i=0;i<6;i++)rtcInput[i]=0;
        if(rtcWaktuValid()){ DateTime n=rtc.now(); rtcInput[0]=n.year(); rtcInput[1]=n.month(); rtcInput[2]=n.day(); }   // TES_127: tanggal diisi awal
        currentState=MENU_SET_RTC_TAHUN;
        tampilkanInputRTC(0);
      } else if(selPengaturanSistem==2){
        selZNHub=0;
        currentState=MENU_ZN_HUB;
        tampilkanMenuZNHub();
      } else if(selPengaturanSistem==3){
        selKunciKanal=0;
        currentState=MENU_KUNCI_KANAL;
        tampilkanMenuKunciKanal();
      } else {
        selResetSistemConfirm=1;
        currentState=RESET_SISTEM_CONFIRM;
        tampilkanResetSistemConfirm();
      }
    }
  }

  // ── HUB PENGATURAN "ZN MANUAL Kp/Ki" ──
  else if(currentState==MENU_ZN_HUB){
    if     (key=='A'){if(selZNHub>0){selZNHub--;tampilkanMenuZNHub();}}
    else if(key=='C'){if(selZNHub<2){selZNHub++;tampilkanMenuZNHub();}}   // TES_145: 3 pilihan
    else if(key=='*'){currentState=MENU_PENGATURAN_SISTEM;lcd.clear();tampilkanMenuPengaturanSistem();}
    else if(key=='B'){
      lcd.clear();
      if(selZNHub==0){              // Uji Reaksi Baru
        ujiKanalDipilih=0;
        currentState=UJI_ZN_PILIH_KANAL;
        tampilkanUjiZNPilihKanal();
      } else if(selZNHub==1){       // TES_145: Rekaman Terakhir
        znRekKanal=(ujiKanalDipilih>=0 && ujiKanalDipilih<=2 && znRekN[ujiKanalDipilih]>0) ? ujiKanalDipilih : znRekKanalPertama(); znRekScroll=0;   // TES_148: kanal uji terakhir dulu
        currentState=MENU_ZN_REKAMAN;
        tampilkanRekamanZN();
      } else {                      // Lihat Kp/Ki Aktif
        halKpKiZN=0;
        currentState=MENU_LIHAT_KPKI_ZN;
        tampilkanKpKiZN();
      }
    }
  }

  // ── LIHAT Kp/Ki AKTIF (poin 3: 3 kategori, bisa digeser) ──
  else if(currentState==MENU_LIHAT_KPKI_ZN){
    if     (key=='A'){if(halKpKiZN>0){halKpKiZN--;tampilkanKpKiZN();}}
    else if(key=='C'){if(halKpKiZN<2){halKpKiZN++;tampilkanKpKiZN();}}
    else if(key=='*'){currentState=MENU_ZN_HUB;lcd.clear();tampilkanMenuZNHub();}
    else if(key=='B'){
      inputBuffer=""; znEditTahap=0;
      currentState=MENU_EDIT_KPKI_ZN;
      lcd.clear();
      tampilkanEditKpKiZN();
    }
  }
  else if(currentState==MENU_EDIT_KPKI_ZN){
    prosesEditKpKiZN(key);
  }

  // ── TES_145: REKAMAN TERAKHIR (A/C geser, B pindah kanal, * kembali) ──
  else if(currentState==MENU_ZN_REKAMAN){
    if(key=='A'){ if(znRekScroll>0){ znRekScroll--; tampilkanRekamanZN(); } }
    else if(key=='C'){
      if(znRekKanal>=0){
        int vis = (znRekKanal==0) ? 3 : 2;
        if(znRekScroll + vis < (int)znRekN[znRekKanal]){ znRekScroll++; tampilkanRekamanZN(); }
      }
    }
    else if(key=='B'){
      if(znRekKanal>=0){
        int nx = znRekKanalBerikut(znRekKanal);
        if(nx != znRekKanal){ znRekKanal = nx; znRekScroll = 0; lcd.clear(); tampilkanRekamanZN(); }
      }
    }
    else if(key=='*'){ currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub(); }
  }

  // ── UJI STEP-RESPONSE: pilih kanal ──
  else if(currentState==UJI_ZN_PILIH_KANAL){
    if     (key=='A'){if(ujiKanalDipilih>0){ujiKanalDipilih--;tampilkanUjiZNPilihKanal();}}
    else if(key=='C'){if(ujiKanalDipilih<2){ujiKanalDipilih++;tampilkanUjiZNPilihKanal();}}
    else if(key=='*'){currentState=MENU_ZN_HUB;lcd.clear();tampilkanMenuZNHub();}
    else if(key=='B'){   // TES_141: volume tandon uji dulu (isi awal = volume uji terakhir / volume tandon)
      inputBuffer=""; currentState=UJI_ZN_INPUT_VOLUME; lcd.clear(); tampilkanUjiZNInputVolume();
    }
  }
  else if(currentState==UJI_ZN_INPUT_VOLUME){
    if(key>='0'&&key<='9'){ if(inputBuffer.length()<4){ inputBuffer+=key; tampilkanUjiZNInputVolume(); } }
    else if(key=='D'){ inputBuffer=""; tampilkanUjiZNInputVolume(); }
    else if(key=='*'){ inputBuffer=""; currentState=UJI_ZN_PILIH_KANAL; lcd.clear(); tampilkanUjiZNPilihKanal(); }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v<UJI_VOL_MIN || v>UJI_VOL_MAKS){
        char e[21]; snprintf_P(e,sizeof(e),PSTR("Batas: %d-%d L"),UJI_VOL_MIN,UJI_VOL_MAKS);
        tampilkanError(e); inputBuffer=""; tampilkanUjiZNInputVolume(); return;
      }
      ujiVolumeL=v; inputBuffer="";
      if(!ujiSiapkanPulsa()) return;   // TES_138: parameter tak lengkap / debit terlalu kecil -> ditolak sebelum pompa nyala
      inputBuffer=String(max(ujiDurasiMaksMenit, ujiRekamMinMenit)); currentState=UJI_ZN_INPUT_DURASI; lcd.clear(); tampilkanUjiZNInputDurasi();
    }
  }
  else if(currentState==UJI_ZN_INPUT_DURASI){
    if(key>='0'&&key<='9'){ if(inputBuffer.length()<3){ inputBuffer+=key; tampilkanUjiZNInputDurasi(); } }   // TES_138: 3 digit (s/d 240)
    else if(key=='D'){ inputBuffer=""; tampilkanUjiZNInputDurasi(); }
    else if(key=='*'){ inputBuffer=""; currentState=UJI_ZN_INPUT_VOLUME; lcd.clear(); tampilkanUjiZNInputVolume(); }   // TES_141
    else if(key=='B'){
      int v=inputBuffer.toInt();
      if(v<ujiRekamMinMenit || v>UJI_REKAM_MAKS_MENIT){
        char e[21]; snprintf_P(e,sizeof(e),PSTR("Batas: %d-%d menit"),ujiRekamMinMenit,UJI_REKAM_MAKS_MENIT);
        tampilkanError(e); inputBuffer=""; tampilkanUjiZNInputDurasi(); return;
      }
      ujiDurasiMaksMenit=v; inputBuffer="";
      if(ujiBentrokJadwal()){   // TES_139: uji dibatalkan; kembali ke input lama rekam (bisa dipersingkat / * kembali)
        tampilkanBentrokJadwalUji();
        inputBuffer=String(v); inputIsiAwal=true;
        lcd.clear(); tampilkanUjiZNInputDurasi();
        return;
      }
      mulaiUjiStepResponse();
    }
  }
  else if(currentState==UJI_ZN_LOGGING){
    // tidak ada input selama proses logging berjalan (non-blocking via tickUjiStepResponse() di loop())
    if(key=='*'){
      hentikanUjiReaksi();   // batal -> D1 & pompa pulsa (bila masih menyala) mati
      currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
    }
  }
  else if(currentState==UJI_ZN_KONFIRMASI){
    prosesKonfirmasiUjiZN(key);
  }
  else if(currentState==ZN_KONFIRMASI_JADWAL){
    prosesZNKonfirmasiJadwal(key);
  }

  else if(currentState==MENU_PENGATURAN_PERSAMAAN){
    if     (key=='A'){if(selPengaturan>0){selPengaturan--;tampilkanMenuPengaturanPersamaan();}}
    else if(key=='C'){if(selPengaturan<2){selPengaturan++;tampilkanMenuPengaturanPersamaan();}}
    else if(key=='*'){currentState=MENU_PENGATURAN_SISTEM;lcd.clear();tampilkanMenuPengaturanSistem();}
    else if(key=='B'){
      lcd.clear();
      if(selPengaturan==0){
        selPengaturanABDil=0;
        currentState=PENGATURAN_AB_PILIH_DIL;
        tampilkanPengaturanABPilihDil();
      } else if(selPengaturan==1){
        selPengaturanPHAsam=0;
        currentState=PENGATURAN_PH_PILIH_ASAM;
        tampilkanPengaturanPHPilihAsam();
      } else {
        selPengaturanResetConfirm=1;
        currentState=PENGATURAN_RESET_CONFIRM;
        tampilkanPengaturanResetConfirm();
      }
    }
  }

  else if(currentState==PENGATURAN_AB_PILIH_DIL){
    if     (key=='A'){if(selPengaturanABDil>0){selPengaturanABDil--;tampilkanPengaturanABPilihDil();}}
    else if(key=='C'){if(selPengaturanABDil<1){selPengaturanABDil++;tampilkanPengaturanABPilihDil();}}
    else if(key=='*'){currentState=MENU_PENGATURAN_PERSAMAAN;lcd.clear();tampilkanMenuPengaturanPersamaan();}
    else if(key=='B'){
      selPengaturanABEC=0; scrollPengaturanABEC=0; inputBuffer="";
      currentState=PENGATURAN_AB_LIST;
      lcd.clear();
      tampilkanPengaturanABList();
    }
  }

  else if(currentState==PENGATURAN_AB_LIST){
    if(key=='*'){
      currentState=PENGATURAN_AB_PILIH_DIL; inputBuffer=""; lcd.clear(); tampilkanPengaturanABPilihDil();
    }
    else if(key=='D'){
      inputBuffer=""; tampilkanPengaturanABList();
    }
    else if(key=='#'){
      if(inputBuffer.indexOf('.')<0 && inputBuffer.length()>=1 && inputBuffer.length()<=3){
        inputBuffer+='.'; tampilkanPengaturanABList();
      }
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      float val=bufferToFloat();
      float defAB = regresiSlopeABDefault[selPengaturanABDil];
      if(val>=0.25f*defAB && val<=4.0f*defAB){   // salah ketik (mis. geser koma) = dosis 10x -> tolak
        inputBuffer="";
        mintaAksiPersamaan(AKSI_PERS_AB, val);
      } else {
        tampilkanError(String(F("Batas: "))+String(0.25f*defAB,1)+F("-")+String(4.0f*defAB,1));
        inputBuffer="";
        tampilkanPengaturanABList();
      }
    }
    else if(key>='0'&&key<='9'){
      int dotPos=inputBuffer.indexOf('.');
      if(dotPos<0){
        if(inputBuffer.length()<3){inputBuffer+=key;tampilkanPengaturanABList();}
      } else {
        int belakang=inputBuffer.length()-dotPos-1;
        if(belakang<2){inputBuffer+=key;tampilkanPengaturanABList();}
      }
    }
  }

  else if(currentState==PENGATURAN_PH_PILIH_ASAM){
    if     (key=='A'){if(selPengaturanPHAsam>0){selPengaturanPHAsam--;tampilkanPengaturanPHPilihAsam();}}
    else if(key=='C'){if(selPengaturanPHAsam<1){selPengaturanPHAsam++;tampilkanPengaturanPHPilihAsam();}}
    else if(key=='*'){currentState=MENU_PENGATURAN_PERSAMAAN;lcd.clear();tampilkanMenuPengaturanPersamaan();}
    else if(key=='B'){
      selPengaturanPHFase=0;
      currentState=PENGATURAN_PH_PILIH_FASE;
      lcd.clear();
      tampilkanPengaturanPHPilihFase();
    }
  }

  else if(currentState==PENGATURAN_PH_PILIH_FASE){
    if     (key=='A'){if(selPengaturanPHFase>0){selPengaturanPHFase--;tampilkanPengaturanPHPilihFase();}}
    else if(key=='C'){if(selPengaturanPHFase<1){selPengaturanPHFase++;tampilkanPengaturanPHPilihFase();}}
    else if(key=='*'){currentState=PENGATURAN_PH_PILIH_ASAM;lcd.clear();tampilkanPengaturanPHPilihAsam();}
    else if(key=='B'){
      selPengaturanPHIdx=0;
      currentState=PENGATURAN_PH_LIST;
      lcd.clear();
      tampilkanPengaturanPHList();
    }
  }

  else if(currentState==PENGATURAN_PH_LIST){
    if     (key=='A'){if(selPengaturanPHIdx>0){selPengaturanPHIdx--;tampilkanPengaturanPHList();}}
    else if(key=='C'){if(selPengaturanPHIdx<TOTAL_TABEL_PH-1){selPengaturanPHIdx++;tampilkanPengaturanPHList();}}
    else if(key=='*'){currentState=PENGATURAN_PH_PILIH_FASE;lcd.clear();tampilkanPengaturanPHPilihFase();}
    else if(key=='B'){
      inputBuffer="";
      fasePengaturanPHInput=0;   // selalu mulai dari tahap Edit PPM saat masuk
      currentState=PENGATURAN_PH_INPUT;
      lcd.clear();
      tampilkanPengaturanPHInput();
    }
  }

  // Layar gabungan edit PPM  & edit Slope 
  else if(currentState==PENGATURAN_PH_INPUT){
    if(key=='D'){
      inputBuffer=""; tampilkanPengaturanPHInput();
    }
    else if(key=='*'){
      if(fasePengaturanPHInput==1){
        fasePengaturanPHInput=0; inputBuffer=""; tampilkanPengaturanPHInput();
      } else {
        currentState=PENGATURAN_PH_LIST; inputBuffer=""; lcd.clear(); tampilkanPengaturanPHList();
      }
    }
    else if(key=='#'){
      if(fasePengaturanPHInput==1){
        if(inputBuffer.indexOf('.')<0 && inputBuffer.length()>=1 && inputBuffer.length()<=1){
          inputBuffer+='.'; tampilkanPengaturanPHInput();
        }
      }
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      if(fasePengaturanPHInput==0){
        // Edit PPM acuan untuk pH       
        int rentangBawah = 100 + 200*selPengaturanPHIdx;
        int rentangAtas  = rentangBawah + 200;
        int v=inputBuffer.toInt();
        if(v>=rentangBawah && v<=rentangAtas){
          float (*tabelEdit)[2];
          if(selPengaturanPHAsam==0) tabelEdit = (selPengaturanPHFase==0)? tabelSlopeAtas : tabelSlopeBawah;
          else                       tabelEdit = (selPengaturanPHFase==0)? tabelSlopeAtasFosfat : tabelSlopeBawahFosfat;
          ppmAcuanTundaPH = v;   // belum ditulis ke tabel: batal di fase slope tidak boleh mengubah PPM acuan
          inputBuffer="";
          fasePengaturanPHInput=1;
          tampilkanPengaturanPHInput();
        } else {
          tampilkanError(String(F("Batas: PPM "))+String(rentangBawah)+F("-")+String(rentangAtas));
          inputBuffer="";
          tampilkanPengaturanPHInput();
        }
      } else {
        // Edit Slope pH
        float val=bufferToFloat();
        const float (*tabelDef)[2];
        if(selPengaturanPHAsam==0) tabelDef = (selPengaturanPHFase==0)? tabelSlopeAtasDefault : tabelSlopeBawahDefault;
        else                       tabelDef = (selPengaturanPHFase==0)? tabelSlopeAtasFosfatDefault : tabelSlopeBawahFosfatDefault;
        float defPH = pgm_read_float(&tabelDef[selPengaturanPHIdx][1]);   // TES_139: tabel bawaan di PROGMEM
        if(val>=0.25f*defPH && val<=4.0f*defPH && val<=9.9999f){   // salah ketik (geser koma) -> dosis pH meleset jauh
          inputBuffer="";
          mintaAksiPersamaan(AKSI_PERS_PH, val);
        } else {
          tampilkanError(String(F("Batas: "))+String(0.25f*defPH,3)+F("-")+String(4.0f*defPH,3));
          inputBuffer="";
          tampilkanPengaturanPHInput();
        }
      }
    }
    else if(key>='0'&&key<='9'){
      if(fasePengaturanPHInput==0){
        if(inputBuffer.length()<4){inputBuffer+=key;tampilkanPengaturanPHInput();}
      } else {
        int dotPos=inputBuffer.indexOf('.');
        if(dotPos<0){
          if(inputBuffer.length()<1){inputBuffer+=key;tampilkanPengaturanPHInput();}
        } else {
          int belakang=inputBuffer.length()-dotPos-1;
          if(belakang<4){inputBuffer+=key;tampilkanPengaturanPHInput();}
        }
      }
    }
  }

  else if(currentState==PENGATURAN_RESET_CONFIRM){
    if     (key=='A'){if(selPengaturanResetConfirm>0){selPengaturanResetConfirm--;tampilkanPengaturanResetConfirm();}}
    else if(key=='C'){if(selPengaturanResetConfirm<1){selPengaturanResetConfirm++;tampilkanPengaturanResetConfirm();}}
    else if(key=='*'){currentState=MENU_PENGATURAN_PERSAMAAN;lcd.clear();tampilkanMenuPengaturanPersamaan();}
    else if(key=='B'){
      if(selPengaturanResetConfirm==0){
        mintaAksiPersamaan(AKSI_PERS_RESET, 0.0f);
      } else {
        currentState=MENU_PENGATURAN_PERSAMAAN;
        lcd.clear(); tampilkanMenuPengaturanPersamaan();
      }
    }
  }

  // ── KONFIRMASI PERUBAHAN PERSAMAAN SAAT JADWAL CEK AKTIF ──
  else if(currentState==PERSAMAAN_KONFIRMASI){
    if     (key=='A'){if(selPersamaanKonfirmasi>0){selPersamaanKonfirmasi--;tampilkanPersamaanKonfirmasi();}}
    else if(key=='C'){if(selPersamaanKonfirmasi<1){selPersamaanKonfirmasi++;tampilkanPersamaanKonfirmasi();}}
    else if(key=='*'){ selesaiAksiPersamaan(false); }
    else if(key=='B'){ selesaiAksiPersamaan(selPersamaanKonfirmasi==0); }
  }

  else if(currentState==RESOLUSI_PERINGATAN){ prosesPeringatanResolusi(key); }

  // ── KUNCI KANAL (TES_123) ──
  else if(currentState==MENU_KUNCI_KANAL){
    if     (key=='A'){ if(selKunciKanal>0){ selKunciKanal--; tampilkanMenuKunciKanal(); } }
    else if(key=='C'){ if(selKunciKanal<1){ selKunciKanal++; tampilkanMenuKunciKanal(); } }
    else if(key=='*'){ currentState=MENU_PENGATURAN_SISTEM; lcd.clear(); tampilkanMenuPengaturanSistem(); }
    else if(key=='B'){
      uint8_t bit = (selKunciKanal==0) ? KUNCI_AB : KUNCI_PH;
      if(kanalTerkunci & bit){ selKunciKonf=1; currentState=KUNCI_KANAL_KONFIRMASI; lcd.clear(); tampilkanKunciKanalKonfirmasi(); }
      else { lcd.clear(); lcdPrintCenter(1, F("Kanal tidak")); lcdPrintCenter(2, F("terkunci")); jedaInfo(1200); lcd.clear(); tampilkanMenuKunciKanal(); }
    }
  }
  else if(currentState==KUNCI_KANAL_KONFIRMASI){
    if     (key=='A'){ if(selKunciKonf>0){ selKunciKonf--; tampilkanKunciKanalKonfirmasi(); } }
    else if(key=='C'){ if(selKunciKonf<1){ selKunciKonf++; tampilkanKunciKanalKonfirmasi(); } }
    else if(key=='*'){ currentState=MENU_KUNCI_KANAL; lcd.clear(); tampilkanMenuKunciKanal(); }
    else if(key=='B'){
      lcd.clear();
      if(selKunciKonf==0){
        kanalTerkunci &= (uint8_t)~((selKunciKanal==0) ? KUNCI_AB : KUNCI_PH);
        eepromAutoSimpan();
        lcdPrintCenter(1, (selKunciKanal==0) ? F("Kunci AB dibuka") : F("Kunci pH dibuka"));
        lcdPrintCenter(2, F("Aktif cek berikutnya"));
      } else lcdPrintCenter(1, F("Tetap terkunci"));
      jedaInfo(1500);
      currentState=MENU_KUNCI_KANAL; lcd.clear(); tampilkanMenuKunciKanal();
    }
  }

  else if(currentState==RESET_SISTEM_CONFIRM){
    if     (key=='A'){if(selResetSistemConfirm>0){selResetSistemConfirm--;tampilkanResetSistemConfirm();}}
    else if(key=='C'){if(selResetSistemConfirm<1){selResetSistemConfirm++;tampilkanResetSistemConfirm();}}
    else if(key=='*'){currentState=MENU_PENGATURAN_SISTEM;lcd.clear();tampilkanMenuPengaturanSistem();}
    else if(key=='B'){
      if(selResetSistemConfirm==0){
        resetPabrik();   // hapus konfigurasi EEPROM lalu reboot -> kembali seperti upload awal
      } else {
        currentState=MENU_PENGATURAN_SISTEM;
        lcd.clear();
        tampilkanMenuPengaturanSistem();
      }
    }
  }

  // ── BATASAN PPM/pH — tahap 1: +/- PPM (30-500) ──
  else if(currentState==MENU_BATASAN_PPM){
    if(key=='D'){ inputBuffer=""; tampilkanInputBatasPPM(); }
    else if(key=='*'){ currentState=SUB_MENU_A; inputBuffer=""; lcd.clear(); tampilkanSubMenuA(); }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      int target = tampilBaru() ? terpilihPPMBaru : terpilihPPM;
      if(batasPPMValid(v) && !batasPPMSesuaiTarget(v,target)){
        tampilkanBatasanTidakSesuai(target);   // batas bawah (target - batas) <= 0 -> AB Mix tak akan pernah didosis
        inputBuffer=""; lcd.clear(); tampilkanInputBatasPPM();
      } else if(batasPPMValid(v)){
        batasPPMTunda=v;               // belum di-commit: PPM & pH disimpan bersamaan setelah tahap pH di-OK
        inputBuffer="";
        currentState=MENU_BATASAN_PH; lcd.clear(); tampilkanInputBatasPH();
      } else {
        tampilkanError(String(F("Batas: "))+String(BATAS_PPM_MIN)+F("-")+String(BATAS_PPM_MAKS)+F(" PPM"));
        inputBuffer=""; tampilkanInputBatasPPM();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<3){inputBuffer+=key;tampilkanInputBatasPPM();}
    }
  }

  // ── BATASAN PPM/pH — tahap 2: +/- pH (0.25-1.00), format otomatis X.XX (wajib 3 digit) ──
  else if(currentState==MENU_BATASAN_PH){
    if(key=='D'){ inputBuffer=""; tampilkanInputBatasPH(); }
    else if(key=='*'){ currentState=MENU_BATASAN_PPM; inputBuffer=""; lcd.clear(); tampilkanInputBatasPPM(); }
    else if(key=='B'){
      if(inputBuffer.length()!=3){ tampilkanError(F("Wajib 3 digit")); tampilkanInputBatasPH(); return; }
      int v=inputBuffer.toInt();       // "050" -> 50 = 0.50 pH
      if(batasPHValid(v)){
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu){ batasPPMBaru=batasPPMTunda; batasPHx100Baru=v; batasanBaruSudahDiatur=true; }
        else                    { batasPPM=batasPPMTunda;     batasPHx100=v;     batasanSudahDiatur=true;     }
        inputBuffer="";
        lcd.clear();
        konfirmasiParam(F("Batasan PPM/pH OK!"),SUB_MENU_A);
      } else {
        tampilkanError(F("Batas: 0.25-1.00"));
        inputBuffer=""; tampilkanInputBatasPH();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<3){inputBuffer+=key;tampilkanInputBatasPH();}
    }
  }

  // ── MENU JENIS DOSING (kini bagian dari Parameter Dosing) ──
  else if(currentState==MENU_JENIS_DOSING){
    if     (key=='A'){if(selJenisDosing>0){selJenisDosing--;tampilkanMenuJenisDosing();}}
    else if(key=='C'){if(selJenisDosing<2){selJenisDosing++;tampilkanMenuJenisDosing();}}
    else if(key=='*'){
      currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();
    }
    else if(key=='B'){
      // 0 = Open Loop, 1 = Closed Loop Auto-Tune, 2 = Closed Loop PI ZN -> lanjut input besar makro dosing
      modeJenisTunda = (selJenisDosing==0) ? 1 : (selJenisDosing==1 ? 3 : 4);
      int pAwal = simpanKeParamBaru() ? (draftBaruAda ? persenMakroBaru : persenMakro) : persenMakro;
      inputBuffer = String(persenMakroValid(pAwal) ? pAwal : PERSEN_MAKRO_DEFAULT);   // isi awal = nilai sekarang
      currentState = MENU_PERSEN_MAKRO;
      lcd.clear(); tampilkanInputPersenMakro();
    }
  }

  // ── INPUT BESAR MAKRO DOSING (50-90%) — di-commit bersama jenis dosing ──
  else if(currentState==MENU_PERSEN_MAKRO){
    if(key>='0' && key<='9'){ if(inputBuffer.length()<2){ inputBuffer+=key; tampilkanInputPersenMakro(); } }
    else if(key=='D'){ inputBuffer=""; tampilkanInputPersenMakro(); }
    else if(key=='*'){ inputBuffer=""; currentState=MENU_JENIS_DOSING; lcd.clear(); tampilkanMenuJenisDosing(); }
    else if(key=='B'){
      int v = inputBuffer.toInt();
      if(inputBuffer.length()==0 || !persenMakroValid(v)){
        tampilkanError(F("Batas: 50-90 %")); inputBuffer=""; tampilkanInputPersenMakro(); return;
      }
      inputBuffer="";
      lcd.clear();
      terapkanJenisDosing(modeJenisTunda, v);
      konfirmasiParam(F("Jenis Dosing OK!"),SUB_MENU_A);
    }
  }

  // ── INPUT "LAMA CEK SENSOR" (Pompa D1) ───
  else if(currentState==DOSING_INPUT_LAMA_CEK){
    if(key=='D'){
      inputBuffer=""; tampilkanDosingInputLamaCek();
    }
    else if(key=='*'){
      currentState=SUB_MENU_A; inputBuffer=""; lcd.clear(); tampilkanSubMenuA();
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=10 && v<=900){
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihLamaCekSensorBaru=v; else terpilihLamaCekSensor=v;
        inputBuffer="";
        lcd.clear();
        konfirmasiParam(F("Lama Bilas OK!"),SUB_MENU_A);
      } else {
        tampilkanError(F("Batas: 10-900 Detik"));
        inputBuffer="";
        tampilkanDosingInputLamaCek();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<3){inputBuffer+=key;tampilkanDosingInputLamaCek();}
    }
  }

  // ── INPUT "JEDA STABILISASI" (D1 OFF sebelum jendela median) ──
  else if(currentState==DOSING_INPUT_STABILISASI){
    if(key=='D'){
      inputBuffer=""; tampilkanDosingInputStabilisasi();
    }
    else if(key=='*'){
      currentState=SUB_MENU_A; inputBuffer=""; lcd.clear(); tampilkanSubMenuA();
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=10 && v<=900){
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihJedaStabilisasiBaru=v; else terpilihJedaStabilisasi=v;
        inputBuffer="";
        lcd.clear();
        konfirmasiParam(F("Jeda Stabil OK!"),SUB_MENU_A);
      } else {
        tampilkanError(F("Batas: 10-900 Detik"));
        inputBuffer="";
        tampilkanDosingInputStabilisasi();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<3){inputBuffer+=key;tampilkanDosingInputStabilisasi();}
    }
  }

  // ── INPUT "JEDA HOMOGENISASI" (kini item mandiri di Parameter Dosing) ─
  else if(currentState==DOSING_INPUT_JEDA){
    if(key=='D'){
      inputBuffer=""; tampilkanDosingInputJeda(dosingJedaInputIdx);
    }
    else if(key=='*'){
      if(dosingJedaInputIdx==0){
        currentState=SUB_MENU_A;
        inputBuffer="";
        lcd.clear();
        tampilkanSubMenuA();
      } else {
        dosingJedaInputIdx--;
        char t[6]=""; if(jedaTunda[dosingJedaInputIdx]>0) itoa(jedaTunda[dosingJedaInputIdx],t,10);
        isiAwalLangsung(t);   // slot sebelumnya: tampilkan nilai yang tadi diketik
        tampilkanDosingInputJeda(dosingJedaInputIdx);
      }
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=JEDA_MIN_MENIT && v<=JEDA_MAKS_MENIT){
        jedaTunda[dosingJedaInputIdx]=v;   // belum di-commit (keluar dgn * di tengah = tidak ada yang berubah)
        inputBuffer="";
        int maxIdx = JUMLAH_SLOT_JEDA-1;   // 5 slot jeda homogenisasi: J0..J4
        if(dosingJedaInputIdx<maxIdx){
          dosingJedaInputIdx++;
          char t[6]=""; if(jedaTunda[dosingJedaInputIdx]>0) itoa(jedaTunda[dosingJedaInputIdx],t,10);
          isiAwalLangsung(t);   // slot berikutnya diisi nilai yang sedang dipakai
          tampilkanDosingInputJeda(dosingJedaInputIdx);
        } else {
          siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
          int* dst = dosingSedangMenunggu ? dosingJedaHomogenisasiBaru : dosingJedaHomogenisasi;
          for(int i=0;i<JUMLAH_SLOT_JEDA;i++) dst[slotJeda(i)] = jedaTunda[i];
          if(dosingSedangMenunggu) jedaBaruSudahDiatur=true; else jedaSudahDiatur=true;
          lcd.clear();
          konfirmasiParam(F("Jeda Homogenisasi OK"),SUB_MENU_A);
        }
      } else {
        tampilkanError(F("Batas: 1-240 Menit"));
        inputBuffer="";
        tampilkanDosingInputJeda(dosingJedaInputIdx);
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<3){inputBuffer+=key;tampilkanDosingInputJeda(dosingJedaInputIdx);}
    }
  }

  // ── SELURUH FASE PROSES DOSING BERJALAN ──
  else if(statusFaseDosingBerjalan(currentState)){
    if(key=='B'){
      // Overlay status tidak berubah
      // TES_147: layar tunggu sudah sama dengan halaman 0 -> B langsung ke halaman berikutnya
      statusOverlayAktif = true; statusHalaman = (currentState==DOSING_TUNGGU_WAKTU) ? 1 : 0;
      lcd.clear();
      tampilkanStatusDosing();
    }
    else if(key=='*'){
      if(currentState==DOSING_TUNGGU_WAKTU){
        matikanSemuaPompa();
        tutupSolenoid();
        currentState=MENU_UTAMA;
        lcd.clear();
        tampilkanMenuUtama();
      } else {
        bukaCancelDosing();
      }
    }
    else if(key=='D' && currentState==DOSING_TUNGGU_WAKTU){
      // Hentikan jadwal cek harian dari layar tunggu (Ya = jadwal dimatikan, Tidak = kembali menunggu)
      bukaCancelDosing();
    }
  }

  // Jam cek tiba saat kalibrasi buffer: B = sensor sudah di chamber -> mulai siklus; * = kembali ke kalibrasi (TES_136)
  else if(currentState==DOSING_TUNDA_KALIBRASI){
    if(key=='B')      mulaiSiklusCekHarian();
    else if(key=='*') kembaliDariTundaKalibrasi();
  }

  // Layar rekap dosing terakhir (otomatis ke status setelah 5 dtk; B/* langsung ke status)
  else if(currentState==DOSING_REKAP){
    if(key=='*' || key=='B'){
      currentState=DOSING_FINAL_STATUS; finalStatusDetikBerjalan=0;
      lcd.clear(); tampilkanStatusDosing();
    }
  }

  // Layar Status Dosing akhir — otomatis lanjut sendiri lewat prosesFinalStatus()
  else if(currentState==DOSING_FINAL_STATUS){
    if(key=='*'){
      lanjutkanSiklusBerikutnya();
    }
    else if(key=='B'){
      // Beralih ke tampilan rekap dosing terakhir
      currentState = DOSING_REKAP;
      rekapDetikBerjalan = 0;
      lcd.clear();
      tampilkanRekapDosing();
    }
  }

  // Cek hari ini gagal: B = maju ke layar tunggu jadwal berikutnya, * = kembali ke Menu Utama (jadwal tetap jalan)
  else if(currentState==DOSING_TANDON_TIMEOUT || currentState==DOSING_SENSOR_INVALID){
    if(key=='B'){ hentikanBuzzer(); lanjutkanSiklusBerikutnya(); }
    else if(key=='*'){ hentikanBuzzer(); currentState=MENU_UTAMA; lcd.clear(); tampilkanMenuUtama(); }
  }


  // ── SUB-MENU A: PARAMETER DOSING ──
  else if(currentState==SUB_MENU_A){
    if     (key=='A'){naik(selMenuA,scrollMenuA,TOTAL_MENU_A_ROWS);tampilkanSubMenuA();}
    else if(key=='C'){turun(selMenuA,scrollMenuA,TOTAL_MENU_A_ROWS);tampilkanSubMenuA();}
    else if(key=='*'){currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
    else if(key=='B'){
      inputBuffer=""; lcd.clear();
      if     (selMenuA==0){currentState=MENU_JENIS_LARUTAN;selJenis=0;tampilkanMenuJenisLarutan();}
      else if(selMenuA==1){currentState=MENU_VOL_TANDON;tampilkanInputVolume();}
      else if(selMenuA==2){currentState=MENU_TARGET_PPM;tampilkanInputPPM();}
      else if(selMenuA==3){currentState=MENU_TARGET_PH;tampilkanInputPH();}
      else if(selMenuA==4){currentState=MENU_WAKTU_DOSING;tampilkanInputWaktuDosing();}
      else if(selMenuA==5){currentState=MENU_DEBIT_POMPA_SEL;selDebitPump=1;scrollDebitPump=1;tampilkanMenuPilihDebitPompa();}
      else if(selMenuA==6){currentState=DOSING_INPUT_LAMA_CEK;tampilkanDosingInputLamaCek();}
      else if(selMenuA==7){currentState=DOSING_INPUT_STABILISASI;tampilkanDosingInputStabilisasi();}
      else if(selMenuA==8){currentState=MENU_BATASAN_PPM;tampilkanInputBatasPPM();}
      else if(selMenuA==9){
        currentState=MENU_JENIS_DOSING;
        { bool bk = tampilBaru(); int m = bk ? dosingModeBaru : dosingMode;   // TES_127: kursor di jenis yg dipakai
          bool ok = bk ? jenisDosingBaruSudahDiatur : jenisDosingSudahDiatur;
          selJenisDosing = (!ok) ? 0 : (m==3 ? 1 : (m==4 ? 2 : 0)); }
        tampilkanMenuJenisDosing();
      }
      else if(selMenuA==10){
        // Semua jenis dosing memakai 5 slot jeda (J0..J4) -> tidak perlu memilih Jenis Dosing lebih dulu
        // TES_127: slot diedit di salinan sementara; Aktif/Baru baru berubah setelah slot terakhir di-OK
        { int* src = tampilBaru() ? dosingJedaHomogenisasiBaru : dosingJedaHomogenisasi;
          for(int i=0;i<JUMLAH_SLOT_JEDA;i++) jedaTunda[i] = src[slotJeda(i)]; }
        dosingJedaInputIdx=0;
        currentState=DOSING_INPUT_JEDA;
        tampilkanDosingInputJeda(0);
      }
    }
  }


// PEMILIHAN PPM/PH TANAMAN: langsung daftar Sayuran Daun (MENU_TANAMAN_LIST)

  // Pilih fase tanaman: Bibit/Persemaian atau Tanaman Dewasa
  else if(currentState==MENU_PPM_PH_TANAMAN){
    if     (key=='A'){naik(selPPMPHTanaman,scrollPPMPHTanaman,TOTAL_PPM_PH_TANAMAN_ROWS);tampilkanMenuPPMPHTanaman();}
    else if(key=='C'){turun(selPPMPHTanaman,scrollPPMPHTanaman,TOTAL_PPM_PH_TANAMAN_ROWS);tampilkanMenuPPMPHTanaman();}
    else if(key=='*'){currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
    else if(key=='B'){
      selTanamanKategori=selPPMPHTanaman;   // 0=Bibit/Persemaian, 1=Tanaman Dewasa (Sayuran Daun)
      selTanamanItem=0; scrollTanamanItem=0;
      currentState=MENU_TANAMAN_LIST;
      lcd.clear();
      tampilkanMenuTanamanList();
    }
  }

  // Daftar nama tanaman pada kategori terpilih
  else if(currentState==MENU_TANAMAN_LIST){
    int totalItem = kategoriTanamanTotal[selTanamanKategori];
    if     (key=='A'){naik(selTanamanItem,scrollTanamanItem,totalItem);tampilkanMenuTanamanList();}
    else if(key=='C'){turun(selTanamanItem,scrollTanamanItem,totalItem);tampilkanMenuTanamanList();}
    else if(key=='*'){currentState=MENU_PPM_PH_TANAMAN;lcd.clear();tampilkanMenuPPMPHTanaman();}
    else if(key=='B'){
      const TanamanData* dt = kategoriTanamanData[selTanamanKategori];
      TanamanData itemTerpilih;
      memcpy_P(&itemTerpilih, &dt[selTanamanItem], sizeof(TanamanData));
      siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
      if(dosingSedangMenunggu){
        terpilihPPMBaru = itemTerpilih.ppm;
        terpilihPHBaru  = itemTerpilih.ph;
      } else {
        terpilihPPM = itemTerpilih.ppm;
        terpilihPH  = itemTerpilih.ph;
        hitungKonstantaPIOtomatis();   // slope pH bergantung PPM -> Kp otomatis ikut
      }
      cekBatasanSetelahTargetBerubah();
      // TES_128: tampilkan nilai yang benar-benar disimpan (dulu hanya "PPM/pH disimpan!")
      { char b[21], f[10];
        lcd.clear();
        lcdPrintCenter(0, String(itemTerpilih.nama));
        snprintf_P(b,sizeof(b),PSTR("PPM %d | pH %s"), itemTerpilih.ppm, fmtF(f,itemTerpilih.ph,2));
        lcdPrintCenter(1, b);
        lcdPrintCenter(2, F("PPM/pH disimpan!"));
        if(simpanKeParamBaru()) lcdPrintCenter(3, F("(Parameter Baru)"));
        jedaAman(2000);
        currentState=MENU_TANAMAN_LIST; lcd.clear(); tampilkanMenuTanamanList(); }
    }
  }

  // Jenis Larutan
  else if(currentState==MENU_JENIS_LARUTAN){
    if     (key=='A'){if(selJenis>0){selJenis--;tampilkanMenuJenisLarutan();}}
    else if(key=='C'){if(selJenis<1){selJenis++;tampilkanMenuJenisLarutan();}}
    else if(key=='*'){currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();}
    else if(key=='B'){
      lcd.clear();
      if(selJenis==0){ecBebasDigits="";ecBebasTujuan=0;currentState=MENU_EC_BEBAS;tampilkanInputECBebas();}
      else{currentState=MENU_JENIS_ASAM;selAcid=((tampilBaru()?terpilihAcidBaru:terpilihAcid)=="Fosfat")?1:0;tampilkanMenuJenisAsam();}
    }
  }

  // ── Target EC AB Mix — input bebas 0.20-4.00, wajib 3 digit format X.XX ──
  else if(currentState==MENU_EC_BEBAS){
    prosesInputECBebas(key);
  }
  else if(currentState==MENU_PENGENCERAN){
    if     (key=='A'){if(selDil>0){selDil--;tampilkanMenuPengenceran();}}
    else if(key=='C'){if(selDil<1){selDil++;tampilkanMenuPengenceran();}}
    else if(key=='*'){currentState=MENU_EC_BEBAS;lcd.clear();tampilkanInputECBebas();}
    else if(key=='B'){
      // Koefisien per-EC diambil dari baris EC2.0 (index 2) tabel kalibrasi yang
      // sudah ada — kalau tabel dikalibrasi ulang lewat menu Pengaturan, nilai
      // EC bebas ini otomatis ikut menyesuaikan (lihat pembahasan sebelumnya).
      float koefisien = regresiSlopeAB[selDil] / EC_ANCHOR[selDil];
      siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
      if(ecTunda.length()==0) ecTunda = dosingSedangMenunggu ? terpilihECBaru : terpilihEC;
      if(dosingSedangMenunggu){
        terpilihECBaru = ecTunda;
        float ecVal = ecTunda.toFloat();
        terpilihDilutionBaru = dilOptions[selDil];
        slopeABMixTerpilihBaru = koefisien * ecVal;
      } else {
        terpilihEC = ecTunda;
        float ecVal = ecTunda.toFloat();
        terpilihDilution = dilOptions[selDil];
        slopeABMixTerpilih = koefisien * ecVal;
        hitungKonstantaPIOtomatis();   // parameter aktif berubah -> Kp otomatis diperbarui
      }
      konfirmasiParam(F("Setup AB Mix OK!"),MENU_JENIS_LARUTAN);
    }
  }

  // Jenis Asam (pH Down)
  else if(currentState==MENU_JENIS_ASAM){
    if     (key=='A'){if(selAcid>0){selAcid--;tampilkanMenuJenisAsam();}}
    else if(key=='C'){if(selAcid<1){selAcid++;tampilkanMenuJenisAsam();}}
    else if(key=='*'){currentState=MENU_JENIS_LARUTAN;lcd.clear();tampilkanMenuJenisLarutan();}
    else if(key=='B'){
      currentState=MENU_KONSENTRASI; selConc=0; scrollConc=0;
      { int c = tampilBaru() ? terpilihConcPersenBaru : terpilihConcPersen;   // TES_127: kursor di konsentrasi yg dipakai
        for(int i=0;i<TOTAL_KONSEN_OPTIONS;i++) if(konsenPHValues[i]==c) selConc=i; }
      lcd.clear(); tampilkanMenuKonsentrasi();
    }
  }

  // ── Konsentrasi Asam — 3 opsi baku ──
  else if(currentState==MENU_KONSENTRASI){
    if     (key=='A'){naik(selConc,scrollConc,TOTAL_KONSEN_OPTIONS);tampilkanMenuKonsentrasi();}
    else if(key=='C'){turun(selConc,scrollConc,TOTAL_KONSEN_OPTIONS);tampilkanMenuKonsentrasi();}
    else if(key=='*'){currentState=MENU_JENIS_ASAM;lcd.clear();tampilkanMenuJenisAsam();}
    else if(key=='B'){
      lcd.clear();
      siapkanDraftBaru();
      if(dosingSedangMenunggu){
        terpilihAcidBaru=acidOptions[selAcid];terpilihConcBaru=konsenPHOptions[selConc];
        terpilihConcPersenBaru=konsenPHValues[selConc];
      } else {
        terpilihAcid=acidOptions[selAcid];terpilihConc=konsenPHOptions[selConc];
        terpilihConcPersen=konsenPHValues[selConc];
        hitungKonstantaPIOtomatis();
      }
      konfirmasiParam(F("Setup pH Down OK!"),MENU_JENIS_LARUTAN);
    }
  }

  // Volume Tandon — rentang 30-3.000 Liter
  else if(currentState==MENU_VOL_TANDON){
    if     (key=='D'){inputBuffer="";tampilkanInputVolume();}
    else if(key=='*'){currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=30 && v<=3000){
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihVolumeBaru=v; else { terpilihVolume=v; hitungKonstantaPIOtomatis(); }
        konfirmasiParam(F("Volume Tandon OK!"),SUB_MENU_A);
      } else {
        tampilkanError(F("Batas: 30-3.000 L"));
        inputBuffer="";
        tampilkanInputVolume();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<4){inputBuffer+=key;tampilkanInputVolume();}
    }
  }

  // Target PPM (rentang 100-2100)
  else if(currentState==MENU_TARGET_PPM){
    if     (key=='D'){inputBuffer="";tampilkanInputPPM();}
    else if(key=='*'){currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(targetPPMValid(v)){
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihPPMBaru=v; else { terpilihPPM=v; hitungKonstantaPIOtomatis(); }
        currentState=MENU_TARGET_PPM_HASIL;
        lcd.clear();
        tampilkanHasilKalkulasiPPM();
        jedaAman(3000);
        cekBatasanSetelahTargetBerubah();   // target turun di bawah batasan -> batasan wajib diatur ulang
        currentState=SUB_MENU_A;
        lcd.clear();
        tampilkanSubMenuA();
      } else {tampilkanError(F("Batas: 100-2100 PPM"));inputBuffer="";tampilkanInputPPM();}
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<4){inputBuffer+=key;tampilkanInputPPM();}
    }
  }

  // Target pH — rentang 3.5-8.0
  else if(currentState==MENU_TARGET_PH){
    if(key=='D'){
      inputBuffer=""; tampilkanInputPH();
    }
    else if(key=='*'){currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();}
    else if(key=='#'){
      if(inputBuffer.indexOf('.')<0 && inputBuffer.length()==1){
        inputBuffer+='.';
        tampilkanInputPH();
      }
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      float v=bufferToFloat();
      if(v>=3.5f&&v<=8.0f){
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihPHBaru=v; else terpilihPH=v;
        currentState=MENU_TARGET_PH_HASIL;
        lcd.clear();
        tampilkanHasilTargetPH();
        jedaAman(3000);
        currentState=SUB_MENU_A;
        lcd.clear();
        tampilkanSubMenuA();
      } else {tampilkanError(F("Batas: pH 3.5-8.0"));inputBuffer="";tampilkanInputPH();}
    }
    else if(key>='0'&&key<='9'){
      int dotPos=inputBuffer.indexOf('.');
      if(dotPos<0){
        if(inputBuffer.length()<1){inputBuffer+=key;tampilkanInputPH();}
      } else {
        int belakang=inputBuffer.length()-dotPos-1;
        if(belakang<2){inputBuffer+=key;tampilkanInputPH();}
      }
    }
  }

  // Waktu Dosing
  else if(currentState==MENU_WAKTU_DOSING){
    if     (key=='D'){inputBuffer="";tampilkanInputWaktuDosing();}
    else if(key=='*'){currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();}
    else if(key=='B'){
      if(inputBuffer.length()!=4){tampilkanError(F("Harus 4 Digit (HHMM)"));inputBuffer="";tampilkanInputWaktuDosing();return;}
      int j=inputBuffer.substring(0,2).toInt(),m=inputBuffer.substring(2,4).toInt();
      if(j<=23&&m<=59){
        String jamStrBaru=inputBuffer.substring(0,2)+":"+inputBuffer.substring(2,4);
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihWaktuDosingBaru=jamStrBaru; else terpilihWaktuDosing=jamStrBaru;
        konfirmasiParam(F("Waktu Dosing OK!"),SUB_MENU_A);
      } else {tampilkanError(F("Jam 00-23, Mnt 00-59"));inputBuffer="";tampilkanInputWaktuDosing();}
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<4){inputBuffer+=key;tampilkanInputWaktuDosing();}
    }
  }

  // Debit Pompa – pilih pompa
  else if(currentState==MENU_DEBIT_POMPA_SEL){
    if     (key=='A'){if(selDebitPump>1){selDebitPump--;if(selDebitPump<scrollDebitPump)scrollDebitPump--;tampilkanMenuPilihDebitPompa();}}
    else if(key=='C'){if(selDebitPump<TOTAL_PUMP_OPTIONS-1){selDebitPump++;if(selDebitPump-scrollDebitPump>2)scrollDebitPump++;tampilkanMenuPilihDebitPompa();}}
    else if(key=='*'){currentState=SUB_MENU_A;lcd.clear();tampilkanSubMenuA();}
    else if(key=='B'){currentState=MENU_DEBIT_POMPA_VAL;inputBuffer="";lcd.clear();tampilkanInputNilaiDebit();}
  }

  // Debit Pompa – input nilai
  else if(currentState==MENU_DEBIT_POMPA_VAL){
    if(key=='D'){
      inputBuffer=""; tampilkanInputNilaiDebit();
    }
    else if(key=='#'){
      if(inputBuffer.indexOf('.')<0&&inputBuffer.length()>=1&&inputBuffer.length()<=2){
        inputBuffer+='.'; tampilkanInputNilaiDebit();
      }
    }
    else if(key=='*'){currentState=MENU_DEBIT_POMPA_SEL;lcd.clear();tampilkanMenuPilihDebitPompa();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      float fv=bufferToFloat();
      if(fv>=0.01f&&fv<=DEBIT_MAKS_LMNT){   // spesifikasi pompa terpasang: 0.01-10 L/menit
        int v=(int)(fv*DEBIT_SKALA+0.5f);   // TES_138: mL/menit
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihDebitPumpBaru[selDebitPump]=v; else { terpilihDebitPump[selDebitPump]=v; hitungKonstantaPIOtomatis(); }
        int berikutnya=-1;
        for(int i=1;i<TOTAL_PUMP_OPTIONS;i++){   // cari pompa kosong PERTAMA dari D2 (index 1); D1 tak dipakai dosing
          int isiCek = dosingSedangMenunggu ? terpilihDebitPumpBaru[i] : terpilihDebitPump[i];
          if(isiCek==0){berikutnya=i;break;}
        }
        if(berikutnya>=0){
          selDebitPump=berikutnya;
          if(selDebitPump-scrollDebitPump>2) scrollDebitPump=selDebitPump-2;
          inputBuffer=""; lcd.clear();
          tampilkanInputNilaiDebit();
        } else {
          konfirmasiParam(F("Debit disimpan!"),MENU_DEBIT_POMPA_SEL);
        }
      } else {tampilkanError(F("Debit 0.01-10 L/mnt"));inputBuffer="";tampilkanInputNilaiDebit();}
    }
    else if(key>='0'&&key<='9'){
      int dotPos=inputBuffer.indexOf('.');
      if(dotPos<0){
        if(inputBuffer.length()<2){inputBuffer+=key;tampilkanInputNilaiDebit();}
      } else {
        int belakang=inputBuffer.length()-dotPos-1;
        if(belakang<3){inputBuffer+=key;tampilkanInputNilaiDebit();}   // TES_138: 3 desimal (0.001 L/mnt = 1 mL/mnt)
      }
    }
  }

  // ── SUB-MENU B: KALIBRASI ──
  else if(currentState==SUB_MENU_B){
    if     (key=='A'){naik(selMenuB,scrollMenuB,TOTAL_MENU_B_ROWS);tampilkanSubMenuB();}
    else if(key=='C'){turun(selMenuB,scrollMenuB,TOTAL_MENU_B_ROWS);tampilkanSubMenuB();}
    else if(key=='*'){currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
    else if(key=='B'){
      inputBuffer=""; lcd.clear();
      if     (selMenuB==0 || selMenuB==1){ kalSensorSel=(byte)selMenuB; selKalMenu=0; scrollKalMenu=0; currentState=MENU_KAL_SENSOR; tampilkanMenuKalSensor(); }
      else if(selMenuB==2){currentState=MENU_WATER_LEVEL;wlIsiAktif=false;waterLevelTestTimeout=false;wlBuzzerSudah=false;wlBuzzerMilikWL=false;tampilkanWaterLevel();}
      else if(selMenuB==3){
        suhuInputNegatif=false;
        currentState=MENU_SUHU_KALIBRASI;
        tampilkanKalkSuhuInput();
      }
      else{
        // Tes Sensor TDS (baris 4) / Tes Sensor pH (baris 5): input lama pembilasan dulu
        tesSensorJenis = (selMenuB==4) ? 0 : 1;
        currentState=TES_SENSOR_INPUT_BILAS;
        tampilkanTesSensorInputBilas();
      }
    }
  }

  // ── TES SENSOR: input lama pembilasan ──
  else if(currentState==TES_SENSOR_INPUT_BILAS){
    if(key=='D'){ inputBuffer=""; tampilkanTesSensorInputBilas(); }
    else if(key=='*'){ currentState=SUB_MENU_B; inputBuffer=""; lcd.clear(); tampilkanSubMenuB(); }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=10 && v<=900){
        tesSensorLamaBilas=v; inputBuffer=""; lcd.clear();
        currentState=TES_SENSOR_INPUT_STABIL; tampilkanTesSensorInputStabil();
      } else { tampilkanError(F("Batas: 10-900 Detik")); inputBuffer=""; tampilkanTesSensorInputBilas(); }
    }
    else if(key>='0'&&key<='9'){ if(inputBuffer.length()<3){inputBuffer+=key;tampilkanTesSensorInputBilas();} }
  }
  // ── TES SENSOR: input jeda stabilisasi -> mulai proses ──
  else if(currentState==TES_SENSOR_INPUT_STABIL){
    if(key=='D'){ inputBuffer=""; tampilkanTesSensorInputStabil(); }
    else if(key=='*'){ currentState=TES_SENSOR_INPUT_BILAS; inputBuffer=""; lcd.clear(); tampilkanTesSensorInputBilas(); }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=10 && v<=900){
        tesSensorJedaStabil=v; inputBuffer=""; matikanSemuaPompa();   // hindari bentrok dgn tes pompa latar belakang
        if(tesSensorJenis==0) nyalakanRelayTDS();   // uji TDS -> daya modul TDS ON
        else                  matikanRelayTDS();    // uji pH   -> pastikan TDS mati (bebas interferensi)
        tesSensorFase=0; tesSensorDetik=0;   // fase 0 = stabilisasi 5 dtk setelah relay berubah
        tesSensorHasilPPM=0; tesSensorHasilPH=0.0f;
        resetPpmRoll15(); resetPhRoll15();
        currentState=TES_SENSOR_PROSES; lcd.clear(); tampilkanTesSensorProses();
      } else { tampilkanError(F("Batas: 10-900 Detik")); inputBuffer=""; tampilkanTesSensorInputStabil(); }
    }
    else if(key>='0'&&key<='9'){ if(inputBuffer.length()<3){inputBuffer+=key;tampilkanTesSensorInputStabil();} }
  }
  // ── TES SENSOR: proses berjalan (batal dengan *) ──
  else if(currentState==TES_SENSOR_PROSES){
    if(key=='*' || key=='B'){
      relayOff(PIN_RELAY_D1);
      matikanRelayTDS();
      currentState=SUB_MENU_B; lcd.clear(); tampilkanSubMenuB();
    }
  }
  // ── TES SENSOR: hasil median 20 detik terakhir ──
  else if(currentState==TES_SENSOR_HASIL){
    if(key=='*' || key=='B'){ matikanRelayTDS(); currentState=SUB_MENU_B; lcd.clear(); tampilkanSubMenuB(); }
  }

  // ── SUBMENU KALIBRASI TDS / pH ──
  else if(currentState==MENU_KAL_SENSOR){
    if     (key=='A'){ naik(selKalMenu, scrollKalMenu, TOTAL_KAL_MENU);  tampilkanMenuKalSensor(); }
    else if(key=='C'){ turun(selKalMenu, scrollKalMenu, TOTAL_KAL_MENU); tampilkanMenuKalSensor(); }
    else if(key=='*'){ matikanRelayTDS(); currentState=SUB_MENU_B; lcd.clear(); tampilkanSubMenuB(); }
    else if(key=='B'){
      inputBuffer=""; lcd.clear();
      if(selKalMenu==0){
        if(kalSensorSel==0){ currentState=MENU_KAL_TDS_SAMPEL; mulaiKalTdsBuffer(); tampilkanInputTDSBuffer(0); }
        else               { currentState=MENU_KAL_PH_SAMPEL;  mulaiKalPhBuffer();  tampilkanInputpHBuffer(0); }
      } else if(selKalMenu==1){
        selKalLihat=0; scrollKalLihat=0; currentState=MENU_KAL_LIHAT; tampilkanKalLihat();
      } else if(selKalMenu==2){
        int idx = (kalSensorSel==0) ? kalDraft.tdsUndoIdx : kalDraft.phUndoIdx;
        if(idx<0){
          lcdPrintCenter(0, F("Tak ada sampel"));
          lcdPrintCenter(1, F("untuk dihapus"));
          lcdPrintCenter(2, F("(hanya 1 langkah;"));
          lcdPrintCenter(3, F("pakai Lihat/Hapus)"));
          jedaInfo(2500);
          lcd.clear(); tampilkanMenuKalSensor();
        } else {
          aksiKal=AKSI_KAL_HAPUS_TERAKHIR; selKalUlang=1; currentState=MENU_KAL_ULANG_KONF; tampilkanKalUlangKonf();
        }
      } else if(selKalMenu==3){
        aksiKal=AKSI_KAL_MULAI_ULANG; selKalUlang=1; currentState=MENU_KAL_ULANG_KONF; tampilkanKalUlangKonf();
      } else {
        // TES_129: Hapus Data Kalibrasi -> kalibrasi aktif & draft sensor ini dihapus, kembali ke rumus bawaan
        bool aktif = (kalSensorSel==0) ? tdsKalibrasiAktif : phKalibrasi3TitikAktif;
        if(!aktif && kalJumlahSampel(kalSensorSel)==0){
          lcdPrintCenter(0, F("Tidak ada data"));
          lcdPrintCenter(1, F("kalibrasi"));
          jedaInfo(2000);
          lcd.clear(); tampilkanMenuKalSensor();
        } else {
          aksiKal=AKSI_KAL_HAPUS_KALIBRASI; selKalUlang=1; currentState=MENU_KAL_ULANG_KONF; tampilkanKalUlangKonf();
        }
      }
    }
  }
  // Lihat & hapus data per buffer: A/C pilih buffer, B = hapus buffer terpilih, * = kembali
  else if(currentState==MENU_KAL_LIHAT){
    int n = (kalSensorSel==0) ? TDS_TITIK_KAL : PH_TITIK_KAL;
    if     (key=='A'){ naik(selKalLihat, scrollKalLihat, n);  tampilkanKalLihat(); }
    else if(key=='C'){ turun(selKalLihat, scrollKalLihat, n); tampilkanKalLihat(); }
    else if(key=='B'){
      int nSampel = (kalSensorSel==0) ? kalDraft.tdsN[selKalLihat] : kalDraft.phN[selKalLihat];
      if(nSampel>0){ aksiKal=AKSI_KAL_HAPUS_BUFFER; selKalUlang=1; currentState=MENU_KAL_ULANG_KONF; lcd.clear(); tampilkanKalUlangKonf(); }
    }
    else if(key=='*'){ currentState=MENU_KAL_SENSOR; lcd.clear(); tampilkanMenuKalSensor(); }
  }
  else if(currentState==MENU_KAL_ULANG_KONF){
    if     (key=='A'){ if(selKalUlang>0){ selKalUlang--; tampilkanKalUlangKonf(); } }
    else if(key=='C'){ if(selKalUlang<1){ selKalUlang++; tampilkanKalUlangKonf(); } }
    else if(key=='*'){ selesaiKonfKal(); }
    else if(key=='B'){
      if(selKalUlang==0){
        bool aktifSebelum = (kalSensorSel==0) ? tdsKalibrasiAktif : phKalibrasi3TitikAktif;
        if(aksiKal==AKSI_KAL_MULAI_ULANG){
          kalDraftKosongkan(kalSensorSel); kalDraftSimpan();
          lcd.clear(); lcdPrintCenter(1, F("Draft dikosongkan"));
          jedaAman(1800);
        } else if(aksiKal==AKSI_KAL_HAPUS_KALIBRASI){
          kalHapusKalibrasi(kalSensorSel);
          lcd.clear();
          lcdPrintCenter(0, kalSensorSel==0 ? F("KALIBRASI TDS") : F("KALIBRASI pH"));
          lcdPrintCenter(1, F("DIHAPUS"));
          lcdPrintCenter(2, F("Pakai rumus bawaan"));
          if(dosingSedangMenunggu) lcdPrintCenter(3, F("(Jadwal ikut ubah)"));
          jedaInfo(2500);
        } else {
          if(aksiKal==AKSI_KAL_HAPUS_TERAKHIR) kalHapusTerakhir(kalSensorSel);
          else                                 kalHapusKelompok(kalSensorSel, selKalLihat);
          byte h = kalEvaluasi(kalSensorSel);     // hitung ulang kalibrasi aktif dari data yang tersisa
          lcd.clear();
          lcdPrintCenter(0, F("Sampel dihapus"));
          if(h==KAL_AKTIF)   { lcdPrintCenter(1, F("Kalibrasi aktif")); lcdPrintCenter(2, F("diperbarui")); }
          else if(h==KAL_TIDAK_WAJAR){ lcdPrintCenter(1, F("Data tidak wajar")); lcdPrintCenter(2, F("Pakai kalibrasi lama")); }
          else if(aktifSebelum){ lcdPrintCenter(1, F("Data belum cukup:")); lcdPrintCenter(2, F("Kalibrasi aktif")); lcdPrintCenter(3, F("tetap dipakai")); }
          else               { lcdPrintCenter(1, F("Kalibrasi belum")); lcdPrintCenter(2, F("aktif")); }
          jedaInfo(2500);
        }
      }
      selesaiKonfKal();
    }
  }

  // ── AMBIL 1 SAMPEL BUFFER TDS ──
  else if(currentState==MENU_KAL_TDS_SAMPEL){
    if(key=='D'){ inputBuffer=""; tampilkanInputTDSBuffer(0); }
    else if(key=='*'){ matikanRelayTDS(); inputBuffer=""; currentState=MENU_KAL_SENSOR; lcd.clear(); tampilkanMenuKalSensor(); }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int val = inputBuffer.toInt();
      if(val < 0 || val > TARGET_PPM_MAKS){ tampilkanError(F("Batas: 0-2100 PPM")); inputBuffer=""; tampilkanInputTDSBuffer(0); return; }
      if(millis() - tKalTdsBufMulai < KAL_TDS_MIN_MS){   // wajib tunggu stabil minimal 30 detik
        kalTdsTungguTampil = true; tampilkanPesanKalTunggu(); tampilkanInputTDSBuffer(0); return;
      }
      kalTdsTungguTampil = false;
      float cv = adcKeVoltTDS(getMedianKalTds()) / (1.0f+0.02f*(suhularutan-25.0f));   // median 10 dtk terakhir
      inputBuffer="";
      matikanRelayTDS();
      if(!kalTambahSampelTDS(val, cv)){ tampilkanError(F("Maks 5 jenis buffer")); }
      else tampilkanHasilSampelKal(0, kalEvaluasi(0));
      currentState=MENU_KAL_SENSOR; lcd.clear(); tampilkanMenuKalSensor();
    }
    else if(key>='0' && key<='9'){ if(inputBuffer.length()<4){ inputBuffer+=key; tampilkanInputTDSBuffer(0); } }
  }

  // ── AMBIL 1 SAMPEL BUFFER pH ──
  else if(currentState==MENU_KAL_PH_SAMPEL){
    if(key=='D'){ inputBuffer=""; tampilkanInputpHBuffer(0); }
    else if(key=='*'){ inputBuffer=""; currentState=MENU_KAL_SENSOR; lcd.clear(); tampilkanMenuKalSensor(); }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int x100 = inputBuffer.toInt();   // 2 desimal otomatis: 686 -> 6.86
      if(x100 < 0 || x100 > 1400){ tampilkanError(F("Batas: pH 0.00-14.00")); inputBuffer=""; tampilkanInputpHBuffer(0); return; }
      if(millis() - tKalPhBufMulai < KAL_PH_MIN_MS){     // wajib tunggu stabil minimal 120 detik
        kalPhTungguTampil = true; tampilkanPesanKalTunggu(); tampilkanInputpHBuffer(0); return;
      }
      kalPhTungguTampil = false;
      float volt = adcKeVolt(getMedianKalPh());          // median 10 dtk terakhir
      float suhuKal = (suhuKalPhCount>0) ? (suhuKalPhSum/(float)suhuKalPhCount) : suhularutan;
      inputBuffer="";
      if(!kalTambahSampelPH(x100, volt, suhuKal)){ tampilkanError(F("Maks 3 jenis buffer")); }
      else tampilkanHasilSampelKal(1, kalEvaluasi(1));
      currentState=MENU_KAL_SENSOR; lcd.clear(); tampilkanMenuKalSensor();
    }
    else if(key>='0' && key<='9'){ if(inputBuffer.length()<4){ inputBuffer+=key; tampilkanInputpHBuffer(0); } }
  }

  // ── Set Waktu RTC ──
  else if(currentState==MENU_SET_RTC_TAHUN) handleRTCField(key,0,0,9999,MENU_SET_RTC_BULAN,1);
  else if(currentState==MENU_SET_RTC_BULAN) handleRTCField(key,1,1,12,MENU_SET_RTC_TGL,2);
  else if(currentState==MENU_SET_RTC_TGL){
    if(key=='D'){
      inputBuffer=""; tampilkanInputRTC(2);
    }
    else if(key=='*'){
      currentState=MENU_PENGATURAN_SISTEM; lcd.clear(); tampilkanMenuPengaturanSistem();
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      int maksHari=getMaksHariBulan(rtcInput[1], rtcInput[0]);
      if(v>=1 && v<=maksHari){
        rtcInput[2]=v;
        inputBuffer="";
        currentState=MENU_SET_RTC_JAM;
        lcd.clear();
        tampilkanInputRTC(3);
      } else {
        tampilkanError(String(F("Batas: 1-"))+String(maksHari));
        inputBuffer="";
        tampilkanInputRTC(2);
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<2){inputBuffer+=key; tampilkanInputRTC(2);}
    }
  }
  else if(currentState==MENU_SET_RTC_JAM)   handleRTCField(key,3,0,23,MENU_SET_RTC_MENIT,4);
  else if(currentState==MENU_SET_RTC_MENIT) handleRTCField(key,4,0,59,MENU_SET_RTC_DETIK,5);
  else if(currentState==MENU_SET_RTC_DETIK){
    if     (key=='D'){inputBuffer="";tampilkanInputRTC(5);}
    else if(key=='*'){inputBuffer="";currentState=MENU_PENGATURAN_SISTEM;lcd.clear();tampilkanMenuPengaturanSistem();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=0&&v<=59){
        rtcInput[5]=v; inputBuffer="";
        terapkanWaktuRTC();
        currentState=MENU_SET_RTC_HASIL;
        lcd.clear();
        tampilkanRTCHasil();
        tsRtcTerakhir=millis();
      }
      else{tampilkanError(F("Detik: 0-59"));inputBuffer="";tampilkanInputRTC(5);}
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<2){inputBuffer+=key;tampilkanInputRTC(5);}
    }
  }

  else if(currentState==MENU_SET_RTC_HASIL){
    if(key=='*'){currentState=MENU_PENGATURAN_SISTEM;lcd.clear();tampilkanMenuPengaturanSistem();}
  }

  else if(currentState==MENU_WATER_LEVEL){
    if(key=='*'){
      tutupSolenoid(); wlIsiAktif=false;
      if(wlBuzzerMilikWL && buzzerBunyi) hentikanBuzzer();   // hanya buzzer milik menu ini
      wlBuzzerMilikWL=false;
      currentState=SUB_MENU_B;lcd.clear();tampilkanSubMenuB();
    }
    else if(key=='B'){
      // Katup baru dibuka atas perintah user (B = mulai/hentikan pengisian, maks 15 menit)
      wlIsiAktif = !wlIsiAktif;
      if(wlIsiAktif){ waterLevelTestMulai=millis(); waterLevelTestTimeout=false; }
      tampilkanWaterLevel();
    }
  }

  else if(currentState==MENU_SUHU_KALIBRASI){
    if(key=='D'){
      inputBuffer=""; suhuInputNegatif=false;
      tampilkanKalkSuhuInput();
    }
    else if(key=='A'){
      suhuInputNegatif=!suhuInputNegatif;
      tampilkanKalkSuhuInput();
    }
    else if(key=='*'){
      currentState=SUB_MENU_B; lcd.clear(); tampilkanSubMenuB();
    }
    else if(key=='#'){
      if(inputBuffer.indexOf('.')<0 && inputBuffer.length()>=1 && inputBuffer.length()<=2){
        inputBuffer+='.'; tampilkanKalkSuhuInput();
      }
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      float v=bufferToFloat();
      if(suhuInputNegatif) v=-v;
      if(!suhuSensorValid){
        tampilkanError(F("Sensor suhu ERROR"));   // offset tak boleh dihitung dari bacaan basi
        inputBuffer=""; suhuInputNegatif=false;
        tampilkanKalkSuhuInput();
      } else if(v>=-10.0f && v<=85.0f && fabs(v - suhuRawTerakhir) > SUHU_KOREKSI_MAKS){
        // TES_127: selisih > 5 C hampir pasti salah ketik / probe belum di air acuan -> tolak (koreksi suhu TDS & pH)
        char fs[10], fa[10], bb[21];
        lcd.clear();
        lcdPrintCenter(0, F("SELISIH SUHU > 5 C"));
        snprintf_P(bb,sizeof(bb),PSTR("Sensor : %s C"), fmtF(fs,suhuRawTerakhir,1)); lcdPrintCenter(1, bb);
        snprintf_P(bb,sizeof(bb),PSTR("Acuan  : %s C"), fmtF(fa,v,1));             lcdPrintCenter(2, bb);
        lcdPrintCenter(3, F("Cek probe/termometer"));
        jedaInfo(3500);
        inputBuffer=""; suhuInputNegatif=false;
        lcd.clear(); tampilkanKalkSuhuInput();
      } else if(v>=-10.0f && v<=85.0f){
        suhuOffset = v - suhuRawTerakhir;
        inputBuffer=""; suhuInputNegatif=false;
        konfirmasiSukses(F("Kalibrasi Suhu OK!"), SUB_MENU_B);
      } else {
        tampilkanError(F("Batas: -10 s/d 85"));
        inputBuffer=""; suhuInputNegatif=false;
        tampilkanKalkSuhuInput();
      }
    }
    else if(key>='0'&&key<='9'){
      int dotPos=inputBuffer.indexOf('.');
      if(dotPos<0){
        if(inputBuffer.length()<2){inputBuffer+=key;tampilkanKalkSuhuInput();}
      } else {
        int belakang=inputBuffer.length()-dotPos-1;
        if(belakang<2){inputBuffer+=key;tampilkanKalkSuhuInput();}
      }
    }
  }

  // ── SUB-MENU C: UJI DEBIT POMPA ──
  else if(currentState==SUB_MENU_C){
    if     (key=='A'){
      if(selPump>0){
        selPump--;
        if(selPump < scrollPump) scrollPump = selPump;
        tampilkanMenuPilihPompa();
      }
    }
    else if(key=='C'){
      if(selPump<TOTAL_UJI_OPTIONS-1){
        selPump++;
        if(selPump - scrollPump > 2) scrollPump = selPump - 2;
        tampilkanMenuPilihPompa();
      }
    }
    else if(key=='*'){
      currentState=MENU_UTAMA;
      lcd.clear();
      tampilkanMenuUtama();
    }
    else if(key=='B'){
      lcd.clear();
      if(selPump < TOTAL_PUMP_OPTIONS){
        currentState=MENU_TES_POMPA_RUN;
        inputBuffer="";
        tampilkanInputDelayPompa();
      } else if(selPump == TOTAL_PUMP_OPTIONS){
        // Katup Solenoid
        solenoidTesPenuh=false; solenoidTesPenuhDetik=0;
        currentState=MENU_TES_SOLENOID;
        tampilkanTesSolenoid();
      } else {
        // "Input Debit Pompa"
        currentState=MENU_INPUT_DEBIT_SEL;
        selInputDebit=1; scrollInputDebit=1; // D1 tidak dipakai dosing 
        tampilkanMenuInputDebitPompa();
      }
    }
  }

  // ── INPUT DURASI TES POMPA ──
  else if(currentState==MENU_TES_POMPA_RUN){
    if     (key=='D'){inputBuffer="";tampilkanInputDelayPompa();}
    else if(key=='*'){currentState=SUB_MENU_C;lcd.clear();tampilkanMenuPilihPompa();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      long vDurasi = inputBuffer.toInt();                       // long: cegah luapan int utk input 5 digit
      long batasMaksTesPompa = (selPump<3) ? 1800L : 10800L;   // D1-D3 maks 30mnt, P1-P3 maks 180mnt
      if(vDurasi<=0 || vDurasi>batasMaksTesPompa){
        tampilkanError(selPump<3?F("Batas: 1-1800 detik"):F("Batas: 1-10800 detik"));
        inputBuffer="";
        tampilkanInputDelayPompa();
        return;
      }
      inputPumpDelay = (int)vDurasi;
      if(tesPompaBerjalan){
        tampilkanError(F("Tes pompa lain jalan"));
        inputBuffer="";
        tampilkanInputDelayPompa();
        return;
      }
      tesPompaDetikTarget = inputPumpDelay;
      tesPompaDetikBerjalan = 0;
      tesPompaIdx = selPump;
      tesPompaMulaiMs = millis();
      tesPompaBerjalan = true;
      relayOn(pumpRelayPin[selPump]);
      currentState = TES_POMPA_AKTIF;
      lcd.clear();
      tampilkanTesPompaAktif();
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<5){inputBuffer+=key;tampilkanInputDelayPompa();}
    }
  }

  // ── PROSES TES POMPA AKTIF (non-blocking, bisa dibatalkan) ──
  else if(currentState==TES_POMPA_AKTIF){
    if(key=='*'){
      tesPompaSelCancel=1;
      currentState=TES_POMPA_CANCEL_CONFIRM;
      lcd.clear();
      tampilkanCancelTesPompa();
    }
  }
  else if(currentState==TES_POMPA_CANCEL_CONFIRM){
    if     (key=='A'){if(tesPompaSelCancel>0){tesPompaSelCancel--;tampilkanCancelTesPompa();}}
    else if(key=='C'){if(tesPompaSelCancel<1){tesPompaSelCancel++;tampilkanCancelTesPompa();}}
    else if(key=='B'){
      if(tesPompaSelCancel==0){
        // Ya: hentikan pompa & batalkan tes
        relayOff(pumpRelayPin[tesPompaIdx]);
        tesPompaBerjalan=false;
        currentState=SUB_MENU_C; lcd.clear(); tampilkanMenuPilihPompa();
      } else {
        currentState=TES_POMPA_AKTIF; lcd.clear(); tampilkanTesPompaAktif();
      }
    }
    else if(key=='*'){
      currentState=TES_POMPA_AKTIF; lcd.clear(); tampilkanTesPompaAktif();
    }
  }

  else if(currentState==MENU_TES_SOLENOID){
    if(key=='*'){
      if(solenoidTerbuka) tutupSolenoid();   // pastikan valve tertutup saat keluar tes
      currentState=SUB_MENU_C;
      lcd.clear();
      tampilkanMenuPilihPompa();
    }
    else if(key=='B'){
      if(solenoidTerbuka) tutupSolenoid();
      else if(!bolehBukaSolenoid()){ lcd.clear(); lcdPrintCenter(1, F("Katup baru berganti")); lcdPrintCenter(2, F("Tunggu 3 detik")); jedaAman(1500); lcd.clear(); }   // TES_149: cegah on-off beruntun
      else { bukaSolenoid(); solenoidTesMulai = millis(); solenoidTesPenuh = false; solenoidTesPenuhDetik = 0; }
      tampilkanTesSolenoid();
    }
  }

  // ── SUB MENU "INPUT DEBIT POMPA" ──
  else if(currentState==MENU_INPUT_DEBIT_SEL){
    if     (key=='A'){
      if(selInputDebit>1){
        selInputDebit--;
        if(selInputDebit<scrollInputDebit) scrollInputDebit--;
        tampilkanMenuInputDebitPompa();
      }
    }
    else if(key=='C'){
      if(selInputDebit<TOTAL_PUMP_OPTIONS-1){
        selInputDebit++;
        if(selInputDebit-scrollInputDebit>2) scrollInputDebit++;
        tampilkanMenuInputDebitPompa();
      }
    }
    else if(key=='*'){
      currentState=SUB_MENU_C; lcd.clear(); tampilkanMenuPilihPompa();
    }
    else if(key=='B'){
      inputBuffer="";
      currentState=INPUT_DEBIT_ML;
      lcd.clear();
      tampilkanInputDebitMl();
    }
  }

  else if(currentState==INPUT_DEBIT_ML){
    if(key=='D'){
      inputBuffer=""; tampilkanInputDebitMl();
    }
    else if(key=='*'){
      currentState=MENU_INPUT_DEBIT_SEL; inputBuffer=""; lcd.clear(); tampilkanMenuInputDebitPompa();
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      long v=inputBuffer.toInt();      // long: int AVR 16-bit meluap utk 32768-99999 mL
      if(v>0 && v<=99999L){
        inputDebitMl=v; inputBuffer="";
        currentState=INPUT_DEBIT_DETIK;
        lcd.clear();
        tampilkanInputDebitDetik();
      } else {
        tampilkanError(F("Nilai harus > 0 ml"));
        inputBuffer="";
        tampilkanInputDebitMl();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<5){inputBuffer+=key;tampilkanInputDebitMl();}
    }
  }

  else if(currentState==INPUT_DEBIT_DETIK){
    if(key=='D'){
      inputBuffer=""; tampilkanInputDebitDetik();
    }
    else if(key=='*'){
      currentState=INPUT_DEBIT_ML; inputBuffer=""; lcd.clear(); tampilkanInputDebitMl();
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>0 && v<=9999){
        // Hitung debit (L/menit) = ml * 0.06 / detik dalam float DULU, baru dicek rentang -> tidak ada luapan int
        float hasil = ((float)inputDebitMl * 0.06f) / (float)v;
        if(hasil < 0.01f || hasil > DEBIT_MAKS_LMNT){
          tampilkanError(F("Debit 0.01-10 L/mnt"));   // di luar spesifikasi pompa -> ulangi pengukuran
          inputBuffer=""; tampilkanInputDebitDetik();
          return;
        }
        inputDebitDetik=v; inputBuffer="";
        inputDebitHasil = hasil;
        int debitSimpan = (int)(hasil * DEBIT_SKALA + 0.5f);   // TES_138: mL/menit (dulu x100 -> pembulatan 0.01 L/mnt)
        if(debitSimpan < 10) debitSimpan = 10;
        siapkanDraftBaru();   // jadwal menunggu: salin Aktif -> Baru saat edit pertama
        if(dosingSedangMenunggu) terpilihDebitPumpBaru[selInputDebit] = debitSimpan;
        else { terpilihDebitPump[selInputDebit] = debitSimpan; hitungKonstantaPIOtomatis(); }
        currentState=INPUT_DEBIT_HASIL;
        lcd.clear();
        tampilkanInputDebitHasil();
      } else {
        tampilkanError(F("Harus > 0 detik"));
        inputBuffer="";
        tampilkanInputDebitDetik();
      }
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<4){inputBuffer+=key;tampilkanInputDebitDetik();}
    }
  }

  else if(currentState==INPUT_DEBIT_HASIL){
    if(key=='B' || key=='*'){
      currentState=MENU_INPUT_DEBIT_SEL;
      lcd.clear();
      tampilkanMenuInputDebitPompa();
    }
  }

  // ── RINGKASAN PARAMETER AKTIF ──
  else if(currentState==MENU_RINGKASAN){
    if     (key=='A'){if(scrollSummary>0){scrollSummary--;tampilkanMenuRingkasan();}}
    else if(key=='C'){if(scrollSummary<maksScrollRingkasan(false)){scrollSummary++;tampilkanMenuRingkasan();}}
    else{currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
  }

  // ── SUB-MENU : MODE KALKULATOR ──
  else if(currentState==SUB_MENU_D){
    if     (key=='A'){naik(selMenuD,scrollMenuD,TOTAL_MENU_D_ROWS);tampilkanSubMenuD();}
    else if(key=='C'){turun(selMenuD,scrollMenuD,TOTAL_MENU_D_ROWS);tampilkanSubMenuD();}
    else if(key=='*'){currentState=MENU_UTAMA;lcd.clear();tampilkanMenuUtama();}
    else if(key=='B'){
      lcd.clear();
      if     (selMenuD==0){
        selKalkWLBentuk=0; kalkWLVolume=0; kalkWLJariJari=0; kalkWLPanjang=0; kalkWLLebar=0; kalkWLHasil=0.0f;
        currentState=KALK_WATER_LEVEL_BENTUK;
        tampilkanKalkWLBentuk();
      }
      else if(selMenuD==1){currentState=KALK_INPUT_MENU;selKalkInput=0;tampilkanKalkInputMenu();}
    }
  }

  // ── KALKULATOR "TINGGI WATER LEVEL" ──
  // Tahap 1: Pilih Bentuk Tandon
  else if(currentState==KALK_WATER_LEVEL_BENTUK){
    if     (key=='A'){if(selKalkWLBentuk>0){selKalkWLBentuk--;tampilkanKalkWLBentuk();}}
    else if(key=='C'){if(selKalkWLBentuk<1){selKalkWLBentuk++;tampilkanKalkWLBentuk();}}
    else if(key=='*'){currentState=SUB_MENU_D;lcd.clear();tampilkanSubMenuD();}
    else if(key=='B'){
      lcd.clear();
      kalkWLVolume=0; kalkWLJariJari=0; kalkWLPanjang=0; kalkWLLebar=0; inputBuffer="";
      if(selKalkWLBentuk==0){currentState=KALK_WL_TABUNG_VOL;tampilkanKalkWLTabungVol();}
      else{currentState=KALK_WL_BALOK_VOL;tampilkanKalkWLBalokVol();}
    }
  }

  // ── TABUNG: Tahap 1 - V Target (Liter, maks 3000) ──
  else if(currentState==KALK_WL_TABUNG_VOL){
    if     (key=='D'){inputBuffer="";tampilkanKalkWLTabungVol();}
    else if(key=='*'){currentState=KALK_WATER_LEVEL_BENTUK;lcd.clear();tampilkanKalkWLBentuk();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>0 && v<=3000){kalkWLVolume=v;inputBuffer="";currentState=KALK_WL_TABUNG_JARI;lcd.clear();tampilkanKalkWLTabungJari();}
      else{tampilkanError(F("Batas: 1-3.000 Liter"));inputBuffer="";tampilkanKalkWLTabungVol();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<4){inputBuffer+=key;tampilkanKalkWLTabungVol();}}
  }
  // ── TABUNG: Tahap 2 - Jari-jari (cm, maks 3 digit) ──
  else if(currentState==KALK_WL_TABUNG_JARI){
    if     (key=='D'){inputBuffer="";tampilkanKalkWLTabungJari();}
    else if(key=='*'){currentState=KALK_WL_TABUNG_VOL;inputBuffer="";lcd.clear();tampilkanKalkWLTabungVol();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>0 && v<=999){
        kalkWLJariJari=v;inputBuffer="";
        hitungKalkWLTabung();
        currentState=KALK_WL_TABUNG_HASIL;lcd.clear();tampilkanKalkWLTabungHasil();
      } else{tampilkanError(F("Batas: 1-999 cm"));inputBuffer="";tampilkanKalkWLTabungJari();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<3){inputBuffer+=key;tampilkanKalkWLTabungJari();}}
  }
  // ── TABUNG: Hasil ──
  else if(currentState==KALK_WL_TABUNG_HASIL){
    if     (key=='*'){currentState=KALK_WL_TABUNG_JARI;inputBuffer="";lcd.clear();tampilkanKalkWLTabungJari();}
    else if(key=='B'||key=='D'){
      currentState=KALK_WATER_LEVEL_BENTUK;kalkWLVolume=0;kalkWLJariJari=0;
      inputBuffer="";lcd.clear();tampilkanKalkWLBentuk();
    }
  }

  // ── BALOK: Tahap 1 - V Target (Liter, maks 3000) ──
  else if(currentState==KALK_WL_BALOK_VOL){
    if     (key=='D'){inputBuffer="";tampilkanKalkWLBalokVol();}
    else if(key=='*'){currentState=KALK_WATER_LEVEL_BENTUK;lcd.clear();tampilkanKalkWLBentuk();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>0 && v<=3000){kalkWLVolume=v;inputBuffer="";currentState=KALK_WL_BALOK_PANJANG;lcd.clear();tampilkanKalkWLBalokPanjang();}
      else{tampilkanError(F("Batas: 1-3.000 Liter"));inputBuffer="";tampilkanKalkWLBalokVol();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<4){inputBuffer+=key;tampilkanKalkWLBalokVol();}}
  }
  // ── BALOK: Tahap 2 - Panjang (cm, maks 3 digit) ──
  else if(currentState==KALK_WL_BALOK_PANJANG){
    if     (key=='D'){inputBuffer="";tampilkanKalkWLBalokPanjang();}
    else if(key=='*'){currentState=KALK_WL_BALOK_VOL;inputBuffer="";lcd.clear();tampilkanKalkWLBalokVol();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>0 && v<=999){kalkWLPanjang=v;inputBuffer="";currentState=KALK_WL_BALOK_LEBAR;lcd.clear();tampilkanKalkWLBalokLebar();}
      else{tampilkanError(F("Batas: 1-999 cm"));inputBuffer="";tampilkanKalkWLBalokPanjang();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<3){inputBuffer+=key;tampilkanKalkWLBalokPanjang();}}
  }
  // ── BALOK: Tahap 3 - Lebar (cm, maks 3 digit) ──
  else if(currentState==KALK_WL_BALOK_LEBAR){
    if     (key=='D'){inputBuffer="";tampilkanKalkWLBalokLebar();}
    else if(key=='*'){currentState=KALK_WL_BALOK_PANJANG;inputBuffer="";lcd.clear();tampilkanKalkWLBalokPanjang();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>0 && v<=999){
        kalkWLLebar=v;inputBuffer="";
        hitungKalkWLBalok();
        currentState=KALK_WL_BALOK_HASIL;lcd.clear();tampilkanKalkWLBalokHasil();
      } else{tampilkanError(F("Batas: 1-999 cm"));inputBuffer="";tampilkanKalkWLBalokLebar();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<3){inputBuffer+=key;tampilkanKalkWLBalokLebar();}}
  }
  // ── BALOK: Hasil ──
  else if(currentState==KALK_WL_BALOK_HASIL){
    if     (key=='*'){currentState=KALK_WL_BALOK_LEBAR;inputBuffer="";lcd.clear();tampilkanKalkWLBalokLebar();}
    else if(key=='B'||key=='D'){
      currentState=KALK_WATER_LEVEL_BENTUK;kalkWLVolume=0;kalkWLPanjang=0;kalkWLLebar=0;
      inputBuffer="";lcd.clear();tampilkanKalkWLBentuk();
    }
  }

  // ── KALKULATOR PPM/pH DARI INPUT ──
  else if(currentState==KALK_INPUT_MENU){
    if     (key=='A'){if(selKalkInput>0){selKalkInput--;tampilkanKalkInputMenu();}}
    else if(key=='C'){if(selKalkInput<1){selKalkInput++;tampilkanKalkInputMenu();}}
    else if(key=='*'){currentState=SUB_MENU_D;lcd.clear();tampilkanSubMenuD();}
    else if(key=='B'){
      lcd.clear();
      if(selKalkInput==0){kalkEC="";kalkSelDil=0;kalkVolume=0;kalkPPMAwal=0;kalkPPMTarget=0;ecBebasTujuan=1;ecBebasDigits="";currentState=KALK_PPM_EC;tampilkanInputECBebas();}
      else{
        kalkPHVolume=0; kalkPHAcid=0; kalkPHKonsen=0; scrollKalkPHKonsen=0; kalkPHPpm=0; kalkPHAwal=0.0f; kalkPHTarget=0.0f;
        inputBuffer="";
        currentState=KALK_PH_VOL;
        tampilkanKalkPHInputVolume();
      }
    }
  }
  // EC bebas dari Input (Kalkulator)
  else if(currentState==KALK_PPM_EC){
    prosesInputECBebas(key);
  }
  else if(currentState==KALK_PPM_DIL){
    if     (key=='A'){if(kalkSelDil>0){kalkSelDil--;tampilkanKalkMenuDil();}}
    else if(key=='C'){if(kalkSelDil<1){kalkSelDil++;tampilkanKalkMenuDil();}}
    else if(key=='*'){ecBebasTujuan=1;ecBebasDigits="";currentState=KALK_PPM_EC;lcd.clear();tampilkanInputECBebas();}
    else if(key=='B'){
      kalkSlopeTerpilih = hitungSlopeAB(kalkEC.toFloat(), kalkSelDil);
      currentState=KALK_PPM_VOL;inputBuffer="";lcd.clear();tampilkanKalkInputVolume();
    }
  }
  // Volume — rentang 30-3.000L
  else if(currentState==KALK_PPM_VOL){
    if     (key=='D'){inputBuffer="";tampilkanKalkInputVolume();}
    else if(key=='*'){currentState=KALK_PPM_DIL;lcd.clear();tampilkanKalkMenuDil();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=30&&v<=3000){kalkVolume=v;inputBuffer="";currentState=KALK_PPM_AWAL;lcd.clear();tampilkanKalkInputPPMAwal();}
      else{tampilkanError(F("Batas: 30-3.000 L"));inputBuffer="";tampilkanKalkInputVolume();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<4){inputBuffer+=key;tampilkanKalkInputVolume();}}
  }
  else if(currentState==KALK_PPM_AWAL){
    if     (key=='D'){inputBuffer="";tampilkanKalkInputPPMAwal();}
    else if(key=='*'){currentState=KALK_PPM_VOL;inputBuffer="";lcd.clear();tampilkanKalkInputVolume();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=0&&v<=TARGET_PPM_MAKS){kalkPPMAwal=v;inputBuffer="";currentState=KALK_PPM_TARGET;lcd.clear();tampilkanKalkInputPPMTarget();}
      else{tampilkanError(F("Batas: 0-2100 PPM"));inputBuffer="";tampilkanKalkInputPPMAwal();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<4){inputBuffer+=key;tampilkanKalkInputPPMAwal();}}
  }
  else if(currentState==KALK_PPM_TARGET){
    if     (key=='D'){inputBuffer="";tampilkanKalkInputPPMTarget();}
    else if(key=='*'){currentState=KALK_PPM_AWAL;inputBuffer="";lcd.clear();tampilkanKalkInputPPMAwal();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(targetPPMValid(v)){
        kalkPPMTarget=v;hitungKalkPPM();
        currentState=KALK_PPM_HASIL;scrollKalkHasil=0;lcd.clear();tampilkanKalkHasilPPM();
      } else{tampilkanError(F("Batas: 100-2100 PPM"));inputBuffer="";tampilkanKalkInputPPMTarget();}
    }
    else if(key>='0'&&key<='9'){
      if(inputBuffer.length()<4){
        inputBuffer+=key;
        tampilkanKalkInputPPMTarget();
      }
    }
  }
  else if(currentState==KALK_PPM_HASIL){
    if     (key=='*'){currentState=KALK_PPM_TARGET;inputBuffer="";lcd.clear();tampilkanKalkInputPPMTarget();}
    else if(key=='B'||key=='D'){
      kalkEC="";ecBebasTujuan=1;ecBebasDigits="";currentState=KALK_PPM_EC;kalkSelDil=0;kalkVolume=0;
      kalkPPMAwal=0;kalkPPMTarget=0;inputBuffer="";lcd.clear();tampilkanInputECBebas();
    }
  }

  // ── KALKULATOR pH DOWN DARI INPUT ──
  else if(currentState==KALK_PH_VOL){
    if     (key=='D'){inputBuffer="";tampilkanKalkPHInputVolume();}
    else if(key=='*'){currentState=KALK_INPUT_MENU;inputBuffer="";lcd.clear();tampilkanKalkInputMenu();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(v>=30&&v<=3000){
        kalkPHVolume=v;inputBuffer="";kalkPHAcid=0;
        currentState=KALK_PH_ASAM;lcd.clear();tampilkanKalkPHMenuAsam();
      }
      else{tampilkanError(F("Batas: 30-3.000 L"));inputBuffer="";tampilkanKalkPHInputVolume();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<4){inputBuffer+=key;tampilkanKalkPHInputVolume();}}
  }
  else if(currentState==KALK_PH_ASAM){
    if     (key=='A'){if(kalkPHAcid>0){kalkPHAcid--;tampilkanKalkPHMenuAsam();}}
    else if(key=='C'){if(kalkPHAcid<1){kalkPHAcid++;tampilkanKalkPHMenuAsam();}}
    else if(key=='*'){currentState=KALK_PH_VOL;inputBuffer="";lcd.clear();tampilkanKalkPHInputVolume();}
    else if(key=='B'){kalkPHKonsen=0;scrollKalkPHKonsen=0;currentState=KALK_PH_KONSEN;lcd.clear();tampilkanKalkPHMenuKonsen();}
  }
  // Konsentrasi Asam — 3 opsi baku
  else if(currentState==KALK_PH_KONSEN){
    if     (key=='A'){naik(kalkPHKonsen,scrollKalkPHKonsen,TOTAL_KONSEN_OPTIONS);tampilkanKalkPHMenuKonsen();}
    else if(key=='C'){turun(kalkPHKonsen,scrollKalkPHKonsen,TOTAL_KONSEN_OPTIONS);tampilkanKalkPHMenuKonsen();}
    else if(key=='*'){currentState=KALK_PH_ASAM;lcd.clear();tampilkanKalkPHMenuAsam();}
    else if(key=='B'){
      lcd.clear();
      kalkPHKonsenPersen=konsenPHValues[kalkPHKonsen];
      inputBuffer="";currentState=KALK_PH_PPM;tampilkanKalkPHInputPPM();
    }
  }
  else if(currentState==KALK_PH_PPM){
    if     (key=='D'){inputBuffer="";tampilkanKalkPHInputPPM();}
    else if(key=='*'){currentState=KALK_PH_KONSEN;lcd.clear();tampilkanKalkPHMenuKonsen();}
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      int v=inputBuffer.toInt();
      if(targetPPMValid(v)){kalkPHPpm=v;inputBuffer="";currentState=KALK_PH_AWAL;lcd.clear();tampilkanKalkPHInputAwal();}
      else{tampilkanError(F("Batas: 100-2100 PPM"));inputBuffer="";tampilkanKalkPHInputPPM();}
    }
    else if(key>='0'&&key<='9'){if(inputBuffer.length()<4){inputBuffer+=key;tampilkanKalkPHInputPPM();}}
  }
  else if(currentState==KALK_PH_AWAL){
    if(key=='D'){inputBuffer="";tampilkanKalkPHInputAwal();}
    else if(key=='*'){currentState=KALK_PH_PPM;inputBuffer="";lcd.clear();tampilkanKalkPHInputPPM();}
    else if(key=='#'){
      if(inputBuffer.indexOf('.')<0 && inputBuffer.length()==1){inputBuffer+='.';tampilkanKalkPHInputAwal();}
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      float v=bufferToFloat();
      if(v>=3.5f&&v<=8.0f){kalkPHAwal=v;inputBuffer="";currentState=KALK_PH_TARGET;lcd.clear();tampilkanKalkPHInputTarget();}
      else{tampilkanError(F("Batas: pH 3.5-8.0"));inputBuffer="";tampilkanKalkPHInputAwal();}
    }
    else if(key>='0'&&key<='9'){
      int dotPos=inputBuffer.indexOf('.');
      if(dotPos<0){if(inputBuffer.length()<1){inputBuffer+=key;tampilkanKalkPHInputAwal();}}
      else{int belakang=inputBuffer.length()-dotPos-1; if(belakang<2){inputBuffer+=key;tampilkanKalkPHInputAwal();}}
    }
  }
  else if(currentState==KALK_PH_TARGET){
    if(key=='D'){inputBuffer="";tampilkanKalkPHInputTarget();}
    else if(key=='*'){currentState=KALK_PH_AWAL;inputBuffer="";lcd.clear();tampilkanKalkPHInputAwal();}
    else if(key=='#'){
      if(inputBuffer.indexOf('.')<0 && inputBuffer.length()==1){inputBuffer+='.';tampilkanKalkPHInputTarget();}
    }
    else if(key=='B'){
      if(inputBuffer.length()==0) return;
      float v=bufferToFloat();
      if(v>=3.5f && v<kalkPHAwal){
        kalkPHTarget=v;
        hitungKalkPH();
        currentState=KALK_PH_HASIL;scrollKalkPHHasil=0;lcd.clear();tampilkanKalkPHHasil();
      } else{tampilkanError(F("< pH awal, >= 3.5"));inputBuffer="";tampilkanKalkPHInputTarget();}
    }
    else if(key>='0'&&key<='9'){
      int dotPos=inputBuffer.indexOf('.');
      if(dotPos<0){if(inputBuffer.length()<1){inputBuffer+=key;tampilkanKalkPHInputTarget();}}
      else{int belakang=inputBuffer.length()-dotPos-1; if(belakang<2){inputBuffer+=key;tampilkanKalkPHInputTarget();}}
    }
  }
  else if(currentState==KALK_PH_HASIL){
    if     (key=='*'){currentState=KALK_PH_TARGET;inputBuffer="";lcd.clear();tampilkanKalkPHInputTarget();}
    else if(key=='B'||key=='D'){
      currentState=KALK_PH_VOL;kalkPHVolume=0;kalkPHAcid=0;kalkPHKonsen=0;scrollKalkPHKonsen=0;kalkPHPpm=0;kalkPHAwal=0.0f;kalkPHTarget=0.0f;
      inputBuffer="";lcd.clear();tampilkanKalkPHInputVolume();
    }
  }

}

// ===================
// RENDER – MENU UTAMA
// ===================
void tampilkanMenuUtama() {
  // Saat jadwal cek menunggu di latar belakang, judul menampilkan jadwal berikutnya sebagai pengingat
  if(dosingSedangMenunggu){
    char b[21];
    snprintf_P(b,sizeof(b),PSTR("[Cek %02d/%02d %s]"),dosingTargetHari,dosingTargetBulan,terpilihWaktuDosing.c_str());
    lcdPrintCenter(0,b);
  } else lcdPrintCenter(0,F("MENU UTAMA"));
  for(int i=0;i<3;i++){
    int idx=scrollMenuUtama+i;
    if(idx<TOTAL_MENU_UTAMA_ROWS){
      String line=(idx==selMenuUtama?"> ":"  ");
      line+=menuUtamaOptions[idx];
      lcdPrint20(0,i+1,line);
    } else lcdPrint20(0,i+1,"");
  }
}

// ====================
// RENDER – MENU KREDIT
// ====================
void tampilkanMenuKredit(){
  lcd.clear();
  if(kreditHalaman==1){
    unsigned long jam = millis()/3600000UL;
    lcdPrintCenter(0,F("INFO SISTEM"));
    lcdCenterP(1, PSTR("RAM bebas: %d B"), freeRam());
    lcdCenterP(2, PSTR("RAM terendah: %d B"), ramMinimum);
    lcdCenterP(3, PSTR("Nyala: %lu hr %lu j"), jam/24UL, jam%24UL);
    return;
  }
  lcdPrintCenter(0,F("Sistem Dosing"));
  lcdPrintCenter(1,F("Hidroponik v1.0"));
  lcdPrintCenter(2,F("dibuat oleh:"));
  lcdPrintCenter(3,F("Syahid_TMB IPB 59"));
}

// ====================
// RENDER – SUB-MENU A
// ====================
void tampilkanSubMenuA(){
  lcdPrintCenter(0,F("PARAMETER DOSING"));
  for(int i=0;i<3;i++){
    int idx=scrollMenuA+i;
    if(idx<TOTAL_MENU_A_ROWS) lcdPrint20(0,i+1,(idx==selMenuA?"> ":"  ")+String(menuAOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}
void tampilkanMenuJenisLarutan(){
  lcdPrintCenter(0,F("JENIS LARUTAN"));
  lcdPrint20(0,1, String(selJenis==0?"> ":"  ") + F("AB Mix"));
  lcdPrint20(0,2, String(selJenis==1?"> ":"  ") + F("pH Down"));
  lcdPrint20(0,3,F(""));
}
void tampilkanMenuPengenceran(){
  lcdPrint20(0,0,F("Pengenceran AB Mix:"));
  lcdPrint20(0,1, String(selDil==0?"> ":"  ") + F("5 ml/L"));
  lcdPrint20(0,2, String(selDil==1?"> ":"  ") + F("10 ml/L"));
  lcdPrint20(0,3,F(""));
}
void tampilkanMenuJenisAsam(){
  lcdPrint20(0,0,F("Jenis Asam pH Down:"));
  lcdPrint20(0,1, String(selAcid==0?"> ":"  ") + F("Nitrat"));
  lcdPrint20(0,2, String(selAcid==1?"> ":"  ") + F("Fosfat"));
  lcdPrint20(0,3,F(""));
}
void tampilkanMenuKonsentrasi(){
  lcdPrintCenter(0,F("Konsentrasi Asam:"));
  for(int i=0;i<3;i++){
    int idx=scrollConc+i;
    if(idx<TOTAL_KONSEN_OPTIONS) lcdPrint20(0,i+1,(idx==selConc?"> ":"  ")+String(konsenPHOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}

// ── Input numerik ──
void tampilkanInputVolume(){
  lcdPrintCenter(0,F("Volume Tandon"));
  lcdPrintCenter(1,F("(30-3.000 Liter):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" Liter"));
  lcdPrint20(0,3,F(""));
}
void tampilkanInputPPM(){
  lcdPrintCenter(0,F("Target PPM"));
  lcdPrintCenter(1,F("(Rentang 100-2100):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" PPM"));
  lcdPrint20(0,3,F(""));
}
void tampilkanInputPH(){
  lcdPrintCenter(0,F("Target pH"));
  lcdPrintCenter(1,F("(Rentang 3.5-8.0):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" pH"));
  lcdPrint20(0,3,F(""));
}
void tampilkanInputWaktuDosing(){
  lcdPrintCenter(0,F("Waktu Dosing"));
  lcdPrintCenter(1,F("(00:00-23:59):"));
  String h1=(inputBuffer.length()>0)?inputBuffer.substring(0,1):"_";
  String h2=(inputBuffer.length()>1)?inputBuffer.substring(1,2):"_";
  String m1=(inputBuffer.length()>2)?inputBuffer.substring(2,3):"_";
  String m2=(inputBuffer.length()>3)?inputBuffer.substring(3,4):"_";
  lcdPrintCenter(2,h1+h2+F(":")+m1+m2);
  lcdPrint20(0,3,F(""));
}

// ── Debit Pompa ──
void tampilkanMenuPilihDebitPompa(){
  lcdPrintCenter(0,F("DEBIT POMPA"));
  for(int i=0;i<3;i++){
    int idx=scrollDebitPump+i;
    if(idx<TOTAL_PUMP_OPTIONS){
      int debitKonteks = tampilBaru() ? terpilihDebitPumpBaru[idx] : terpilihDebitPump[idx];
      String cek=(debitKonteks>0)?" OK":"   ";   // OK hilang bila debit (Baru/Aktif sesuai konteks) masih kosong
      String prefix=(idx==selDebitPump?"> ":"  ");
      lcdPrint20(0,i+1, prefix + String(pumpNamaRingkas[idx]) + cek);
    } else lcdPrint20(0,i+1,"");
  }
}
void tampilkanInputNilaiDebit(){
  lcdPrintCenter(0, String(pumpNamaRingkas[selDebitPump]));
  lcdPrintCenter(1, F("Debit :"));
  lcdPrintCenter(2, (inputBuffer.length()>0?inputBuffer:"_")+F(" L/menit"));
  lcdPrint20(0,3,F(""));
}

// ── Hasil Target ──
void tampilkanHasilKalkulasiPPM(){
  lcdPrintCenter(0,F("TARGET PPM TERSIMPAN"));
  char buf[21];
  snprintf_P(buf,sizeof(buf),PSTR("Target : %d PPM"), tampilBaru() ? terpilihPPMBaru : terpilihPPM);
  lcdPrintCenter(1, String(buf));
  lcdPrintCenter(2, F("Sensor TDS: OFF"));   // sensor mati saat setting target
  if(simpanKeParamBaru()) lcdPrintCenter(3,F("(Parameter Baru)")); else lcdPrint20(0,3,F(""));
}

void tampilkanHasilTargetPH(){
  lcdPrintCenter(0,F("TARGET pH TERSIMPAN"));
  lcdPrintCenter(1, String(F("Target : ")) + String(tampilBaru() ? terpilihPHBaru : terpilihPH,2) + F(" pH"));
  lcdPrintCenter(2, F("Sensor pH: OFF"));    // sensor mati saat setting target
  if(simpanKeParamBaru()) lcdPrintCenter(3,F("(Parameter Baru)")); else lcdPrint20(0,3,F(""));
}

// ==============================
// RENDER – MENU "PPM/pH TANAMAN"
// ==============================
void tampilkanMenuPPMPHTanaman(){
  lcdPrintCenter(0,F("PPM/pH TANAMAN"));
  for(int i=0;i<3;i++){
    int idx=scrollPPMPHTanaman+i;
    if(idx<TOTAL_PPM_PH_TANAMAN_ROWS) lcdPrint20(0,i+1,(idx==selPPMPHTanaman?"> ":"  ")+String(ppmPhTanamanOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}

void tampilkanMenuTanamanList(){
  lcdPrintCenter(0, String(tanamanKategoriJudul[selTanamanKategori]));
  int totalItem = kategoriTanamanTotal[selTanamanKategori];
  const TanamanData* dt = kategoriTanamanData[selTanamanKategori];
  for(int i=0;i<3;i++){
    int idx=scrollTanamanItem+i;
    if(idx<totalItem){
      TanamanData itemBaris;
      memcpy_P(&itemBaris, &dt[idx], sizeof(TanamanData));
      lcdPrint20(0,i+1,(idx==selTanamanItem?"> ":"  ")+String(itemBaris.nama));
    }
    else lcdPrint20(0,i+1,"");
  }
}

// ==============================
// RENDER – SUB-MENU B: KALIBRASI
// ==============================
void tampilkanSubMenuB(){
  lcdPrintCenter(0,F("MODE KALIBRASI"));
  for(int i=0;i<3;i++){
    int idx=scrollMenuB+i;
    if(idx<TOTAL_MENU_B_ROWS) lcdPrint20(0,i+1,(idx==selMenuB?"> ":"  ")+String(menuBOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}

void tampilkanKalibrasiTDSLayar(int bufferIndex) {
  int raw = bacaAdcTDS();
  float v = adcKeVoltTDS(raw);
  float cv = v/(1.0f+0.02f*(suhularutan-25.0f));
  int ppmL = tdsKalibrasiAktif ? (int)round(interpolasiTDS(cv))
                                     : (int)((133.42f*cv*cv*cv-255.86f*cv*cv+857.39f*cv)*0.5f);

  if(kalTdsTungguTampil){
    unsigned long elapsedMs = millis() - tKalTdsBufMulai;
    int detikTampil = (int)(elapsedMs/1000UL);
    if(detikTampil > 30) detikTampil = 30;
    lcdCenterP(0, PSTR("%d/30 detik"), detikTampil);
  } else {
    lcdPrintCenter(0, F("KALIBRASI TDS"));
  }
  char lbuf[21]; labelSampelKal(lbuf, 0);
  lcdPrint20(0, 1, lbuf);

  char adcb[12], ppm1b[12];
  snprintf(adcb,sizeof(adcb),"ADC:%d", raw);
  snprintf(ppm1b,sizeof(ppm1b),"PPM1:%d", ppmL);
  lcdPrintLR(2, adcb, ppm1b);

  char vb[14], ppm2b[14], f[10];
  snprintf_P(vb,sizeof(vb),PSTR("V:%s"), fmtF(f,v,3));
  snprintf_P(ppm2b,sizeof(ppm2b),PSTR("PPM2:%s"), (inputBuffer.length() > 0 ? inputBuffer.c_str() : "_"));
  lcdPrintLR(3, vb, ppm2b);
}

// "Sampel 3/5 (2 jenis)" sebelum aktif; "Sampel ke-7" sesudah jumlah minimum terpenuhi
void labelSampelKal(char *buf, byte sensor){
  int n = kalJumlahSampel(sensor), mn = (sensor==0)?KAL_TDS_SAMPEL_MIN:KAL_PH_SAMPEL_MIN;
  if(n < mn) snprintf_P(buf,21,PSTR("Sampel %d/%d %d jenis"), n+1, mn, kalJumlahJenis(sensor));
  else       snprintf_P(buf,21,PSTR("Sampel ke-%d"), n+1);
}

// Submenu kalibrasi per sensor
void tampilkanMenuKalSensor(){
  bool aktif = (kalSensorSel==0) ? tdsKalibrasiAktif : phKalibrasi3TitikAktif;
  if(kalSensorSel==0) lcdPrintCenter(0, aktif ? F("KALIBRASI TDS AKTIF") : F("KALIBRASI TDS BAWAAN"));
  else                lcdPrintCenter(0, aktif ? F("KALIBRASI pH AKTIF") : F("KALIBRASI pH BAWAAN"));
  int n  = kalJumlahSampel(kalSensorSel);
  int mn = (kalSensorSel==0) ? KAL_TDS_SAMPEL_MIN : KAL_PH_SAMPEL_MIN;
  char b[21];
  // 5 item, 3 baris terlihat (gulir)
  for(int r=0;r<3;r++){
    int i = scrollKalMenu + r;
    const char* pre = (i==selKalMenu) ? "> " : "  ";
    if(i==0){
      if(n < mn) snprintf_P(b,sizeof(b),PSTR("%sAmbil Sampel %d/%d"), pre, n, mn);
      else if(n < 100) snprintf_P(b,sizeof(b),PSTR("%sTambah Sampel (%d)"), pre, n);
      else             snprintf_P(b,sizeof(b),PSTR("%sTambah Smpl (%d)"), pre, n);   // TES_128: >= 100 sampel muat 20 kolom
    }
    else if(i==1) snprintf_P(b,sizeof(b),PSTR("%sLihat/Hapus Data"), pre);
    else if(i==2) snprintf_P(b,sizeof(b),PSTR("%sHapus Sampel Akhir"), pre);
    else if(i==3) snprintf_P(b,sizeof(b),PSTR("%sMulai Ulang Draft"), pre);
    else          snprintf_P(b,sizeof(b),PSTR("%sHapus Data Kalibr."), pre);   // TES_129 (judul konfirmasi: nama lengkap)
    lcdPrint20(0,r+1,b);
  }
}
// Kembali dari konfirmasi: hapus buffer -> layar Lihat/Hapus; lainnya -> submenu kalibrasi
void selesaiKonfKal(){
  lcd.clear();
  if(aksiKal==AKSI_KAL_HAPUS_BUFFER){ currentState=MENU_KAL_LIHAT; tampilkanKalLihat(); }
  else { currentState=MENU_KAL_SENSOR; tampilkanMenuKalSensor(); }
}
// Daftar kelompok buffer di draft: nilai, jumlah sampel, tegangan rata-rata
void tampilkanKalLihat(){
  char b[21];
  snprintf_P(b,sizeof(b),PSTR("DATA %S %d sampel"), kalSensorSel==0?PSTR("TDS"):PSTR("pH"), kalJumlahSampel(kalSensorSel));
  lcdPrintCenter(0,b);
  int n = (kalSensorSel==0) ? TDS_TITIK_KAL : PH_TITIK_KAL;
  for(int r=0;r<3;r++){
    int i = scrollKalLihat + r;
    if(i>=n){ lcdPrint20(0,r+1,F("")); continue; }
    char v[8];
    const char* pre = (i==selKalLihat) ? ">" : " ";
    if(kalSensorSel==0){
      if(kalDraft.tdsN[i]==0){ snprintf_P(b,sizeof(b),PSTR("%s%d. -"), pre, i+1); }
      else { dtostrf(kalDraft.tdsSumCV[i]/kalDraft.tdsN[i],1,3,v);
             snprintf_P(b,sizeof(b),PSTR("%s%dppm n%d %sV"), pre, kalDraft.tdsPPM[i], kalDraft.tdsN[i], v); }
    } else {
      if(kalDraft.phN[i]==0){ snprintf_P(b,sizeof(b),PSTR("%s%d. -"), pre, i+1); }
      else { char p[8]; dtostrf(kalDraft.phX100[i]/100.0f,1,2,p); dtostrf(kalDraft.phSumV[i]/kalDraft.phN[i],1,3,v);
             snprintf_P(b,sizeof(b),PSTR("%spH%s n%d %sV"), pre, p, kalDraft.phN[i], v); }
    }
    lcdPrint20(0,r+1,b);
  }
}
void tampilkanKalUlangKonf(){
  char b[21], v[8], p[8];
  if(aksiKal==AKSI_KAL_HAPUS_TERAKHIR){
    lcdPrintCenter(0, F("Hapus sampel?"));
    if(kalSensorSel==0){ int i=kalDraft.tdsUndoIdx; dtostrf(kalDraft.tdsUndoCV,1,3,v);
      snprintf_P(b,sizeof(b),PSTR("%dppm %sV"), (i>=0)?kalDraft.tdsPPM[i]:0, v); }
    else { int i=kalDraft.phUndoIdx; dtostrf((i>=0)?kalDraft.phX100[i]/100.0f:0.0f,1,2,p); dtostrf(kalDraft.phUndoV,1,3,v);
      snprintf_P(b,sizeof(b),PSTR("pH%s %sV"), p, v); }
    lcdPrintCenter(1, b);
  } else if(aksiKal==AKSI_KAL_HAPUS_BUFFER){
    lcdPrintCenter(0, F("Hapus semua sampel"));
    int i=selKalLihat;
    if(kalSensorSel==0) snprintf_P(b,sizeof(b),PSTR("%dppm (n=%d)?"), kalDraft.tdsPPM[i], kalDraft.tdsN[i]);
    else { dtostrf(kalDraft.phX100[i]/100.0f,1,2,p); snprintf_P(b,sizeof(b),PSTR("pH%s (n=%d)?"), p, kalDraft.phN[i]); }
    lcdPrintCenter(1, b);
  } else if(aksiKal==AKSI_KAL_HAPUS_KALIBRASI){
    lcdPrintCenter(0, F("HAPUS DATA KALIBRASI"));
    lcdPrintCenter(1, kalSensorSel==0 ? F("TDS -> rumus bawaan") : F("pH -> rumus bawaan"));
  } else {
    lcdPrintCenter(0, F("Mulai ulang draft?"));
  }
  lcdPrint20(0,2, String(selKalUlang==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,3, String(selKalUlang==1?"> ":"  ") + F("Tidak"));
}
// Pesan setelah 1 sampel disimpan
void tampilkanHasilSampelKal(byte sensor, byte hasil){
  lcd.clear();
  char b[21];
  if(hasil==KAL_AKTIF){
    lcdPrintCenter(0, sensor==0 ? F("KALIBRASI TDS") : F("KALIBRASI pH"));
    lcdPrintCenter(1, F("AKTIF / DIPERBARUI"));
    snprintf_P(b,sizeof(b),PSTR("%d sampel, %d jenis"), kalJumlahSampel(sensor), kalJumlahJenis(sensor));
    lcdPrintCenter(2, b);
  } else if(hasil==KAL_BUTUH_JENIS){
    lcdPrintCenter(0, F("Sampel disimpan"));
    lcdPrintCenter(1, F("Butuh min. 2 jenis"));
    lcdPrintCenter(2, F("buffer berbeda"));
  } else if(hasil==KAL_TIDAK_WAJAR){
    lcdPrintCenter(0, F("Sampel disimpan"));
    lcdPrintCenter(1, F("DATA TIDAK WAJAR"));
    lcdPrintCenter(2, F("Cek buffer/probe,"));
    lcdPrintCenter(3, F("atau Mulai Ulang"));
  } else {
    int mn = (sensor==0)?KAL_TDS_SAMPEL_MIN:KAL_PH_SAMPEL_MIN;
    lcdPrintCenter(0, F("Sampel disimpan"));
    snprintf_P(b,sizeof(b),PSTR("%d/%d sampel"), kalJumlahSampel(sensor), mn);
    lcdPrintCenter(1, b);
    lcdPrintCenter(2, F("Ganti/ulang buffer"));
    lcdPrintCenter(3, F("lalu Ambil Sampel"));
  }
  jedaInfo(2500);
}

void tampilkanInputTDSBuffer(int bufferIndex) {
  lcd.clear();
  tampilkanKalibrasiTDSLayar(bufferIndex);
}

void tampilkanKalibrasiPHLayar(int bufferIndex) {
  int raw = bacaAdcPH();
  float v = adcKeVolt(raw);
  float phRaw = phKalibrasi3TitikAktif ? interpolasiPH(v) : (phSlope*v + phOffset);
  float ph = constrain(terapkanKompensasiSuhuPH(phRaw), 0.0f, 14.0f);

  if(kalPhTungguTampil){
    unsigned long elapsedMs = millis() - tKalPhBufMulai;
    int detikTampil = (int)(elapsedMs/1000UL);
    if(detikTampil > 120) detikTampil = 120;
    lcdCenterP(0, PSTR("%d/120 detik"), detikTampil);
  } else {
    lcdPrintCenter(0, F("KALIBRASI pH"));
  }
  char lbuf[21]; labelSampelKal(lbuf, 1);
  lcdPrint20(0, 1, lbuf);

  char adcb[12], ph1b[12], phv[8]; dtostrf(ph,1,2,phv);
  snprintf(adcb,sizeof(adcb),"ADC:%d", raw);
  snprintf(ph1b,sizeof(ph1b),"pH1:%s", phv);
  lcdPrintLR(2, adcb, ph1b);

  char ph2Str[10], vb[14], ph2b[14], f[10];
  if(inputBuffer.length() == 0) strcpy(ph2Str, "_");
  else fmtF(ph2Str, (float)inputBuffer.toInt() / 100.0f, 2);
  snprintf_P(vb,sizeof(vb),PSTR("V:%s"), fmtF(f,v,3));
  snprintf_P(ph2b,sizeof(ph2b),PSTR("pH2:%s"), ph2Str);
  lcdPrintLR(3, vb, ph2b);
}

void tampilkanInputpHBuffer(int bufferIndex) {
  lcd.clear();
  tampilkanKalibrasiPHLayar(bufferIndex);
}

void tampilkanInputRTC(int idx) {
  const char* fn[6] = {"Tahun (2000-2099):", "Bulan (01-12):", "Hari (01-31):",
                       "Jam (00-23):", "Menit (00-59):", "Detik (00-59):"};

  lcdPrintCenter(0, F("WAKTU RTC"));
  lcdPrintCenter(1, String(fn[idx]));

  String inputDisplay = "Input : ";
  if(inputBuffer.length() > 0) {
    inputDisplay += inputBuffer;
  } else {
    inputDisplay += "_";
  }
  lcdPrintCenter(2, inputDisplay);

  char buf[21];
  sprintf(buf, "%04d/%02d/%02d %02d:%02d:%02d",
          rtcInput[0], rtcInput[1], rtcInput[2],
          rtcInput[3], rtcInput[4], rtcInput[5]);
  lcdPrintCenter(3, String(buf));
}

void tampilkanRTCHasil() {
  lcdPrintCenter(0, F("Waktu RTC :"));
  lcdPrintCenter(1, getTanggalLengkapStr() + F(" ") + getWaktuStr());
  lcdPrint20(0, 2, F(""));
  lcdPrint20(0, 3, F(""));
}

// Uji sensor water level. Katup TIDAK dibuka otomatis: B = mulai/hentikan pengisian (berhenti sendiri saat penuh
// atau setelah 15 menit). Buzzer "tercapai" hanya dimatikan bila memang dinyalakan oleh menu ini.
void tampilkanWaterLevel() {
  bool tercapai = (digitalRead(PIN_WATER_LEVEL) == HIGH);

  if(tercapai){
    if(!wlBuzzerSudah){ mulaiBuzzer(BUZZER_DURASI_MS); wlBuzzerSudah = true; wlBuzzerMilikWL = true; }
    wlIsiAktif = false;                              // penuh -> pengisian selesai
  } else {
    if(wlBuzzerMilikWL && buzzerBunyi) hentikanBuzzer();
    wlBuzzerMilikWL = false;
    wlBuzzerSudah = false;
  }

  lcdPrintCenter(0, F("SENSOR WATER LEVEL"));

  if(wlIsiAktif && !waterLevelTestTimeout && (millis() - waterLevelTestMulai >= WL_TEST_TIMEOUT_MS)){
    waterLevelTestTimeout = true; wlIsiAktif = false;
  }
  bool bolehBuka = (wlIsiAktif && !tercapai);

  lcdPrint20(0, 1, tercapai ? F("Kondisi : Di Atas") : F("Kondisi : Di Bawah"));
  if(tercapai)                 lcdPrint20(0, 2, F("Status  : Terpenuhi"));
  else if(bolehBuka)           lcdPrint20(0, 2, F("Status  : Mengisi..."));
  else if(waterLevelTestTimeout) lcdPrint20(0, 2, F("Status  : Timeout!"));
  else                         lcdPrint20(0, 2, F("Status  : Diam"));

  if(bolehBuka) { if(!solenoidTerbuka) bukaSolenoid();  }
  else          { if(solenoidTerbuka)  tutupSolenoid(); }
  if(bolehBuka)      lcdPrint20(0, 3, F("Katup:Buka OK=Stop"));
  else if(tercapai)  lcdPrint20(0, 3, F("Penuh           <-"));
  else               lcdPrint20(0, 3, F("OK=Isi tandon   <-"));
}

void tampilkanKalkSuhuInput() {
  char f[10], b[21];
  lcdPrintCenter(0, F("SENSOR SUHU"));
  if(suhuSensorValid) fmtF(f, suhuRawTerakhir, 1); else strcpy(f, "ERR");
  lcdCenterP(1, PSTR("Suhu 1 : %s C"), f);
  if(inputBuffer.length() > 0) snprintf_P(b,sizeof(b),PSTR("Referensi : %s%s C"), suhuInputNegatif ? "-" : "", inputBuffer.c_str());
  else                         snprintf_P(b,sizeof(b),PSTR("Referensi : _ C"));
  lcdPrint20(0, 2, b);
  snprintf_P(b,sizeof(b),PSTR("Suhu 2 : %s C"), teksSuhu(f));
  lcdPrint20(0, 3, b);
}

// ====================================
// RENDER – SUB-MENU C: UJI DEBIT POMPA
// ====================================
void tampilkanMenuPilihPompa(){
  lcdPrintCenter(0, F("UJI DEBIT POMPA"));
  for(int i=0; i<3; i++){
    int idx=scrollPump+i;
    if(idx<TOTAL_UJI_OPTIONS){
      String prefix = (idx==selPump) ? "> " : "  ";
      lcdPrint20(0, i+1, prefix + String(ujiOptions[idx]));
    } else {
      lcdPrint20(0, i+1, "");
    }
  }
}

void tampilkanInputDelayPompa(){
  lcdPrintCenter(0, String(F("Tes ")) + String(pumpNamaRingkas[selPump]));
  lcdPrintCenter(1, F("Durasi aktif(detik):"));
  lcdPrintCenter(2, (inputBuffer.length()>0 ? inputBuffer : "_") + F(" detik"));
  lcdPrintCenter(3, selPump<3 ? F("(Maks 1800 detik)") : F("(Maks 10800 detik)"));
}

// Tampilan tes pompa non-blocking dengan bar progres
void tampilkanTesPompaAktif(){
  lcdPrintCenter(0, String(F("Tes ")) + String(pumpNamaRingkas[tesPompaIdx]));
  lcdPrintCenter(1, F("Pompa aktif..."));
  lcdCenterP(2, PSTR("%d/%d detik"), tesPompaDetikBerjalan, tesPompaDetikTarget);
  lcdBar(3, tesPompaDetikBerjalan, tesPompaDetikTarget);
}

void tampilkanCancelTesPompa(){
  lcdPrintCenter(0,F("Cancel Tes Pompa?"));
  lcdPrint20(0,1, String(tesPompaSelCancel==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,2, String(tesPompaSelCancel==1?"> ":"  ") + F("Tidak"));
  lcdPrint20(0,3,F(""));
}

void tampilkanTesSolenoid(){
  lcdPrintCenter(0, F("Tes Katup Solenoid"));
  lcdPrintCenter(1, solenoidTerbuka ? F("Status : TERBUKA") : F("Status : TERTUTUP"));
  if(solenoidTerbuka){
    unsigned long jalan = millis() - solenoidTesMulai;
    unsigned long sisa  = (jalan < SOLENOID_TES_MAKS_MS) ? (SOLENOID_TES_MAKS_MS - jalan)/1000UL : 0;
    char b[21]; snprintf_P(b,sizeof(b),PSTR("Tutup otomatis %02lu:%02lu"), sisa/60UL, sisa%60UL);
    lcdPrintCenter(2, b);
  } else if(solenoidTesPenuh) lcdPrintCenter(2, F("Tutup: tandon penuh"));
  else lcdPrint20(0, 2, F(""));
  lcdPrintCenter(3, solenoidTerbuka ? F("<-   OK=TUTUP") : F("<-   OK=BUKA"));
}

// =====================================
// RENDER – SUB MENU "INPUT DEBIT POMPA"
// =====================================
void tampilkanMenuInputDebitPompa(){
  lcdPrintCenter(0, F("INPUT DEBIT POMPA"));
  for(int i=0; i<3; i++){
    int idx=scrollInputDebit+i;
    if(idx<TOTAL_PUMP_OPTIONS){
      String prefix = (idx==selInputDebit) ? "> " : "  ";
      lcdPrint20(0, i+1, prefix + String(pumpNamaRingkas[idx]));
    } else {
      lcdPrint20(0, i+1, "");
    }
  }
}

void tampilkanInputDebitMl(){
  lcdPrintCenter(0, String(F("DEBIT ")) + String(pumpNamaRingkas[selInputDebit]));
  lcdPrintCenter(1, F("Air keluaran (ml):"));
  lcdPrintCenter(2, (inputBuffer.length()>0 ? inputBuffer : "_") + F(" ml"));
  lcdPrint20(0, 3, F(""));
}

void tampilkanInputDebitDetik(){
  lcdPrintCenter(0, String(F("DEBIT ")) + String(pumpNamaRingkas[selInputDebit]));
  lcdPrintCenter(1, F("Waktu (detik):"));
  lcdPrintCenter(2, (inputBuffer.length()>0 ? inputBuffer : "_") + F(" detik"));
  lcdPrint20(0, 3, F(""));
}

void tampilkanInputDebitHasil(){
  lcdPrintCenter(0, String(F("DEBIT ")) + String(pumpNamaRingkas[selInputDebit]));
  lcdPrintCenter(1, String(inputDebitHasil,3) + F(" L/menit"));   // TES_138: 3 desimal
  lcdPrintCenter(2, F("Parameter Disimpan"));
  lcdPrintCenter(3, simpanKeParamBaru() ? F("Dosing (Baru)") : F("Dosing (Aktif)"));
}

// ── Baris ringkasan 17+ (sama utk Aktif & Baru): jenis, makro, Kp/Ki PI (TES_121), jeda ──
// Kp/Ki yang ditampilkan = nilai acuan pada PPM TARGET konteks. CL Auto-Tune: dari persamaan & debit (Kp = 0.5/g,
// Ki = 0.1 Kp; kanal pH dihitung ulang tiap iterasi dari rentang pH & PPM terukur). CL PI ZN: nilai EFEKTIF
// (sudah dibatasi batas aman model), status profil ZN ditampilkan lebih dulu.
// TES_138: Kp/Ki tampil >= 3 angka penting, maks 7 karakter (dulu 3 desimal tetap: Ki 0.00235 tampil "0.002",
// 0.000425 tampil "0.000"). < 0.001 -> notasi ilmiah ringkas "4.2e-4".
char* fmtKp(char* b, float v){
  if(!(v>0.0f)){ b[0]='0'; b[1]='\0'; return b; }
  if(v < 0.001f){
    int e=0; float m=v; while(m<1.0f && e<9){ m*=10.0f; e++; }
    char t[8]; dtostrf(m,1,1,t);
    if(t[0]=='1' && t[1]=='0'){ t[0]='1'; t[1]='.'; t[2]='0'; t[3]='\0'; e--; }   // 9.96 -> "10.0" -> 1.0e-(e-1)
    snprintf_P(b,8,PSTR("%se-%d"),t,e);
    return b;
  }
  int dec = (v<0.01f)?5 : (v<0.1f)?4 : (v<10.0f)?3 : (v<100.0f)?2 : (v<1000.0f)?1 : 0;
  return fmtF(b, v, dec);
}
// TES_141: faktor skala ringkas ("x20.0", "x1.00", "x0.017")
char* fmtSkala(char* b, float v){ return fmtF(b, v, v>=100.0f?0 : v>=10.0f?1 : v>=1.0f?2 : 3); }
// TES_138: dua nilai Kp/Ki dalam satu baris LCD: "La=a Lb=b"; bila > 20 kolom tanda '=' dibuang (tak pernah terpotong)
void fmtDuaKp(char* out, const char* la, float a, const char* lb, float b){
  char x[12], y[12]; fmtKp(x,a); fmtKp(y,b);
  if(strlen(la)+strlen(lb)+strlen(x)+strlen(y)+3 <= 20) snprintf_P(out,21,PSTR("%s=%s %s=%s"),la,x,lb,y);
  else                                                    snprintf_P(out,21,PSTR("%s%s %s%s"),la,x,lb,y);
}
bool gainModelValidVol(int kanal, bool baru, int vol){
  int* deb = baru ? terpilihDebitPumpBaru : terpilihDebitPump;
  if(vol<=0) return false;
  if(kanal==0) return deb[4]>0 && deb[5]>0 && (baru ? slopeABMixTerpilihBaru : slopeABMixTerpilih)>0.0f;   // TES_138: P2 & P3 keduanya
  return deb[3]>0 && (baru ? terpilihConcPersenBaru : terpilihConcPersen)>0 && (baru ? terpilihPPMBaru : terpilihPPM)>0;
}
bool gainModelValid(int kanal, bool baru){ return gainModelValidVol(kanal, baru, baru ? terpilihVolumeBaru : terpilihVolume); }
bool gainModelValidUji(int kanal, bool baru){ return gainModelValidVol(kanal, baru, ujiVolumeL); }   // TES_141
// ── TES_141: skala Kp/Ki profil ZN (acuan uji) -> Parameter konteks ──
float slopePHKanal(int kanal, bool baru, int ppm){
  bool fosfat = ((baru ? terpilihAcidBaru : terpilihAcid)=="Fosfat");
  if(kanal==1) return interpSlopePH(fosfat?tabelSlopeAtasFosfat:tabelSlopeAtas,(float)ppm);
  return interpSlopePH(fosfat?tabelSlopeBawahFosfat:tabelSlopeBawah,(float)ppm);
}
float faktorSkalaVolZN(bool baru){   // gain proses ~ 1/V -> Kp,Ki ~ V
  const ProfilKpKiZN &p = profilZNKonteks(baru);
  int vol = baru ? terpilihVolumeBaru : terpilihVolume;
  return (p.terisi && p.sig.volume > 0.5f && vol > 0) ? (float)vol / p.sig.volume : 1.0f;
}
float faktorSkalaPPMZN(int kanal, bool baru){ return faktorSkalaPPMZNPada(kanal, baru, baru ? terpilihPPMBaru : terpilihPPM); }
// TES_148: tgt = PPM tempat Kp/Ki dipakai (tampilan: PPM target; dosing: PPM TERUKUR siklus berjalan)
float faktorSkalaPPMZNPada(int kanal, bool baru, int tgt){   // kanal pH: gain pH Down ~ slope(PPM) -> Kp,Ki ~ 1/slope
  if(kanal==0) return 1.0f;
  int ref = znPPMUji[baru?1:0][kanal-1];
  if(ref<=0 || tgt<=0 || abs(tgt-ref) <= ZN_TOL_PPM_SKALA) return 1.0f;   // diuji di PPM target (+/- toleransi sensor): pakai apa adanya
  float sRef = slopePHKanal(kanal, baru, ref), sTgt = slopePHKanal(kanal, baru, tgt);
  return (sRef>0.0f && sTgt>0.0f) ? sRef/sTgt : 1.0f;
}
float faktorSkalaZN(int kanal, bool baru){ return faktorSkalaVolZN(baru) * faktorSkalaPPMZN(kanal, baru); }
// Batas aman Kp/Ki ZN thd gain model g (per iterasi PI: a = g.Kp, b = g.Ki):
//   a <= 0.5 (FRAKSI_AMAN_ZN; Ki ikut diredam sebanding)  dan  TES_141: b <= a^2/4 (akar riil -> tanpa osilasi/overshoot)
void batasAmanKpKiZN(float &kp, float &ki, float g){
  if(!(g>0.0f)) return;
  float kpMaks = FRAKSI_AMAN_ZN/g;
  if(kp>kpMaks){ ki *= kpMaks/kp; kp = kpMaks; }
  float kiMaks = ZN_RASIO_KI_AKAR_RIIL*g*kp*kp;
  if(ki>kiMaks) ki = kiMaks;
}
// Status profil ZN konteks: 0 = OK, 1 = belum ada, 2 = tidak cocok konfigurasi, 3 = belum lengkap
// TES_144: bit kanal 'uji lama' yang dipakai proses (PPM & pH atas selalu; pH bawah bila target < 5.85) & terisi
uint8_t znKanalLamaDipakai(bool baru){
  const ProfilKpKiZN &p = profilZNKonteks(baru);
  uint8_t m = 0;
  if(p.kpPPM>0.0f) m |= 1;
  if(p.kpPHAtas>0.0f) m |= 2;
  if(p.kpPHBawah>0.0f && (baru ? terpilihPHBaru : terpilihPH) < batasFasePH) m |= 4;
  return znKanalLama[baru?1:0] & m;
}
byte statusProfilZN(bool baru){
  const ProfilKpKiZN &p = profilZNKonteks(baru);
  if(!p.terisi) return 1;
  if(!signatureSama(p.sig, ambilSignatureKonteks(baru)) || znKanalLamaDipakai(baru)) return 2;
  if(!profilZNLengkap(p, baru ? terpilihPHBaru : terpilihPH)) return 3;
  return 0;
}
// Kp/Ki ZN efektif satu kanal (0=PPM, 1=pH atas, 2=pH bawah) setelah batas aman model
void kpKiZNEfektif(int kanal, bool baru, float &kp, float &ki){
  const ProfilKpKiZN &p = profilZNKonteks(baru);
  kp = (kanal==0) ? p.kpPPM : (kanal==1 ? p.kpPHAtas : p.kpPHBawah);
  ki = (kanal==0) ? p.kiPPM : (kanal==1 ? p.kiPHAtas : p.kiPHBawah);
  float f = faktorSkalaZN(kanal, baru); kp *= f; ki *= f;   // TES_141: volume uji -> Parameter, PPM uji -> target
  batasAmanKpKiZN(kp, ki, hitungGainModel(kanal, baru));
}
int jumlahBarisPI(bool baru){
  int mode = baru ? dosingModeBaru : dosingMode;
  if(!(baru ? jenisDosingBaruSudahDiatur : jenisDosingSudahDiatur) || mode<0) return 0;
  if(mode==3) return 5;                            // Kp/Ki PPM, Kp pH atas/bawah, Ki pH
  if(mode==4) return (statusProfilZN(baru)!=1) ? 10 : 1;   // TES_141: status, vol uji, skala, 6 nilai efektif, PPM uji pH (profil terisi)
  return 0;                                        // Open Loop: tanpa PI
}
// ── RESOLUSI POMPA (TES_125) ──
// Perubahan akibat 1 detik pompa halus menurut persamaan: P1 (pH, fase tempat target berada, pada PPM target)
// dan P2/P3 (PPM, per sisi). Dosis terkecil yang bisa diberikan = 1 dtk; bila > 1/2 lebar batasan, dosis
// terakhir mudah melewati batas (aktuator satu arah, tak bisa dikoreksi) -> peringatan sebelum dosing.
float resolusiPH1dtk(bool baru){
  // Fase bawah (pH Down ~3x lebih kuat) dipakai bila batas bawah target masuk fase bawah (< 5.85): dosis terakhir
  // yang kelebihan akan jatuh di fase itu.
  float t = baru ? terpilihPHBaru : terpilihPH;
  int   b = baru ? batasPHx100Baru : batasPHx100;
  int kanal = (t - (float)b/100.0f < batasFasePH) ? 2 : 1;
  return gainModelValid(kanal, baru) ? hitungGainModel(kanal, baru) : 0.0f;
}
float resolusiPPM1dtk(bool baru){ return gainModelValid(0, baru) ? hitungGainModel(0, baru) : 0.0f; }
bool resolusiPHKasar(bool baru){
  int b = baru ? batasPHx100Baru : batasPHx100;
  return b>0 && resolusiPH1dtk(baru) > 0.5f*(float)b/100.0f;
}
bool resolusiPPMKasar(bool baru){
  int b = baru ? batasPPMBaru : batasPPM;
  return b>0 && resolusiPPM1dtk(baru) > 0.5f*(float)b;
}
// ── KAPASITAS POMPA PER SIKLUS (TES_131) ──
// Perubahan maksimum yang bisa dicapai dalam 1 siklus menurut persamaan, dgn batas aman tiap pompa & jendela waktu
// dosing 1 siklus (TES_133: tanpa batas waktu -> CL dibatasi makro + maks 30 pulsa PI, OL oleh batas aman pompa). Acuan: AB dari air baku 0 PPM -> target; pH dari 7,5 (air baku umum) -> target.
const float PH_ACUAN_AIR_BAKU = 7.5f;
float kapasitasPPMSiklus(bool baru){
  int*  deb  = baru ? terpilihDebitPumpBaru : terpilihDebitPump;
  int   vol  = baru ? terpilihVolumeBaru : terpilihVolume;
  float slope= baru ? slopeABMixTerpilihBaru : slopeABMixTerpilih;
  int   mode = baru ? dosingModeBaru : dosingMode;
  if(vol<=0 || slope<=0.0f) return 0.0f;
  // TES_138: batas volume per sisi saat dosing = min(T.qA, T.qB) -> kapasitas memakai pompa yg LEBIH LAMBAT (dulu rata-rata)
  float qD = ((float)min(deb[1],deb[2])/DEBIT_SKALA)*(1000.0f/60.0f);   // mL/dtk per sisi (D2-D3, makro)
  float qP = ((float)min(deb[4],deb[5])/DEBIT_SKALA)*(1000.0f/60.0f);   // mL/dtk per sisi (P2-P3, mikro/PI)
  float tD = (float)PUMP_MAKS_DETIK[1];
  float tP = (mode==1) ? (float)PUMP_MAKS_DETIK[4] : (float)PI_MAKS_ITERASI*(float)PUMP_MAKS_DETIK[4];
  float capD = qD*tD*slope/(float)vol, capP = qP*tP*slope/(float)vol;   // PPM maks makro (D2-D3) & mikro/PI (P2-P3)
  float m = (float)(baru ? persenMakroBaru : persenMakro)/100.0f;
  if(m < 0.0f) m = 0.0f; if(m > 0.95f) m = 0.95f;
  // Makro = m x kebutuhan (dibatasi capD), sisanya oleh P2-P3 (dibatasi capP) -> kebutuhan N tercapai bila
  // min(m.N, capD) + capP >= N. Bila P2-P3 yg membatasi: N maks = capP/(1-m) (makro tidak dipakai penuh).
  if(capP*m <= capD*(1.0f-m)) return capP/(1.0f-m);
  return capD + capP;
}
float kapasitasPHSiklus(bool baru){
  int* deb = baru ? terpilihDebitPumpBaru : terpilihDebitPump;
  int  mode= baru ? dosingModeBaru : dosingMode;
  float q1 = ((float)deb[3]/DEBIT_SKALA)*(1000.0f/60.0f);
  float t  = (mode==1) ? 2.0f*(float)PUMP_MAKS_DETIK[3] : (float)(1+PI_MAKS_ITERASI)*(float)PUMP_MAKS_DETIK[3];   // OL: makro+mikro; CL: makro+PI
  return prediksiTurunPHK(baru, PH_ACUAN_AIR_BAKU, q1*t, baru ? terpilihPPMBaru : terpilihPPM);
}
bool kapasitasPPMKurang(bool baru){ int t = baru ? terpilihPPMBaru : terpilihPPM; return t>0 && kapasitasPPMSiklus(baru) < (float)t; }
bool kapasitasPHKurang(bool baru){
  float t = baru ? terpilihPHBaru : terpilihPH;
  return t>0.0f && t < PH_ACUAN_AIR_BAKU && kapasitasPHSiklus(baru) < (PH_ACUAN_AIR_BAKU - t);
}
// TES_141: CL PI ZN dgn Kp/Ki dari konfigurasi lama (selain volume)
bool znKonfigLama(bool baru){
  int mode = baru ? dosingModeBaru : dosingMode;
  const ProfilKpKiZN &p = profilZNKonteks(baru);
  return mode==4 && p.terisi && (!signatureSama(p.sig, ambilSignatureKonteks(baru)) || znKanalLamaDipakai(baru));
}
// Tahap layar peringatan: 0 Kp/Ki ZN konfigurasi lama (TES_141), 1 resolusi pH, 2 resolusi PPM, 3 kapasitas AB, 4 kapasitas pH
bool tahapPeringatanPerlu(byte tahap, bool baru){
  if(tahap==0) return znKonfigLama(baru);
  if(tahap==1) return resolusiPHKasar(baru);
  if(tahap==2) return resolusiPPMKasar(baru);
  if(tahap==3) return kapasitasPPMKurang(baru);
  if(tahap==4) return kapasitasPHKurang(baru);
  return false;
}
int tahapPeringatanBerikut(byte dari, bool baru){ for(byte t=dari; t<5; t++) if(tahapPeringatanPerlu(t,baru)) return t; return -1; }
bool perluPeringatanResolusi(bool baru){ return tahapPeringatanBerikut(0, baru) >= 0; }
byte resolusiTahap = 0;      // 0 = Kp/Ki ZN konfig lama (TES_141), 1 = resolusi pH, 2 = resolusi PPM, 3 = kapasitas AB, 4 = kapasitas pH
bool resolusiBaru  = false;  // konteks: Parameter Baru (commit) / Aktif (Mulai Dosing)
void tampilkanPeringatanResolusi(){
  char b[21], f[12];
  if(resolusiTahap==0){   // TES_141: Kp/Ki ZN hasil uji dgn konfigurasi lain (selain volume) -> saran uji ulang, boleh lanjut
    const __FlashStringHelper* beda = signatureBeda(profilZNKonteks(resolusiBaru).sig, ambilSignatureKonteks(resolusiBaru));
    lcdPrintCenter(0, F("Kp/Ki ZN KONFIG LAMA"));
    if(beda) lcdPrintCenter(1, String(F("Beda: ")) + String(beda));
    else {   // TES_144: konfigurasi profil = sekarang, tetapi ada kanal hasil uji konfigurasi lama
      uint8_t m = znKanalLamaDipakai(resolusiBaru);
      int n = ((m&1)?1:0) + ((m&2)?1:0) + ((m&4)?1:0);
      if(n>1) lcdPrintCenter(1, String(F("Uji lama: ")) + String(n) + F(" kanal"));
      else    lcdPrintCenter(1, (m&1) ? F("Uji lama: PPM") : (m&2) ? F("Uji lama: pH Atas") : F("Uji lama: pH Bawah"));
    }
    lcdPrintCenter(2, F("Lakukan Uji Reaksi"));
    lcdPrintCenter(3, F("<-   |   OK"));
    return;
  }
  if(resolusiTahap==3){   // TES_131: kapasitas AB
    lcdPrintCenter(0, F("KAPASITAS POMPA AB"));
    snprintf_P(b,sizeof(b),PSTR("Maks 0->%ld PPM"), (long)kapasitasPPMSiklus(resolusiBaru)); lcdPrintCenter(1,b);   // 1 siklus, dari air baku
    snprintf_P(b,sizeof(b),PSTR("Target: %d PPM"), resolusiBaru ? terpilihPPMBaru : terpilihPPM); lcdPrintCenter(2,b);
    lcdPrintCenter(3, F("<-   |   OK"));
    return;
  }
  if(resolusiTahap==4){   // TES_131: kapasitas pH Down
    lcdPrintCenter(0, F("KAPASITAS POMPA pH"));
    char g[10]; float phMin = PH_ACUAN_AIR_BAKU - kapasitasPHSiklus(resolusiBaru);   // 1 siklus, dari pH 7,5
    if(phMin < 0.0f) phMin = 0.0f;   // TES_141: debit P1 sangat besar -> prediksi negatif ("7.5->-1268.22" > 20 kolom)
    snprintf_P(b,sizeof(b),PSTR("Maks %s->%s pH"), fmtF(f,PH_ACUAN_AIR_BAKU,1), fmtF(g,phMin,2)); lcdPrintCenter(1,b);
    snprintf_P(b,sizeof(b),PSTR("Target: %s pH"), fmtF(f,resolusiBaru ? terpilihPHBaru : terpilihPH,2)); lcdPrintCenter(2,b);
    lcdPrintCenter(3, F("<-   |   OK"));
    return;
  }
  lcdPrintCenter(0, F("RESOLUSI POMPA KASAR"));
  if(resolusiTahap==1){
    float r = resolusiPH1dtk(resolusiBaru);
    snprintf_P(b,sizeof(b),PSTR("1 dtk P1 = %s pH"), fmtF(f,r,r<1.0f?2:1)); lcdPrintCenter(1,b);
    int bt = resolusiBaru ? batasPHx100Baru : batasPHx100;
    snprintf_P(b,sizeof(b),PSTR("Batas: +/-%d.%02d pH"), bt/100, bt%100); lcdPrintCenter(2,b);
  } else {
    float r = resolusiPPM1dtk(resolusiBaru);
    snprintf_P(b,sizeof(b),PSTR("1dtk P2-P3=%s PPM"), fmtF(f,r,r<10.0f?1:0)); lcdPrintCenter(1,b);
    snprintf_P(b,sizeof(b),PSTR("Batas: +/-%d PPM"), resolusiBaru ? batasPPMBaru : batasPPM); lcdPrintCenter(2,b);
  }
  lcdPrintCenter(3, F("<-   |   OK"));
}
void mulaiPeringatanResolusi(bool baru){
  resolusiBaru = baru;
  { int t = tahapPeringatanBerikut(0, baru); resolusiTahap = (t<0) ? 0 : (byte)t; }
  mulaiBuzzer(BUZZER_DURASI_MS > 2000UL ? 2000UL : BUZZER_DURASI_MS);
  currentState = RESOLUSI_PERINGATAN;
  lcd.clear(); tampilkanPeringatanResolusi();
}
void prosesPeringatanResolusi(char key){
  if(key=='*'){ hentikanBuzzer(); currentState=MENU_UTAMA; lcd.clear(); tampilkanMenuUtama(); return; }
  if(key!='B') return;
  hentikanBuzzer();
  { int t = tahapPeringatanBerikut(resolusiTahap+1, resolusiBaru);   // TES_131: resolusi -> kapasitas
    if(t>=0){ resolusiTahap=(byte)t; lcd.clear(); tampilkanPeringatanResolusi(); return; } }
  lcd.clear();
  if(resolusiBaru) komitParameterBaruLaluMulai();
  else             mulaiDosingSekarang();
}

void ringkasanBarisLanjut(char* buf, size_t n, int idx, bool baru){
  int  mode    = baru ? dosingModeBaru : dosingMode;
  bool jenisOk = (baru ? jenisDosingBaruSudahDiatur : jenisDosingSudahDiatur) && mode>=0;
  int  persen  = baru ? persenMakroBaru : persenMakro;
  char f[12];
  buf[0]='\0';
  if(idx==17){
    if(!jenisOk) snprintf_P(buf,n,PSTR("Jenis : -"));
    else snprintf_P(buf,n,PSTR("Jenis : %s"), mode==3?"CL Auto-Tune":mode==4?"CL PI ZN":"Open Loop");
    return;
  }
  if(idx==18){
    if(!jenisOk)       snprintf_P(buf,n,PSTR("Makro : -"));
    else if(mode==1)   snprintf_P(buf,n,PSTR("Makro : %d%% + Mikro"),persen);   // mikro = sisa (100-makro)%
    else               snprintf_P(buf,n,PSTR("Makro : %d%% + PI"),persen);
    return;
  }
  if(idx==19 || idx==20){   // TES_125: resolusi 1 dtk pompa halus ("!" = lebih dari 1/2 batasan -> kasar)
    if(idx==19){
      float r = resolusiPH1dtk(baru);
      if(r<=0.0f) snprintf_P(buf,n,PSTR("1dtk P1 : -"));
      else snprintf_P(buf,n,PSTR("1dtk P1 : %s pH%s"), fmtF(f,r,r<1.0f?2:1), resolusiPHKasar(baru)?"!":"");
    } else {
      float r = resolusiPPM1dtk(baru);
      if(r<=0.0f) snprintf_P(buf,n,PSTR("1dtk P2-3: -"));
      else snprintf_P(buf,n,PSTR("1dtk P2-3: %s PPM%s"), fmtF(f,r,r<10.0f?1:0), resolusiPPMKasar(baru)?"!":"");
    }
    return;
  }
  int nPI = jumlahBarisPI(baru);
  int r = idx-21;
  if(r < nPI){
    if(mode==3){
      if(r<=1){
        if(!gainModelValid(0,baru)){ snprintf_P(buf,n, r==0?PSTR("Kp PPM    : -"):PSTR("Ki PPM    : -")); return; }
        float kp = PI_FRAKSI_KOREKSI/hitungGainModel(0,baru);
        snprintf_P(buf,n, r==0?PSTR("Kp PPM    : %s"):PSTR("Ki PPM    : %s"), fmtKp(f, r==0?kp:PI_RASIO_KI*kp));
      } else if(r<=3){
        int kanal = r-1;   // 1 = fase atas, 2 = fase bawah
        if(!gainModelValid(kanal,baru)){ snprintf_P(buf,n, kanal==1?PSTR("Kp pH>5.85: -"):PSTR("Kp pH<5.85: -")); return; }
        snprintf_P(buf,n, kanal==1?PSTR("Kp pH>5.85: %s"):PSTR("Kp pH<5.85: %s"), fmtKp(f, PI_FRAKSI_KOREKSI/hitungGainModel(kanal,baru)));
      } else {
        snprintf_P(buf,n,PSTR("Ki pH     : 0.1 x Kp"));
      }
    } else {   // mode 4
      if(r==0){
        byte st = statusProfilZN(baru);
        snprintf_P(buf,n,PSTR("Profil ZN: %S"), st==0?PSTR("OK"):st==1?PSTR("BELUM ADA"):st==2?PSTR("TDK COCOK"):PSTR("BLM LKP"));
        return;
      }
      const ProfilKpKiZN &pz = profilZNKonteks(baru);
      if(r==1){ snprintf_P(buf,n,PSTR("Vol. uji : %d L"), (int)(pz.sig.volume+0.5f)); return; }   // TES_141
      if(r==2){ snprintf_P(buf,n,PSTR("Skala vol: x%s"), fmtSkala(f, faktorSkalaVolZN(baru))); return; }
      if(r==9){   // PPM tandon saat uji kanal pH atas / bawah
        char a[8], b2[8];
        int pa = znPPMUji[baru?1:0][0], pb = znPPMUji[baru?1:0][1];
        if(pz.kpPHAtas>0.0f && pa>0) itoa(pa,a,10); else strcpy(a,"-");
        if(pz.kpPHBawah>0.0f && pb>0) itoa(pb,b2,10); else strcpy(b2,"-");
        snprintf_P(buf,n,PSTR("PPM uji pH: %s/%s"),a,b2);
        if(strlen(buf)>20) snprintf_P(buf,n,PSTR("PPM uji pH:%s/%s"),a,b2);
        return;
      }
      int kanal = (r-3)/2;           // 0 = PPM, 1 = pH atas, 2 = pH bawah
      bool barisKi = ((r-3)%2)==1;
      float kp, ki; kpKiZNEfektif(kanal, baru, kp, ki);
      static const char lblPPM[] PROGMEM = "PPM    ";
      static const char lblAtas[] PROGMEM = "pH>5.85";
      static const char lblBawah[] PROGMEM = "pH<5.85";
      const char* lbl = kanal==0 ? lblPPM : (kanal==1 ? lblAtas : lblBawah);
      if(kp<=0.0f) snprintf_P(buf,n,PSTR("%s %S: -"), barisKi?"Ki":"Kp", lbl);
      else         snprintf_P(buf,n,PSTR("%s %S: %s"), barisKi?"Ki":"Kp", lbl, fmtKp(f, barisKi?ki:kp));
    }
    return;
  }
  int j = r - nPI;   // baris jeda
  if(j<0 || j>=JUMLAH_SLOT_JEDA) return;
  bool jedaOk = baru ? jedaBaruSudahDiatur : jedaSudahDiatur;
  int* arr = baru ? dosingJedaHomogenisasiBaru : dosingJedaHomogenisasi;
  if(!jedaOk){ if(j==0) snprintf_P(buf,n,PSTR("Jeda  : -")); }
  else if(j==0) snprintf_P(buf,n,PSTR("Jeda  : J0 %d min"),arr[slotJeda(0)]);
  else          snprintf_P(buf,n,PSTR("        J%d %d min"),j,arr[slotJeda(j)]);
}

// ==================================
// RENDER – RINGKASAN PARAMETER AKTIF
// ==================================
String ringkasanBarisKe(int idx){
  char buf[21];
  switch(idx){
    case 0:
      if(terpilihPPM==0) snprintf(buf,sizeof(buf),"PPM      : -");
      else                snprintf(buf,sizeof(buf),"PPM      : %d",terpilihPPM);
      break;
    case 1: {
      if(terpilihPH==0.0f){
        snprintf(buf,sizeof(buf),"pH       : -");
      } else {
        char phbuf[8]; dtostrf(terpilihPH,4,2,phbuf);   // TES_128: 2 desimal sesuai input
        snprintf(buf,sizeof(buf),"pH       : %s",phbuf);
      }
      break;
    }
    case 2:
      snprintf(buf,sizeof(buf),"AB Mix   : %s",terpilihEC.c_str());
      break;
    case 3:
      snprintf(buf,sizeof(buf),"           %s",terpilihDilution.c_str());
      break;
    case 4:
      snprintf(buf,sizeof(buf),"pH Down  : %s",terpilihAcid.c_str());
      break;
    case 5:
      snprintf(buf,sizeof(buf),"           %s",terpilihConc.c_str());
      break;
    case 6:
      if(terpilihVolume==0) snprintf(buf,sizeof(buf),"V Tandon : -");
      else                  snprintf(buf,sizeof(buf),"V Tandon : %d L",terpilihVolume);
      break;
    case 7:
      snprintf(buf,sizeof(buf),"Waktu    : %s",terpilihWaktuDosing.c_str());
      break;
    default: {
      int p = idx-8;
      if(p==0){
        if(terpilihLamaCekSensor<=0) snprintf(buf,sizeof(buf),"Bilas D1 : -");
        else                         snprintf(buf,sizeof(buf),"Bilas D1 : %d detik",terpilihLamaCekSensor);
      } else if(p==1){
        if(terpilihJedaStabilisasi<=0) snprintf(buf,sizeof(buf),"Stabil   : -");
        else                           snprintf(buf,sizeof(buf),"Stabil   : %d detik",terpilihJedaStabilisasi);
      } else if(p>=2 && p<=TOTAL_PUMP_OPTIONS){
        snprintf(buf,sizeof(buf),"%s: %s",pumpNamaRingkas[p-1],formatDebit(terpilihDebitPump[p-1]).c_str());
      } else if(idx==15){
        if(!batasanSudahDiatur) snprintf_P(buf,sizeof(buf),PSTR("Batas PPM: -"));
        else      snprintf_P(buf,sizeof(buf),PSTR("Batas PPM: +/-%d"),batasPPM);
      } else if(idx==16){
        if(!batasanSudahDiatur) snprintf_P(buf,sizeof(buf),PSTR("Batas pH : -"));
        else      snprintf_P(buf,sizeof(buf),PSTR("Batas pH : +/-%d.%02d"),batasPHx100/100,batasPHx100%100);
      } else if(idx>=17){
        ringkasanBarisLanjut(buf, sizeof(buf), idx, false);   // jenis, makro, Kp/Ki PI, jeda
      } else {
        buf[0]='\0';
      }
      break;
    }
  }
  return String(buf);
}

void tampilkanMenuRingkasan(){
  lcdPrintCenter(0,F("PARAMETER AKTIF"));
  if(scrollSummary<0) scrollSummary=0;
  { int mx=maksScrollRingkasan(false); if(scrollSummary>mx) scrollSummary=mx; }
  for(int i=0;i<3;i++){
    int idx=scrollSummary+i;
    lcdPrint20(0,i+1, idx<TOTAL_SUMMARY_ROWS ? ringkasanBarisKe(idx) : String(""));
  }
}

// Ringkasan Parameter Baru 
String ringkasanBarisKeBaru(int idx){
  char buf[21];
  switch(idx){
    case 0:
      if(terpilihPPMBaru==0) snprintf(buf,sizeof(buf),"PPM      : -");
      else                   snprintf(buf,sizeof(buf),"PPM      : %d",terpilihPPMBaru);
      break;
    case 1: {
      if(terpilihPHBaru==0.0f){
        snprintf(buf,sizeof(buf),"pH       : -");
      } else {
        char phbuf[8]; dtostrf(terpilihPHBaru,4,2,phbuf);
        snprintf(buf,sizeof(buf),"pH       : %s",phbuf);
      }
      break;
    }
    case 2:
      snprintf(buf,sizeof(buf),"AB Mix   : %s",terpilihECBaru.c_str());
      break;
    case 3:
      snprintf(buf,sizeof(buf),"           %s",terpilihDilutionBaru.c_str());
      break;
    case 4:
      snprintf(buf,sizeof(buf),"pH Down  : %s",terpilihAcidBaru.c_str());
      break;
    case 5:
      snprintf(buf,sizeof(buf),"           %s",terpilihConcBaru.c_str());
      break;
    case 6:
      if(terpilihVolumeBaru==0) snprintf(buf,sizeof(buf),"V Tandon : -");
      else                      snprintf(buf,sizeof(buf),"V Tandon : %d L",terpilihVolumeBaru);
      break;
    case 7:
      snprintf(buf,sizeof(buf),"Waktu    : %s",terpilihWaktuDosingBaru.c_str());
      break;
    default: {
      int p = idx-8;
      if(p==0){
        if(terpilihLamaCekSensorBaru<=0) snprintf(buf,sizeof(buf),"Bilas D1 : -");
        else                             snprintf(buf,sizeof(buf),"Bilas D1 : %d detik",terpilihLamaCekSensorBaru);
      } else if(p==1){
        if(terpilihJedaStabilisasiBaru<=0) snprintf(buf,sizeof(buf),"Stabil   : -");
        else                               snprintf(buf,sizeof(buf),"Stabil   : %d detik",terpilihJedaStabilisasiBaru);
      } else if(p>=2 && p<=TOTAL_PUMP_OPTIONS){
        snprintf(buf,sizeof(buf),"%s: %s",pumpNamaRingkas[p-1],formatDebit(terpilihDebitPumpBaru[p-1]).c_str());
      } else if(idx==15){
        if(!batasanBaruSudahDiatur) snprintf_P(buf,sizeof(buf),PSTR("Batas PPM: -"));
        else      snprintf_P(buf,sizeof(buf),PSTR("Batas PPM: +/-%d"),batasPPMBaru);
      } else if(idx==16){
        if(!batasanBaruSudahDiatur) snprintf_P(buf,sizeof(buf),PSTR("Batas pH : -"));
        else      snprintf_P(buf,sizeof(buf),PSTR("Batas pH : +/-%d.%02d"),batasPHx100Baru/100,batasPHx100Baru%100);
      } else if(idx>=17){
        ringkasanBarisLanjut(buf, sizeof(buf), idx, true);
      } else {
        buf[0]='\0';
      }
      break;
    }
  }
  return String(buf);
}

void tampilkanMenuRingkasanBaru(){
  lcdPrintCenter(0,F("PARAMETER BARU"));
  if(!draftBaruAda){   // belum ada draft: tak ada perubahan yang menunggu dipakai
    lcdPrintCenter(1, F("Belum ada perubahan"));
    lcdPrintCenter(2, dosingSedangMenunggu ? F("(Parameter Aktif)") : F("(jadwal tidak aktif)"));
    lcdPrint20(0,3,F(""));
    return;
  }
  if(scrollSummaryBaru<0) scrollSummaryBaru=0;
  { int mx=maksScrollRingkasan(true); if(scrollSummaryBaru>mx) scrollSummaryBaru=mx; }
  for(int i=0;i<3;i++){
    int idx=scrollSummaryBaru+i;
    lcdPrint20(0,i+1, idx<TOTAL_SUMMARY_ROWS ? ringkasanBarisKeBaru(idx) : String(""));
  }
}

// ====================================
// RENDER – SUB-MENU D: MODE KALKULATOR
// ====================================
void tampilkanSubMenuD(){
  lcdPrintCenter(0,F("MODE KALKULATOR"));
  for(int i=0;i<3;i++){
    int idx=scrollMenuD+i;
    if(idx<TOTAL_MENU_D_ROWS) lcdPrint20(0,i+1,(idx==selMenuD?"> ":"  ")+String(menuDOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}

// ==========================================
// KALKULATOR "TINGGI WATER LEVEL"
// ==========================================
void hitungKalkWLTabung(){
  if(kalkWLJariJari<=0){ kalkWLHasil=0.0f; return; }
  float volCm3 = (float)kalkWLVolume * 1000.0f;
  kalkWLHasil = volCm3 / ((float)PI * (float)kalkWLJariJari * (float)kalkWLJariJari);
}
void hitungKalkWLBalok(){
  if(kalkWLPanjang<=0 || kalkWLLebar<=0){ kalkWLHasil=0.0f; return; }
  float volCm3 = (float)kalkWLVolume * 1000.0f;
  kalkWLHasil = volCm3 / ((float)kalkWLPanjang * (float)kalkWLLebar);
}

// Tahap 1: Pilih Bentuk Tandon
void tampilkanKalkWLBentuk(){
  lcdPrintCenter(0,F("Bentuk Tandon :"));
  lcdPrint20(0,1, String(selKalkWLBentuk==0?"> ":"  ") + F("Tabung"));
  lcdPrint20(0,2, String(selKalkWLBentuk==1?"> ":"  ") + F("Balok"));
  lcdPrint20(0,3,F(""));
}

// ── TABUNG ──
void tampilkanKalkWLTabungVol(){
  lcdPrintCenter(0,F("TABUNG"));
  lcdPrint20(0,1, String(F("V target : ")) + (inputBuffer.length()>0?inputBuffer:"_") + F(" L"));
  lcdPrint20(0,2,F(""));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkWLTabungJari(){
  lcdPrintCenter(0,F("TABUNG"));
  lcdPrint20(0,1, String(F("V target : ")) + String(kalkWLVolume) + F(" L"));
  lcdPrint20(0,2, String(F("Jari-jari : ")) + (inputBuffer.length()>0?inputBuffer:"_") + F(" cm"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkWLTabungHasil(){
  lcdPrint20(0,0,F("Penempatan sensor :"));
  lcdPrintCenter(1, String(kalkWLHasil,1) + F(" cm"));
  lcdPrintCenter(2, F("dari bawah tandon"));
  lcdPrint20(0,3,F(""));
}

// ── BALOK ──
void tampilkanKalkWLBalokVol(){
  lcdPrintCenter(0,F("BALOK"));
  lcdPrint20(0,1, String(F("V target : ")) + (inputBuffer.length()>0?inputBuffer:"_") + F(" L"));
  lcdPrint20(0,2,F(""));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkWLBalokPanjang(){
  lcdPrintCenter(0,F("BALOK"));
  lcdPrint20(0,1, String(F("V target : ")) + String(kalkWLVolume) + F(" L"));
  lcdPrint20(0,2, String(F("Panjang : ")) + (inputBuffer.length()>0?inputBuffer:"_") + F(" cm"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkWLBalokLebar(){
  lcdPrintCenter(0,F("BALOK"));
  lcdPrint20(0,1, String(F("V target : ")) + String(kalkWLVolume) + F(" L"));
  lcdPrint20(0,2, String(F("Panjang : ")) + String(kalkWLPanjang) + F(" cm"));
  lcdPrint20(0,3, String(F("Lebar : ")) + (inputBuffer.length()>0?inputBuffer:"_") + F(" cm"));
}
void tampilkanKalkWLBalokHasil(){
  lcdPrint20(0,0,F("Penempatan sensor :"));
  lcdPrintCenter(1, String(kalkWLHasil,1) + F(" cm"));
  lcdPrintCenter(2, F("dari bawah tandon"));
  lcdPrint20(0,3,F(""));
}

void tampilkanKalkInputMenu(){
  lcdPrintCenter(0,F("PPM/pH DARI INPUT"));
  lcdPrint20(0,1, String(selKalkInput==0?"> ":"  ") + F("PPM"));
  lcdPrint20(0,2, String(selKalkInput==1?"> ":"  ") + F("pH"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkMenuDil(){
  lcdPrint20(0,0,F("Pengenceran:"));
  lcdPrint20(0,1, String(kalkSelDil==0?"> ":"  ") + F("5 ml/L"));
  lcdPrint20(0,2, String(kalkSelDil==1?"> ":"  ") + F("10 ml/L"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkInputVolume(){
  lcdPrintCenter(0,F("Volume Tandon"));
  lcdPrintCenter(1,F("(Liter, 30-3.000):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" Liter"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkInputPPMAwal(){
  lcdPrintCenter(0,String(F("Vol: "))+String(kalkVolume)+F(" L"));
  lcdPrintCenter(1,F("PPM Awal (0-2100):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" PPM"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkInputPPMTarget(){
  lcdPrintCenter(0,String(F("PPM Awal : "))+String(kalkPPMAwal)+F(" PPM"));
  lcdPrintCenter(1,F("Target (100-2100):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" PPM"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkHasilPPM(){
  lcdPrintCenter(0, String(F("PPM: ")) + String(kalkPPMAwal) + F(" -> ") + String(kalkPPMTarget));
  String sat=(kalkVolume>0)?" ml":" ml/L";
  if(kalkHasilTambah<=0.0f){
    lcdPrintCenter(1, F("AB Mix: tidak perlu"));
    lcdPrint20(0,2,F(""));
    lcdPrint20(0,3,F(""));
  } else {
    // Volume berlaku untuk MASING-MASING larutan A dan B (pompa D2 = A, D3 = B)
    lcdPrintCenter(1, F("Butuh A dan B:"));
    lcdPrintCenter(2, String(kalkHasilTambah,1)+sat);
    // Estimasi waktu nyala dari debit D2 & D3 tersimpan (berjalan bersamaan -> ambil terlama)
    if(kalkVolume>0 && terpilihDebitPump[1]>0 && terpilihDebitPump[2]>0){
      float t2=hitungDurasiPompaDetik(kalkHasilTambah,1), t3=hitungDurasiPompaDetik(kalkHasilTambah,2);
      lcdPrintCenter(3, String(F("Pompa: "))+String((long)round(t2>t3?t2:t3))+F(" detik"));
    } else {
      lcdPrintCenter(3, F("Pompa: -"));
    }
  }
}

void tampilkanKalkPHInputVolume(){
  lcdPrintCenter(0,F("Vol Tandon"));
  lcdPrintCenter(1,F("(Liter, 30-3.000):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" Liter"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkPHMenuAsam(){
  lcdPrint20(0,0,F("Jenis Asam pH Down:"));
  lcdPrint20(0,1, String(kalkPHAcid==0?"> ":"  ") + F("Nitrat"));
  lcdPrint20(0,2, String(kalkPHAcid==1?"> ":"  ") + F("Fosfat"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkPHMenuKonsen(){
  lcdPrintCenter(0,F("Konsentrasi pH Down:"));
  for(int i=0;i<3;i++){
    int idx=scrollKalkPHKonsen+i;
    if(idx<TOTAL_KONSEN_OPTIONS) lcdPrint20(0,i+1,(idx==kalkPHKonsen?"> ":"  ")+String(konsenPHOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}
void tampilkanKalkPHInputPPM(){
  lcdPrintCenter(0,String(F("Vol:"))+String(kalkPHVolume)+F("L"));
  lcdPrintCenter(1,F("PPM (100-2100):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" PPM"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkPHInputAwal(){
  lcdPrintCenter(0,String(F("PPM: "))+String(kalkPHPpm));
  lcdPrintCenter(1,F("pH Awal (3.5-8.0):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" pH"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkPHInputTarget(){
  lcdPrintCenter(0, String(F("pH Awal : ")) + String(kalkPHAwal,2));
  lcdPrintCenter(1,F("pH Target (>=3.5):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" pH"));
  lcdPrint20(0,3,F(""));
}
void tampilkanKalkPHHasil(){
  lcdPrintCenter(0, String(F("pH: ")) + String(kalkPHAwal,2) + F(" -> ") + String(kalkPHTarget,2));   // TES_128: 2 desimal
  lcdPrintCenter(1, String(F("pH Down: "))+String(kalkPHHasilTotal,2)+F(" ml"));
  lcdPrintCenter(2, F("Waktu Nyala Pompa:"));
  // Estimasi dari debit P1 tersimpan
  if(kalkPHHasilTotal>0.0f && terpilihDebitPump[3]>0){
    lcdPrintCenter(3, String((long)round(hitungDurasiPompaDetik(kalkPHHasilTotal,3)))+F(" Detik"));
  } else {
    lcdPrintCenter(3, F("-"));
  }
}

// ==================================
// RENDER – SUB-MENU "BATASAN PPM/pH"
// ==================================
// Baris 4 menampilkan rentang hasil (target -/+ batas) bila target pada konteks (Aktif/Baru) sudah diisi.
void tampilkanInputBatasPPM(){
  int target = tampilBaru() ? terpilihPPMBaru : terpilihPPM;
  char b[21];
  lcdPrintCenter(0,F("BATASAN PPM"));
  snprintf_P(b,sizeof(b),PSTR("(+/- %d-%d PPM):"),BATAS_PPM_MIN,batasPPMMaksUntuk(target));   // maks mengikuti target
  lcdPrintCenter(1,b);
  lcdPrintCenter(2,String(F("+/- "))+(inputBuffer.length()>0?inputBuffer:"_")+F(" PPM"));
  int v = inputBuffer.toInt();
  if(target>0 && batasPPMValid(v) && !batasPPMSesuaiTarget(v,target)){
    lcdPrintCenter(3,F("Melebihi target!"));
  } else if(target>0 && batasPPMValid(v)){
    snprintf_P(b,sizeof(b),PSTR("Rentang %d-%d"),target-v,target+v);
    lcdPrintCenter(3,b);
  } else lcdPrint20(0,3,F(""));
}
void tampilkanInputBatasPH(){
  lcdPrintCenter(0,F("BATASAN pH"));
  lcdPrintCenter(1,F("(+/- 0.25-1.00):"));
  String tampil;
  int n = inputBuffer.length();
  if(n==0)      tampil = F("_.__");
  else if(n==1) tampil = inputBuffer + F(".__");
  else if(n==2) tampil = inputBuffer.substring(0,1) + "." + inputBuffer.substring(1,2) + "_";
  else          tampil = inputBuffer.substring(0,1) + "." + inputBuffer.substring(1,3);
  lcdPrintCenter(2,String(F("+/- "))+tampil);
  float target = tampilBaru() ? terpilihPHBaru : terpilihPH;
  int v = inputBuffer.toInt();
  if(target>0.0f && n==3 && batasPHValid(v)){
    float d = v/100.0f;
    lcdPrintCenter(3,String(F("Rentang "))+String(target-d,2)+F("-")+String(target+d,2));
  } else lcdPrintCenter(3,F("<-   |   OK"));
}

// =========================================
// RENDER – MENU JENIS DOSING & FASE LAINNYA
// =========================================
void tampilkanMenuJenisDosing(){
  lcdPrintCenter(0,F("JENIS DOSING"));
  for(int i=0;i<TOTAL_JENIS_DOSING_OPTIONS;i++){
    lcdPrint20(0,i+1, String(selJenisDosing==i?"> ":"  ") + String(jenisDosingOptions[i]));
  }
}
// Layar input besar makro dosing. Baris 2 menjelaskan pembagian sesuai jenis yang dipilih.
void tampilkanInputPersenMakro(){
  char b[21];
  lcdPrintCenter(0,F("Besar Makro Dosing"));
  int v = inputBuffer.toInt();
  if(inputBuffer.length()==2 && persenMakroValid(v)){
    if(modeJenisTunda==1) snprintf_P(b,sizeof(b),PSTR("Makro %d%% Mikro %d%%"),v,100-v);
    else                  snprintf_P(b,sizeof(b),PSTR("Makro %d%% + PI"),v);
    lcdPrintCenter(1,b);
  } else lcdPrintCenter(1,F("(50 - 90 %)"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:String("_"))+F(" %"));
  lcdPrintCenter(3,F("<-   |   OK"));
}

// ============================================================================
// ============  EC BEBAS (0.20-4.00), format otomatis "X.XX"  ==============
// ============================================================================
// (deklarasi EC_BEBAS_MIN/MAKS & ecBebasDigits dipindah ke dekat awal file — lihat dekat deklarasi dilOptions/acidOptions)

void tampilkanInputECBebas(){
  lcdPrintCenter(0, F("Target EC:"));
  lcdPrintCenter(1, F("(0.20 - 4.00)"));
  String tampil;
  int n = ecBebasDigits.length();
  if(n==0)      tampil = F("_._ _");
  else if(n==1) tampil = ecBebasDigits + F("._ _");
  else if(n==2) tampil = ecBebasDigits.substring(0,1) + "." + ecBebasDigits.substring(1,2) + "_";
  else          tampil = ecBebasDigits.substring(0,1) + "." + ecBebasDigits.substring(1,3);
  lcdPrintCenter(2, tampil);
  lcdPrintCenter(3, F("<-   |   OK"));
}

void prosesInputECBebas(char key){
  if(key>='0' && key<='9'){
    if(ecBebasDigits.length()<3){ ecBebasDigits+=key; tampilkanInputECBebas(); }
    return;
  }
  if(key=='D'){ ecBebasDigits=""; tampilkanInputECBebas(); return; }
  if(key=='*'){
    ecBebasDigits="";
    if(ecBebasTujuan==1){ currentState=KALK_INPUT_MENU; lcd.clear(); tampilkanKalkInputMenu(); }
    else { currentState=MENU_JENIS_LARUTAN; lcd.clear(); tampilkanMenuJenisLarutan(); }
    return;
  }
  if(key=='B'){
    if(ecBebasDigits.length()!=3){ tampilkanError(F("Wajib 3 digit")); tampilkanInputECBebas(); return; }
    float nilai = ecBebasDigits.toFloat()/100.0f;
    if(nilai<EC_BEBAS_MIN || nilai>EC_BEBAS_MAKS){
      tampilkanError(F("Batas: 0.20-4.00"));
      ecBebasDigits=""; tampilkanInputECBebas();
      return;
    }
    char buf[8]; dtostrf(nilai,4,2,buf);
    if(ecBebasTujuan==1){ kalkEC=String(buf); ecBebasDigits=""; kalkSelDil=0; currentState=KALK_PPM_DIL; lcd.clear(); tampilkanKalkMenuDil(); return; }
    ecTunda = String(buf);   // jangan commit dulu: EC, pengenceran & slope harus berubah bersamaan
    ecBebasDigits="";
    currentState=MENU_PENGENCERAN; selDil=0;
    { int d = cariIndexDil(tampilBaru() ? terpilihDilutionBaru : terpilihDilution); if(d>=0 && d<=1) selDil=d; }   // TES_127
    lcd.clear(); tampilkanMenuPengenceran();
  }
}

// ============================================================================
// ================  HUB PENGATURAN "ZN MANUAL Kp/Ki"  =======================
// ============================================================================
// Konteks Kp/Ki PI ZN mengikuti Parameter Dosing: saat jadwal menunggu & draft Parameter Baru ada -> profil BARU
// (tidak mengganggu jadwal yang sedang berjalan; dipakai saat Parameter Baru di-commit). Selain itu -> profil AKTIF.
void tampilkanMenuZNHub(){
  lcdPrintCenter(0, tampilBaru() ? F("Kp/Ki PI ZN (Baru)") : F("Kp/Ki PI ZN"));
  lcdPrint20(0,1, String(selZNHub==0?"> ":"  ")+String(znHubOptions[0]));
  lcdPrint20(0,2, String(selZNHub==1?"> ":"  ")+String(znHubOptions[1]));   // TES_145: Rekaman Terakhir
  lcdPrint20(0,3, String(selZNHub==2?"> ":"  ")+(tampilBaru() ? String(F("Lihat Kp/Ki Baru")) : String(znHubOptions[2])));   // TES_127
}

// TES_145: layar 'Rekaman Terakhir'. AB Mix: judul baris 0, data baris 1-3. pH Down: judul baris 0, fase baris 1, data baris 2-3.
// Tiap baris data: waktu (detik sejak awal pulsa) - nilai; baris pertama '0s' = nilai awal. Penanda ^ / v di kolom 19 = masih ada data di atas / bawah.
void tampilkanRekamanZN(){
  if(znRekKanal < 0 || znRekN[znRekKanal] == 0){
    lcdPrintCenter(0, F("Rekaman Terakhir"));
    lcdPrintCenter(1, F("Belum ada rekaman"));
    lcdPrintCenter(2, F("Jalankan Uji Reaksi"));
    lcdPrint20(0,3,F(""));
    return;
  }
  int k = znRekKanal;
  int baris0 = (k==0) ? 1 : 2;     // baris LCD untuk data pertama
  int vis = 4 - baris0;            // jumlah baris data: 3 (AB Mix) / 2 (pH)
  int n = znRekN[k];
  if(k==0) lcdPrintCenter(0, F("Korelasi AB Mix:"));
  else {
    lcdPrintCenter(0, F("Korelasi pH Down"));
    lcdPrintCenter(1, (k==1) ? F("Fase Atas:") : F("Fase Bawah:"));
  }
  if(znRekScroll + vis > n) znRekScroll = (n > vis) ? (n - vis) : 0;
  if(znRekScroll < 0) znRekScroll = 0;
  for(int r=0; r<vis; r++){
    int i = znRekScroll + r;
    char b[21], f[12], ln[21];
    b[0] = 0;
    if(i < n){
      float v = (float)znRekNilai[k][i] / ((k==0) ? 10.0f : 1000.0f);
      fmtF(f, v, (k==0) ? 1 : 3);
      snprintf_P(b, sizeof(b), (k==0) ? PSTR("%5lus - %s PPM") : PSTR("%5lus - %s pH"), (unsigned long)znRekWaktu[k][i], f);
    }
    int L = (int)strlen(b); if(L > 19) L = 19;
    for(int c=0; c<20; c++) ln[c] = (c < L) ? b[c] : ' ';
    ln[20] = 0;
    if(r==0 && znRekScroll > 0) ln[19] = '^';
    else if(r==vis-1 && znRekScroll + vis < n) ln[19] = 'v';
    lcd.setCursor(0, baris0 + r); lcd.print(ln);
  }
}

// -- Signature konfigurasi (poin 1), Aktif atau Baru --
SignatureKpKi ambilSignatureKonteks(bool baru){
  SignatureKpKi s;
  s.volume         = baru ? terpilihVolumeBaru : terpilihVolume;
  s.jenisAsam      = ((baru ? terpilihAcidBaru : terpilihAcid)=="Fosfat") ? 1 : 0;
  s.konsentrasiAsam= baru ? terpilihConcPersenBaru : terpilihConcPersen;
  s.jenisABMixDil  = ((baru ? terpilihDilutionBaru : terpilihDilution)=="10 ml/L") ? 1 : 0;
  s.targetEC       = (baru ? terpilihECBaru : terpilihEC).toFloat();
  int* deb = baru ? terpilihDebitPumpBaru : terpilihDebitPump;
  s.debitP1        = ((float)deb[3]/DEBIT_SKALA) * (1000.0f/60.0f);            // mL/detik, pompa pH
  s.debitP2        = ((float)(deb[4]+deb[5])/DEBIT_SKALA) * (1000.0f/60.0f);   // mL/detik, pompa AB Mix
  return s;
}
SignatureKpKi ambilSignatureAktif(){ return ambilSignatureKonteks(false); }
ProfilKpKiZN& profilZNKonteks(bool baru){ return baru ? profilZNBaru : profilZNAktif; }

// PI yang dipakai proses dosing mode CL PI ZN = profil AKTIF
void muatPIZNDariProfilAktif(){
  piPPM_ZN.kp=profilZNAktif.kpPPM;  piPPM_ZN.ki=profilZNAktif.kiPPM;  piPPM_ZN.integral=0.0f;
  piPHKpAtas_ZN=profilZNAktif.kpPHAtas;   piPHKiAtas_ZN=profilZNAktif.kiPHAtas;
  piPHKpBawah_ZN=profilZNAktif.kpPHBawah; piPHKiBawah_ZN=profilZNAktif.kiPHBawah;
}

// Simpan Kp/Ki satu kanal (0=PPM, 1=pH Atas, 2=pH Bawah) ke profil konteks. Bila konfigurasi (signature) berbeda
// dari profil tersimpan, kanal lain berasal dari konfigurasi lama -> TES_144: dipertahankan & ditandai 'uji lama'.
// TES_141: volRef = volume tempat Kp/Ki berlaku (volume tandon uji), ppmRef = PPM tandon saat uji (kanal pH).
// Konfigurasi selain volume sama dgn profil tersimpan -> kanal lain DIPERTAHANKAN; bila volume acuan berbeda, kanal
// lain diskalakan ke volume acuan baru (Kp ~ V) sehingga 1 profil = 1 volume acuan.
void simpanKanalZNRef(bool baru, int kanal, float kp, float ki, float volRef, int ppmRef){
  ProfilKpKiZN &p = profilZNKonteks(baru);
  int16_t* pr = znPPMUji[baru?1:0];
  SignatureKpKi sig = ambilSignatureKonteks(baru);
  sig.volume = volRef;
  uint8_t &lama = znKanalLama[baru?1:0];
  if(!p.terisi){
    p.kpPPM=0; p.kiPPM=0; p.kpPHAtas=0; p.kiPHAtas=0; p.kpPHBawah=0; p.kiPHBawah=0;
    pr[0]=0; pr[1]=0; lama=0;
  } else {
    // TES_144: konfigurasi (selain volume) beda -> kanal lain DIPERTAHANKAN & ditandai 'uji lama' (dulu dihapus: jadwal
    // yang sedang berjalan langsung memakai Kp = 0 utk kanal itu). Peringatan KONFIG LAMA tampil sampai kanal diuji ulang.
    if(!signatureSama(p.sig, sig)){
      if(p.kpPPM>0.0f)     lama |= 1;
      if(p.kpPHAtas>0.0f)  lama |= 2;
      if(p.kpPHBawah>0.0f) lama |= 4;
    }
    if(p.sig.volume > 0.5f && volRef > 0.5f && fabs(p.sig.volume - volRef) >= EPS_VOL_ZN){
      float f = volRef / p.sig.volume;
      p.kpPPM*=f; p.kiPPM*=f; p.kpPHAtas*=f; p.kiPHAtas*=f; p.kpPHBawah*=f; p.kiPHBawah*=f;
    }
  }
  p.terisi = true; p.sig = sig;
  lama &= (uint8_t)~(1 << kanal);   // kanal ini kini sesuai konfigurasi sekarang
  if(kanal==0){ p.kpPPM=kp; p.kiPPM=ki; }
  else if(kanal==1){ p.kpPHAtas=kp; p.kiPHAtas=ki; pr[0]=(int16_t)ppmRef; }
  else { p.kpPHBawah=kp; p.kiPHBawah=ki; pr[1]=(int16_t)ppmRef; }
  if(!baru) muatPIZNDariProfilAktif();
  eepromAutoSimpan();
}
// Nilai utk Parameter konteks apa adanya (edit manual): acuan = volume tandon & PPM target konteks
void simpanKanalZN(bool baru, int kanal, float kp, float ki){
  simpanKanalZNRef(baru, kanal, kp, ki, (float)(baru ? terpilihVolumeBaru : terpilihVolume), baru ? terpilihPPMBaru : terpilihPPM);
}

// Kp/Ki ZN dianggap lengkap bila kanal yang dipakai proses sudah terisi: PPM & pH Atas selalu; pH Bawah bila target pH < 5.85.
// (Kanal berisi 0 = PI kanal itu langsung 'selesai' tanpa dosis -> koreksi diam-diam tidak jalan.)
bool profilZNLengkap(const ProfilKpKiZN &p, float phTarget){
  if(!p.terisi) return false;
  if(!(p.kpPPM>0.0f) || !(p.kpPHAtas>0.0f)) return false;   // TES_148: NaN = tidak lengkap
  if(phTarget < batasFasePH && !(p.kpPHBawah>0.0f)) return false;
  return true;
}
void tampilkanZNBelumLengkap(){
  lcd.clear();
  lcdPrintCenter(0, F("Kp/Ki ZN belum"));
  lcdPrintCenter(1, F("lengkap (PPM, pH"));
  lcdPrintCenter(2, F("Atas; pH Bawah bila"));
  lcdPrintCenter(3, F("target pH < 5.85"));
  jedaInfo(3000);
}

// -- Poin 4 & 6: pengecekan blocking sebelum mulai dosing (Parameter Aktif) --
// TES_141: volume beda = diskalakan otomatis; konfigurasi lain beda = peringatan "KONFIG LAMA" (B=Lanjut) di tahap
// peringatan Mulai Dosing (tahapPeringatanPerlu 0). Yang tetap menolak: belum ada / kanal yg dipakai belum terisi.
bool validasiKpKiZNSebelumDosing(){
  if(dosingMode!=4) return true;
  if(!profilZNAktif.terisi){
    lcd.clear();
    lcdPrintCenter(0, F("Atur dahulu"));
    lcdPrintCenter(1, F("Kp dan Ki"));
    lcdPrintCenter(2, F("di pengaturan"));
    lcdPrintCenter(3, F(""));
    jedaInfo(2500);
    return false;
  }
  if(!profilZNLengkap(profilZNAktif, terpilihPH)){ tampilkanZNBelumLengkap(); return false; }
  muatPIZNDariProfilAktif();
  return true;
}

// -- Validasi Kp/Ki ZN terhadap draft Parameter BARU (dipanggil SEBELUM salinParameterBaruKeAktif()) --
// Profil Baru dibuat lewat Pengaturan Sistem > Kp/Ki PI ZN saat draft Baru ada. Bila gagal: draft Baru & jadwal
// tunggu yang sedang berjalan TIDAK disentuh sama sekali.
bool validasiKpKiZNParameterBaru(){
  if(dosingModeBaru!=4) return true;      // hanya mode CL PI ZN yg butuh profil Kp/Ki
  if(profilZNBaru.terisi){   // TES_141: konfigurasi lain beda -> peringatan (bisa lanjut) di tahap peringatan
    if(profilZNLengkap(profilZNBaru, terpilihPHBaru)) return true;
    tampilkanZNBelumLengkap();   // draft Baru & jadwal tunggu dibiarkan utuh
    return false;
  }
  lcd.clear();
  lcdPrintCenter(0, F("Kp/Ki ZN belum ada"));
  lcdPrintCenter(1, F("Untuk Parameter Baru"));
  lcdPrintCenter(2, F("Uji/isi Kp/Ki PI ZN"));
  lcdPrintCenter(3, F("(Pengaturan Sistem)"));
  jedaInfo(3500);
  return false;
}

// -- Poin 3: tampilan Kp/Ki, 3 kategori bisa digeser --
void tampilkanKpKiZN(){
  // AVR snprintf tidak mendukung %f -> angka diformat dgn dtostrf (fmtKp).
  // Bila nilai tersimpan melewati batas aman model (mis. PPM target/debit berubah setelah uji), nilai EFEKTIF
  // yang benar-benar dipakai saat dosing ditampilkan di samping ("ef").
  char judul[21],baris1[21],baris2[21],a[12],b[12];
  bool baru = tampilBaru();   // Baru bila draft Parameter Baru ada
  const ProfilKpKiZN &p = profilZNKonteks(baru);
  float kp = (halKpKiZN==0) ? p.kpPPM : (halKpKiZN==1 ? p.kpPHAtas : p.kpPHBawah);
  float ki = (halKpKiZN==0) ? p.kiPPM : (halKpKiZN==1 ? p.kiPHAtas : p.kiPHBawah);
  float kpEf, kiEf; kpKiZNEfektif(halKpKiZN, baru, kpEf, kiEf);
  if(halKpKiZN==0)      snprintf_P(judul,sizeof(judul),PSTR("Kp/Ki PPM (AB Mix)"));
  else if(halKpKiZN==1) snprintf_P(judul,sizeof(judul),PSTR("Kp/Ki pH Fase Atas"));
  else                  snprintf_P(judul,sizeof(judul),PSTR("Kp/Ki pH Fase Bawah"));
  // TES_138: status profil di judul (dulu nilai lama tampil seolah valid walau konfigurasi sudah berubah)
  byte st = statusProfilZN(baru);
  // TES_144: TDK COCOK per kanal: konfigurasi profil beda (semua kanal) atau kanal ini hasil uji konfigurasi lama
  bool tdk = (st==2) && (!signatureSama(p.sig, ambilSignatureKonteks(baru)) || ((znKanalLama[baru?1:0] >> halKpKiZN) & 1));
  if(tdk || kp<=0.0f){
    if(halKpKiZN==0)      snprintf_P(judul,sizeof(judul),tdk?PSTR("PPM: TDK COCOK"):PSTR("PPM: BELUM ADA"));
    else if(halKpKiZN==1) snprintf_P(judul,sizeof(judul),tdk?PSTR("pH Atas: TDK COCOK"):PSTR("pH Atas: BELUM ADA"));
    else                  snprintf_P(judul,sizeof(judul),tdk?PSTR("pH Bawah: TDK COCOK"):PSTR("pH Bawah: BELUM ADA"));
  }
  if(kp>0.0f && (fabs(kpEf-kp) > kp*0.001f || fabs(kiEf-ki) > ki*0.001f+1e-9f)){   // TES_141: skala volume/PPM atau batas aman
    fmtDuaKp(baris1,"Kp",kp,"ef",kpEf);
    fmtDuaKp(baris2,"Ki",ki,"ef",kiEf);
  } else {
    snprintf_P(baris1,sizeof(baris1),PSTR("Kp = %s"),fmtKp(a,kp));
    snprintf_P(baris2,sizeof(baris2),PSTR("Ki = %s"),fmtKp(a,ki));
  }
  lcdPrint20(0,0,judul); lcdPrint20(0,1,baris1); lcdPrint20(0,2,baris2);
  if(halKpKiZN==0)      lcdPrint20(0,3,F(">>>   OK=EDIT"));
  else if(halKpKiZN==2) lcdPrint20(0,3,F("<<<   OK=EDIT"));
  else                  lcdPrint20(0,3,F("<<< >>>  OK=EDIT"));
}

// TES_141: Ki maks edit manual = min(0.6 Kp, 0.25.g.Kp^2) pada gain model Parameter konteks
float kiMaksEdit(int kanal, float kp){
  float m = 0.6f*kp;
  float g = hitungGainModel(kanal, tampilBaru());
  if(g>0.0f){ float r = ZN_RASIO_KI_AKAR_RIIL*g*kp*kp; if(r<m) m=r; }
  return m;
}
void tampilkanEditKpKiZN(){
  lcdPrintCenter(0, znEditTahap==0?F("Input Kp:"):F("Input Ki:"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_"));
  lcdPrintCenter(3,F("<-   OK"));
}
void prosesEditKpKiZN(char key){
  if(key>='0'&&key<='9'){ if(inputBuffer.length()<9){inputBuffer+=key; tampilkanEditKpKiZN();} return; }   // TES_138: 9 karakter (mis. 0.000425)
  if(key=='#'){ if(inputBuffer.indexOf('.')<0 && inputBuffer.length()>0){inputBuffer+='.'; tampilkanEditKpKiZN();} return; }
  if(key=='D'){ inputBuffer=""; tampilkanEditKpKiZN(); return; }
  if(key=='*'){ inputBuffer=""; currentState=MENU_LIHAT_KPKI_ZN; lcd.clear(); tampilkanKpKiZN(); return; }
  if(key=='B'){
    if(inputBuffer.length()==0) return;
    float nilai = inputBuffer.toFloat();
    if(znEditTahap==0){
      // Kp harus > 0 dan <= batas aman kanal ini (sama dgn cap pada hasil Uji Reaksi: FRAKSI_AMAN_ZN / gain model)
      float kpMaks = kpMaksModel(halKpKiZN, tampilBaru());   // pemetaan kanal sama: 0=PPM, 1=pH Atas, 2=pH Bawah
      if(kpMaks<=0.0f){   // TES_138: gain model tak valid (parameter belum lengkap) -> batas aman tak bisa dihitung
        lcd.clear();
        lcdPrintCenter(0,F("Parameter belum"));
        lcdPrintCenter(1,F("lengkap: batas aman"));
        lcdPrintCenter(2,F("Kp tak bisa dihitung"));
        jedaInfo(3000);
        inputBuffer=""; currentState=MENU_LIHAT_KPKI_ZN; lcd.clear(); tampilkanKpKiZN();
        return;
      }
      // TES_138: batas bawah 1% (dulu 10%: rasionalnya 'dosis < deadband' tak berlaku lagi sejak pulsa min 1 dtk;
      // Kp hasil uji reaksi < 10% batas aman tak bisa dikonfirmasi ulang). 1% = penjaga salah ketik.
      float kpMin  = 0.01f*kpMaks;
      if(nilai<=0.0f || nilai>kpMaks*1.002f || nilai<kpMin*0.998f){   // toleransi pembulatan 0,2%
        lcd.clear();
        lcdPrintCenter(0,F("Kp di luar batas"));
        char ka[12], kb[12]; fmtKp(ka,kpMin*1.006f); fmtKp(kb,kpMaks*0.994f);   // dibulatkan ke DALAM rentang
        char bm[21]; snprintf(bm,sizeof(bm),"%s - %s",ka,kb);
        lcdPrintCenter(1,bm);
        lcdPrintCenter(2,F("(1-100% batas aman"));
        lcdPrintCenter(3,F("anti overshoot)"));
        jedaInfo(3000);
        inputBuffer=""; lcd.clear(); tampilkanEditKpKiZN();
        return;
      }
      znEditKpSementara = nilai;
      inputBuffer=""; znEditTahap=1;
      { const ProfilKpKiZN &pz = profilZNKonteks(tampilBaru());
        float kpE, ki; kpKiZNEfektif(halKpKiZN, tampilBaru(), kpE, ki);   // TES_141: nilai efektif
        float kiMaks = kiMaksEdit(halKpKiZN, nilai);
        if(ki > kiMaks) ki = kiMaks;
        char t[12]=""; if(pz.terisi && ki>0.0f) fmtIsiKp(t, ki);
        isiAwalLangsung(t); }   // TES_127: Ki diisi nilai yang sedang dipakai
      tampilkanEditKpKiZN();
    } else {
      // Ki boleh 0 (= kontrol P murni). Batas atas: Ki <= 0.6 Kp (rumus ZN PI: Ki/Kp = 0.3/D, dan D >= 0.5 dtk) dan
      // TES_141: g.Ki <= (g.Kp)^2/4 (akar karakteristik PI per iterasi riil -> tanpa osilasi/overshoot)
      float kiMaks = kiMaksEdit(halKpKiZN, znEditKpSementara);
      if(nilai > kiMaks*1.002f){   // toleransi 0,2%: pembulatan tampilan isi awal
        lcd.clear();
        lcdPrintCenter(0,F("Ki di luar batas"));
        char kb[12]; fmtKp(kb,kiMaks*0.994f);   // TES_138: dibulatkan ke dalam rentang
        char bm[21]; snprintf(bm,sizeof(bm),"Rentang 0 - %s",kb);
        lcdPrintCenter(1,bm);
        lcdPrintCenter(2,F("(Ki maks: tanpa"));
        lcdPrintCenter(3,F("osilasi/overshoot)"));
        jedaInfo(3000);
        inputBuffer=""; lcd.clear(); tampilkanEditKpKiZN();
        return;
      }
      // simpan Kp+Ki ke kanal halaman yang sedang aktif (halKpKiZN), pada profil konteks (Aktif/Baru)
      inputBuffer=""; znEditTahap=0;
      znTundaDariUji = false;
      { bool bru = tampilBaru();   // TES_141: nilai yg diketik berlaku utk volume tandon & PPM target Parameter konteks
        mintaSimpanKanalZN(halKpKiZN, znEditKpSementara, nilai, MENU_LIHAT_KPKI_ZN,
                           (float)(bru ? terpilihVolumeBaru : terpilihVolume), bru ? terpilihPPMBaru : terpilihPPM); }
    }
  }
}

// ============================================================================
// ==========  UJI STEP-RESPONSE (METODE B: 28.3% / 63.2%)  ==================
// ============================================================================

void tampilkanUjiZNPilihKanal(){
  lcdPrintCenter(0, tampilBaru() ? F("Uji kanal (P. Baru)") : F("Uji kanal mana?"));
  for(int i=0;i<3;i++) lcdPrint20(0,i+1, String(ujiKanalDipilih==i?"> ":"  ")+String(ujiKanalOptions[i]));
}

// Baca 1 nilai sensor sesuai kanal uji yang dipilih (0=PPM/TDS, 1&2=pH)
// Bacaan sensor uji reaksi = rumus yang SAMA dengan proses dosing (kalibrasi bila aktif, rumus bawaan bila belum,
// kompensasi suhu), median 3 bacaan (masing-masing rata-rata 10 sampel ADC) agar tahan noise.
float bacaSensorUjiSekali(){
  if(ujiKanalDipilih==0){
    float cv = bacaCVTDS();   // sudah terkompensasi suhu
    return tdsKalibrasiAktif ? interpolasiTDS(cv) : ((133.42f*cv*cv*cv-255.86f*cv*cv+857.39f*cv)*0.5f);
  } else {
    float v = bacaVoltPH();
    float ph = phKalibrasi3TitikAktif ? interpolasiPH(v) : (phSlope*v+phOffset);
    ph = terapkanKompensasiSuhuPH(ph);
    if(!(ph >= PH_VALID_MIN && ph <= PH_VALID_MAKS)) phSensorValid = false;   // TES_148
    return constrain(ph, 0.0f, 14.0f);
  }
}
float bacaSensorTerpilihRataRata(){
  float a=bacaSensorUjiSekali(), b=bacaSensorUjiSekali(), c=bacaSensorUjiSekali();
  if(a>b){ float t=a; a=b; b=t; }
  if(b>c){ float t=b; b=c; c=t; }
  if(a>b){ float t=a; a=b; b=t; }
  return b;   // median
}

// TES_138: pompa pulsa uji NON-BLOCKING (dulu jedaAman = tombol & jadwal terkunci selama pulsa; kini pulsa s/d 600 dtk).
// Nyala/mati pompa peristaltik sesuai kanal uji; durasi diawasi tickUjiStepResponse().
void setelPompaUji(bool nyala){
  if(ujiKanalDipilih==0){ if(nyala){ relayOn(pumpRelayPin[4]); relayOn(pumpRelayPin[5]); } else { relayOff(pumpRelayPin[4]); relayOff(pumpRelayPin[5]); } }  // P2 & P3, kanal AB Mix
  else                  { if(nyala) relayOn(pumpRelayPin[3]); else relayOff(pumpRelayPin[3]); }                                                          // P1, kanal pH
  ujiPulsaAktif = nyala;
}
void hentikanUjiReaksi(){   // batal / keluar logging: semua aktuator uji mati
  ujiTdsUntukPH = false;
  if(ujiPulsaAktif) setelPompaUji(false);
  if(ujiD1Aktif){ relayOff(PIN_RELAY_D1); ujiD1Aktif=false; }
}

// Gain model acuan (dipakai HANYA sbg batas cap faktor aman, bukan pengganti uji fisik).
// kanal: 0 = PPM, 1 = pH fase atas, 2 = pH fase bawah. baru = pakai Parameter Baru (konteks ZN Baru).
float hitungGainModelPPM(int kanal, bool baru, int ppmAcuan);
float hitungGainModel(int kanal, bool baru){ return hitungGainModelPPM(kanal, baru, baru ? terpilihPPMBaru : terpilihPPM); }
float hitungGainModelPPMVol(int kanal, bool baru, int ppmAcuan, float vol);
float hitungGainModelPPM(int kanal, bool baru, int ppmAcuan){
  return hitungGainModelPPMVol(kanal, baru, ppmAcuan, (float)(baru ? terpilihVolumeBaru : terpilihVolume));
}
// TES_141: gain model pada volume tandon UJI (pulsa uji, batas aman hasil uji)
float gainUji(int kanal, int ppmAcuan){ return hitungGainModelPPMVol(kanal, tampilBaru(), ppmAcuan, (float)ujiVolumeL); }
// TES_139: ppmAcuan = PPM tempat slope pH Down diambil (dosing & batas aman: PPM target; pulsa uji pH: PPM terukur)
// TES_141: vol eksplisit (Parameter: volume tandon; Uji Reaksi: volume tandon uji)
float hitungGainModelPPMVol(int kanal, bool baru, int ppmAcuan, float vol){
  int*  deb    = baru ? terpilihDebitPumpBaru : terpilihDebitPump;
  float slopeAB= baru ? slopeABMixTerpilihBaru : slopeABMixTerpilih;
  int   conc   = baru ? terpilihConcPersenBaru : terpilihConcPersen;
  int   ppm    = ppmAcuan;
  bool  fosfat = ((baru ? terpilihAcidBaru : terpilihAcid)=="Fosfat");
  if(kanal==0){
    // per SISI (A = B): makro memberi tiap pompa volume penuh, jadi regresi slope = PPM per mL/L per sisi (bukan jumlah A+B)
    float debitAB = ((float)(deb[4]+deb[5])/(2.0f*DEBIT_SKALA)) * (1000.0f/60.0f);
    if(debitAB<=0.0f || slopeAB<=0.0f || vol<=0.0f) return 0.0f;   // TES_138: 0 = tak valid (dulu 0.0762 -> uji reaksi tanpa parameter memompa 600 dtk)
    return debitAB * slopeAB / vol;
  } else {
    float debitPH = ((float)deb[3]/DEBIT_SKALA) * (1000.0f/60.0f);
    float skalaVolume = (conc>0) ? ((vol/10.0f) * (10.0f/(float)conc)) : 0.0f;   // sama dgn hitungPHDownVolume
    bool faseAtas = (kanal==1);
    float slope;
    if(fosfat) slope = faseAtas ? interpSlopePH(tabelSlopeAtasFosfat,(float)ppm) : interpSlopePH(tabelSlopeBawahFosfat,(float)ppm);
    else       slope = faseAtas ? interpSlopePH(tabelSlopeAtas,(float)ppm)       : interpSlopePH(tabelSlopeBawah,(float)ppm);
    if(debitPH<=0.0f || skalaVolume<=0.0f || slope<=0.0f) return 0.0f;   // TES_138: 0 = tak valid
    return debitPH * slope / skalaVolume;
  }
}

// TES_141: PPM acuan uji: kanal pH = PPM tandon terukur saat bilas (bila valid), selain itu PPM target konteks
int ujiPPMAcuan(){
  if(ujiKanalDipilih!=0 && ujiPPMTerukur > 0.0f) return (int)(ujiPPMTerukur+0.5f);
  return tampilBaru() ? terpilihPPMBaru : terpilihPPM;
}
void tampilkanUjiZNInputVolume(){
  char b[21];
  lcdPrintCenter(0, F("Volume tandon uji"));
  snprintf_P(b,sizeof(b),PSTR("(%d-%d L):"),UJI_VOL_MIN,UJI_VOL_MAKS); lcdPrintCenter(1, b);
  lcdPrintCenter(2, (inputBuffer.length()>0?inputBuffer:String(F("_")))+F(" L"));
  int v = tampilBaru() ? terpilihVolumeBaru : terpilihVolume;
  if(v>0) snprintf_P(b,sizeof(b),PSTR("Parameter: %d L"),v); else snprintf_P(b,sizeof(b),PSTR("Param.: -"));
  lcdPrintCenter(3, b);
}
void tampilkanUjiZNInputDurasi(){
  char b[21];
  lcdPrintCenter(0, F("Lama rekam maksimum"));
  snprintf_P(b,sizeof(b),PSTR("(%d-%d menit):"),ujiRekamMinMenit,UJI_REKAM_MAKS_MENIT); lcdPrintCenter(1, b);
  lcdPrintCenter(2, (inputBuffer.length()>0?inputBuffer:String(F("_")))+F(" menit"));
  if(ujiPulsaMaksDetik > (int)ujiDurasiPulsa)   // TES_142: pulsa bisa diperbesar s/d respon maks bila sensor berderau
    snprintf_P(b,sizeof(b),PSTR("Pulsa %d-%d dtk"),(int)ujiDurasiPulsa,ujiPulsaMaksDetik);
  else snprintf_P(b,sizeof(b),PSTR("Pulsa uji %d detik"),(int)ujiDurasiPulsa);
  lcdPrintCenter(3, b);   // TES_138: lama pulsa diketahui sebelum mulai
}
// Lama bilas chamber sebelum nilai awal uji = Lama Bilas Sensor konteks (Aktif/Baru); tak valid -> 30 dtk
int ujiLamaBilasDetik(){
  int bilas = tampilBaru() ? terpilihLamaCekSensorBaru : terpilihLamaCekSensor;
  if(bilas < 10 || bilas > 900) bilas = 30;
  if(bilas < JEDA_STABIL_SENSOR_DETIK) bilas = JEDA_STABIL_SENSOR_DETIK;
  return bilas;
}
// TES_139: lama uji MAKSIMUM (bilas + lama rekam + 1 mnt cadangan) melewati jadwal cek harian berikutnya?
// Hanya saat jadwal menunggu & RTC valid. Jam cek yg sudah tiba (cek tertunda) = pasti bentrok.
bool ujiBentrokJadwalDetik(long perlu);
bool ujiBentrokJadwal(){ return ujiBentrokJadwalDetik((long)ujiLamaBilasDetik() + (long)UJI_DERAU_DETIK + (long)ujiDurasiMaksMenit*60L + 60L); }
// TES_144: perlu = detik yang masih dibutuhkan uji dari sekarang
bool ujiBentrokJadwalDetik(long perlu){
  if(!dosingSedangMenunggu || !rtcWaktuValid() || !waktuDosingFormatValid(terpilihWaktuDosing)) return false;
  DateTime n = rtc.now();
  int jamT = terpilihWaktuDosing.substring(0,2).toInt();
  int menT = terpilihWaktuDosing.substring(3,5).toInt();
  DateTime target(dosingTargetTahun, dosingTargetBulan, dosingTargetHari, jamT, menT, 0);
  long sisa  = (long)target.unixtime() - (long)n.unixtime();
  return sisa < perlu;
}
void tampilkanBentrokJadwalUji(){
  lcd.clear();
  lcdPrintCenter(0, F("WAKTU UJI REAKSI"));
  lcdPrintCenter(1, F("KEMUNGKINAN"));
  lcdPrintCenter(2, F("BENTROK DENGAN"));
  lcdPrintCenter(3, F("WAKTU DOSING"));
  jedaInfo(5000);   // 5 dtk (tombol apa pun mempercepat)
}
// TES_138: hitung pulsa uji & cek kelayakan SEBELUM pompa apa pun menyala. false = ditolak (layar info sudah tampil, kembali ke hub).
bool ujiSiapkanPulsa(){
  bool baru = tampilBaru();
  if(ujiVolumeL < UJI_VOL_MIN || ujiVolumeL > UJI_VOL_MAKS){   // TES_141: tanpa input volume uji (mis. pemanggilan langsung) -> volume tandon
    int v = baru ? terpilihVolumeBaru : terpilihVolume;
    ujiVolumeL = (v>=UJI_VOL_MIN && v<=UJI_VOL_MAKS) ? v : 0;
  }
  if(!gainModelValidUji(ujiKanalDipilih, baru)){   // tanpa EC / debit / asam gain model = 0 -> lama pulsa tak bisa dihitung aman
    lcd.clear();
    lcdPrintCenter(0, F("Parameter belum"));
    lcdPrintCenter(1, F("lengkap untuk uji:"));
    lcdPrintCenter(2, (ujiKanalDipilih==0) ? F("AB Mix, debit P2/P3") : F("Asam,PPM,debit P1"));
    lcdPrintCenter(3, F("Uji dibatalkan"));
    jedaInfo(3500);
    currentState = MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
    return false;
  }
  // Pulsa otomatis: respon target / gain model (detik), dibatasi 2-600 dtk (TES_138: dulu 30 dtk)
  float g = gainUji(ujiKanalDipilih, baru ? terpilihPPMBaru : terpilihPPM);   // TES_141: pada volume tandon uji
  float target = (ujiKanalDipilih==0) ? UJI_TARGET_RESPON_PPM : UJI_TARGET_RESPON_PH;
  ujiDurasiPulsa = target/g;
  if(ujiDurasiPulsa < UJI_PULSA_MIN_DETIK) ujiDurasiPulsa = UJI_PULSA_MIN_DETIK;
  if(ujiDurasiPulsa > UJI_PULSA_MAKS_DETIK) ujiDurasiPulsa = UJI_PULSA_MAKS_DETIK;
  ujiDurasiPulsa = (float)((int)(ujiDurasiPulsa+0.5f));   // pompa ber-resolusi 1 detik
  // Debit pompa terlalu kecil utk tandon -> pulsa maks pun respon < 2x ambang deteksi -> tolak
  float minR = (ujiKanalDipilih==0) ? UJI_RESPON_MIN_PPM : UJI_RESPON_MIN_PH;
  float prediksi = g*ujiDurasiPulsa;
  if(prediksi < 2.0f*minR){
    char b[21], f[12];
    lcd.clear();
    lcdPrintCenter(0, F("DEBIT POMPA KECIL"));
    if(ujiKanalDipilih==0) snprintf_P(b,sizeof(b),PSTR("Pulsa %ds = %s PPM"),(int)ujiDurasiPulsa,fmtF(f,prediksi,1));
    else                   snprintf_P(b,sizeof(b),PSTR("Pulsa %ds = %s pH"),(int)ujiDurasiPulsa,fmtF(f,prediksi,2));
    lcdPrintCenter(1, b);
    lcdPrintCenter(2, (ujiKanalDipilih==0) ? F("Minimal 10 PPM") : F("Minimal 0.10 pH"));
    lcdPrintCenter(3, F("Uji dibatalkan"));
    jedaInfo(3500);
    currentState = MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
    return false;
  }
  // TES_142: pulsa terpanjang yg mungkin (respon tuju maks, sensor berderau) -> dasar lama rekam minimum
  { float tMaks = ((ujiKanalDipilih==0) ? UJI_TARGET_RESPON_PPM_MAKS : UJI_TARGET_RESPON_PH_MAKS)/g;
    if(tMaks > UJI_PULSA_MAKS_DETIK) tMaks = UJI_PULSA_MAKS_DETIK;
    if(tMaks < ujiDurasiPulsa) tMaks = ujiDurasiPulsa;
    ujiPulsaMaksDetik = (int)(tMaks+0.5f); }
  // Lama rekam minimum = pulsa + 3 mnt (penilaian stabil baru dimulai 60 dtk sesudah pulsa & butuh jendela >= 60 dtk)
  ujiRekamMinMenit = (int)(((float)ujiPulsaMaksDetik + 180.0f + 59.0f)/60.0f);
  if(ujiRekamMinMenit < UJI_REKAM_MIN_MENIT) ujiRekamMinMenit = UJI_REKAM_MIN_MENIT;
  return true;
}
void mulaiUjiStepResponse(){
  if(!ujiSiapkanPulsa()) return;           // juga dipanggil dari 'D' (ulang) di layar hasil
  if(ujiDurasiMaksMenit < ujiRekamMinMenit){
    ujiDurasiMaksMenit = ujiRekamMinMenit;
    // TES_144: lama rekam dinaikkan ke minimum -> cek ulang bentrok jadwal (pompa belum menyala)
    if(ujiBentrokJadwal()){ currentState = MENU_ZN_HUB; tampilkanBentrokJadwalUji(); lcd.clear(); tampilkanMenuZNHub(); return; }
  }
  ujiIntervalMs = UJI_INTERVAL_AWAL_MS;
  // TES_138: chamber sensor dibilas D1 selama Lama Bilas Sensor (sama dgn cek dosing) SEBELUM nilai awal dibaca.
  // Dulu hanya 5 dtk -> pergantian sampel lama -> sampel tandon jatuh di menit pertama kurva (tempat D ditentukan).
  ujiBilasDetik = ujiLamaBilasDetik();
  // TES_139: kanal pH juga menyalakan TDS selama bilas -> PPM tandon terukur (slope pH Down bergantung PPM: di 700 PPM
  // ~1.4x lebih kuat dari di 1000 PPM). TDS dimatikan 6 dtk sebelum nilai awal pH (bebas interferensi, sama dgn cek dosing).
  nyalakanRelayTDS();
  ujiTdsUntukPH = (ujiKanalDipilih!=0); ujiPPMTerukur = -1.0f;
  if(ujiTdsUntukPH && ujiBilasDetik < 12) ujiBilasDetik = 12;
  relayOn(PIN_RELAY_D1); ujiD1Aktif = true;   // sirkulasikan sampel ke sensor selama uji (D1 mati lagi saat uji selesai/batal)
  ujiFase = 0; ujiBilasMulaiMs = millis();
  ujiJumlahSampel = 0; ujiStabilBerturut = 0;
  ujiRawN = 0; ujiRawIdx = 0; ujiStN = 0; ujiStIdx = 0; ujiStTerakhirT = 0.0f;
  ujiDerauSd = 0.0f; ujiResponTuju = 0.0f; ujiSubSum = 0.0f; ujiSubN = 0;   // TES_142
  ujiInvalidBerturut = 0;   // TES_144
  ujiDetikTampil = 0xFFFFFFFFUL;
  currentState = UJI_ZN_LOGGING;           // non-blocking: bilas -> nilai awal -> pulsa -> rekam (tickUjiStepResponse)
  lcd.clear();
  tampilkanUjiLogging();
}
// Akhir bilas: baca nilai awal, cek fase pH, lalu pulsa dimulai. false = uji dibatalkan (fase pH salah).
bool ujiMulaiPulsa(){
  // TES_142: ujiC0 & ujiDerauSd sudah dihitung di fase ukur derau (ujiHitungDerau)
  // Kanal pH: pH awal harus berada di fase yang diuji, kalau tidak kurva reaksi yg terekam milik fase lain
  // (slope pH Down fase atas & bawah berbeda jauh). Fase atas: >= 5.85 + 0.2 = 6.05 (cadangan agar respons tidak
  // langsung melintasi 5.85); fase bawah: < 5.85. Dicek SEBELUM pulsa -> tidak ada pH Down yang terbuang.
  if(ujiKanalDipilih!=0){
    bool fasaOk = (ujiKanalDipilih==1) ? (ujiC0 >= batasFasePH + 0.2f) : (ujiC0 < batasFasePH);
    if(!fasaOk){
      hentikanUjiReaksi();
      char b[21], f[12];
      lcd.clear();
      lcdPrintCenter(0, F("FASE pH TAK SESUAI"));
      snprintf_P(b,sizeof(b),PSTR("pH awal: %s"),fmtF(f,ujiC0,2)); lcdPrintCenter(1, b);
      lcdPrintCenter(2, (ujiKanalDipilih==1) ? F("Fase atas: pH>=6.05") : F("Fase bawah: pH<5.85"));
      lcdPrintCenter(3, F("Uji dibatalkan"));
      jedaInfo(3500);
      currentState = MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
      return false;
    }
  }
  // TES_142: respon tuju = maks(minimum, 25 x derau), maks 150 PPM / 0.60 pH. Kanal pH memakai slope pada PPM terukur.
  // TES_139: kanal pH fase atas -> penurunan dibatasi separuh jarak ke batas fase (5.85): asam 2x lebih kuat dari
  // persamaan pun berhenti di 5.85 (melewati = fase bawah, slope ~4x lebih curam; dulu pH bisa jatuh ke ~4.7).
  { bool kanalPH = (ujiKanalDipilih!=0);
    float minT = kanalPH ? UJI_TARGET_RESPON_PH : UJI_TARGET_RESPON_PPM;
    float maksT = kanalPH ? UJI_TARGET_RESPON_PH_MAKS : UJI_TARGET_RESPON_PPM_MAKS;
    float tuju = UJI_SNR_TUJU*ujiDerauSd;
    if(tuju < minT) tuju = minT;
    if(tuju > maksT) tuju = maksT;
    bool dibatasiFase = false;
    // fase atas: turun <= separuh jarak ke 5.85 (asam 2x lebih kuat pun tetap di fase atas)
    // TES_144: fase bawah: turun <= separuh jarak ke pH 3.5 (batas bawah data slope) - dulu tanpa batas
    float maksTurun = 1.0e9f;
    if(ujiKanalDipilih==1)      maksTurun = (ujiC0 - batasFasePH)/2.0f;
    else if(ujiKanalDipilih==2) maksTurun = (ujiC0 - UJI_PH_DATA_MIN)/2.0f;
    if(maksTurun < tuju){ tuju = maksTurun; maksT = maksTurun; dibatasiFase = true; }
    float g = gainUji(ujiKanalDipilih, ujiPPMAcuan());
    if(!(g > 0.0f)){   // TES_144: gain tak valid pada PPM terukur -> dulu batas fase dilewati & pulsa lama tetap jalan
      ujiBatalInfo(F("Parameter belum"), F("lengkap utk uji:"), kanalPH ? F("Asam,PPM,debit P1") : F("AB Mix, debit P2/P3"));
      return false;
    }
    {
      float pls = tuju/g;
      pls = dibatasiFase ? (float)((int)pls) : (float)((int)(pls+0.5f));   // fase atas dibulatkan ke bawah
      if(maksTurun < 1.0e8f && g*pls > maksTurun) pls = (float)((int)(maksTurun/g));   // TES_148: pembulatan ke atas tak boleh melewati batas fase
      if(pls < UJI_PULSA_MIN_DETIK) pls = UJI_PULSA_MIN_DETIK;
      if(pls > UJI_PULSA_MAKS_DETIK) pls = UJI_PULSA_MAKS_DETIK;
      ujiDurasiPulsa = pls;
      ujiResponTuju = g*pls;
      // TES_144: pulsa minimum 2 dtk sudah melewati batas fase / rentang data (tandon uji kecil, pH awal dekat 5.85)
      if(maksTurun < 1.0e8f && ujiResponTuju > maksTurun*1.001f){
        char b1[21], b2[21], f1[12], f2[12];
        snprintf_P(b1,sizeof(b1),PSTR("Pulsa min %d dtk:"),(int)UJI_PULSA_MIN_DETIK);
        snprintf_P(b2,sizeof(b2),PSTR("%s > maks %s pH"),fmtF(f1,ujiResponTuju,2),fmtF(f2,maksTurun>0.0f?maksTurun:0.0f,2));
        ujiBatalInfo(F("RESPON TERLALU BESAR"), b1, b2);
        return false;
      }
      float responMaks = g*UJI_PULSA_MAKS_DETIK; if(responMaks > maksT) responMaks = maksT;
      if(responMaks < ujiResponTuju) responMaks = ujiResponTuju;
      // derau terlalu besar dibanding respon yang boleh/dapat dibuat -> plateau tak terbedakan dari derau
      if(ujiDerauSd > 0.0f && responMaks < UJI_SNR_MIN*ujiDerauSd){
        hentikanUjiReaksi();
        char b[21], f[12];
        lcd.clear();
        lcdPrintCenter(0, F("SENSOR BERDERAU"));
        snprintf_P(b,sizeof(b),kanalPH?PSTR("Derau +/-%s pH"):PSTR("Derau +/-%s PPM"),fmtF(f,2.0f*ujiDerauSd,kanalPH?3:1)); lcdPrintCenter(1, b);
        snprintf_P(b,sizeof(b),kanalPH?PSTR("Respon maks %s pH"):PSTR("Respon maks %s PPM"),fmtF(f,responMaks,kanalPH?2:0)); lcdPrintCenter(2, b);
        lcdPrintCenter(3, F("Uji dibatalkan"));
        mulaiBuzzerPutus(3000UL);
        jedaInfo(4000);
        currentState = MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
        return false;
      }
      // TES_144: respon dgn gain pada PPM TERUKUR tetap >= 2x ambang deteksi (dulu hanya dicek pada PPM target)
      if(ujiResponTuju < 2.0f*ujiAmbangDeteksi()){
        char b1[21], b2[21], f1[12];
        snprintf_P(b1,sizeof(b1),kanalPH?PSTR("Pulsa %ds = %s pH"):PSTR("Pulsa %ds = %s PPM"),(int)ujiDurasiPulsa,fmtF(f1,ujiResponTuju,kanalPH?2:0));   // <= 20 karakter
        snprintf_P(b2,sizeof(b2),kanalPH?PSTR("Minimal %s pH"):PSTR("Minimal %s PPM"),fmtF(f1,2.0f*ujiAmbangDeteksi(),kanalPH?2:0));
        ujiBatalInfo(F("DEBIT POMPA KECIL"), b1, b2);
        return false;
      }
    }
    // lama rekam minimum ikut pulsa sebenarnya (pengaman; ujiRekamMinMenit sudah memakai pulsa maks)
    int mn = (int)((ujiDurasiPulsa + 180.0f + 59.0f)/60.0f);
    if(ujiDurasiMaksMenit < mn){
      ujiDurasiMaksMenit = (mn <= UJI_REKAM_MAKS_MENIT) ? mn : UJI_REKAM_MAKS_MENIT;
      // TES_144: lama rekam diperpanjang (pulsa pada PPM terukur lebih panjang) -> cek ulang bentrok jadwal
    }
    // TES_148: SELALU dicek tepat sebelum pompa uji menyala (mis. jam cek tiba saat bilas/ukur derau & ditunda B)
    if(ujiBentrokJadwalDetik((long)ujiDurasiMaksMenit*60L + 60L)){
      hentikanUjiReaksi(); currentState = MENU_ZN_HUB;
      if(!promptTibaAktif){ tampilkanBentrokJadwalUji(); lcd.clear(); tampilkanMenuZNHub(); }
      else ujiBatalTundaJudul = F("Bentrok jadwal cek");
      return false;
    }
  }
  ujiFase = 1;
  setelPompaUji(true);                     // TES_138: non-blocking, dimatikan tickUjiStepResponse() sesudah ujiDurasiPulsa
  ujiTimerAwal = millis();                 // t=0 = AWAL pulsa (saat 'langkah' input diberikan), sesuai kurva reaksi Ziegler-Nichols
  ujiTimerSampelTerakhir = ujiTimerAwal;   // TES_138: sampel juga direkam selama pulsa (pulsa panjang -> respon mulai sebelum pulsa selesai)
  ujiSubSum = 0.0f; ujiSubN = 0; ujiSubTerakhirMs = ujiTimerAwal;   // TES_142: rata-rata sub-sampel per interval
  ujiDetikTampil = 0xFFFFFFFFUL;
  lcdPrintCenter(0,F("Merekam s/d stabil"));
  { char b[21], f[12]; bool kanalPH = (ujiKanalDipilih!=0);   // sampai sampel pertama: respon yang dituju
    snprintf_P(b,sizeof(b),kanalPH?PSTR("Respon tuju %s pH"):PSTR("Respon tuju %s PPM"),fmtF(f,ujiResponTuju,kanalPH?2:0));
    lcdPrintCenter(2, b); }
  return true;
}
// ── TES_144: pembatalan uji dgn layar info (layar TIBA WAKTU DOSING terbuka -> tanpa info agar tombol tak termakan) ──
void ujiBatalInfo(const __FlashStringHelper* b0, const char* b1, const char* b2){
  hentikanUjiReaksi();
  currentState = MENU_ZN_HUB;
  if(promptTibaAktif){ ujiBatalTundaJudul = b0; return; }   // hub & info digambar saat prompt ditutup (TES_148)
  lcd.clear();
  lcdPrintCenter(0, b0); lcdPrintCenter(1, b1); lcdPrintCenter(2, b2); lcdPrintCenter(3, F("Uji dibatalkan"));
  mulaiBuzzerPutus(3000UL);
  jedaInfo(3500);
  lcd.clear(); tampilkanMenuZNHub();
}
void ujiBatalInfo(const __FlashStringHelper* b0, const __FlashStringHelper* b1, const __FlashStringHelper* b2){
  char t1[21], t2[21];
  strncpy_P(t1, (PGM_P)b1, 20); t1[20] = 0; strncpy_P(t2, (PGM_P)b2, 20); t2[20] = 0;
  ujiBatalInfo(b0, t1, t2);
}
// TES_144: sensor uji tidak valid (probe lepas / ADC di luar rentang) 3 sub-sampel berturut -> uji batal, pompa mati.
// Dulu bacaan tak valid ikut direkam: TDS lepas -> nilai awal ~0 & derau 0 -> pulsa tetap memompa s/d 250 PPM.
bool ujiSensorValid(){ return (ujiKanalDipilih==0) ? tdsSensorValid : phSensorValid; }
// true = sub-sampel boleh dipakai; uji dibatalkan bila 3 berturut tak valid (cek currentState sesudahnya)
bool ujiSubSampelValid(){
  if(ujiSensorValid()){ ujiInvalidBerturut = 0; return true; }
  if(++ujiInvalidBerturut >= 3){
    ujiInvalidBerturut = 0;
    ujiBatalInfo(F("SENSOR TAK VALID"), (ujiKanalDipilih==0) ? F("TDS: cek probe/kabel") : F("pH: cek probe/kabel"), F("Pompa uji dimatikan"));
  }
  return false;
}
// ── TES_142: fase UKUR DERAU (sesudah bilas, D1 tetap mengalir, pompa uji mati) ──
// Sub-sampel tiap 0,5 dtk dikelompokkan per blok 4 dtk (= sampel kurva pertama). Derau = sd rata-rata blok terhadap
// garis tren (tren pelan tak dihitung derau); nilai awal = garis tren pada akhir jendela.
void ujiMulaiUkurDerau(){
  ujiFase = 2;
  unsigned long now = millis();
  ujiDerauMulaiMs = now; ujiBlokMulaiMs = now; ujiSubTerakhirMs = now - UJI_SUB_MS;
  ujiSubSum = 0.0f; ujiSubN = 0;
  ujiRegN = 0; ujiRegSx = ujiRegSy = ujiRegSxx = ujiRegSxy = ujiRegSyy = 0.0f; ujiRegY0 = 0.0f; ujiRegXAkhir = 0.0f;
  ujiDerauSd = 0.0f;
  ujiDetikTampil = 0xFFFFFFFFUL;
  lcd.clear(); tampilkanUjiLogging();
}
float ujiSdRegresi(float &yAkhir){
  int n = ujiRegN;
  float mx = ujiRegSx/n, my = ujiRegSy/n;
  float sxx = ujiRegSxx - n*mx*mx, sxy = ujiRegSxy - n*mx*my, syy = ujiRegSyy - n*my*my;
  float b = (sxx > 0.0f) ? sxy/sxx : 0.0f;
  float res = syy - b*sxy; if(res < 0.0f) res = 0.0f;
  yAkhir = ujiRegY0 + my + b*(ujiRegXAkhir - mx);
  return (n > 2) ? sqrt(res/(float)(n-2)) : 0.0f;
}
void tickUkurDerau(unsigned long now){
  unsigned long dtk = (now-ujiDerauMulaiMs)/1000UL;
  if(dtk != ujiDetikTampil){ ujiDetikTampil = dtk; tampilkanUjiLoggingBaris1(); }
  if(now - ujiSubTerakhirMs >= UJI_SUB_MS){
    ujiSubTerakhirMs = now;
    float v = bacaSensorUjiSekali();
    if(ujiSubSampelValid()){ ujiSubSum += v; ujiSubN++; }
    else if(currentState != UJI_ZN_LOGGING) return;   // TES_144: dibatalkan (sensor tak valid)
  }
  if(now - ujiBlokMulaiMs >= UJI_BLOK_DERAU_MS && ujiSubN > 0){
    float m = ujiSubSum/(float)ujiSubN;
    float x = (float)(ujiBlokMulaiMs - ujiDerauMulaiMs + (now - ujiBlokMulaiMs)/2UL)/1000.0f;   // tengah blok (dtk)
    if(ujiRegN == 0) ujiRegY0 = m;
    float y = m - ujiRegY0;   // offset -> presisi float terjaga
    ujiRegN++; ujiRegSx += x; ujiRegSy += y; ujiRegSxx += x*x; ujiRegSxy += x*y; ujiRegSyy += y*y;
    ujiRegXAkhir = (float)(now - ujiDerauMulaiMs)/1000.0f;
    ujiBlokMulaiMs = now; ujiSubSum = 0.0f; ujiSubN = 0;
    if(ujiRegN >= 3){
      float ya; ujiDerauSd = ujiSdRegresi(ya);
      char b[21], f[12]; bool kanalPH = (ujiKanalDipilih!=0);
      snprintf_P(b,sizeof(b),kanalPH?PSTR("Derau +/-%s pH"):PSTR("Derau +/-%s PPM"),fmtF(f,2.0f*ujiDerauSd,kanalPH?3:1));
      lcdPrintCenter(2, b);
    }
  }
  if(!promptTibaAktif && now - ujiDerauMulaiMs >= (unsigned long)UJI_DERAU_DETIK*1000UL){   // TES_144: ditunda selama prompt TIBA
    if(ujiRegN >= 3){ float ya; ujiDerauSd = ujiSdRegresi(ya); ujiC0 = ya; }
    else { ujiDerauSd = 0.0f; ujiC0 = bacaSensorTerpilihRataRata(); }
    ujiSubSum = 0.0f; ujiSubN = 0;
    ujiMulaiPulsa();
  }
}
// Baris 2 layar rekam: hitung mundur pulsa / waktu rekam berjalan
void tampilkanUjiLoggingBaris1(){
  char b[21];
  if(ujiFase==0){ snprintf_P(b,sizeof(b),PSTR("Bilas %lu/%d dtk"),(millis()-ujiBilasMulaiMs)/1000UL,ujiBilasDetik); lcdPrintCenter(1,b); return; }
  if(ujiFase==2){ snprintf_P(b,sizeof(b),PSTR("Derau %lu/%d dtk"),(millis()-ujiDerauMulaiMs)/1000UL,UJI_DERAU_DETIK); lcdPrintCenter(1,b); return; }   // TES_142
  unsigned long dt = (millis()-ujiTimerAwal)/1000UL;
  if(ujiPulsaAktif) snprintf_P(b,sizeof(b),PSTR("Pulsa %lu/%d dtk"),dt,(int)ujiDurasiPulsa);
  else              snprintf_P(b,sizeof(b),PSTR("Rekam %lu/%d mnt"),dt/60UL,ujiDurasiMaksMenit);
  lcdPrintCenter(1,b);
}
// Layar rekam kurva (TES_135: juga dipakai gambar ulang sesudah layar TIBA WAKTU DOSING / LCD re-init)
void tampilkanUjiLogging(){
  lcdPrintCenter(0, (ujiFase==0) ? F("Bilas sensor") : (ujiFase==2) ? F("Ukur derau sensor") : F("Merekam s/d stabil"));
  tampilkanUjiLoggingBaris1();
  if(ujiJumlahSampel>0){
    char buf[21], fy[12]; int k=ujiJumlahSampel-1;
    snprintf_P(buf,sizeof(buf),PSTR("t=%lddt y=%s"),(long)ujiWaktu[k],fmtF(fy,ujiNilai[k],ujiKanalDipilih==0?1:3));
    lcdPrintCenter(2,buf);
  }
  lcdPrintCenter(3,F("<- = Batal"));
}

float cariWaktuPadaFraksi(float fraksiTarget,float deltaTotal){
  float targetY = fraksiTarget*deltaTotal;
  for(int i=0;i<ujiJumlahSampel;i++){
    if(ujiNilai[i]>=targetY){
      // i==0: segmen dari titik awal kurva (t=0 saat pulsa mulai, y=0) ke sampel pertama
      float y0=(i==0)?0.0f:ujiNilai[i-1], y1=ujiNilai[i];
      float t0=(i==0)?0.0f:ujiWaktu[i-1], t1=ujiWaktu[i];
      if(y1==y0) return t1;
      return t0+(targetY-y0)/(y1-y0)*(t1-t0);
    }
  }
  return ujiWaktu[ujiJumlahSampel-1];
}

void tampilkanHasilUjiZN(){
  // AVR snprintf tidak mendukung %f -> dtostrf. Baris 4 dulu (Kp,Ki + " [B]Simpan") melebihi 20 kolom.
  char b0[21],b1[21],b2[21],x[12],y[12];
  snprintf_P(b0,sizeof(b0),PSTR("tau=%s D=%s"),fmtF(x,hasilUjiTau,hasilUjiTau<1000.0f?1:0),fmtF(y,hasilUjiD,hasilUjiD<1000.0f?1:0));
  fmtDuaKp(b1,"Kp",hasilUjiKp,"Ki",hasilUjiKi);
  snprintf_P(b2,sizeof(b2),PSTR("K=%s OK=Simpan"),fmtF(x,hasilUjiK,4));
  char j[21]; snprintf_P(j,sizeof(j),PSTR("Hasil Uji (%d L):"),ujiVolumeL);   // TES_141: nilai utk tandon uji
  lcdPrint20(0,0,j);
  lcdPrint20(0,1,b0); lcdPrint20(0,2,b1); lcdPrint20(0,3,b2);
}

// TES_145: salin rekaman kurva uji (waktu + nilai ABSOLUT) ke penyimpanan 'Rekaman Terakhir' kanal yang diuji.
// Dipanggil tiap rekaman SELESAI (stabil / waktu habis), termasuk yang gagal dinilai, agar bisa dianalisis.
void simpanRekamanZN(){
  int k = ujiKanalDipilih; if(k < 0 || k > 2) return;
  int n = ujiJumlahSampel; if(n > UJI_MAKS_SAMPEL) n = UJI_MAKS_SAMPEL;
  if(n < 1) return;
  float skala = (k==0) ? 10.0f : 1000.0f;
  // titik ke-0: nilai awal (t = 0 dtk)
  float v0 = ujiC0 * skala;
  if(v0 > 32767.0f) v0 = 32767.0f; if(v0 < -32768.0f) v0 = -32768.0f;
  znRekWaktu[k][0] = 0;
  znRekNilai[k][0] = (int16_t)((v0 >= 0.0f) ? (v0 + 0.5f) : (v0 - 0.5f));
  for(int i=0; i<n; i++){
    float t = ujiWaktu[i]; if(t < 0.0f) t = 0.0f; if(t > 65535.0f) t = 65535.0f;
    znRekWaktu[k][i+1] = (uint16_t)(t + 0.5f);
    float v = (ujiC0 + ujiNilai[i]) * skala;
    if(v > 32767.0f) v = 32767.0f; if(v < -32768.0f) v = -32768.0f;
    znRekNilai[k][i+1] = (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
  }
  znRekN[k] = (byte)(n + 1);
  eepromSimpanZNRek(k);   // TES_145: uji terakhir kanal ini menimpa rekaman lama (RAM + EEPROM)
}
void selesaiUjiStepResponse(bool stabil){
  hentikanUjiReaksi();   // uji selesai -> D1 (dan pompa pulsa bila masih menyala) mati
  simpanRekamanZN();     // TES_145: rekaman kurva disalin dulu (sebelum penilaian hasil)
  // Plateau = rata-rata 3 sampel mentah terakhir (lebih tahan derau daripada 1 sampel; TES_138: tak terpengaruh penjarangan)
  float deltaTotal = ujiNilaiSekarang();
  float minRespon = ujiAmbangDeteksi();   // TES_142
  if(fabs(deltaTotal) < minRespon){   // tidak ada respons berarti (sensor mati / belum kalibrasi / tak tersirkulasi)
    lcd.clear();
    lcdPrintCenter(0,F("Respon tidak"));
    lcdPrintCenter(1,F("terdeteksi!"));
    lcdPrintCenter(2,F("Cek sensor/kalibrasi"));
    if(!stabil) lcdPrintCenter(3,F("/tambah lama rekam"));   // TES_138: waktu habis sebelum respon muncul
    jedaInfo(3000);
    currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
    return;
  }
  // Arah respon wajib sesuai fisika: AB Mix MENAIKKAN PPM, pH Down MENURUNKAN pH. Arah terbalik = sensor/selang/
  // pompa tertukar atau larutan terganggu (mis. air baku masuk) -> Kp/Ki dari kurva ini tidak boleh dipakai.
  bool arahOk = (ujiKanalDipilih==0) ? (deltaTotal > 0.0f) : (deltaTotal < 0.0f);
  if(!arahOk){
    lcd.clear();
    lcdPrintCenter(0,F("ARAH RESPON SALAH"));
    lcdPrintCenter(1,(ujiKanalDipilih==0) ? F("PPM turun") : F("pH naik"));
    lcdPrintCenter(2,F("Cek sensor/pompa"));
    lcdPrintCenter(3,F("Hasil tak disimpan"));
    mulaiBuzzerPutus(3000UL);
    jedaInfo(3500);
    currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
    return;
  }
  // TES_138: waktu rekam habis & kurva masih berubah -> plateau belum tercapai -> K, tau, D, Kp, Ki salah -> tidak disimpan
  if(!stabil){
    lcd.clear();
    lcdPrintCenter(0,F("KURVA BELUM STABIL"));
    lcdPrintCenter(1,(ujiDurasiMaksMenit < UJI_REKAM_MAKS_MENIT) ? F("Tambah lama rekam /") : F("Pengadukan lambat:"));
    lcdPrintCenter(2,F("cek sirkulasi"));
    lcdPrintCenter(3,F("Hasil tak disimpan"));
    mulaiBuzzerPutus(3000UL);
    jedaInfo(3500);
    currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
    return;
  }
  if(deltaTotal < 0.0f){   // pH Down menurunkan pH -> balik tanda agar K, tau, D positif
    for(int i=0;i<ujiJumlahSampel;i++) ujiNilai[i] = -ujiNilai[i];
    deltaTotal = -deltaTotal;
  }
  float K_uji = deltaTotal/ujiDurasiPulsa;
  // Parameter kurva reaksi bentuk S: metode GARIS SINGGUNG di titik belok (sesuai materi PID/Ziegler-Nichols)
  //   D   = perpotongan garis singgung dgn sumbu waktu (y=0)
  //   tau = selisih waktu garis singgung mencapai plateau dan D
  // TES_138: kemiringan maks diukur pada TALI BUSUR yang naik >= 25% respon total (bukan antar-sampel berurutan),
  // memakai kurva yang dihaluskan median-3. Dulu: segmen antar-sampel 4 dtk -> derau sensor di awal kurva lambat
  // menghasilkan 'kemiringan maks' palsu (D jatuh ke 0,5 dtk -> Ki melonjak, atau D > tau di kanal pH berderau).
  // Utk kurva S / orde-1 + waktu mati yang bersih, D hasil tali busur 25% ~ D garis singgung (tau ~15% lebih panjang).
  float slopeMaks=0.0f, tI=0.0f, yI=0.0f;
  const float FRAKSI_BUSUR = 0.25f;
  for(int i=-1;i<ujiJumlahSampel;i++){            // i = -1 : titik awal kurva (0,0) = awal pulsa
    float ti = (i<0) ? 0.0f : ujiWaktu[i], yi = (i<0) ? 0.0f : ujiNilaiHalus(i);
    for(int k=i+1;k<ujiJumlahSampel;k++){
      float yk = ujiNilaiHalus(k);
      if(yk-yi >= FRAKSI_BUSUR*deltaTotal){
        float dtSeg = ujiWaktu[k]-ti;
        if(dtSeg>0.0f){
          float sg = (yk-yi)/dtSeg;
          if(sg>slopeMaks){ slopeMaks=sg; tI=0.5f*(ti+ujiWaktu[k]); yI=0.5f*(yi+yk); }
        }
        break;
      }
    }
  }
  float tau=0.0f, D=0.0f; bool tanOk=false;
  if(slopeMaks>0.0f){
    D   = tI - yI/slopeMaks;
    tau = (tI + (deltaTotal-yI)/slopeMaks) - D;
    tanOk = (D>=0.0f && tau>0.0f);
  }
  if(!tanOk){   // cadangan bila garis singgung tak valid (kurva terlalu kasar/berderau): metode dua titik 28.3% / 63.2%
    float t283 = cariWaktuPadaFraksi(0.283f,deltaTotal);
    float t632 = cariWaktuPadaFraksi(0.632f,deltaTotal);
    tau = 1.5f*(t632-t283);
    D   = t632-tau;
  }
  // TES_142: KOREKSI LAMA PULSA. Rumus Ziegler-Nichols mengasumsikan input LANGKAH (seketika). Dosis pompa selama Tp
  // ke tandon = langkah yang 'dimiringkan' selama Tp -> kurva tampak lebih lambat. Utk proses orde-1 + waktu mati:
  //   tau_tampak = Tp/(1-e^(-Tp/tau)),  D_tampak = D + Tp - tau_tampak + tau   (garis singgung di kemiringan maks)
  // Dibalik (bisection) -> tau & D PROSES, tidak bergantung lama pulsa (respon tuju besar saat sensor berderau =
  // pulsa s/d 3x lebih panjang; tanpa koreksi D ikut membesar -> Ki mengecil).
  { float Tp = ujiDurasiPulsa;
    if(tanOk && Tp > 0.5f && tau < 0.9f*Tp && ujiDerauSd > 0.0f && deltaTotal < UJI_SNR_TUJU*ujiDerauSd){
      // TES_143: garis singgung lebih curam dari laju dosis (respon naik lebih cepat dari pulsanya) -> mustahil secara
      // fisik (maks = respon/Tp; toleransi sampling 10%). Bila respon < 25 x derau = kurva dirusak derau -> ditolak (dulu
      // tau dijepit 0,5 dtk -> Kp ZN sangat kecil, dosing lambat). Sensor tenang (respon >= 25 x derau) = tangga
      // kuantisasi ADC / pengadukan jauh lebih cepat dari pulsa -> tau di batas resolusi (~0.14 Tp), D tetap terpakai.
      lcd.clear();
      lcdPrintCenter(0,F("KURVA TAK WAJAR"));
      lcdPrintCenter(1,F("Naik lebih cepat"));
      lcdPrintCenter(2,F("dari pulsa (derau)"));
      lcdPrintCenter(3,F("Hasil tak disimpan"));
      mulaiBuzzerPutus(3000UL);
      jedaInfo(3500);
      currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub();
      return;
    }
    if(!tanOk && Tp > 0.5f){ D -= 0.5f*Tp; }   // cadangan dua titik: acuan waktu = tengah pulsa (pendekatan orde-1)
    else if(Tp > 0.5f && tau > 0.0f){
      if(tau < 1.001f*Tp) tau = 1.001f*Tp;   // TES_143: toleransi sampling / kuantisasi -> batas resolusi
      float tauT = tau, dT = D, tauP;
      {
        float lo = 0.01f, hi = tauT;
        for(int it=0; it<40; it++){ float mid = 0.5f*(lo+hi); float v = Tp/(1.0f-exp(-Tp/mid)); if(v > tauT) hi = mid; else lo = mid; }
        tauP = 0.5f*(lo+hi);
      }
      D = dT - Tp + tauT - tauP;
      tau = tauP;
    }
  }
  if(D<0.5f) D=0.5f;
  if(tau<0.5f) tau=0.5f;
  float KpRaw = 0.9f*tau/D;
  float KiRaw = 0.27f*tau/(D*D);
  float Kp = KpRaw/K_uji;
  float Ki = KiRaw/K_uji;
  // TES_141: batas aman thd gain model TANGKI UJI (volume uji, kanal pH: PPM terukur) + batas Ki akar riil
  hasilUjiPPMAcuan = ujiPPMAcuan();
  float KModel = gainUji(ujiKanalDipilih, hasilUjiPPMAcuan);
  batasAmanKpKiZN(Kp, Ki, KModel);
  hasilUjiTau=tau; hasilUjiD=D; hasilUjiK=K_uji; hasilUjiKp=Kp; hasilUjiKi=Ki;
  currentState=UJI_ZN_KONFIRMASI;
  lcd.clear();
  tampilkanHasilUjiZN();
}

// TES_138: nilai kurva dihaluskan median-3 (titik ujung apa adanya) -> 1 sampel berderau tak membentuk lereng palsu
float ujiNilaiHalus(int i){
  if(i<=0 || i>=ujiJumlahSampel-1) return ujiNilai[i];
  float a=ujiNilai[i-1], b=ujiNilai[i], c=ujiNilai[i+1];
  if(a>b){ float t=a; a=b; b=t; }
  if(b>c){ float t=b; b=c; c=t; }
  if(a>b){ float t=a; a=b; b=t; }
  return b;
}
// TES_138: rata-rata sampel mentah terakhir (maks 3) = nilai respon sekarang (tahan derau)
float ujiNilaiSekarang(){
  float s=0.0f; for(int k=0;k<ujiRawN;k++) s+=ujiRaw[k];
  return (ujiRawN>0) ? s/(float)ujiRawN : 0.0f;
}
// TES_138: kurva dianggap DATAR bila perubahan dalam jendela W = maks(60 dtk, 30% waktu berjalan) < maks(ambang, 3% respon).
// Dulu: 2 selisih berurutan (jarak 4 dtk) < 3 PPM -> tandon yg teraduk lambat / 2 bacaan sama dianggap stabil di 11-22% plateau.
// Jendela ikut waktu berjalan -> kurva lambat (tandon besar) otomatis butuh masa datar lebih panjang.
// TES_142: ambang respon terdeteksi = maks(5 PPM / 0.05 pH, 3 x derau terukur)
float ujiAmbangDeteksi(){
  float m = (ujiKanalDipilih==0) ? UJI_RESPON_MIN_PPM : UJI_RESPON_MIN_PH;
  float d = UJI_K_DETEKSI_DERAU*ujiDerauSd;
  return (d > m) ? d : m;
}
bool ujiKurvaDatar(float t){
  if(ujiPulsaAktif) return false;
  if(t < ujiDurasiPulsa + UJI_STABIL_JEDA_MIN_DETIK) return false;
  float W = UJI_STABIL_FRAKSI_WAKTU * t; if(W < UJI_STABIL_JEDA_MIN_DETIK) W = UJI_STABIL_JEDA_MIN_DETIK;
  int j=-1; float tj=-1.0f;
  for(int i=0;i<ujiStN;i++){ float ti=(float)ujiStT[i]; if(ti <= t - W && ti > tj){ tj=ti; j=i; } }   // acuan terbaru yg >= W lalu
  if(j < 0) return false;
  float yNow = ujiNilaiSekarang();
  float minR = ujiAmbangDeteksi();
  if(fabs(yNow) < minR) return false;
  float amb = (ujiKanalDipilih==0) ? UJI_STABIL_AMB_PPM : UJI_STABIL_AMB_PH;
  float rel = UJI_STABIL_RELATIF*fabs(yNow); if(rel > amb) amb = rel;
  float ad = UJI_K_DATAR_DERAU*ujiDerauSd; if(ad > amb) amb = ad;   // TES_142: ambang datar ikut derau terukur
  float d = fabs(yNow - ujiStY[j]);
  return d < amb && d < UJI_STABIL_MAKS_RELATIF*fabs(yNow);
}

void tickUjiStepResponse(){
  if(currentState!=UJI_ZN_LOGGING) return;
  unsigned long now = millis();
  if(ujiFase==0){   // TES_138: bilas chamber non-blocking (*=Batal tetap bisa), lalu nilai awal & pulsa
    unsigned long dtkB = (now-ujiBilasMulaiMs)/1000UL;
    if(dtkB != ujiDetikTampil){ ujiDetikTampil = dtkB; tampilkanUjiLoggingBaris1(); }
    if(ujiTdsUntukPH && now-ujiBilasMulaiMs >= (unsigned long)(ujiBilasDetik-6)*1000UL){   // TES_139: PPM tandon utk kanal pH
      float cv = bacaCVTDS();
      float ppm = tdsKalibrasiAktif ? interpolasiTDS(cv) : ((133.42f*cv*cv*cv-255.86f*cv*cv+857.39f*cv)*0.5f);
      if(tdsSensorValid && ppm > 0.0f && ppm < 5000.0f) ujiPPMTerukur = ppm;
      ujiTdsUntukPH = false;
      matikanRelayTDS();
      // TES_144: PPM tandon tak terbaca -> slope pH Down tak diketahui (pulsa dari PPM target bisa melewati batas fase
      // & PPM acuan Kp/Ki salah) -> uji batal (dulu diam-diam memakai PPM target)
      if(ujiPPMTerukur < 0.0f){ ujiBatalInfo(F("SENSOR TDK VALID"), F("TDS: cek probe/kabel"), F("PPM perlu utk uji pH")); return; }
    }
    if(!promptTibaAktif && now-ujiBilasMulaiMs >= (unsigned long)ujiBilasDetik*1000UL) ujiMulaiUkurDerau();   // TES_142: ukur derau dulu
    return;
  }
  if(ujiFase==2){ tickUkurDerau(now); return; }
  // TES_138: pulsa non-blocking -> matikan pompa tepat sesudah ujiDurasiPulsa
  if(ujiPulsaAktif && (now-ujiTimerAwal) >= (unsigned long)(ujiDurasiPulsa*1000.0f)){ setelPompaUji(false); ujiDetikTampil = 0xFFFFFFFFUL; }
  unsigned long dtk = (now-ujiTimerAwal)/1000UL;
  if(dtk != ujiDetikTampil){ ujiDetikTampil = dtk; tampilkanUjiLoggingBaris1(); }   // hitung mundur pulsa / menit rekam
  bool waktuHabis = (now-ujiTimerAwal) >= (unsigned long)ujiDurasiMaksMenit*60000UL;
  if(waktuHabis && promptTibaAktif) return;   // TES_148: hasil ditunda selama prompt TIBA -> jangan merekam tiap putaran loop
  // TES_142: sub-sampel tiap 0,5 dtk (dilewati < 0,2 dtk sebelum pulsa berakhir -> pompa tetap mati tepat waktu)
  bool dekatAkhirPulsa = ujiPulsaAktif && ((unsigned long)(ujiDurasiPulsa*1000.0f) - (now-ujiTimerAwal)) < 200UL;
  if(!dekatAkhirPulsa && now-ujiSubTerakhirMs >= UJI_SUB_MS){
    ujiSubTerakhirMs = now;
    unsigned long tB = millis();
    float v = bacaSensorUjiSekali();
    if(ujiSubSampelValid()){
      ujiSubSum += v;
      if(ujiSubN==0) ujiSubPertamaMs = tB;
      ujiSubAkhirMs = tB; ujiSubN++;
    } else if(currentState != UJI_ZN_LOGGING) return;   // TES_144: dibatalkan (sensor tak valid), pompa sudah mati
  }
  if(!waktuHabis && now-ujiTimerSampelTerakhir < ujiIntervalMs) return;   // TES_138: waktu habis -> sampel terakhir langsung (tak menunggu interval)
  ujiTimerSampelTerakhir = now;
  // TES_142: sampel = rata-rata sub-sampel sepanjang interval; waktu = tengah jendela rata-rata
  float t, vRata;
  if(ujiSubN > 0){
    vRata = ujiSubSum/(float)ujiSubN;
    t = (float)(ujiSubPertamaMs + (ujiSubAkhirMs - ujiSubPertamaMs)/2UL - ujiTimerAwal)/1000.0f;
  } else { vRata = bacaSensorTerpilihRataRata(); t = (now-ujiTimerAwal)/1000.0f; }
  ujiSubSum = 0.0f; ujiSubN = 0;
  float nilai = vRata-ujiC0;
  // Buffer penuh -> separuh titik dibuang & interval sampel digandakan.
  // TES_138: yang dibuang titik paling 'lurus' (kesalahan interpolasi dari tetangganya terkecil), bukan selang-seling:
  // dulu titik di lekuk awal kurva (akhir waktu mati) ikut terbuang pada rekaman panjang -> D salah (0,5-4 dtk).
  if(ujiJumlahSampel>=UJI_MAKS_SAMPEL){
    while(ujiJumlahSampel > UJI_MAKS_SAMPEL/2){
      int iMin=-1; float eMin=0.0f;
      for(int i=1;i<ujiJumlahSampel-1;i++){
        float rentang = ujiWaktu[i+1]-ujiWaktu[i-1];
        float yInt = (rentang>0.0f) ? ujiNilai[i-1] + (ujiNilai[i+1]-ujiNilai[i-1])*(ujiWaktu[i]-ujiWaktu[i-1])/rentang : ujiNilai[i];
        float e = fabs(ujiNilai[i]-yInt);
        if(iMin<0 || e<eMin){ eMin=e; iMin=i; }
      }
      if(iMin<0) break;
      for(int k=iMin;k<ujiJumlahSampel-1;k++){ ujiWaktu[k]=ujiWaktu[k+1]; ujiNilai[k]=ujiNilai[k+1]; }
      ujiJumlahSampel--;
    }
    ujiIntervalMs *= 2UL;
    if(ujiIntervalMs > UJI_INTERVAL_MAKS_MS) ujiIntervalMs = UJI_INTERVAL_MAKS_MS;
  }
  ujiWaktu[ujiJumlahSampel]=t; ujiNilai[ujiJumlahSampel]=nilai; ujiJumlahSampel++;
  ujiRaw[ujiRawIdx]=nilai; ujiRawIdx=(ujiRawIdx+1)%3; if(ujiRawN<3) ujiRawN++;
  float jarakAcuan = 0.05f*t; if(jarakAcuan < 8.0f) jarakAcuan = 8.0f;
  if(ujiStN==0 || t - ujiStTerakhirT >= jarakAcuan){   // titik acuan tiap maks(8 dtk, 5% waktu): cincin 10 titik >= jendela W
    ujiStT[ujiStIdx]=(uint16_t)t; ujiStY[ujiStIdx]=ujiNilaiSekarang(); ujiStIdx=(ujiStIdx+1)%UJI_TITIK_ACUAN; if(ujiStN<UJI_TITIK_ACUAN) ujiStN++;
    ujiStTerakhirT=t;
  }
  char buf[21], fy[12]; snprintf_P(buf,sizeof(buf),PSTR("t=%lddt y=%s"),(long)t,fmtF(fy,nilai,ujiKanalDipilih==0?1:3));
  lcdPrintCenter(2,buf);
  if(ujiKurvaDatar(t)){ if(ujiStabilBerturut < 255) ujiStabilBerturut++; } else ujiStabilBerturut = 0;
  bool stabil = (ujiStabilBerturut >= UJI_STABIL_KONFIRMASI);
  if((stabil || waktuHabis) && !promptTibaAktif) selesaiUjiStepResponse(stabil);   // TES_144: hasil ditunda selama prompt TIBA
}

// ── Simpan Kp/Ki ZN dgn konfirmasi (TES_121) ──
// Profil AKTIF dipakai langsung oleh jadwal yang sedang ter-arm. Bila jadwal aktif & jenis dosing Aktif = CL PI ZN,
// minta konfirmasi dulu (pola sama dgn perubahan Persamaan Nutrisi). Profil BARU / jadwal tak aktif -> langsung simpan.
int znTundaKanal = 0; float znTundaKp = 0.0f, znTundaKi = 0.0f;
float znTundaVolRef = 0.0f; int znTundaPPMRef = 0;   // TES_141: acuan volume & PPM nilai yg ditunda
MenuState znTundaKembali = MENU_ZN_HUB;
bool znTundaDariUji = false;   // TES_138: konfirmasi berasal dari layar hasil uji reaksi -> "Tidak" kembali ke hasil (tak hilang)
byte selZNKonfirmasi = 1;
void mintaSimpanKanalZN(int kanal, float kp, float ki, MenuState kembali, float volRef, int ppmRef){
  bool baru = tampilBaru();
  if(!baru && (dosingSedangMenunggu || jadwalCekAktif) && dosingMode==4){
    znTundaKanal=kanal; znTundaKp=kp; znTundaKi=ki; znTundaKembali=kembali; znTundaVolRef=volRef; znTundaPPMRef=ppmRef;
    selZNKonfirmasi = 1;   // default "Tidak" (aman)
    currentState = ZN_KONFIRMASI_JADWAL;
    lcd.clear(); tampilkanZNKonfirmasiJadwal();
    return;
  }
  simpanKanalZNRef(baru, kanal, kp, ki, volRef, ppmRef);
  konfirmasiSuksesInti(F("Kp/Ki ZN disimpan!"), kembali, baru);
}
void tampilkanZNKonfirmasiJadwal(){
  lcdPrintCenter(0, F("JADWAL CEK AKTIF"));
  lcdPrintCenter(1, F("Terapkan ke jadwal?"));
  lcdPrint20(0,2, String(selZNKonfirmasi==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,3, String(selZNKonfirmasi==1?"> ":"  ") + F("Tidak"));
}
void prosesZNKonfirmasiJadwal(char key){
  if     (key=='A'){ if(selZNKonfirmasi>0){ selZNKonfirmasi--; tampilkanZNKonfirmasiJadwal(); } }
  else if(key=='C'){ if(selZNKonfirmasi<1){ selZNKonfirmasi++; tampilkanZNKonfirmasiJadwal(); } }
  else if(key=='B' || key=='*'){
    bool terapkan = (key=='B' && selZNKonfirmasi==0);
    if(terapkan) simpanKanalZNRef(false, znTundaKanal, znTundaKp, znTundaKi, znTundaVolRef, znTundaPPMRef);
    lcd.clear();
    lcdPrintCenter(1, terapkan ? F("Kp/Ki ZN disimpan!") : F("Tidak Disimpan"));
    if(terapkan) lcdPrintCenter(2, F("(Jadwal ikut ubah)"));
    jedaAman(1500);
    if(!terapkan && znTundaDariUji){   // TES_138: hasil uji (bisa berjam-jam) tidak dibuang -> kembali ke layar hasil
      znTundaDariUji = false;
      currentState = UJI_ZN_KONFIRMASI; lcd.clear(); tampilkanHasilUjiZN();
      return;
    }
    znTundaDariUji = false;
    currentState = znTundaKembali; lcd.clear();
    if(znTundaKembali==MENU_LIHAT_KPKI_ZN) tampilkanKpKiZN(); else tampilkanMenuZNHub();
  }
}

void prosesKonfirmasiUjiZN(char key){
  if(key=='B'){
    znTundaDariUji = true;
    mintaSimpanKanalZN(ujiKanalDipilih, hasilUjiKp, hasilUjiKi, MENU_ZN_HUB, (float)ujiVolumeL, hasilUjiPPMAcuan);   // ke profil konteks (Aktif/Baru); acuan = tandon uji
    if(currentState!=ZN_KONFIRMASI_JADWAL) znTundaDariUji = false;              // langsung tersimpan (tanpa konfirmasi)
  }
  else if(key=='D'){
    if(ujiBentrokJadwal()){ tampilkanBentrokJadwalUji(); lcd.clear(); tampilkanHasilUjiZN(); return; }   // TES_139: hasil tetap
    mulaiUjiStepResponse();
  }
  else if(key=='*'){ currentState=MENU_ZN_HUB; lcd.clear(); tampilkanMenuZNHub(); }
}

void tampilkanDosingInputLamaCek(){
  lcdPrintCenter(0,F("Lama Bilas Sensor"));
  lcdPrintCenter(1,F("(10-900 detik):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" detik"));
  lcdPrint20(0,3,F(""));
}

void tampilkanDosingInputStabilisasi(){
  lcdPrintCenter(0,F("Jeda Stabilisasi"));
  lcdPrintCenter(1,F("(10-900 detik):"));
  lcdPrintCenter(2,(inputBuffer.length()>0?inputBuffer:"_")+F(" detik"));
  lcdPrint20(0,3,F(""));
}

// Jeda Homogenisasi 
void tampilkanDosingInputJeda(int idx){
  lcdPrintCenter(0,F("Jeda Homogenisasi"));
  lcdPrintCenter(1,F("(1-240 menit):"));
  lcdPrintCenter(2,String(F("Jeda "))+String(idx)+F(" : ")+(inputBuffer.length()>0?inputBuffer:"_")+F(" menit"));   // TES_134: J0..J4

  // TES_127: slot sebelumnya = nilai yang baru diketik (salinan sementara, di-commit di slot terakhir)
  int* arrJeda = jedaTunda;

  int totalJeda = JUMLAH_SLOT_JEDA;   // semua jenis dosing memakai 5 slot jeda (J0..J4)
  String ringkas="";
  for(int i=0;i<totalJeda;i++){
    String nilai;
    if(i<idx) nilai=String(arrJeda[i]);
    else if(i==idx) nilai="_";
    else nilai="-";
    ringkas += "J"+String(i)+":"+nilai+" ";
  }

  // Reset posisi scroll setiap kali berpindah tahap 
  static int lastIdxJedaTampil = -1;
  if(idx != lastIdxJedaTampil){
    lastIdxJedaTampil = idx;
    scrollJedaOffset = 0;
  }

  if((int)ringkas.length() <= 20){
    lcdPrint20(0,3,ringkas);
  } else {
    String extended = ringkas + "   " + ringkas;
    if(scrollJedaOffset > (int)ringkas.length()+2) scrollJedaOffset = 0;
    String window = extended.substring(scrollJedaOffset, scrollJedaOffset+20);
    lcdPrint20(0,3, window);
    scrollJedaOffset++;
  }
}

// TES_147: layar tunggu dosing (jadwal cek aktif, belum waktunya) memakai tampilan yang SAMA dengan halaman pertama
// Status Dosing saat tidak dosing: PPM/pH, Target, Cek berikutnya, waktu kini (diperbarui tiap detik oleh prosesTungguWaktu).
void tampilkanDosingTungguWaktu(bool tercapai){
  (void)tercapai;
  tampilkanStatusMenunggu();
}

void tampilkanDosingIsiTandon(){
  bool penuh=(digitalRead(PIN_WATER_LEVEL)==HIGH);
  lcdPrintCenter(0,F("DOSING AKTIF..."));
  lcdPrintCenter(1,F("Larutan Tandon :"));
  lcdPrintCenter(2, penuh?F("Tercapai"):F("Kurang"));
  lcdPrintCenter(3, penuh?F("Status : OK"):F("Status : Mengisi.."));
}

void tampilkanDosingJedaIsi(){
  long tgt = jedaIsiTargetDetik();
  lcdPrintCenter(0,F("DOSING AKTIF..."));
  lcdPrintCenter(1,F("Tandon Terpenuhi"));
  if(tgt > (long)JEDA_ISI_DETIK){   // TES_133: air baru masuk -> homogenisasi (Jeda Homogenisasi 1)
    lcdPrintCenter(2,F("Homogenisasi air"));
    lcdCenterP(3, PSTR("%d/%ld menit"), dosingJedaIsiDetikBerjalan/60, tgt/60L);
  } else {
    lcdPrintCenter(2,F("Stabilisasi.."));
    lcdCenterP(3, PSTR("%d/%d detik"), dosingJedaIsiDetikBerjalan, JEDA_ISI_DETIK);
  }
}

// Peringatan pengisian tandon > 6 jam & belum penuh (cek hari ini dilewati, jadwal besok tetap)
void tampilkanDosingTandonTimeout(){
  lcdPrintCenter(0,F("WAKTU PENGISIAN"));
  lcdPrintCenter(1,F("BERLEBIH! Cek tandon"));
  lcdPrintCenter(2,F("Pengecekan hari ini"));
  lcdPrintCenter(3,F("dibatalkan"));
}

// Peringatan sensor tidak valid 
void tampilkanDosingSensorInvalid(){
  lcdPrintCenter(0,F("SENSOR TIDAK VALID"));
  lcdPrintCenter(1,F("Cek kabel/probe"));
  lcdPrintCenter(2,F("Cek hari ini batal"));
  lcdPrintCenter(3,F("OK= lanjut jadwal"));
}

void hentikanDosingSensorInvalid(){
  gagalkanSiklusHariIni(CEK_ERR_SENSOR);   // sensor tak terbaca: lewati hari ini, jadwal besok tetap jalan
}

// ============================================================
// SENSOR "OFF" SECARA LOGIKA (hemat umur probe TDS & noise pH)
// ============================================================
// Sensor TDS HANYA aktif (relay daya ON) pada: kalibrasi TDS, fase cek PPM dosing,
// tahap PPM pada Cek Akhir, Tes Sensor TDS, dan sekali saat boot.
bool sensorTdsPerluAktif(){
  switch(currentState){
    case MENU_KAL_TDS_SAMPEL:
    case DOSING_CEK_PPM1: case DOSING_CEK_PPM2: case DOSING_CEK_PPM3: case DOSING_PI_CEK_PPM:
      return true;
    case DOSING_CEK_AKHIR:
      return (cekAkhirTahap==0);            // hanya saat tahap ukur PPM
    case UJI_ZN_LOGGING:
      return (ujiKanalDipilih==0) || ujiTdsUntukPH;   // kanal PPM: TDS tetap ON; kanal pH: hanya saat ukur PPM (bilas)
    case TES_SENSOR_PROSES: case TES_SENSOR_HASIL:
      return (tesSensorJenis==0);           // hanya Tes Sensor TDS
    default:
      return false;
  }
}
// Sensor pH dibaca pada: kalibrasi pH, fase cek pH dosing, tahap pH Cek Akhir, Tes Sensor pH.
bool sensorPhPerluAktif(){
  switch(currentState){
    case MENU_KAL_PH_SAMPEL:
    case DOSING_CEK_PH1:  case DOSING_CEK_PH2:  case DOSING_PI_CEK_PH:
      return true;
    case DOSING_CEK_AKHIR:
      return (cekAkhirTahap==1);            // hanya saat tahap ukur pH
    case TES_SENSOR_PROSES: case TES_SENSOR_HASIL:
      return (tesSensorJenis==1);           // hanya Tes Sensor pH
    default:
      return false;
  }
}
bool sensorTdsPhAktif(){ return sensorTdsPerluAktif() || sensorPhPerluAktif(); }

void primingBufferSensorSelektif(bool bacaTds, bool bacaPh){
  if(bacaTds){ for(int i=0;i<TDS_SCOUNT;i++){ tdsBuffer[i]=bacaAdcTDS(); delay(5); } tdsBufferIdx=0; }
  if(bacaPh){  for(int i=0;i<PH_SCOUNT;i++){  phBuffer[i]=bacaAdcPH();  delay(5); } phBufferIdx=0; }
  bacaSensorSelektif(bacaTds, bacaPh);   // segarkan ppmTerbaca/phTerbaca + flag valid
}
void primingBufferSensor(){ primingBufferSensorSelektif(true, true); }

// ============================================================
// PEMULIHAN LCD OTOMATIS (non-blocking, TANPA kedip rutin)
// ============================================================
void gambarUlangLayarAktif(){
  if(promptTibaAktif){ tampilkanPromptTiba(); return; }   // TES_135
  if(statusOverlayAktif){ tampilkanStatusDosing(); return; }
  if(cancelConfirmAktif){ tampilkanCancelDosing(); return; }  
  switch(currentState){
    // Layar utama & mode jalan
    case MENU_UTAMA:            tampilkanMenuUtama();          break;
    case MENU_KREDIT:           tampilkanMenuKredit();         break;
    // Loop / Jenis / Konfirmasi dosing
    case MENU_RINGKASAN_BARU:   tampilkanMenuRingkasanBaru();  break;
    case MULAI_DOSING_KONFIRMASI: tampilkanMulaiDosingKonfirmasi(); break;
    case MENU_BATASAN_PPM:      tampilkanInputBatasPPM();      break;
    case MENU_BATASAN_PH:       tampilkanInputBatasPH();       break;
    case MENU_JENIS_DOSING:     tampilkanMenuJenisDosing();    break;
    case MENU_PERSEN_MAKRO:     tampilkanInputPersenMakro();   break;
    case ZN_KONFIRMASI_JADWAL:  tampilkanZNKonfirmasiJadwal(); break;
    case MENU_KUNCI_KANAL:      tampilkanMenuKunciKanal();     break;
    case RESOLUSI_PERINGATAN:   tampilkanPeringatanResolusi(); break;
    case KUNCI_KANAL_KONFIRMASI: tampilkanKunciKanalKonfirmasi(); break;
    case MENU_EC_BEBAS:         tampilkanInputECBebas();       break;
    case MENU_ZN_HUB:           tampilkanMenuZNHub();          break;
    case MENU_LIHAT_KPKI_ZN:    tampilkanKpKiZN();             break;
    case MENU_ZN_REKAMAN:       tampilkanRekamanZN();          break;   // TES_145
    case MENU_EDIT_KPKI_ZN:     tampilkanEditKpKiZN();         break;
    case UJI_ZN_PILIH_KANAL:    tampilkanUjiZNPilihKanal();    break;
    case UJI_ZN_LOGGING:        tampilkanUjiLogging();         break;   // TES_135: kerangka + sampel terakhir
    case UJI_ZN_KONFIRMASI:     tampilkanHasilUjiZN();         break;
    case UJI_ZN_INPUT_DURASI:   tampilkanUjiZNInputDurasi();   break;
    case UJI_ZN_INPUT_VOLUME:   tampilkanUjiZNInputVolume();   break;
    case DOSING_INPUT_LAMA_CEK: tampilkanDosingInputLamaCek(); break;
    case DOSING_INPUT_STABILISASI: tampilkanDosingInputStabilisasi(); break;
    case DOSING_INPUT_JEDA:     tampilkanDosingInputJeda(dosingJedaInputIdx); break;
    case DOSING_TANDON_TIMEOUT: tampilkanDosingTandonTimeout(); break;
    case DOSING_SENSOR_INVALID: tampilkanDosingSensorInvalid(); break;
    case DOSING_FINAL_STATUS:   tampilkanStatusDosing();       break;
    case DOSING_REKAP:          tampilkanRekapDosing();        break;
    case DOSING_TUNDA_KALIBRASI: tampilkanTundaKalibrasi();    break;
    // SUB MENU A: parameter dosing
    case SUB_MENU_A:            tampilkanSubMenuA();           break;
    case MENU_JENIS_LARUTAN:    tampilkanMenuJenisLarutan();   break;
    case MENU_PENGENCERAN:      tampilkanMenuPengenceran();    break;
    case MENU_JENIS_ASAM:       tampilkanMenuJenisAsam();      break;
    case MENU_KONSENTRASI:      tampilkanMenuKonsentrasi();    break;
    case MENU_VOL_TANDON:       tampilkanInputVolume();        break;
    case MENU_TARGET_PPM:       tampilkanInputPPM();           break;
    case MENU_TARGET_PPM_HASIL: tampilkanHasilKalkulasiPPM();  break;
    case MENU_TARGET_PH:        tampilkanInputPH();            break;
    case MENU_TARGET_PH_HASIL:  tampilkanHasilTargetPH();      break;
    case MENU_WAKTU_DOSING:     tampilkanInputWaktuDosing();   break;
    case MENU_DEBIT_POMPA_SEL:  tampilkanMenuPilihDebitPompa(); break;
    case MENU_DEBIT_POMPA_VAL:  tampilkanInputNilaiDebit();    break;
    // PPM/pH tanaman
    case MENU_PPM_PH_TANAMAN:  tampilkanMenuPPMPHTanaman();   break;
    case MENU_TANAMAN_LIST:     tampilkanMenuTanamanList();    break;
    // SUB MENU B: kalibrasi / RTC / water level / suhu
    case SUB_MENU_B:            tampilkanSubMenuB();           break;
    case MENU_KAL_SENSOR:     tampilkanMenuKalSensor();      break;
    case MENU_KAL_TDS_SAMPEL: tampilkanKalibrasiTDSLayar(0); break;
    case MENU_KAL_PH_SAMPEL:  tampilkanKalibrasiPHLayar(0);  break;
    case MENU_KAL_LIHAT:      tampilkanKalLihat();           break;
    case MENU_KAL_ULANG_KONF: tampilkanKalUlangKonf();       break;
    case MENU_SET_RTC_TAHUN: tampilkanInputRTC(0); break;
    case MENU_SET_RTC_BULAN: tampilkanInputRTC(1); break;
    case MENU_SET_RTC_TGL:   tampilkanInputRTC(2); break;
    case MENU_SET_RTC_JAM:   tampilkanInputRTC(3); break;
    case MENU_SET_RTC_MENIT: tampilkanInputRTC(4); break;
    case MENU_SET_RTC_DETIK: tampilkanInputRTC(5); break;
    case MENU_SET_RTC_HASIL: tampilkanRTCHasil();   break;
    case MENU_WATER_LEVEL:      tampilkanWaterLevel();         break;
    case MENU_SUHU_KALIBRASI:   tampilkanKalkSuhuInput();      break;
    case TES_SENSOR_INPUT_BILAS:  tampilkanTesSensorInputBilas();  break;
    case TES_SENSOR_INPUT_STABIL: tampilkanTesSensorInputStabil(); break;
    case TES_SENSOR_PROSES:       tampilkanTesSensorProses();      break;
    case TES_SENSOR_HASIL:        tampilkanTesSensorHasil();       break;
    // SUB MENU C: tes pompa / solenoid & input debit
    case SUB_MENU_C:            tampilkanMenuPilihPompa();     break;
    case MENU_TES_POMPA_RUN:    tampilkanInputDelayPompa();    break;
    case TES_POMPA_AKTIF:       tampilkanTesPompaAktif();      break;
    case TES_POMPA_CANCEL_CONFIRM: tampilkanCancelTesPompa();  break;
    case MENU_TES_SOLENOID:     tampilkanTesSolenoid();        break;
    case MENU_INPUT_DEBIT_SEL:  tampilkanMenuInputDebitPompa(); break;
    case INPUT_DEBIT_ML:        tampilkanInputDebitMl();       break;
    case INPUT_DEBIT_DETIK:     tampilkanInputDebitDetik();    break;
    case INPUT_DEBIT_HASIL:     tampilkanInputDebitHasil();    break;
    case MENU_RINGKASAN:        tampilkanMenuRingkasan();      break;
    // SUB MENU D: kalkulator
    case SUB_MENU_D:            tampilkanSubMenuD();           break;
    case KALK_WATER_LEVEL_BENTUK: tampilkanKalkWLBentuk();     break;
    case KALK_WL_TABUNG_VOL:    tampilkanKalkWLTabungVol();    break;
    case KALK_WL_TABUNG_JARI:   tampilkanKalkWLTabungJari();   break;
    case KALK_WL_TABUNG_HASIL:  tampilkanKalkWLTabungHasil();  break;
    case KALK_WL_BALOK_VOL:     tampilkanKalkWLBalokVol();     break;
    case KALK_WL_BALOK_PANJANG: tampilkanKalkWLBalokPanjang(); break;
    case KALK_WL_BALOK_LEBAR:   tampilkanKalkWLBalokLebar();   break;
    case KALK_WL_BALOK_HASIL:   tampilkanKalkWLBalokHasil();   break;
    case KALK_INPUT_MENU:       tampilkanKalkInputMenu();      break;
    case KALK_PPM_EC:           tampilkanInputECBebas();       break;
    case KALK_PPM_DIL:          tampilkanKalkMenuDil();        break;
    case KALK_PPM_VOL:          tampilkanKalkInputVolume();    break;
    case KALK_PPM_AWAL:         tampilkanKalkInputPPMAwal();   break;
    case KALK_PPM_TARGET:       tampilkanKalkInputPPMTarget(); break;
    case KALK_PPM_HASIL:        tampilkanKalkHasilPPM();       break;
    case KALK_PH_VOL:           tampilkanKalkPHInputVolume();  break;
    case KALK_PH_ASAM:          tampilkanKalkPHMenuAsam();     break;
    case KALK_PH_KONSEN:        tampilkanKalkPHMenuKonsen();   break;
    case KALK_PH_PPM:           tampilkanKalkPHInputPPM();     break;
    case KALK_PH_AWAL:          tampilkanKalkPHInputAwal();    break;
    case KALK_PH_TARGET:        tampilkanKalkPHInputTarget();  break;
    case KALK_PH_HASIL:         tampilkanKalkPHHasil();        break;
    // Pengaturan sistem / persamaan
    case MENU_PENGATURAN_SISTEM:    tampilkanMenuPengaturanSistem();    break;
    case MENU_PENGATURAN_PERSAMAAN: tampilkanMenuPengaturanPersamaan(); break;
    case PENGATURAN_AB_PILIH_DIL:   tampilkanPengaturanABPilihDil();    break;
    case PENGATURAN_AB_LIST:        tampilkanPengaturanABList();        break;
    case PENGATURAN_PH_PILIH_ASAM:  tampilkanPengaturanPHPilihAsam();   break;
    case PENGATURAN_PH_PILIH_FASE:  tampilkanPengaturanPHPilihFase();   break;
    case PENGATURAN_PH_LIST:        tampilkanPengaturanPHList();        break;
    case PENGATURAN_PH_INPUT:       tampilkanPengaturanPHInput();       break;
    case PENGATURAN_RESET_CONFIRM:  tampilkanPengaturanResetConfirm();  break;
    case PERSAMAAN_KONFIRMASI:      tampilkanPersamaanKonfirmasi();     break;
    case RESET_SISTEM_CONFIRM:      tampilkanResetSistemConfirm();      break;
    default:
      if(statusFaseDosingBerjalan(currentState)) tampilkanUlangFaseDosing();
      break;
  }
}

// Cek keberadaan LCD di bus I2C secara berkala. Re-init HANYA bila LCD lepas/
void pantauLCD(unsigned long now){
  // Pemulihan paksa setelah on/off solenoid valve: 
  if(lcdPerluReinit){
    if(now - lcdReinitMs < LCD_REINIT_TUNDA_MS) return;   // TES_149: tunggu lonjakan reda
    lcdPerluReinit = false;
    lcd.init(); lcd.backlight();
    gambarUlangLayarAktif();
    tsLcdPantau = now;
    return;
  }
  if(now - tsLcdPantau < INTERVAL_LCD_PANTAU) return;
  tsLcdPantau = now;
  Wire.beginTransmission(0x27);
  if(Wire.endTransmission() != 0){
    lcd.init(); lcd.backlight();
    gambarUlangLayarAktif();
  }
}

// Dosing PPM
void tampilkanDosingCekPPM(){
  lcdPrintCenter(0,F("DOSING AKTIF..."));
  if(!dosingPPMTerkunci){
    if(cekSensorFase==0)      lcdPrintCenter(1,F("Stabilisasi TDS.."));
    else if(cekSensorFase==1) lcdPrintCenter(1,F("Membilas Sensor.."));
    else if(cekSensorFase==2) lcdPrintCenter(1,F("Stabilisasi Sensor.."));
    else                      lcdPrintCenter(1,F("Mengukur PPM.."));
    char t[10];
    lcdCenterP(2, PSTR("PPM saat ini: %s"), teksPPM(t));
    int target = (cekSensorFase==0)?JEDA_STABIL_SENSOR_DETIK:(cekSensorFase==1)?terpilihLamaCekSensor:(cekSensorFase==2)?terpilihJedaStabilisasi:ROLL_MED_N;
    lcdCenterP(3, PSTR("%d/%d detik"), dosingCekDetikBerjalan, target);
  } else {
    char f[10];
    // Label cek (TES_122): cek pertama "Awal", cek mikro "Cek 2", cek di loop PI "PI#n" (n = urutan cek PI)
    if(currentState==DOSING_PI_CEK_PPM)    lcdCenterP(1, PSTR("PPM PI#%d : %d"), piCekKe(piIterasiPPM, piPulsaDetikAB, piPPMSelesai), dosingPPMHasilCek);
    else if(currentState==DOSING_CEK_PPM2) lcdCenterP(1, PSTR("PPM Cek 2 : %d"), dosingPPMHasilCek);
    else if(currentState==DOSING_CEK_PPM3) lcdCenterP(1, PSTR("PPM Cek 3 : %d"), dosingPPMHasilCek);
    else                                   lcdCenterP(1, PSTR("PPM Awal : %d"), dosingPPMHasilCek);
    if(makroABDilewati && currentState==DOSING_CEK_PPM1) lcdPrintCenter(2, F("Makro kecil->P2-P3"));
    else lcdCenterP(2, PSTR("PPM Target : %d"), terpilihPPM);
    if(dosingTujuanSetelahCekPPM==DOSING_CEK_PPM3){
      lcdPrintCenter(3, F("Acuan pH Down"));    // cek ukur saja (Open Loop, setelah mikro)
    } else if(dosingTujuanSetelahCekPPM==DOSING_PI_POMPA_AB){
      if(piPPMSelesai) lcdPrintCenter(3, (piFlagSiklus & PI_FLAG_AB_RESPON)  ? F("Respon PPM tdk ada!") :
                                         (piFlagSiklus & PI_FLAG_AB_ITERASI) ? F("PI Batas Iterasi!") : F("PI AB Selesai"));
      else lcdCenterP(3, PSTR("Pulsa PI : %s dtk"), fmtF(f,piPulsaDetikAB,piPulsaDetikAB<100.0f?1:0));
    } else if(dosingLewatiAB){
      if(dosingTujuanSetelahCekPPM==DOSING_MACRO_AB)   // cek pertama: hasil gerbang Batasan
        lcdPrintCenter(3, (kanalTerkunci & KUNCI_AB) ? F("Kanal AB terkunci") :
                          ppmDiAtasBatas(dosingPPMHasilCek) ? F("Di Atas Batas!") : F("Dalam Batas"));
      else
        lcdPrintCenter(3, (piFlagSiklus & PI_FLAG_AB_RESPON) ? F("Respon PPM tdk ada!") : F("PPM Sudah Tercapai"));
    } else if(dosingTujuanSetelahCekPPM==DOSING_MACRO_AB){
      lcdCenterP(3, PSTR("AB Mix 1 : %s ml"), fmtF(f,dosingABMixTahap1,dosingABMixTahap1<1000.0f?1:0));
    } else {
      lcdCenterP(3, PSTR("AB Mix 2 : %s ml"), fmtF(f,dosingABMixTahap2,dosingABMixTahap2<1000.0f?1:0));
    }
  }
}

// Urutan cek di loop PI (1 = cek pertama setelah makro). piIterasi sudah bertambah bila cek ini menghasilkan pulsa.
int piCekKe(int iterasi, float pulsa, bool selesai){ return (!selesai && pulsa>0.0f) ? iterasi : iterasi+1; }
// Dosing pH
void tampilkanDosingCekPH(){
  lcdPrintCenter(0,F("DOSING AKTIF..."));
  if(!dosingPHTerkunci){
    if(cekSensorFase==0)      lcdPrintCenter(1,F("Stabil Sensor pH.."));
    else if(cekSensorFase==1) lcdPrintCenter(1,F("Membilas Sensor.."));
    else if(cekSensorFase==2) lcdPrintCenter(1,F("Stabilisasi Sensor.."));
    else                      lcdPrintCenter(1,F("Mengukur pH.."));
    char t[10];
    lcdCenterP(2, PSTR("pH saat ini: %s"), teksPH(t));
    int target = (cekSensorFase==0)?JEDA_STABIL_SENSOR_DETIK:(cekSensorFase==1)?terpilihLamaCekSensor:(cekSensorFase==2)?terpilihJedaStabilisasi:ROLL_MED_N;
    lcdCenterP(3, PSTR("%d/%d detik"), dosingCekDetikBerjalan, target);
  } else {
    char f[10];
    if(currentState==DOSING_PI_CEK_PH)    lcdCenterP(1, PSTR("pH PI#%d : %s"), piCekKe(piIterasiPH, piPulsaDetikPH, piPHSelesai), fmtF(f,dosingPHHasilCek,2));
    else if(currentState==DOSING_CEK_PH2) lcdCenterP(1, PSTR("pH Cek 2 : %s"), fmtF(f,dosingPHHasilCek,2));
    else                                  lcdCenterP(1, PSTR("pH Awal : %s"), fmtF(f,dosingPHHasilCek,2));
    lcdCenterP(2, PSTR("pH Target : %s"), fmtF(f,terpilihPH,2));
    if(dosingTujuanSetelahCekPH==DOSING_PI_POMPA_PH){
      if(piPHSelesai) lcdPrintCenter(3, (piFlagSiklus & PI_FLAG_PH_RESPON)  ? F("Respon pH tdk ada!") :
                                        (piFlagSiklus & PI_FLAG_PH_ITERASI) ? F("PI Batas Iterasi!") : F("PI pH Selesai"));
      else lcdCenterP(3, PSTR("Pulsa PI : %s dtk"), fmtF(f,piPulsaDetikPH,piPulsaDetikPH<100.0f?1:0));
    } else if(dosingLewatiPH){
      if(dosingTujuanSetelahCekPH==DOSING_MACRO_PH)    // cek pertama: hasil gerbang Batasan
        lcdPrintCenter(3, (kanalTerkunci & KUNCI_PH) ? F("Kanal pH terkunci") :
                          phDiBawahBatas(dosingPHHasilCek) ? F("Di Bawah Batas!") : F("Dalam Batas"));
      else
        lcdPrintCenter(3, (piFlagSiklus & PI_FLAG_PH_RESPON) ? F("Respon pH tdk ada!") : F("pH Sudah Tercapai"));
    } else if(dosingTujuanSetelahCekPH==DOSING_MACRO_PH){
      lcdCenterP(3, PSTR("pH Down 1 : %s ml"), fmtF(f,dosingPHDownTahap1,dosingPHDownTahap1<100.0f?2:(dosingPHDownTahap1<1000.0f?1:0)));
    } else {
      lcdCenterP(3, PSTR("pH Down 2 : %s ml"), fmtF(f,dosingPHDownTahap2,dosingPHDownTahap2<100.0f?2:(dosingPHDownTahap2<1000.0f?1:0)));
    }
  }
}

// Menampilkan progres persentase pompa aktif
void tampilkanDosingPompaAktif(){
  lcdPrintCenter(0,F("DOSING AKTIF..."));
  lcdCenterP(1, PSTR("%s"), dosingPumpLabel.c_str());
  lcdCenterP(2, PSTR("%d/%d detik"), dosingPumpDetikBerjalan, dosingPumpDetikTarget);
  lcdBar(3, dosingPumpDetikBerjalan, dosingPumpDetikTarget);
}

// Progres Dosing
void tampilkanDosingHomogenisasi(){
  lcdPrintCenter(0,F("DOSING AKTIF..."));
  lcdPrintCenter(1,(sedangUlangAB || sedangUlangPH) ? F("Homogenisasi ulang") : F("Homogenisasi Larutan"));   // TES_137
  int jedaMenit = jedaUntukState(currentState);
  int menitBerjalan = dosingHomogenDetikBerjalan/60;
  lcdCenterP(2, PSTR("%d/%d menit"), menitBerjalan, jedaMenit);
  lcdBar(3, dosingHomogenDetikBerjalan, (long)jedaMenit*60L);
}

void tampilkanDosingCekAkhir(){
  bool tahapPPM = (cekAkhirTahap==0);
  lcdPrintCenter(0, tahapPPM?F("CEK AKHIR PPM (1/2)"):F("CEK AKHIR pH (2/2)"));
  if(cekSensorFase==0)      lcdPrintCenter(1, tahapPPM?F("Stabil Daya TDS.."):F("Stabil Sensor pH.."));
  else if(cekSensorFase==1) lcdPrintCenter(1,F("Bilas Sensor Akhir.."));
  else if(cekSensorFase==2) lcdPrintCenter(1,F("Stabilisasi Akhir.."));
  else                      lcdPrintCenter(1, tahapPPM?F("Mengukur PPM.."):F("Mengukur pH.."));
  char t[10];
  if(tahapPPM) lcdCenterP(2, PSTR("PPM: %s"), teksPPM(t));
  else         lcdCenterP(2, PSTR("pH : %s"), teksPH(t));
  int target = (cekSensorFase==0)?JEDA_STABIL_SENSOR_DETIK:(cekSensorFase==1)?terpilihLamaCekSensor:(cekSensorFase==2)?terpilihJedaStabilisasi:ROLL_MED_N;
  lcdCenterP(3, PSTR("%d/%d detik"), dosingCekDetikBerjalan, target);
}

void tampilkanCancelDosing(){
  lcdPrintCenter(0,F("Cancel Dosing?"));
  lcdPrint20(0,1, String(selCancelDosing==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,2, String(selCancelDosing==1?"> ":"  ") + F("Tidak"));
  lcdPrint20(0,3,F(""));
}

// ======================================================================
// STATUS DOSING — 2 halaman. B = maju, * = kembali (pola sama dgn menu lain)
//   Hal. 1 (utama): saat siklus berjalan -> bacaan LIVE seperti mode kalibrasi, lalu nilai median TETAP setelah
//                   tiap cek selesai; saat menunggu -> hasil cek terakhir dinilai thd target & batasan aktif.
//   Hal. 2        : jadwal cek berikutnya, suhu, dan rekap dosing terakhir.
// ======================================================================
bool siklusSedangBerjalan(){ return statusFaseDosingBerjalan(currentState) && currentState!=DOSING_TUNGGU_WAKTU; }

const __FlashStringHelper* labelFaseDosing(){
  switch(currentState){
    case DOSING_ISI_TANDON:    return F("Mengisi tandon..");
    case DOSING_JEDA_PULIH:    return F("Homogenisasi pulih");
    case DOSING_JEDA_ISI:      return isiAdaAirBaru ? F("Homogenisasi air") : F("Stabilisasi tandon");
    case DOSING_CEK_PPM1: case DOSING_CEK_PPM2: case DOSING_CEK_PPM3: case DOSING_PI_CEK_PPM:
      return dosingPPMTerkunci ? F("Cek PPM: selesai") : F("Cek PPM: live");
    case DOSING_CEK_PH1:  case DOSING_CEK_PH2:  case DOSING_PI_CEK_PH:
      return dosingPHTerkunci  ? F("Cek pH: selesai")  : F("Cek pH: live");
    case DOSING_MACRO_AB: case DOSING_MICRO_AB: case DOSING_PI_POMPA_AB: return F("Pompa AB Mix aktif");
    case DOSING_MACRO_PH: case DOSING_MICRO_PH: case DOSING_PI_POMPA_PH: return F("Pompa pH Down aktif");
    case DOSING_HOMOGEN1: case DOSING_HOMOGEN2: case DOSING_HOMOGEN3: case DOSING_HOMOGEN4:
    case DOSING_PI_HOMOGEN_AB: case DOSING_PI_HOMOGEN_PH: return F("Homogenisasi..");
    case DOSING_CEK_AKHIR:
      if(cekAkhirTahap==0) return dosingPPMTerkunci ? F("Cek akhir PPM ok") : F("Cek akhir PPM live");
      return dosingPHTerkunci ? F("Cek akhir pH ok") : F("Cek akhir pH live");
    default: return F("Proses dosing..");
  }
}

// Catatan Status Dosing: kanal terkunci (kondisi SEKARANG) didahulukan, lalu catatan PI siklus terakhir
const __FlashStringHelper* teksCatatan(uint8_t pf){
  if((kanalTerkunci & KUNCI_AB) && (kanalTerkunci & KUNCI_PH)) return F("AB & pH TERKUNCI");
  if(kanalTerkunci & KUNCI_AB) return F("Kanal AB TERKUNCI");
  if(kanalTerkunci & KUNCI_PH) return F("Kanal pH TERKUNCI");
  return teksPIFlag(pf);
}
// Teks catatan PI (Status Dosing): satu kanal -> pesan lengkap; dua kanal -> ringkas
const __FlashStringHelper* teksPIFlag(uint8_t pf){
  bool ab = pf & (PI_FLAG_AB_RESPON|PI_FLAG_AB_ITERASI);
  bool ph = pf & (PI_FLAG_PH_RESPON|PI_FLAG_PH_ITERASI);
  if(ab && ph) return F("PI AB&pH bermasalah");
  if(pf & PI_FLAG_AB_RESPON)  return F("AB: TAK ADA RESPON");
  if(pf & PI_FLAG_PH_RESPON)  return F("pH: TAK ADA RESPON");
  if(pf & PI_FLAG_AB_ITERASI) return F("AB: BATAS ITERASI");
  return F("pH: BATAS ITERASI");
}
void tampilkanStatusHal1(){
  char b[21];
  bool berjalan = siklusSedangBerjalan();
  char ppmStr[10], phStr[10], f[10];
  if(berjalan){
    // Live selama sensor sedang mengukur & belum terkunci; setelah terkunci -> nilai median siklus ini (tetap)
    bool ppmLive = sensorTdsPerluAktif() && !dosingPPMTerkunci;
    bool phLive  = sensorPhPerluAktif()  && !dosingPHTerkunci;
    if(ppmLive) teksPPM(ppmStr); else if(statusPPMTerakhir>=0) itoa(statusPPMTerakhir,ppmStr,10); else strcpy(ppmStr,"-");
    if(phLive)  teksPH(phStr);   else if(statusPHTerakhir>=0.0f) fmtF(phStr,statusPHTerakhir,2); else strcpy(phStr,"-");
  } else {
    if(cekTerakhir.ppm<0) strcpy(ppmStr,"-"); else itoa(cekTerakhir.ppm,ppmStr,10);
    if(cekTerakhir.ph<0.0f) strcpy(phStr,"-"); else fmtF(phStr,cekTerakhir.ph,2);
  }
  { char b0[32]; snprintf_P(b0,sizeof(b0),PSTR("PPM : %s pH : %s"), ppmStr, phStr);
    if(strlen(b0) > 20) snprintf_P(b0,sizeof(b0),PSTR("PPM:%s pH:%s"), ppmStr, phStr);   // TES_128: PPM 4 digit + pH >= 10
    lcdPrintCenter(0, b0); }
  // Target AKTIF (otomatis berganti saat Parameter Baru dipakai)
  if(terpilihPPM>0 && terpilihPH>0.0f) lcdCenterP(berjalan?2:1, PSTR("Target %d | %s"), terpilihPPM, fmtF(f,terpilihPH,2));
  else                                 lcdPrintCenter(berjalan?2:1, F("Target -"));
  if(berjalan){
    lcdPrintCenter(1, labelFaseDosing());
    lcdCenterP(3, PSTR("Suhu : %s C"), teksSuhu(f));
    return;
  }
  // Hasil cek terakhir
  uint8_t pf = piFlagCek();
  bool lambat = (kodeErrCek()==CEK_ERR_LAMBAT);   // TES_137
  bool tampilPI = (pf || kanalTerkunci || lambat) && ((millis()/2000UL)%2UL==1UL);   // catatan: bergantian dgn status batasan tiap 2 dtk
  if(cekTerakhir.ppm<0 && cekTerakhir.ph<0.0f && kodeErrCek()==CEK_ERR_TIDAK) lcdPrintCenter(2, F("Hasil: -"));
  else if(kodeErrCek()==CEK_ERR_TANDON) lcdPrintCenter(2, F("Gagal: tandon"));
  else if(kodeErrCek()==CEK_ERR_SENSOR) lcdPrintCenter(2, F("Gagal: sensor"));
  else if(tampilPI){ if(lambat && !kanalTerkunci && !(pf & (PI_FLAG_AB_RESPON|PI_FLAG_PH_RESPON))) lcdPrintCenter(2, F("Respon dosis lambat"));
                     else lcdPrintCenter(2, teksCatatan(pf)); }
  else {
    // TES_147: baris 3 menampilkan BATASAN yang diinput (bukan status dalam/luar batas): "Batas +- 200 | 0.20"
    if(batasanSudahDiatur && batasPPM>0 && batasPHx100>0){
      snprintf_P(b,sizeof(b),PSTR("Batas +- %d | %s"), batasPPM, fmtF(f, batasPHx100/100.0f, 2));
      lcdPrintCenter(2, b);
    } else lcdPrintCenter(2, F("Batas +- - | -"));
  }
  if(cekTerakhir.ppm<0 && cekTerakhir.ph<0.0f && kodeErrCek()==CEK_ERR_TIDAK) snprintf_P(b,sizeof(b),PSTR("Cek  : -"));
  else if(cekTerakhir.tgl==0) snprintf_P(b,sizeof(b),PSTR("Cek  : RTC"));
  else snprintf_P(b,sizeof(b),PSTR("Cek %02d/%02d/%02d %02d:%02d"),cekTerakhir.tgl,cekTerakhir.bln,cekTerakhir.thn%100,cekTerakhir.jam,cekTerakhir.mnt);
  lcdPrintCenter(3, b);
}

void tampilkanStatusHal2(){
  char b[21];
  if(dosingSedangMenunggu) snprintf_P(b,sizeof(b),PSTR("Jadwal %02d/%02d %s"),dosingTargetHari,dosingTargetBulan,terpilihWaktuDosing.c_str());
  else if(jadwalCekAktif)  snprintf_P(b,sizeof(b),PSTR("Jadwal: cek berjalan"));
  else                     snprintf_P(b,sizeof(b),PSTR("Jadwal: tidak aktif"));
  lcdPrintCenter(0, b);
  { char f[10]; lcdCenterP(1, PSTR("Suhu : %s C"), teksSuhu(f)); }
  if(cekTerakhir.dTgl==0) snprintf_P(b,sizeof(b),PSTR("Dosis : belum ada"));
  else snprintf_P(b,sizeof(b),PSTR("Dosis %02d/%02d/%02d %02d:%02d"),cekTerakhir.dTgl,cekTerakhir.dBln,cekTerakhir.dThn%100,cekTerakhir.dJam,cekTerakhir.dMnt);
  lcdPrintCenter(2, b);
  { char kiri[12], kanan[12];
    fmtVolRingkas(kiri,sizeof(kiri),"AB",cekTerakhir.dABmL);
    fmtVolRingkas(kanan,sizeof(kanan),"pH",cekTerakhir.dPHmL);
    lcdPrintLR(3, kiri, kanan); }
}

// TES_146: halaman "menunggu cek berikutnya": PPM/pH hasil cek terakhir, target aktif, jadwal cek berikutnya, waktu kini
void tampilkanStatusMenunggu(){
  char b[24], f[10], ppmStr[10], phStr[10];
  if(cekTerakhir.ppm<0) strcpy(ppmStr,"-"); else itoa(cekTerakhir.ppm,ppmStr,10);
  if(cekTerakhir.ph<0.0f) strcpy(phStr,"-"); else fmtF(phStr,cekTerakhir.ph,2);
  snprintf_P(b,sizeof(b),PSTR("PPM : %s pH : %s"), ppmStr, phStr);
  if(strlen(b) > 20) snprintf_P(b,sizeof(b),PSTR("PPM:%s pH:%s"), ppmStr, phStr);   // PPM 4 digit + pH >= 10
  lcdPrintCenter(0, b);
  if(terpilihPPM>0 && terpilihPH>0.0f){
    snprintf_P(b,sizeof(b),PSTR("Target : %d | %s"), terpilihPPM, fmtF(f,terpilihPH,2));
    if(strlen(b) > 20) snprintf_P(b,sizeof(b),PSTR("Target %d | %s"), terpilihPPM, fmtF(f,terpilihPH,2));
    lcdPrintCenter(1, b);
  } else lcdPrintCenter(1, F("Target : - | -"));   // TES_147: target belum ada
  if(dosingSedangMenunggu) snprintf_P(b,sizeof(b),PSTR("Cek : %02d/%02d/%02d %s"),dosingTargetHari,dosingTargetBulan,dosingTargetTahun%100,terpilihWaktuDosing.c_str());
  else                     snprintf_P(b,sizeof(b),PSTR("Cek : -"));   // TES_147: tidak ada jadwal
  lcdPrintCenter(2, b);
  if(rtcWaktuValid()){
    DateTime n = rtc.now();
    snprintf_P(b,sizeof(b),PSTR("%02d/%02d/%02d %02d:%02d:%02d"),n.day(),n.month(),n.year()%100,n.hour(),n.minute(),n.second());
    lcdPrintCenter(3, b);
  } else lcdPrintCenter(3, F("RTC belum diatur"));
}

// TES_146: true bila TIDAK ada siklus dosing berjalan (menunggu cek berikutnya / tanpa jadwal).
// Layar hasil siklus (DOSING_FINAL_STATUS) dikecualikan agar hasil cek yang baru selesai tetap tampil lebih dulu.
bool statusModeMenunggu(){ return !siklusSedangBerjalan() && currentState!=DOSING_FINAL_STATUS; }

void tampilkanStatusDosing(){
  { byte hm = statusModeMenunggu() ? 2 : 1; if(statusHalaman > hm) statusHalaman = hm; }   // TES_148: mode berganti saat dilihat (3 -> 2 halaman)
  if(statusModeMenunggu()){
    if(statusHalaman==0)      tampilkanStatusMenunggu();
    else if(statusHalaman==1) tampilkanStatusHal1();
    else                      tampilkanStatusHal2();
  } else {
    if(statusHalaman>=1) tampilkanStatusHal2();
    else                 tampilkanStatusHal1();
  }
}

// ===========================
// MENU "PENGATURAN PERSAMAAN"
// ===========================
void tampilkanMenuPengaturanSistem(){
  lcdPrintCenter(0,F("PENGATURAN SISTEM"));
  for(int i=0;i<3;i++){
    int idx=scrollPengaturanSistem+i;
    if(idx<TOTAL_PENGATURAN_SISTEM_ROWS) lcdPrint20(0,i+1,(idx==selPengaturanSistem?"> ":"  ")+String(pengaturanSistemOptions[idx]));
    else lcdPrint20(0,i+1,"");
  }
}

// Pengaturan Sistem > Kunci Kanal: kanal dikunci otomatis saat dosis tidak menghasilkan respon (sensor macet,
// D1 tak sirkulasi, stok habis, pompa macet). Buka kunci hanya setelah penyebabnya dicek.
void tampilkanMenuKunciKanal(){
  lcdPrintCenter(0, F("KUNCI KANAL"));
  lcdPrint20(0,1, String(selKunciKanal==0?"> ":"  ") + F("AB Mix : ") + ((kanalTerkunci&KUNCI_AB)?F("TERKUNCI"):F("normal")));
  lcdPrint20(0,2, String(selKunciKanal==1?"> ":"  ") + F("pH Down: ") + ((kanalTerkunci&KUNCI_PH)?F("TERKUNCI"):F("normal")));
  lcdPrint20(0,3, F("<-   OK=Buka"));
}
void tampilkanKunciKanalKonfirmasi(){
  lcdPrintCenter(0, (selKunciKanal==0) ? F("Buka kunci AB Mix?") : F("Buka kunci pH Down?"));
  lcdPrintCenter(1, F("Sensor/pompa dicek?"));
  lcdPrint20(0,2, String(selKunciKonf==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,3, String(selKunciKonf==1?"> ":"  ") + F("Tidak"));
}
void tampilkanResetSistemConfirm(){
  lcdPrintCenter(0,F("Reset Sistem ke"));
  lcdPrintCenter(1,F("Setelan Pabrik?"));
  lcdPrint20(0,2, String(selResetSistemConfirm==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,3, String(selResetSistemConfirm==1?"> ":"  ") + F("Tidak"));
}

void resetPabrik(){
  // Matikan semua aktuator SEKETIKA (mis. tes pompa latar belakang) sebelum menghapus EEPROM
  matikanSemuaPompa();
  tutupSolenoid();
  matikanRelayTDS();
  hentikanBuzzer();
  lcd.clear();
  lcdPrintCenter(1,F("Mereset ke"));
  lcdPrintCenter(2,F("Setelan Pabrik.."));
  // Hapus seluruh area konfigurasi EEPROM (parameter dosing, persamaan, kalibrasi sensor)
  for(unsigned int i=0;i<sizeof(KonfigTersimpan);i++){
    EEPROM.update(EEPROM_ALAMAT + i, 0xFF); wdt_reset();
  }
  // Hapus juga kalibrasi suhu pH (disimpan terpisah dari struct utama).
  // Waktu RTC (jam hardware DS3231) TIDAK ikut direset -> tetap berjalan normal.
  for(unsigned int i=0;i<sizeof(suhuKalibrasiPH);i++){
    EEPROM.update(EEPROM_ALAMAT_SUHUKALPH + i, 0xFF); wdt_reset();
  }
  // Hapus juga riwayat hasil cek harian terakhir (Status Dosing kembali "-")
  for(unsigned int i=0;i<sizeof(RekamCek);i++){
    EEPROM.update(EEPROM_ALAMAT_REKAMCEK + i, 0xFF); wdt_reset();
  }
  // Hapus data tambahan (draft Parameter Baru, profil PI ZN Baru, persen makro dosing, kunci kanal)
  for(unsigned int i=0;i<sizeof(KonfigEkstra);i++){
    EEPROM.update(EEPROM_ALAMAT_EKSTRA + i, 0xFF); wdt_reset();
  }
  // Hapus draft kalibrasi TDS/pH
  for(unsigned int i=0;i<sizeof(KalibDraft);i++){
    EEPROM.update(EEPROM_ALAMAT_DRAFTKAL + i, 0xFF); wdt_reset();
  }
  // TES_141: acuan uji Kp/Ki ZN
  for(unsigned int i=0;i<sizeof(KonfigZNUji);i++){
    EEPROM.update(EEPROM_ALAMAT_ZNUJI + i, 0xFF); wdt_reset();
  }
  // TES_145: rekaman kurva uji terakhir (cukup kosongkan magic + jumlah titik; data kanal tak dibaca tanpa magic)
  for(unsigned int i=0;i<8;i++){
    EEPROM.update(EEPROM_ALAMAT_ZNREK + i, 0xFF); wdt_reset();
  }
  jedaAman(1500);
  // TES_150: restart TANPA watchdog. Sesudah reset oleh watchdog, ATmega2560 menyalakan watchdog lagi dgn waktu
  // terpendek (16 ms, berapa pun nilai yg dipasang program - TES_146 memakai 4 dtk) & bootloader bawaan Mega yang
  // lama tidak mematikannya -> bootloader ter-reset terus (boot loop, layar tetap "Mereset ke Setelan Pabrik..").
  // Kini: semua interupsi & periferal dimatikan lalu lompat ke alamat 0 (program mulai dari awal, setup() berjalan).
  restartLunak();
}
// TES_150: restart program dari awal tanpa watchdog (aman utk bootloader Mega lama).
void restartLunak(){
  cli();
  wdt_disable();
  TWCR = 0;                              // I2C (LCD, RTC, ADS1115)
  UCSR0B = 0; UCSR1B = 0; UCSR2B = 0; UCSR3B = 0;   // UART
  ADCSRA = 0;
  TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; TIMSK4 = 0; TIMSK5 = 0;
  EIMSK = 0; PCICR = 0;
  asm volatile("jmp 0");
}

void tampilkanMenuPengaturanPersamaan(){
  lcdPrintCenter(0,F("PERSAMAAN NUTRISI"));
  lcdPrint20(0,1, String(selPengaturan==0?"> ":"  ") + F("AB Mix"));
  lcdPrint20(0,2, String(selPengaturan==1?"> ":"  ") + F("pH Down"));
  lcdPrint20(0,3, String(selPengaturan==2?"> ":"  ") + F("Reset Persamaan"));
}

// "KONSTANTA AB MIX"
void tampilkanPengaturanABPilihDil(){
  lcdPrintCenter(0,F("KONSTANTA AB MIX"));
  lcdPrintCenter(1,F("Pilih pengenceran:"));
  lcdPrint20(0,2, String(selPengaturanABDil==0?"> ":"  ") + F("5 ml/L"));
  lcdPrint20(0,3, String(selPengaturanABDil==1?"> ":"  ") + F("10 ml/L"));
}

// Layar gabungan browsing + edit slope AB Mix
void tampilkanPengaturanABList(){
  lcdPrintCenter(0, String(dilOptions[selPengaturanABDil]) + F(" @ EC ") + String(EC_ANCHOR[selPengaturanABDil],1));
  lcdPrintCenter(1, String(F("Slope : ")) + String(regresiSlopeAB[selPengaturanABDil],2));
  lcdPrint20(0,2, String(F("Edit Slope: ")) + (inputBuffer.length()>0?inputBuffer:"_"));
  lcdPrint20(0,3,F(""));
}

// KONSTANTA pH DOWN
void tampilkanPengaturanPHPilihAsam(){
  lcdPrintCenter(0,F("KONSTANTA pH DOWN"));
  lcdPrintCenter(1,F("Pilih Jenis Down:"));
  lcdPrint20(0,2, String(selPengaturanPHAsam==0?"> ":"  ") + F("Nitrat"));
  lcdPrint20(0,3, String(selPengaturanPHAsam==1?"> ":"  ") + F("Fosfat"));
}

// ASAM NITRAT

void tampilkanPengaturanPHPilihFase(){
  lcdPrintCenter(0, String(F("ASAM ")) + String(acidOptions[selPengaturanPHAsam]));
  lcdPrintCenter(1, F("Pilih Fase:"));
  lcdPrint20(0,2, String(selPengaturanPHFase==0?"> ":"  ") + F("Fase Atas (>5.85)"));
  lcdPrint20(0,3, String(selPengaturanPHFase==1?"> ":"  ") + F("Fase Bawah (<5.85)"));
}

void tampilkanPengaturanPHList(){
  const float (*tabel)[2];
  if(selPengaturanPHAsam==0) tabel = (selPengaturanPHFase==0)? tabelSlopeAtas : tabelSlopeBawah;
  else                       tabel = (selPengaturanPHFase==0)? tabelSlopeAtasFosfat : tabelSlopeBawahFosfat;

  String judul = String(acidOptions[selPengaturanPHAsam]) + " Fase " + String(selPengaturanPHFase==0?"Atas":"Bawah");
  lcdPrintCenter(0, judul);

  int rentangBawah = 100 + 200*selPengaturanPHIdx;
  int rentangAtas  = rentangBawah + 200;
  lcdPrintCenter(1, String(F("PPM (")) + String(rentangBawah) + F("-") + String(rentangAtas) + F(")"));

  lcdPrintCenter(2, String(F("PPM   : ")) + String((int)tabel[selPengaturanPHIdx][0]));
  lcdPrintCenter(3, String(F("Slope : -")) + String(tabel[selPengaturanPHIdx][1],4));
}

void tampilkanPengaturanPHInput(){
  float (*tabel)[2];
  if(selPengaturanPHAsam==0) tabel = (selPengaturanPHFase==0)? tabelSlopeAtas : tabelSlopeBawah;
  else                       tabel = (selPengaturanPHFase==0)? tabelSlopeAtasFosfat : tabelSlopeBawahFosfat;

  lcdPrintCenter(0, String(F("PPM : ")) + String(fasePengaturanPHInput==1 ? ppmAcuanTundaPH : (int)tabel[selPengaturanPHIdx][0]));

  if(fasePengaturanPHInput==0){
    lcdPrintCenter(1, String(F("Edit PPM : ")) + (inputBuffer.length()>0?inputBuffer:"_"));
    lcdPrintCenter(2, String(F("Slope : -")) + String(tabel[selPengaturanPHIdx][1],4));
    lcdPrint20(0,3,F(""));
  } else {
    lcdPrint20(0,1,F(""));
    lcdPrintCenter(2, String(F("Slope : -")) + String(tabel[selPengaturanPHIdx][1],4));
    lcdPrintCenter(3, String(F("Edit Slope : -")) + (inputBuffer.length()>0?inputBuffer:"_"));
  }
}

// Reset Konstanta
void tampilkanPengaturanResetConfirm(){
  lcdPrintCenter(0,F("Reset Konstanta"));
  lcdPrintCenter(1,F("ke Nilai Pabrik?"));
  lcdPrint20(0,2, String(selPengaturanResetConfirm==0?"> ":"  ") + F("Ya"));
  lcdPrint20(0,3, String(selPengaturanResetConfirm==1?"> ":"  ") + F("Tidak"));
}
