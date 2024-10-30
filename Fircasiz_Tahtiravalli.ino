#include <Wire.h>   // I2C iletişimi için Wire kütüphanesini dahil et
#include <Servo.h>  // Servo motor kontrolü için Servo kütüphanesini dahil et

// Servo motorları tanımlıyoruz
Servo sagMotor;   // Sağ motorun kontrolü için bir Servo nesnesi oluştur
Servo solMotor;   // Sol motorun kontrolü için bir Servo nesnesi oluştur

// MPU-6050 sensöründen alınacak ham veriler için değişkenler
//Veriler ham olduğu için ön kalibirasyon gerekmez
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY;

// Açı hesaplamaları için diziler
float Acceleration_angle[2]; // İvme açısı (X, Y) için diziyi tanımla
float Gyro_angle[2];         // Jiroskop açısı (X, Y) için diziyi tanımla
float Total_angle[2];        // Toplam açı (X, Y) için diziyi tanımla

// Zaman hesaplamaları için değişkenler
float elapsedTime, currentTime, previousTime;

// PID kontrol değişkenleri
float PID, pwmSol, pwmSag, hata, onceki_hata; // PID hesaplamaları için gerekli değişkenler
//Alttaki PID değerleri su altı şartları için denenmiş değerlerdir. Hava ortamı için yeni hesap gerekir.
double kp = 3.75; // Proportional katsayısı; hata ile çıkış arasındaki çarpan
double ki = 0.0045; // Integral katsayısı; hata toplamını hesaba katar
double kd = 2.65; // Derivative katsayısı; hata değişim hızını hesaba katar

//throttle değeri motor ve ESC ye göre farklılık gösterebilmekte.
double throttle = 1300; // Motorlar için başlangıç gaz değeri (minimum PWM sinyali)
float hedef_ac = 0; // Denge açısı (hedef açı); sistemin dengede kalması gereken açı

void setup() {
  // I2C iletişimini başlat
  Wire.begin();

  // MPU6050 sensörünü başlat
  Wire.beginTransmission(0x68); // MPU6050 adresine iletişim başlat
  Wire.write(0x6B); // Güç kaynağı kayıt adresi
  Wire.write(0); // Güç kaynağını aktif etmek için 0 yaz
  Wire.endTransmission(true); // İletişimi sonlandır

  Serial.begin(250000); // Seri haberleşmeyi başlat; debug için yüksek hız

  // Motorları bağlantı pinlerine ata
  sagMotor.attach(3); // Sağ motor için kontrol pinini ayarla (pin 3)
  solMotor.attach(5); // Sol motor için kontrol pinini ayarla (pin 5)

  // ESC'lerin çalışması için başlangıç sinyali gönder
  sagMotor.writeMicroseconds(1000); // Sağ motor için 1000 mikro saniye sinyal gönder
  solMotor.writeMicroseconds(1000); // Sol motor için 1000 mikro saniye sinyal gönder
  delay(7000); // Motorların kalibrasyon için 7 saniye bekle
  //Bip bi bi bipp
}

void loop() {
  // Zaman hesaplaması
  previousTime = currentTime; // Önceki zamanı güncelle
  currentTime = millis(); // Şu anki zamanı al (milisaniye cinsinden)
  elapsedTime = (currentTime - previousTime) / 1000; // Geçen zamanı saniye cinsine çevir

  // İvme verilerini oku
  Wire.beginTransmission(0x68); // MPU6050 ile iletişim başlat
  Wire.write(0x3B); // İvme verileri için başlangıç adresi (0x3B)
  Wire.endTransmission(false); // İletişimi sonlandırmadan veri iste
  Wire.requestFrom(0x68, 6, true); // 6 adet veri iste (X, Y, Z eksenleri)

  // Ham ivme verilerini oku ve açılara çevir
  Acc_rawX = Wire.read() << 8 | Wire.read(); // X eksenindeki ivme verisini oku
  Acc_rawY = Wire.read() << 8 | Wire.read(); // Y eksenindeki ivme verisini oku
  Acc_rawZ = Wire.read() << 8 | Wire.read(); // Z eksenindeki ivme verisini oku

  // İvme açılarını hesapla (radyandan dereceye çevir)
  Acceleration_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * (180 / 3.141592654); // X açısını hesapla
  Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * (180 / 3.141592654); // Y açısını hesapla

  // Jiroskop verilerini oku
  Wire.beginTransmission(0x68); // MPU6050 ile iletişim başlat
  Wire.write(0x43); // Jiroskop verileri için başlangıç adresi (0x43)
  Wire.endTransmission(false); // İletişimi sonlandırmadan veri iste
  Wire.requestFrom(0x68, 4, true); // 4 adet veri iste (X, Y eksenleri)

  // Ham jiroskop verilerini oku
  Gyr_rawX = Wire.read() << 8 | Wire.read(); // X eksenindeki jiroskop verisini oku
  Gyr_rawY = Wire.read() << 8 | Wire.read(); // Y eksenindeki jiroskop verisini oku

  // Jiroskop verilerini derece/saniye cinsine çevir
  Gyro_angle[0] = Gyr_rawX / 131.0; // X açısını derece/saniye cinsine çevir
  Gyro_angle[1] = Gyr_rawY / 131.0; // Y açısını derece/saniye cinsine çevir

  // Toplam açıları hesapla (ağırlıklı ortalama ile)
  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0]; // X toplam açısını hesapla
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1]; // Y toplam açısını hesapla

  // Hata hesaplama (hedef açı ile ölçülen açı arasındaki fark)
  hata = Total_angle[1] - hedef_ac; // Y eksenindeki hata değerini hesapla

  // PID hesaplamaları
  float pid_p = kp * hata; // Proportional bileşeni; hata ile çarpan
  static float pid_i = 0; // Integral bileşeni (sabit değişken); hata toplamını saklar

  // Integral yalnızca hata küçükse etkinleştir
  if (-2 < hata && hata < 2) {
    pid_i += ki * hata; // Hata küçükse, integral bileşenini güncelle
  }

  float pid_d = kd * ((hata - onceki_hata) / elapsedTime); // Derivative bileşeni; hata değişim hızını hesapla

  // PID toplamı
  PID = pid_p + pid_i + pid_d; // Tüm bileşenleri topla

  // PWM değerlerini sınırla (-1000 ile 1000 arasında)
  if (PID < -1000) PID = -1000; // PID değerini -1000 ile sınırlama
  if (PID > 1000) PID = 1000; // PID değerini 1000 ile sınırlama

  // Motor PWM hesaplamaları
  pwmSol = throttle + PID; // Sol motor için PWM değerini hesapla
  pwmSag = throttle - PID; // Sağ motor için PWM değerini hesapla

  // PWM değerlerini sınırla (1000 ile 2000 arasında)
  //Her motor için ayrı sınır değeri gerekebilir!!! (Bi-directional ESC)
  if (pwmSag < 1000) pwmSag = 1000; // Sağ motor PWM değerini 1000 ile sınırlama
  if (pwmSag > 2000) pwmSag = 2000; // Sağ motor PWM değerini 2000 ile sınırlama
  if (pwmSol < 1000) pwmSol = 1000; // Sol motor PWM değerini 1000 ile sınırlama
  if (pwmSol > 2000) pwmSol = 2000; // Sol motor PWM değerini 2000 ile sınırlama

  // Motorları kontrol et
  //writeMicroseconds() kullanmamızın nedeni, motor hızını çok daha hassas/kontrol edilebilir ayarlamak istememiz
  solMotor.writeMicroseconds(pwmSol); // Sol motor için PWM sinyalini gönder
  sagMotor.writeMicroseconds(pwmSag); // Sağ motor için PWM sinyalini gönder

  onceki_hata = hata; // Önceki hatayı güncelle; bir sonraki döngüde kullanılacak
}
