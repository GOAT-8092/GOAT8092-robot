# FRC Limelight Görüş Sistemi - GOAT8092 Robot

Bu depo, GOAT8092 FRC robotunun Limelight kamera kullanarak geliştirilen görüş sistemi kodunu içerir.

## Limelight Görüş Sistemi Genel Bakış

Limelight, FRC için reflektif bant, AprilTag'ler, oyun parçaları ve diğer nesneleri hedeflemek için görüntü işlemeyi basitleştiren akıllı bir kameradır. Uygulamamız aşağıdaki özellikleri sağlar:

- Hassas hizalama için PID-kontrollü hedefleme
- Görüş verilerinin istikrarını artırmak için gürültü filtreleme
- Farklı oyun durumları için çoklu pipeline desteği
- Hata ayıklama ve görselleştirme için panel entegrasyonu
- Görüş destekli otonom çalışma
- Gelişmiş AprilTag algılama ve tepki sistemi

## Temel İyileştirmeler

Limelight görüş sisteminde aşağıdaki iyileştirmeler yapılmıştır:

1. **PID Kontrolü**: Basit açık/kapalı kontrol yerine, yumuşak ve hassas hedefleme için PID kontrolörleri eklendi
2. **Filtreleme**: Görüş verilerindeki gürültüyü azaltmak için medyan filtreleri eklendi
3. **Çoklu Pipeline**: Farklı Limelight pipeline'ları arasında geçiş için destek
4. **Panel Entegrasyonu**: SmartDashboard'a kapsamlı telemetri veri entegrasyonu
5. **Otonom Desteği**: Görüş tabanlı hizalama ile geliştirilmiş otonom mod
6. **AprilTag Takibi**: Gelişmiş AprilTag algılama yetenekleri
7. **Etiket Tabanlı Tepkiler**: Robot, çeşitli AprilTag kimliklerine farklı tepkiler verebilir

## AprilTag İşlevleri

Son güncelleme, kapsamlı AprilTag desteği ekler:

### Özellikler
- AprilTag'lerin algılanması ve takibi
- AprilTag'lere mesafe ve açı hesaplaması
- Etiket kimliği tanıma ve önceden tanımlanmış tepkiler
- Buton tetiklemeli etiket-özel hareketler
- Etiket seçimi ve durum izleme için SmartDashboard entegrasyonu

### AprilTag Modunu Kullanma
1. **AprilTag Modunu Aç/Kapa**: Normal sürüş modundayken AprilTag algılamayı etkinleştirmek için X düğmesine basın. Limelight, AprilTag pipeline'ına geçiş yapacaktır.
   
2. **Etiket Tepkisini Tetikleme**: Bir AprilTag algılandığında, o etiket türüne özgü bir hareket paterni gerçekleştirmek için A düğmesine basın. Robot, etiket kimliğine göre farklı tepkiler verecektir:
   - Mavi İttifak Etiketleri: 1.0 metre mesafeye hareket et
   - Kırmızı İttifak Etiketleri: 1.5 metre mesafeye hareket et 
   - Sahne Etiketleri: Sadece dönüşsel hizalama
   
3. **Etiket Tepkisini Durdurma**: Mevcut etiket tepkisini herhangi bir anda durdurmak için B düğmesine basın.

4. **Belirli Etiketleri Hedefleme**: SmartDashboard'daki "Target Tag" açılır menüsünü kullanarak belirli bir etiket kimliği seçin. A düğmesine bastığınızda, başka bir etiket şu anda görünür olsa bile robot bu belirli etikete tepki verecektir.

### Otonom AprilTag Davranışı
Otonom mod sırasında robot:
1. Yavaşça dönerek AprilTag'leri tarayacak
2. Bir etiket algılandığında, etikete özgü bir yaklaşma hareketi gerçekleştirecek
3. Mavi etiketlere, kırmızı etiketlere ve sahne etiketlerine farklı tepkiler verecek

## Limelight Sistemini Ayarlama

### Pipeline Yapılandırması (Limelight Web Arayüzü)

En iyi sonuçlar için, Limelight web arayüzünde (http://limelight.local:5801) bu pipeline'ları yapılandırın:

1. **Pipeline 0 (Varsayılan)**: Normal sürücü kamera görünümü
2. **Pipeline 1 (AprilTag)**: AprilTag algılama için optimize edilmiş
3. **Pipeline 2 (Reflektif)**: Reflektif bant için optimize edilmiş

Her pipeline için ayarlama ipuçları:

#### AprilTag Pipeline
- Aile: TagStandard52h13 (FRC için varsayılan)
- Algılama kazancı: 0.2-0.7 (daha fazla hassasiyet için düşük, daha fazla algılama mesafesi için yüksek)
- Küçültme: 1-2 (daha fazla doğruluk için 1, daha fazla hız için 2)
- Bulanıklaştırma: 0-0.5 (zayıf ışık koşullarında yardımcı olur)
- Pose tahmini: Doğru robot boyutları ile etkinleştirin
- Doğru kamera montaj pozisyonunu ve açısını ayarlayın

#### Reflektif Pipeline
- HSV Aralığı: Belirli hedefleriniz için ayarlayın (FRC yeşil reflektif bant için tipik değerler):
  - Ton: 55-100
  - Doygunluk: 150-255
  - Değer: 30-255
- Erozyon ve Genişleme: 1-3 (gürültüyü filtrelemek için yardımcı olur)
- Hedef Alan Filtreleme: Yanlış pozitifleri azaltmak için küçük konturları filtreleyin
- Hedef Gruplama: Eşleştirilmiş hedefleri algılıyorsanız etkinleştirin

### Kod İçinde PID Ayarlaması

PID kontrolörleri, `Limelight.java` dosyasındaki şu sabitler ayarlanarak düzenlenebilir:

```java
// PID Controllers for movement control
private static final PIDController rotationPID = new PIDController(0.03, 0.0, 0.001);
private static final PIDController strafePID = new PIDController(0.03, 0.0, 0.0);
private static final PIDController distancePID = new PIDController(0.04, 0.0, 0.0);
```

Ayarlama ipuçları:
1. Sadece P (orantısal) kontrolü ile başlayın (I ve D'yi sıfıra ayarlayın)
2. Robot hedefin etrafında salınım yapana kadar P'yi artırın
3. Salınımları azaltmak için küçük bir D (türev) değeri ekleyin
4. P'nin gideremediği kalıcı bir sapma varsa, I (integral) ekleyin
5. Hassasiyet ve istikrarı dengelemek için tolerans değerlerini ayarlayın

## Limelight Görüş Takibi için En İyi Uygulamalar

1. **Limelight'ı Sağlam Monte Edin**: Kameranın, takip doğruluğunu etkileyebilecek titreşimi önlemek için sağlam şekilde monte edildiğinden emin olun.

2. **Montaj Konumunu Kalibre Edin**: Limelight web arayüzünde doğru montaj açısını ve yüksekliğini ayarlayın.

3. **LED Modunu Uygun Şekilde Kullanın**:
   - Hedefleme sırasında LED AÇIK
   - Aktif olarak görüş kullanılmadığında LED KAPALI
   - Belirli robot durumları veya hata ayıklama için LED YANIP SÖNME

4. **Pipeline Geçişi**: Mevcut görev için uygun pipeline'a geçiş yapın:
   - Manuel çalışma gerektiğinde sürücü pipeline'ı
   - Otonom veya hedefleme modları sırasında hedef odaklı pipeline'lar

5. **Görüş Verilerini Filtreleyin**: İstikrarı artırmak için ham kamera değerlerini her zaman filtreleyin.

6. **Maç Koşullarında Test Edin**: Aydınlatma koşulları görüş performansını önemli ölçüde etkiler - müsabakaya benzer ortamlarda test edin.

7. **Doğru Kontrolörü Kullanın**:
   - Basit orantısal kontrol, temel uygulamalar için işe yarar
   - PID kontrolü, kritik hedefleme için daha iyi hassasiyet sağlar
   - Sürtünme/ataleti aşmak için bir min_command değeri ekleyin

## Kullanım

### Temel Manuel Mod
Bu modda, sürücü robotun tam kontrolüne sahiptir ve görüş işleme devre dışıdır.

### Görüş Destekli Mod
Görüş takip modunu açıp kapatmak için Y düğmesine basın. Etkinleştirildiğinde:
- Robot otomatik olarak algılanan hedefle hizalanacaktır
- Hizalama sağlandığında kontrolcü titreşim verecektir
- Hedef görünmüyorsa otomatik arama davranışını etkinleştirmek/devre dışı bırakmak için X düğmesine basın

### AprilTag Modu
Manuel moddayken AprilTag modunu açıp kapatmak için X düğmesine basın. Etkinleştirildiğinde:
- Robot AprilTag'leri algılayacak ve takip edecektir
- Mevcut etikete bir tepki yürütmek için A düğmesine basın
- Bir etiket tepkisini iptal etmek için B düğmesine basın
- SmartDashboard açılır menüsünden belirli etiket kimliklerini seçin

### Otonom Mod
Otonom mod, konumlandırma için AprilTag'leri ve hizalama için reflektif hedefleme kullanır.

## PID Ayarlama Süreci

Robotun takip davranışının ayarlanması gerekiyorsa:

1. **Robot salınım yapıyorsa** (yerleşmeden ileri geri hareket ediyorsa):
   - PID denetleyicisinde P değerini azaltın
   - Salınımları azaltmak için D değerini artırın

2. **Robot hedefe yavaş ulaşıyorsa**:
   - P değerini artırın
   - Maksimum çıkışın çok kısıtlayıcı olmadığından emin olun

3. **Robot hedeflere tam olarak merkezlenemiyorsa**:
   - Sabit durum hatasını gidermek için küçük bir I değeri ekleyin
   - Hedefleme toleransınızın uygun olduğundan emin olun

## Sorun Giderme

Yaygın sorunlar ve çözümleri:

1. **Hedef algılanmıyor**:
   - LED'lerin açık olduğunu kontrol edin
   - Pipeline ayarlarının hedefinizle eşleştiğini doğrulayın
   - HSV eşik değerlerini ayarlayın

2. **Düzensiz takip davranışı**:
   - Aydınlatmadaki hızlı değişimleri kontrol edin
   - Gürültüyü azaltmak için medyan filtre boyutunu artırın
   - PID değerlerinin uygun olduğunu doğrulayın

3. **Gecikmeli tepki**:
   - Limelight'a ağ gecikmesini kontrol edin
   - Robot kodunun verimli çalıştığından emin olun
   - Pipeline'ı basitleştirerek Limelight üzerindeki işleme yükünü azaltmayı düşünün

4. **AprilTag algılama sorunları**:
   - Uygun aydınlatma sağlayın (çok parlak veya çok karanlık olmamalı)
   - Etiketlerin doğru ailede olduğunu doğrulayın (TagStandard52h13)
   - Etiket boyutu ve mesafesinin algılama aralığında olduğunu kontrol edin
   - Etiketlerin hasar görmediğinden veya kısmen engellenmediğinden emin olun

## Kontrolcü Düğme Kılavuzu

Xbox kontrolcüsü, robotun işlevlerini kontrol etmek için kullanılır. İşte düğme yapılandırması:

### Ana Sürüş Kontrolleri
- **Sol Çubuk**: İleri/geri hareket (Y ekseni) ve sola/sağa kayma (X ekseni)
- **Sağ Çubuk**: Robotun dönüşü (X ekseni)
- **Sol ve Sağ Tetikler**: Hız kontrolü (daha sert basış = daha hızlı hareket)

### Mod Düğmeleri
- **Y Düğmesi**: Görüş takip modunu aç/kapat
  - LED'ler otomatik olarak açılır
  - Robot algılanan hedeflere doğru otomatik olarak hizalanır
  - Hedef algılandığında kontrolcü titreşim verir
  
- **X Düğmesi**: AprilTag modunu aç/kapat
  - Limelight, AprilTag pipeline'ına geçiş yapar
  - Şu anki AprilTag kimliği ve mesafesi SmartDashboard'da gösterilir
  - Bu, A düğmesi ile etiket tepkilerini tetiklemek için gereklidir

### AprilTag Düğmeleri
- **A Düğmesi**: AprilTag tepkisini tetikle
  - Robot, algılanan etikete (veya SmartDashboard'da seçilen etikete) tepki verir
  - Mavi etiketler: 1.0 metreye sürüş
  - Kırmızı etiketler: 1.5 metreye sürüş
  - Sahne etiketleri: Sadece dönüşsel hizalama
  
- **B Düğmesi**: Etkin AprilTag tepkisini durdur
  - Mevcut etiket tepkisini herhangi bir anda iptal eder
  - Robot normal kontrolcü girişlerini kabul etmeye geri döner

### Diğer Kontroller
- **Tampon Düğmeleri (Bumpers)**: 
  - **Sol Tampon**: İnce ayar modu (daha hassas kontroller)
  - **Sağ Tampon**: Hızlı mod (daha hızlı hareket)

- **Back/Select Düğmesi**: Limelight LED'leri aç/kapat
- **Start Düğmesi**: Mevcut robotik durum verilerini yeniden başlat

- **POV Pad (D-Pad)**:
  - **Yukarı/Aşağı**: İkincil aktüatör kontrolü
  - **Sol/Sağ**: İlave fonksiyonlar için ayarlanabilir

### SmartDashboard Etkileşimi
Kontrolcü ile birlikte, SmartDashboard'daki "Target Tag" açılır menüsünü kullanarak belirli etiket kimliklerini hedefleyebilirsiniz. Bu, A düğmesi ile tepki tetiklendiğinde mevcut görünür etiketlere bakılmaksızın robotun belirli bir etikete tepki vermesini sağlar.

## Simülasyon ve Test

Robot fiziksel olarak mevcut olmadığında bile geliştirme ve test yapabilmeniz için birkaç seçenek bulunmaktadır:

### WPILib Simülasyon Araçları

WPILib, kod geliştirme ve test için güçlü simülasyon araçları sunar:

1. **Robot Simülatörü**:
   - WPILib'in entegre robot simülatörü, kodunuzu fiziksel robot olmadan test etmenizi sağlar
   - Komut `./gradlew simulateJava` ile simülatör başlatılabilir
   - Joystick girişleri ve motor çıkışları sanal olarak izlenebilir

2. **Görselleştirme Araçları**:
   - Robot sürücü istasyonu (Driver Station) simülatörü, kontrolcü girişlerini taklit etmenizi sağlar
   - Field2d widget, robotun saha üzerindeki simüle edilmiş konumunu görselleştirir
   - SmartDashboard, gerçek robotta olduğu gibi telemetri verileri görüntüler

### Limelight Simülasyonu

Limelight kamerasını simüle etmek için:

1. **PhotonVision Simülasyonu**:
   - PhotonVision, Limelight benzeri görüş fonksiyonlarını simüle etmek için kullanılabilir
   - Sanal AprilTag görüntüleri oluşturabilir ve bu etiketleri algılamanızı simüle edebilir
   - GitHub: [https://github.com/PhotonVision/photonvision](https://github.com/PhotonVision/photonvision)

2. **LLSimulator Tool**:
   - Limelight resmi simülasyon aracı, NetworkTables üzerinden gerçek Limelight davranışını taklit eder
   - Yapılandırması: [https://docs.limelightvision.io/en/latest/simulator_config.html](https://docs.limelightvision.io/en/latest/simulator_config.html)
   - Python3 ile çalıştırılabilir

### Simülasyondan Gerçek Robota Geçiş

Simülasyonda kodunuzu test ettikten sonra, gerçek robota geçerken şunlara dikkat edin:

1. **NetworkTables Yapılandırması**:
   - Simülasyon ortamında genellikle "localhost" kullanılırken, gerçek robotta IP adresi gereklidir
   - `NetworkTableInstance.getDefault().setServerTeam(8092);` robot kodunuzda olduğundan emin olun

2. **Fiziksel Kalibrasyonlar**:
   - Simülasyonda test edilmiş PID değerleri, gerçek robotta ince ayar gerektirebilir
   - Kamera montaj açıları ve yüksekliği gerçek robotla eşleşmelidir

3. **Test Modu**:
   - Tam otonom modlarını test etmeden önce, test modunda tek bileşenleri kontrol edin
   - `./gradlew testJava` ile birim testlerini çalıştırın

### Simülasyon Ortamını Kurma

Simülasyon ortamını kurmak için:

1. WPILib VSCode uzantısını yükleyin
2. WPILib örnek simülasyon projelerini referans alın
3. NetworkTables ve SmartDashboard'ın doğru çalıştığını kontrol edin
4. `src/main/java/frc/robot/simulation` klasörü altında simülasyon özel kodları oluşturun

Bu simülasyon araçları, gerçek robota erişmeden bile görüş sisteminizi geliştirmeye ve test etmeye devam etmenizi sağlar. 