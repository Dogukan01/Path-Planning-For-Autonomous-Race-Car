# Path Planning for Autonomous Race Car

Otonom yarış araçları için yol planlama simülasyonu. Kinematik Bisiklet Modeli ve Pure Pursuit algoritması kullanarak aracı pistin merkez çizgisinde takip ettiren bir simülasyon projesi.

## Proje Açıklaması

Bu proje, otonom araçların parkur takip kontrolünü simüle eder. Araç, Pure Pursuit kontrolcüsü kullanarak önceden tanımlanmış bir pistin merkez çizgisini takip eder.

### Temel Bileşenler

- **Araç Modeli** (`car.py`): Kinematik Bisiklet Modeli kullanarak aracın fiziğini simüle eder
- **Kontrolcü** (`controller.py`): Pure Pursuit algoritması ile yörünge takip kontrolü sağlar
- **Pist** (`track.py`): Fıstık şekilli kapalı bir yarış pistini matematiksel olarak modelleyer
- **Simülasyon** (`main.py`): Tüm bileşenleri bir araya getirerek animasyonlu simülasyon çalıştırır
- **Analiz** (`analysis.py`): Simülasyon sonuçlarını görselleştirir

## Gereksinimler

- Python 3.6+
- NumPy
- Matplotlib

## Kurulum

```bash
pip install numpy matplotlib
```

## Kullanım

Simülasyonu çalıştırmak için:

```bash
python main.py
```

Simülasyon şu bilgileri gösterir:
- Aracın pistte hareket eden canlı animasyonu
- Hedef noktası (yeşil X işareti)
- Aracın geçmiş yörüngesi (mavi çizgi)
- Gerçek zamanlı simülasyon parametreleri (hız, hata, direksiyon açısı)

Simülasyon tamamlandıktan sonra, analiz grafiklerini görmek için:

```bash
# main.py çalıştıktan sonra otomatik olarak analysis grafikleri açılır
```

## Simülasyon Parametreleri

[main.py](main.py) dosyasında özelleştirilebilir parametreler:

| Parameter | Değer | Birim | Açıklama |
|-----------|-------|-------|----------|
| DT | 0.05 | s | Zaman adımı |
| TARGET_SPEED | 10.0 | m/s | Aracın hedef hızı |
| MAX_TIME | 100.0 | s | Maksimum simülasyon süresi |
| L | 2.5 | m | Dingil mesafesi (Wheelbase) |
| LD | 6.0 | m | Pure Pursuit ileriye bakma mesafesi |

## Proje Yapısı

```
.
├── README.md           # Bu dosya
├── .gitignore         # Git'ten hariç tutulacak dosyalar
├── main.py            # Ana simülasyon döngüsü
├── car.py             # Araç kinematik modeli
├── controller.py      # Pure Pursuit kontrolcüsü
├── track.py           # Pist modeli
└── analysis.py        # Analiz ve görselleştirme
```

## Algoritma Açıklaması

### Kinematik Bisiklet Modeli

Aracı basit bir bisiklet modeli olarak temsil eder:
- Ön tekerlek tarafından yönlendirilir (δ açısı)
- Arka tekerlekler hızı belirler (v hızı)
- Dingil mesafesi L'ye bağımlı kinemalık denklemler kullanılır

### Pure Pursuit Kontrolcüsü

Path tracking algoritması:
1. Aracın mevcut konumundan ld (look-ahead distance) kadar uzaklıkta bir hedef nokta bulur
2. Aracın o noktaya ulaşması için gerekli direksiyon açısını hesaplar
3. Kinematik kısıtlamaları sağlayan steering komutu üretir

## Çıktılar

Simülasyon aşağıdaki analizleri sunar:

1. **Yörünge Haritası**: Pistin görünümü ve aracın izlediği yol
2. **Cross-Track Error Grafiği**: Zamanla yörüngeden sapma hatasının değişimi
3. **Direksiyon Açısı Grafiği**: Zamanla kontrolcünün komut ettiği direksiyon açısı

## Notlar

- Pistin şekli fıstık (peanut) formülü ile üretilen kapalı bir döngüdür
- Araç hedef hızında sabit olarak hareket eder (hız kontrolü yapılmaz)
- Sim Seçme algoritması, aracın tur tamamlanmasına kadar çalışır