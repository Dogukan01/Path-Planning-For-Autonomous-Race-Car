import matplotlib.pyplot as plt
import numpy as np

def plot_analysis(history):
    """
    Simülasyon verilerinden yörünge sapma hatası ve direksiyon açısı grafiklerini çizer.
    """
    t = history['t']
    cte = history['cte']
    # Analizin daha okunabilir olması için radyanı dereceye çeviriyoruz
    steer_deg = np.degrees(history['steer'])
    
    # 2 satır, 1 sütun şeklinde alt alta iki grafik oluştur
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # --- 1. Cross-Track Error Grafiği ---
    ax1.plot(t, cte, 'r-', linewidth=2)
    ax1.axhline(y=0, color='black', linestyle='--', linewidth=1.5, alpha=0.7)
    ax1.set_title('Zaman Ekseninde Yörüngeden Sapma Hatası (Cross-Track Error vs Time)', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Zaman [s]', fontsize=12)
    ax1.set_ylabel('Hata (CTE) [m]', fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.8)
    
    # --- 2. Direksiyon Açısı Grafiği ---
    ax2.plot(t, steer_deg, 'b-', linewidth=2)
    ax2.axhline(y=0, color='black', linestyle='--', linewidth=1.5, alpha=0.7)
    ax2.set_title('Zaman Ekseninde Direksiyon Açısı Değişimi (Steering Angle vs Time)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Zaman [s]', fontsize=12)
    ax2.set_ylabel('Direksiyon Açısı [Derece]', fontsize=12)
    ax2.grid(True, linestyle=':', alpha=0.8)
    
    plt.tight_layout()
    plt.show()
