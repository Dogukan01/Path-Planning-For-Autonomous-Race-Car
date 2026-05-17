import matplotlib.pyplot as plt
import numpy as np

def plot_analysis(history):
    """
    Optimal test senaryosunun sonuçlarını çizer.
    """
    fig, axes = plt.subplots(4, 1, figsize=(10, 12))
    
    lap_time = history['t'][-1] if len(history['t']) > 0 else 0
    
    title = f"Simülasyon Veri Analizi\nOptimal Lap Time: {lap_time:.2f}s"
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Verileri hazırla (Radyandan dereceye çevirme vb.)
    t = history['t']
    cte = history['cte']
    steer = np.degrees(history['steer'])
    v = history['v']
    ld = history['ld']
    
    # 1. Hız
    axes[0].plot(t, v, 'lime', linewidth=2)
    axes[0].set_title('Optimal - Araç Hızı')
    axes[0].set_ylabel('Hız [m/s]')
    axes[0].grid(True, linestyle=':', alpha=0.8)
    
    # 2. İleri Bakma Mesafesi
    axes[1].plot(t, ld, 'lime', linewidth=2)
    axes[1].set_title('Optimal - İleri Bakma Mesafesi (Ld)')
    axes[1].set_ylabel('Ld [m]')
    axes[1].grid(True, linestyle=':', alpha=0.8)
    
    # 3. Yörüngeden Sapma Hatası
    axes[2].plot(t, cte, 'lime', linewidth=2)
    axes[2].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[2].set_title('Optimal - Yörüngeden Sapma Hatası (CTE)')
    axes[2].set_ylabel('Hata [m]')
    axes[2].grid(True, linestyle=':', alpha=0.8)
    
    # 4. Direksiyon Açısı
    axes[3].plot(t, steer, 'lime', linewidth=2)
    axes[3].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[3].set_title('Optimal - Direksiyon Açısı')
    axes[3].set_xlabel('Zaman [s]')
    axes[3].set_ylabel('Açı [Derece]')
    axes[3].grid(True, linestyle=':', alpha=0.8)
    
    plt.tight_layout()
    plt.show()
