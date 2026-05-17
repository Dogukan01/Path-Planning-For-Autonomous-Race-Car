import matplotlib.pyplot as plt
import numpy as np

def plot_analysis(history_s, history_d):
    """
    Baseline ve Optimal test senaryolarının sonuçlarını yan yana (dashboard şeklinde) çizer.
    """
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    
    lap_time_s = history_s['t'][-1] if len(history_s['t']) > 0 else 0
    lap_time_d = history_d['t'][-1] if len(history_d['t']) > 0 else 0
    
    title = f"Simülasyon Veri Analizi\nBaseline Lap Time: {lap_time_s:.2f}s | Optimal Lap Time: {lap_time_d:.2f}s"
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Verileri hazırla (Radyandan dereceye çevirme vb.)
    t_s = history_s['t']
    cte_s = history_s['cte']
    steer_s = np.degrees(history_s['steer'])
    v_s = history_s['v']
    ld_s = history_s['ld']
    
    t_d = history_d['t']
    cte_d = history_d['cte']
    steer_d = np.degrees(history_d['steer'])
    v_d = history_d['v']
    ld_d = history_d['ld']
    
    # --- Sol Kolon: BASELINE SENARYO ---
    
    # 1. Hız
    axes[0, 0].plot(t_s, v_s, 'r-', linewidth=2)
    axes[0, 0].set_title('Baseline - Araç Hızı')
    axes[0, 0].set_ylabel('Hız [m/s]')
    axes[0, 0].grid(True, linestyle=':', alpha=0.8)
    
    # 2. İleri Bakma Mesafesi
    axes[1, 0].plot(t_s, ld_s, 'r-', linewidth=2)
    axes[1, 0].set_title('Baseline - İleri Bakma Mesafesi (Ld)')
    axes[1, 0].set_ylabel('Ld [m]')
    axes[1, 0].grid(True, linestyle=':', alpha=0.8)
    
    # 3. Yörüngeden Sapma Hatası
    axes[2, 0].plot(t_s, cte_s, 'r-', linewidth=2)
    axes[2, 0].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[2, 0].set_title('Baseline - Yörüngeden Sapma Hatası (CTE)')
    axes[2, 0].set_ylabel('Hata [m]')
    axes[2, 0].grid(True, linestyle=':', alpha=0.8)
    
    # 4. Direksiyon Açısı
    axes[3, 0].plot(t_s, steer_s, 'r-', linewidth=2)
    axes[3, 0].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[3, 0].set_title('Baseline - Direksiyon Açısı')
    axes[3, 0].set_xlabel('Zaman [s]')
    axes[3, 0].set_ylabel('Açı [Derece]')
    axes[3, 0].grid(True, linestyle=':', alpha=0.8)
    
    # --- Sağ Kolon: OPTIMAL SENARYO ---
    
    # 1. Hız
    axes[0, 1].plot(t_d, v_d, 'lime', linewidth=2)
    axes[0, 1].set_title('Optimal - Araç Hızı')
    axes[0, 1].set_ylabel('Hız [m/s]')
    axes[0, 1].grid(True, linestyle=':', alpha=0.8)
    
    # 2. İleri Bakma Mesafesi
    axes[1, 1].plot(t_d, ld_d, 'lime', linewidth=2)
    axes[1, 1].set_title('Optimal - İleri Bakma Mesafesi (Ld)')
    axes[1, 1].set_ylabel('Ld [m]')
    axes[1, 1].grid(True, linestyle=':', alpha=0.8)
    
    # 3. Yörüngeden Sapma Hatası
    axes[2, 1].plot(t_d, cte_d, 'lime', linewidth=2)
    axes[2, 1].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[2, 1].set_title('Optimal - Yörüngeden Sapma Hatası (CTE)')
    axes[2, 1].set_ylabel('Hata [m]')
    axes[2, 1].grid(True, linestyle=':', alpha=0.8)
    
    # 4. Direksiyon Açısı
    axes[3, 1].plot(t_d, steer_d, 'lime', linewidth=2)
    axes[3, 1].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[3, 1].set_title('Optimal - Direksiyon Açısı')
    axes[3, 1].set_xlabel('Zaman [s]')
    axes[3, 1].set_ylabel('Açı [Derece]')
    axes[3, 1].grid(True, linestyle=':', alpha=0.8)
    
    plt.tight_layout()
    # Y eksenlerini aynı skalaya getirelim ki CTE ve Steer kıyaslaması kolay olsun
    max_cte = max(np.max(np.abs(cte_s)), np.max(np.abs(cte_d))) * 1.1
    axes[2, 0].set_ylim(-max_cte, max_cte)
    axes[2, 1].set_ylim(-max_cte, max_cte)
    
    max_steer = max(np.max(np.abs(steer_s)), np.max(np.abs(steer_d))) * 1.1
    axes[3, 0].set_ylim(-max_steer, max_steer)
    axes[3, 1].set_ylim(-max_steer, max_steer)
    
    plt.show()
