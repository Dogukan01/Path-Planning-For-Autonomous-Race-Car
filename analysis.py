import matplotlib.pyplot as plt
import numpy as np

class LiveAnalysisWindow:
    def __init__(self):
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 12))
        self.fig.suptitle("Canlı Simülasyon Veri Analizi", fontsize=16, fontweight='bold')
        
        self.line_v, = self.axes[0].plot([], [], 'lime', linewidth=2)
        self.axes[0].set_title('Araç Hızı')
        self.axes[0].set_ylabel('Hız [m/s]')
        self.axes[0].grid(True, linestyle=':', alpha=0.8)
        
        self.line_ld, = self.axes[1].plot([], [], 'lime', linewidth=2)
        self.axes[1].set_title('İleri Bakma Mesafesi (Ld)')
        self.axes[1].set_ylabel('Ld [m]')
        self.axes[1].grid(True, linestyle=':', alpha=0.8)
        
        self.line_cte, = self.axes[2].plot([], [], 'lime', linewidth=2)
        self.axes[2].axhline(0, color='k', linestyle='--', alpha=0.5)
        self.axes[2].set_title('Yörüngeden Sapma Hatası (CTE)')
        self.axes[2].set_ylabel('Hata [m]')
        self.axes[2].grid(True, linestyle=':', alpha=0.8)
        
        self.line_steer, = self.axes[3].plot([], [], 'lime', linewidth=2)
        self.axes[3].axhline(0, color='k', linestyle='--', alpha=0.5)
        self.axes[3].set_title('Direksiyon Açısı')
        self.axes[3].set_xlabel('Zaman [s]')
        self.axes[3].set_ylabel('Açı [Derece]')
        self.axes[3].grid(True, linestyle=':', alpha=0.8)
        
        plt.tight_layout()
        self.fig.subplots_adjust(hspace=0.5, top=0.9)  # Yazıların iç içe girmesini önler
        plt.show(block=False)
        self.is_open = True
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def on_close(self, event):
        self.is_open = False

    def update(self, history):
        if not self.is_open:
            return
            
        t = history['t']
        if len(t) == 0:
            return
            
        cte = history['cte']
        steer = np.degrees(history['steer'])
        v = history['v']
        ld = history['ld']
        
        self.line_v.set_data(t, v)
        self.line_ld.set_data(t, ld)
        self.line_cte.set_data(t, cte)
        self.line_steer.set_data(t, steer)
        
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
            
        lap_time = t[-1]
        self.fig.suptitle(f"Canlı Simülasyon Veri Analizi\nGüncel Süre: {lap_time:.2f}s", fontsize=16, fontweight='bold')
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def plot_analysis(history):
    """
    Optimal test senaryosunun sonuçlarını statik olarak çizer.
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
