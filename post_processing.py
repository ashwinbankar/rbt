# post_processing.py
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class ReportPlotter:
    def __init__(self):
        self.COLOR_RAW = '#483D8B'   
        self.COLOR_FILT = '#FF4500'  
        self.WINDOW_SECONDS = 1.0    
        # Default limits, but we will adjust dynamically
        self.DEFAULT_Y_MIN = -1
        self.DEFAULT_Y_MAX = 11
        
    def _apply_convolution_filter(self, data, window_size):
        if window_size < 3: return data
        kernel = np.ones(window_size) / window_size
        return np.convolve(data, kernel, mode='same')

    def _get_stats_label(self, name, data):
        if len(data) == 0: return f"{name}\nNo Data"
        s_max = np.max(data)
        s_min = np.min(data)
        s_avg = np.mean(data)
        s_std = np.std(data)
        return (f"{name}\nMax: {s_max:.3f}\nMin: {s_min:.3f}\nAvg: {s_avg:.3f}\nStd: {s_std:.3f}")

    def generate_plot(self, csv_path, output_png_path, title_text):
        if not os.path.exists(csv_path):
            print(f"Error: CSV file not found at {csv_path}")
            return

        print(f"DEBUG: Processing CSV: {csv_path}")

        try:
            # 1. Scan file physically to find the "Timestamp" line
            with open(csv_path, 'r', encoding='utf-8-sig') as f:
                lines = f.readlines()
            
            header_row_index = -1
            for i, line in enumerate(lines):
                if "Timestamp" in line and "Torque" in line:
                    header_row_index = i
                    break
            
            if header_row_index == -1:
                print("Error: Could not find 'Timestamp' header in CSV.")
                return

            # 2. Load CSV using skiprows
            df = pd.read_csv(csv_path, skiprows=header_row_index, encoding='utf-8-sig')
            df.columns = df.columns.str.strip()
            
        except Exception as e:
            print(f"Error reading CSV structure: {e}")
            return

        # 3. Smart Column Identification
        cols_lower = [c.lower() for c in df.columns]

        def find_col(substring):
            for idx, c_name in enumerate(cols_lower):
                if substring in c_name:
                    return df.columns[idx]
            return None

        time_col = find_col("time")       
        torque_col = find_col("torque")   
        angle_col = find_col("angle")    

        if not time_col or not torque_col:
            print(f"Error: Missing essential columns. Found: {df.columns.tolist()}")
            return

        # 4. Prepare Data
        if angle_col:
            x_data = df[angle_col].values
            x_label = angle_col
        else:
            x_data = df[time_col].values
            x_label = "Time (s)"
            
        raw_torque = df[torque_col].values

        # --- DYNAMIC Y-AXIS SCALING LOGIC ---
        # Calculate max torque to determine scale
        max_val_found = np.max(raw_torque) if len(raw_torque) > 0 else 0
        
        # If max value is greater than our default (11), expand to (max + 1)
        if max_val_found > self.DEFAULT_Y_MAX:
            y_upper = max_val_found + 1
        else:
            y_upper = self.DEFAULT_Y_MAX
            
        y_limits = (self.DEFAULT_Y_MIN, y_upper)
        # ------------------------------------

        # 5. Filtering Logic
        t_vals = df[time_col].values
        try:
            dt = np.mean(np.diff(t_vals)) if len(t_vals) > 1 else 0.01
            fs = 1 / dt if dt > 0 else 100
        except:
            fs = 100 

        window_size = int(fs * self.WINDOW_SECONDS)
        if window_size < 3: window_size = 3
        filtered_torque = self._apply_convolution_filter(raw_torque, window_size)

        # 6. Stats & Trimming
        cut_idx = int(len(raw_torque) * 0.1)
        if cut_idx == 0: cut_idx = 1
        
        if len(raw_torque) > 2 * cut_idx:
            raw_valid = raw_torque[cut_idx:-cut_idx]
            filt_valid = filtered_torque[cut_idx:-cut_idx]
        else:
            raw_valid = raw_torque
            filt_valid = filtered_torque

        # 7. Plotting
        try:
            fig, ax = plt.subplots(figsize=(20, 7))
            
            ax.plot(x_data, raw_torque, color=self.COLOR_RAW, label='Raw Data', alpha=0.3, linewidth=3)
            ax.plot(x_data, filtered_torque, color=self.COLOR_FILT, label='Filtered Data', linewidth=3)

            s_raw_avg = np.mean(raw_valid)
            s_filt_avg = np.mean(filt_valid)

            ax.axhline(s_raw_avg, color=self.COLOR_RAW, linestyle='--', linewidth=1.5, alpha=0.5)
            ax.axhline(s_filt_avg, color=self.COLOR_FILT, linestyle='--', linewidth=2, alpha=0.9)

            # Formatting
            ax.set_ylim(y_limits) # Use dynamic limits
            ax.set_xlim(x_data[0], x_data[-1])
            for axis in ['top','bottom','left','right']: ax.spines[axis].set_linewidth(2)
            ax.tick_params(width=2, labelsize=12)
            plt.xticks(fontweight='bold')
            plt.yticks(fontweight='bold')
            ax.set_xlabel(x_label, fontsize=14, fontweight='bold')
            ax.set_ylabel("Torque (Nm)", fontsize=14, fontweight='bold')
            ax.set_title(title_text, fontsize=16, fontweight='bold')

            # Legends
            proxy_filt = Line2D([0], [0], color=self.COLOR_FILT, lw=3)
            label_filt = self._get_stats_label("FILTERED", filt_valid)
            leg_filt = ax.legend([proxy_filt], [label_filt], loc='upper right', bbox_to_anchor=(1, 1),
                               frameon=True, fontsize=12, facecolor='white', framealpha=1, edgecolor='black', shadow=True)
            ax.add_artist(leg_filt)

            proxy_raw = Line2D([0], [0], color=self.COLOR_RAW, lw=3, alpha=0.5)
            label_raw = self._get_stats_label("RAW", raw_valid)
            ax.legend([proxy_raw], [label_raw], loc='upper right', bbox_to_anchor=(0.91, 1),
                      frameon=True, fontsize=12, facecolor='white', framealpha=1, edgecolor='black', shadow=True)

            ax.grid(True, which='both', linestyle='--', alpha=0.5)
            plt.tight_layout()
            
            plt.savefig(output_png_path, dpi=150, bbox_inches='tight')
            print(f"Report plot saved: {output_png_path}")
            
        except Exception as e:
            print(f"Error during plotting: {e}")
        finally:
            plt.close(fig)