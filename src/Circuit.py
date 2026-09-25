# %%
import fastf1
import ipywidgets as widgets
from IPython.display import clear_output, display
import numpy as np
import scipy.interpolate
import pandas as pd
import plotly.graph_objects as go
import logging

# =============================================
#       Standarize internal documentation
#==============================================
mapping = {
    'Zandvoort': 'Zandvoort',
    'Zuzuka': 'Suzuka',
    'Shanghai': 'Shanghai',
    'Cota': 'Austin',
    'Interlagos': 'São Paulo',
    'Yeda': 'Jeddah',
    'Hermanos rodriguez': 'Mexico City',
    'Circuit de Spa-Francorchamps': 'Spa-Francorchamps',
    'Autodromo Nazionale di Monza': 'Monza',
    'Albert Park Circuit': 'Melbourne',
    'Bahrain International Circuit': 'Sakhir',
    'Baku City Circuit': 'Baku',
    'Circuit de Barcelona-Cataluña': 'Barcelona',
    'Circuit de Monaco': 'Monaco',
    'Circuit Gilles-Villeneuve': 'Montréal', # FastF1 exige la tilde
    'Circuit Paul Ricard': 'Le Castellet',
    'Red Bull Ring': 'Spielberg',
    'Silverstone Circuit': 'Silverstone',
    'Hockenheimring': 'Hockenheim',
    'Hungaroring': 'Budapest',
    'Marina Bay Street Circuit': 'Marina Bay',
    'Sochi Autodrom': 'Sochi',
    'Yas Marina Circuit': 'Yas Island',
    'Autódromo Internacional del Algarve / Portimão': 'Portimão',
    'Autodromo Internazionale Enzo e Dino Ferrari / Imola': 'Imola',
    'Mugello Circuit': 'Mugello',
    'Losail International Circuit': 'Lusail', # Qatar
    'Miami International Autodrome': 'Miami',
    'Las Vegas Strip Circuit': 'Las Vegas'
}

# 1. Cargamos tu archivo actual
archivo_csv = '../data/all_track_curve_camber.csv'
df = pd.read_csv(archivo_csv)

# 2. Aplicamos la traducción. Si un circuito no está en el dict, deja el original.
df['Circuito'] = df['Circuito'].map(mapping).fillna(df['Circuito'])

# 3. Sobrescribimos el archivo con los nombres limpios
df.to_csv(archivo_csv, index=False)

# Silenciamos el spam de FastF1
fastf1.set_log_level('ERROR')

# ==========================================
# 1. INTERFAZ GRÁFICA (WIDGETS)
# ==========================================
dropdown_year = widgets.Dropdown(options=list(range(2020, 2026)), value=2025, description='Year:')
dropdown_track = widgets.Dropdown(options=['Press "Load Tracks" ->'], description='Track:', layout=widgets.Layout(width='300px'))

button_load_tracks = widgets.Button(description='Load Tracks', button_style='info')
button_run = widgets.Button(description='Process and Show Track', button_style='success')
output_log = widgets.Output()

def load_tracks(b):
    with output_log:
        clear_output()
        print(f"Downloading {dropdown_year.value} calendar...")
        try:
            schedule = fastf1.get_event_schedule(dropdown_year.value)
            races = schedule[schedule['RoundNumber'] > 0]
            # Usamos 'Location' (ej: 'Silverstone') para que coincida con tu CSV
            dropdown_track.options = races['Location'].tolist()
            clear_output()
            print(f"¡{dropdown_year.value} calendar loaded! Select a track.")
        except Exception as e:
            print(f"Error de conexión: {e}")

button_load_tracks.on_click(load_tracks)

# ==========================================
# 2. MOTOR GEOMÉTRICO Y GRAFICADOR 3D
# ==========================================
def process_and_plot_3d(year, track_location, csv_path=archivo_csv):
    # 1. Carga de datos
    session = fastf1.get_session(year, track_location, 'Q')
    session.load(telemetry=True, weather=False, messages=False)
    
    lap = session.laps.pick_fastest()
    pos = lap.get_pos_data()
    info = session.get_circuit_info()
    s_apex = info.corners.loc[:, 'Distance']
    
    # 2. Limpieza de Data (NaNs y GPS congelado)
    pos = pos.dropna(subset=['X', 'Y', 'Z'])
    X_raw = pos.loc[:, 'X'].values / 10.0
    Y_raw = pos.loc[:, 'Y'].values / 10.0
    Z_raw = pos.loc[:, 'Z'].values / 10.0
    
    dx_raw = np.diff(X_raw, prepend=X_raw[0] - 1)
    dy_raw = np.diff(Y_raw, prepend=Y_raw[0] - 1)
    mask_movimiento = (dx_raw**2 + dy_raw**2) > 0
    
    X = X_raw[mask_movimiento]
    Y = Y_raw[mask_movimiento]
    Z = Z_raw[mask_movimiento]

    # 3. Splines Cúbicos 3D
    tck, u = scipy.interpolate.splprep([X, Y, Z], s=1000)
    x, y, z = scipy.interpolate.splev(u, tck)

    # 4. Longitud de arco
    ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s_array = np.concatenate(([0], np.cumsum(ds)))

    # 5. Detección Arco/Cuerda (Solo X/Y)
    window_m = 25.0 
    arc_chord_ratio = np.ones(len(s_array), dtype=float)

    for i in range(len(s_array)):
        s_current = s_array[i]
        idx_rear = np.argmin(np.abs(s_array - (s_current - window_m)))
        idx_fwd = np.argmin(np.abs(s_array - (s_current + window_m)))
        
        arc = s_array[idx_fwd] - s_array[idx_rear]
        dx_c = x[idx_fwd] - x[idx_rear]
        dy_c = y[idx_fwd] - y[idx_rear]
        chord = np.sqrt(dx_c**2 + dy_c**2)
        
        if chord > 0:
            arc_chord_ratio[i] = arc / chord

    # 6. Fronteras Territoriales de las Curvas
    threshold = 1.0002
    start_indices = []
    end_indices = []

    for i in range(len(s_apex)):
        current_apex = s_apex.iloc[i]
        idx_apex = np.argmin(np.abs(s_array - current_apex))
        
        border_rear = (current_apex + s_apex.iloc[i-1]) / 2.0 if i > 0 else 0.0
        border_fwd = (current_apex + s_apex.iloc[i+1]) / 2.0 if i < len(s_apex)-1 else s_array[-1]
        
        idx_back = idx_apex
        while idx_back > 0 and arc_chord_ratio[idx_back] > threshold and s_array[idx_back] > border_rear:
            idx_back -= 1
            
        idx_forw = idx_apex
        while idx_forw < len(s_array)-1 and arc_chord_ratio[idx_forw] > threshold and s_array[idx_forw] < border_fwd:
            idx_forw += 1
            
        start_indices.append(idx_back)
        end_indices.append(idx_forw)

    start_distances_m = s_array[start_indices]
    end_distances_m = s_array[end_indices]

    # 7. Merge Inteligente con el CSV
    try:
        df = pd.read_csv(csv_path)
        # Filtramos ignorando mayúsculas/minúsculas
        track_mask = df['Circuito'].str.lower() == track_location.lower()
        
        if not track_mask.any():
            print(f"⚠️ ADVERTENCIA: No se encontró la pista '{track_location}' en la columna 'Circuito' del CSV.")
        else:
            modificadas = 0
            for i in range(len(start_distances_m)):
                corner_name = f'T{i+1}'
                row_mask = track_mask & (df['Curva'] == corner_name)
                
                if row_mask.any():
                    df.loc[row_mask, 'Distancia_Inicio_m'] = round(start_distances_m[i], 2)
                    df.loc[row_mask, 'Distancia_Fin_m'] = round(end_distances_m[i], 2)
                    modificadas += 1
                    
            df.to_csv(csv_path, index=False)
          
    except FileNotFoundError:
        print(f"❌ Error: Archivo {csv_path} no encontrado en este directorio.")

    # 8. Visualización 3D Interactiva con Plotly
    # Rectas en gris
    track_trace = go.Scatter3d(
        x=x, y=y, z=z, 
        mode='lines', line=dict(color='gray', width=3), name='Recta'
    )
    plot_data = [track_trace]

    fig = go.Figure(data=plot_data)

    # --- CÁLCULO DE CÁMARA DINÁMICA ---
    rango_x = np.max(x) - np.min(x)
    rango_y = np.max(y) - np.min(y)
    
    # Calculamos qué tan "estirada" es la pista
    elongacion = max(rango_x, rango_y) / min(rango_x, rango_y)
    
    # Zoom base + penalización por elongación
    # Mientras más alargada sea la pista, mayor será el factor de alejamiento
    distancia_base = 1.3
    zoom = distancia_base + (elongacion * 0.25)
    
    fig.update_layout(
        title=f'Análisis Geométrico 3D - {track_location} ({year})',
        scene=dict(
            aspectmode='data', 
            xaxis_title='X (m)', yaxis_title='Y (m)', zaxis_title='Z (m)',
            camera=dict(
                # x e y positivos idénticos dan el ángulo en diagonal (isométrico)
                # z ligeramente menor lo inclina para no verlo plano desde arriba
                eye=dict(x=zoom, y=zoom, z=zoom * 0.6) 
            )
        ) 
    )
    fig.show()

# ==========================================
# 3. LÓGICA DE EJECUCIÓN
# ==========================================
def on_button_clicked(b):
    with output_log:
        clear_output()
        year = dropdown_year.value
        track = dropdown_track.value
        if track == 'Click "Load Tracks" ->':
            print("Please load the calendar first.")
            return
            
        print(f"Extracting {track} ({year}) data...")
        process_and_plot_3d(year, track)

button_run.on_click(on_button_clicked)

# Renderizamos los widgets (HBox para agrupar en línea horizontal)
fila_superior = widgets.HBox([dropdown_year, button_load_tracks])
ui = widgets.VBox([fila_superior, dropdown_track, button_run, output_log])
ui
# %%
