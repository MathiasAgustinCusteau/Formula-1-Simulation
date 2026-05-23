# %% Importado de librerias
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IPython.display import HTML
import pandas as pd



#%% Definicion de datos y funciones

# --- PARÁMETROS DEL AUTO --- #
m = 800.0        # [kg] Masa del auto
A = 1.5          # [m^2] Área frontal
Mapa_Cd = {False: 0.9, True: 0.65}  # [Adimensional] Coeficiente de drag aerodinámico
Mapa_Cl = {False: -3.0, True: -2.4} # [Adimensional] Coeficiente de lift (Negativo = Downforce)
mu_s = 1.5       # [Adimensional] Coeficiente de fricción estática
C_rr = 0.015     # [Adimensional] Coeficiente de resistencia a la rodadura
PotenciaMotor = 1000 # [CV] la potencia del motor
P = PotenciaMotor * 735.5 # [W] Potencia en watts

# --- PARÁMETROS DEL ENTORNO --- #
g = 9.8067       # [m/s^2] Aceleración de la gravedad
rho = 1.225      # [kg/m^3] Densidad del aire a nivel del mar
dt = 0.01        # [s] Paso de tiempo del integrador

# --- REGLAS --- #
margen_seguridad = 15 # [m] Distancia antes del frenado para estabilizar el auto

def aero_map(estado_drs):
    return Mapa_Cd[estado_drs], Mapa_Cl[estado_drs]

def accion_drs(s,margen_seguridad,drs_array,v_target,v_mod, longitud_total):
    drs_legal = drs_array[s]

    if s + margen_seguridad < longitud_total :
        drs_frenada = v_target[s+margen_seguridad] < v_mod
    else:
        drs_frenada = False

    if drs_legal and not drs_frenada:
        estado_drs = True
    else:
        estado_drs = False

    return estado_drs
def piloto_virtual(v_target, v_mod, dt, m, fza_drag, fza_rodadura, fza_traccion_max, fza_freno_max, indice):
    a_req = (v_target[indice] - v_mod) / dt
    fza_lon_req = (m * a_req) + fza_drag + fza_rodadura

    pedal_accel = 0.0
    pedal_freno = 0.0

    if fza_lon_req > 0:
        pedal_accel = min(fza_lon_req / fza_traccion_max, 1.0)
    elif fza_lon_req < 0:
        pedal_freno = min(abs(fza_lon_req) / fza_freno_max, 1.0) if fza_freno_max > 0 else 0

    return pedal_accel, pedal_freno
# %% Mapeado de circuito

# --- 1. DEFINICIÓN DEL CIRCUITO ---
# dir: 0 (recta), 1 (izquierda), -1 (derecha)
# Usamos un radio gigante (1e9) para las rectas, así evitamos la división por cero
circuito = [
    {"tipo": "recta", "longitud": 600.0, "radio": 1e9, "dir": 0, "drs": True, "drs_offset": 150.0},
    {"tipo": "horquilla", "longitud": 157.08, "radio": 50.0, "dir": 1, "drs": False},  # Curva de 180° (pi * R) a la izq
    {"tipo": "recta", "longitud": 400.0, "radio": 1e9, "dir": 0, "drs": True, "drs_offset": 50.0},
    {"tipo": "curva_rapida", "longitud": 157.08, "radio": 100.0, "dir": -1, "drs": False}, # Curva de 90° (pi/2 * R) a la der
    {"tipo": "recta", "longitud": 300.0, "radio": 1e9, "dir": 0, "drs": True, "drs_offset": 50.0}
]

# --- 2. PRE-PROCESAMIENTO GEOMÉTRICO ---
ds = 1.0  # Resolución: vamos a calcular la pista cada 1 metro
longitud_total = sum([tramo["longitud"] for tramo in circuito])

# Creamos el array de distancia (s) desde 0 hasta el final
s_array = np.arange(0, longitud_total, ds)

# Preparamos vectores vacíos para guardar la información metro a metro
x_array = np.zeros_like(s_array)
y_array = np.zeros_like(s_array)
theta_array = np.zeros_like(s_array)
radio_array = np.zeros_like(s_array)
dir_array = np.zeros_like(s_array)
drs_array = np.zeros_like(s_array)

# Función simple para saber en qué tramo estamos parados
def obtener_geometria(s_actual, circuito):
    s_acumulado = 0.0
    for tramo in circuito:
        s_fin = s_acumulado + tramo["longitud"]
        
        # Si estamos dentro de este tramo
        if s_actual >= s_acumulado and s_actual < s_fin:
            # Calculamos la distancia local dentro del tramo
            s_local = s_actual - s_acumulado
            
            # Verificamos si el DRS es legal en este metro exacto
            drs_legal = False
            if tramo.get("drs_habilitado") == True:
                if s_local >= tramo["drs_offset"]:
                    drs_legal = True
                    
            return tramo["radio"], tramo["dir"], drs_legal
            
        s_acumulado = s_fin
        
    return circuito[-1]["radio"], circuito[-1]["dir"], False

# --- 3. CONSTRUCCIÓN DEL MAPA (Integración Espacial) ---
# Empezamos en el origen (0,0) apuntando hacia el eje X positivo (theta = 0)
x, y = 0.0, 0.0
theta = 0.0 

for i, s in enumerate(s_array):
    # Leemos qué pide el circuito en este metro exacto
    R, dir_curva, estado_drs = obtener_geometria(s, circuito)
    
    # Guardamos los datos puros (los vamos a necesitar después para la aerodinámica)
    radio_array[i] = R
    dir_array[i] = dir_curva
    drs_array[i] = estado_drs
    
    # Calculamos cuánto gira la trompa del auto
    d_theta = (ds / R) * dir_curva
    theta += d_theta
    
    # Proyectamos el avance usando trigonometría básica
    x += ds * np.cos(theta)
    y += ds * np.sin(theta)
    
    # Guardamos las coordenadas listas para plotear
    x_array[i] = x
    y_array[i] = y
    theta_array[i] = theta

# --- 4. RENDERIZADO VISUAL ---
plt.figure(figsize=(10, 8))

# Dibujamos la pista
plt.plot(x_array, y_array, color='black', linewidth=4, label='Trazado')

# Marcamos largada y llegada
plt.plot(x_array[0], y_array[0], 'go', markersize=10, label='Largada (s=0)')
plt.plot(x_array[-1], y_array[-1], 'ro', markersize=10, label='Llegada')

plt.title("Generador de Geometría - Pista de Pruebas F1")
plt.xlabel("Posición X (m)")
plt.ylabel("Posición Y (m)")

# El "axis equal" es vital en simulaciones físicas para que las curvas 
# no se aplasten si la ventana es rectangular.
plt.axis('equal') 
plt.grid(True, alpha=0.4)
plt.legend()
plt.show()

# %%  Mapeado de velocidad objetivo
# --- PARÁMETROS DEL AUTO (Si no los tenés definidos arriba, descomentá esto) ---
# m = 800.0; g = 9.8067; mu_s = 1.5; rho = 1.225; A = 1.5; C_rr = 0.015
# Cl_abierto = -2.4; Cd_abierto = 0.65; Cl_cerrado = -3.0; Cd_cerrado = 0.9

v_max_recta = 380.0 / 3.6  # [m/s] Tope máximo por motor/relación de caja

# --- PASADA 1: LÍMITE DE GRIP (Fuerzas Laterales) ---
# Creamos un array vacío para guardar la velocidad máxima teórica de cada punto
v_target_adherencia = np.zeros_like(s_array)

for i in range(len(s_array)):
    R = radio_array[i]
    
    # Si es una recta (radio gigante), la velocidad máxima la pone el motor
    if R > 10000:
        v_target_adherencia[i] = v_max_recta
    else:
        # Ecuación de V_apex (Despejada de: m*v^2/R = mu_s * (m*g + 0.5*rho*A*Cl*v^2))
        Cl_mag = abs(-3.0) # Asumimos DRS cerrado en curva pesada para máximo grip
        numerador = m * g * mu_s
        denominador = (m / R) - (0.5 * rho * A * Cl_mag * mu_s)
        
        if denominador > 0:
            # Margen de seguridad del 2% para no ir exactamente al 100% del límite
            v_target_adherencia[i] = np.sqrt(numerador / denominador) * 0.98 
        else:
            # Si el denominador es negativo, la aerodinámica empuja más que la fuerza centrífuga
            v_target_adherencia[i] = v_max_recta 

# --- PASADA 2: INTEGRACIÓN DE FRENADO (Con Círculo de Kamm) ---
v_target = np.copy(v_target_adherencia)
Cd = 0.9 # Asumimos DRS cerrado al frenar
Cl = -3.0

for i in range(len(s_array) - 2, -1, -1):
    v_futura = v_target[i+1]
    R = radio_array[i] # Leemos el radio en este metro específico
    
    # Fuerzas verticales
    Flift = 0.5 * abs(Cl) * A * (v_futura**2) * rho
    Normal = (m * g) + Flift
    Ffriccion_max = Normal * mu_s
    
    # Demanda Lateral en este punto (Centrípeta)
    Fvolante_req = m * (v_futura**2) / R
    
    # Círculo de Kamm: ¿Cuánta fricción nos sobra para el freno longitudinal?
    if Fvolante_req < Ffriccion_max:
        Ffreno_max = np.sqrt(Ffriccion_max**2 - Fvolante_req**2)
    else:
        Ffreno_max = 0.0 # Si la curva ya nos exige el 100%, no podemos frenar nada
        
    # Fuerzas disipativas (Drag y Rodadura ayudan a frenar gratis)
    Fdrag = 0.5 * Cd * A * (v_futura**2) * rho
    Frod = Normal * C_rr
    
    # Aceleración total de frenado disponible
    Ftot = Ffreno_max + Fdrag + Frod
    a_frenado = Ftot / m
    
    # Ecuación cinemática
    v_requerida_para_frenar = np.sqrt(v_futura**2 + 2 * a_frenado * ds)
    
    # Nos quedamos con el peor escenario (el límite más bajo)
    v_target[i] = min(v_target_adherencia[i], v_requerida_para_frenar)
# --- GRÁFICO DEL ORÁCULO ---
plt.figure(figsize=(12, 5))
plt.plot(s_array, v_target_adherencia * 3.6, color='gray', linestyle='--', label='Pasada 1: Grip Limit')
plt.plot(s_array, v_target * 3.6, color='blue', linewidth=2, label='Pasada 2: Perfil Final Objetivo')

# Pintamos las zonas de frenada para que se vean claras
plt.fill_between(s_array, v_target * 3.6, v_target_adherencia * 3.6, color='red', alpha=0.2, label='Zona de Frenada')

plt.title("Perfil de Velocidad Objetivo - El 'Oráculo'")
plt.xlabel("Distancia en pista (m)")
plt.ylabel("Velocidad Target (km/h)")
plt.legend()
plt.grid(True, alpha=0.4)
plt.show()

#%% Simulacion principal

# Variables iniciales de integración
s = 0.0
t = 0.0
v_mod = 0.0
nafta = 2.0
d_nafta = 0.0277 * dt

x = np.array([0.0,0.0])
v = np.array([0.0,0.0]) 
a = np.array([0.0,0.0])

# --- TELEMETRIA --- #

telemetria_s = np.zeros_like(s_array)
telemetria_x = np.zeros_like(s_array)
telemetria_y = np.zeros_like(s_array)
telemetria_v = np.zeros_like(s_array)
telemetria_n = np.zeros_like(s_array)
telemetria_t = np.zeros_like(s_array)
telemetria_pedal_accel = np.zeros_like(s_array)
telemetria_pedal_freno = np.zeros_like(s_array)
telemetria_estado_DRS = np.zeros_like(s_array)
telemetria_G_lat = np.zeros_like(s_array)
telemetria_G_long = np.zeros_like(s_array)
telemetria_grip = np.zeros_like(s_array)


while s < longitud_total:

    indice_actual = int(np.clip(s / ds, 0, len(s_array) - 1))

    # Adentro del while, obtenemos el ángulo de la pista en esa posición s
    theta_pista = theta_array[indice_actual]

    # Forzamos los versores a la geometría de la pista
    u = np.array([np.cos(theta_pista), np.sin(theta_pista)])
    n = np.array([-u[1], u[0]]) # Versor normal (hacia la izquierda)
    
    R = radio_array[indice_actual]
    dir_curva = dir_array[indice_actual]
    v_objetivo = v_target[indice_actual]
    drs_legal = drs_array[indice_actual]

    estado_drs = accion_drs(indice_actual,margen_seguridad,drs_array,v_target,v_mod,longitud_total)

    Cd, Cl = aero_map(estado_drs)
    auto = m + nafta
    Fgrav = -auto * g
    Flift = 0.5 * Cl * A * (v_mod**2) * rho
    Normal = -(Fgrav + Flift)
    Ffriccion_max = Normal * mu_s

    Fdrag = 0.5 * Cd * A * (v_mod**2) * rho
    Frod = Normal * C_rr

    Fvolante_req = min(auto * (v_mod**2) * (1/R), Ffriccion_max)

    if Fvolante_req < Ffriccion_max:
        Fgrip_long = np.sqrt(Ffriccion_max**2 - Fvolante_req**2)
    else:
        Fgrip_long = 0.0

    Ffreno_max = Fgrip_long
    Ftraccion_max = Fgrip_long if v_mod <= 0.1 else min(P / v_mod, Fgrip_long)

    pedal_accel, pedal_freno = piloto_virtual(v_target, v_mod, dt, auto, Fdrag, Frod, Ftraccion_max, Ffreno_max,indice_actual)
    Fpedal = (Ftraccion_max * pedal_accel) - (Ffreno_max * pedal_freno)

    Flongitudinal = (Fpedal - Fdrag - Frod) * u
    Flateral = (Fvolante_req * dir_curva) * n

    Ftot = Flongitudinal + Flateral
    a = Ftot / auto

    Glong = np.dot(a, u) / g
    Glat = np.dot(a, n) / g

    telemetria_x[indice_actual] = x[0]
    telemetria_y[indice_actual] = x[1]
    telemetria_s[indice_actual] = s
    telemetria_v[indice_actual] = v_mod * 3.6
    telemetria_n[indice_actual] = nafta
    telemetria_t[indice_actual] = t
    telemetria_pedal_accel[indice_actual] = pedal_accel
    telemetria_pedal_freno[indice_actual] = pedal_freno
    telemetria_estado_DRS[indice_actual] = estado_drs
    telemetria_G_lat[indice_actual] = Glat
    telemetria_G_long[indice_actual] = Glong


    uso_grip = ((np.sqrt(Fpedal**2 + Fvolante_req**2)) / Ffriccion_max) *100  if Ffriccion_max > 0 else 0

    telemetria_grip[indice_actual] = uso_grip

    v += a * dt
    x += v * dt
    v_mod = np.linalg.norm(v)
    s += v_mod * dt
    t += dt
    nafta = max(0, nafta - d_nafta)



# %%    GRAFICADO
# --- 1. GRÁFICOS DE DESEMPEÑO (Velocidad y Pedales) ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Velocidad vs Oráculo
ax1.plot(s_array, telemetria_v, color='red', label='Velocidad Real')
ax1.plot(s_array, v_target * 3.6, color='blue', linestyle='--', alpha=0.6, label='Oráculo')
ax1.set_title("Análisis de Velocidad y Trayectoria")
ax1.set_ylabel("Velocidad (km/h)")
ax1.legend()
ax1.grid(True, alpha=0.3)

# Uso de Pedales
ax2.plot(s_array, telemetria_pedal_accel, color='green', label='Acelerador')
ax2.plot(s_array, telemetria_pedal_freno, color='red', label='Freno')
ax2.set_xlabel("Distancia (m)")
ax2.set_ylabel("Input (%)")
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# --- 2. DIAGRAMA G-G (Círculo de Kamm Dinámico) ---
plt.figure(figsize=(8, 8))
# Graficamos G-Lat vs G-Long
plt.scatter(telemetria_G_lat, telemetria_G_long, c=telemetria_v, cmap='magma', alpha=0.5, s=10)
plt.colorbar(label='Velocidad (km/h)')

# Dibujamos el límite mecánico estático (0 km/h)
theta_circle = np.linspace(0, 2*np.pi, 100)
plt.plot(mu_s * np.cos(theta_circle), mu_s * np.sin(theta_circle), 'k--', label='Límite Mecánico (0 km/h)')

# Dibujamos cómo se expande el límite con la aerodinámica (Ej: 150 y 300 km/h)
for v_ref in [150, 300]: # Velocidades en km/h
    v_ms = v_ref / 3.6
    F_lift_ref = 0.5 * abs(-3.0) * A * (v_ms**2) * rho # Usamos el Cl máximo (DRS cerrado)
    Normal_ref = (m * g) + F_lift_ref
    G_max_ref = (Normal_ref * mu_s) / (m * g)
    
    plt.plot(G_max_ref * np.cos(theta_circle), G_max_ref * np.sin(theta_circle), 
             linestyle='--', alpha=0.6, label=f'Límite Aero ({v_ref} km/h)')

plt.axhline(0, color='black', lw=1)
plt.axvline(0, color='black', lw=1)
plt.title("Diagrama G-G: Envolvente de Performance Dinámica")
plt.xlabel("Aceleración Lateral (g)")
plt.ylabel("Aceleración Longitudinal (g)")
plt.axis('equal')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()

# --- 3. MAPA DE CALOR EN EL CIRCUITO ---
plt.figure(figsize=(10, 8))
plt.plot(x_array, y_array, color='black', lw=5, alpha=0.1) # Trazado base
plt.scatter(telemetria_x, telemetria_y, c=telemetria_v, cmap='magma', s=5)
plt.colorbar(label='Velocidad (km/h)')
plt.axis('equal')
plt.title("Velocidad en el Trazado Real")
plt.show()


# %% ANIMACION   
# --- ANIMACIÓN DEL CIRCUITO (Basada en Tiempo Real) ---
from matplotlib.animation import FuncAnimation
import matplotlib as mpl
mpl.rcParams['animation.embed_limit'] = 50.0 # Aumenta el límite a 50 MB

fig_anim, ax_anim = plt.subplots(figsize=(8, 8))
ax_anim.plot(x_array, y_array, color='gray', alpha=0.3, lw=4, label='Pista') 
auto_dot, = ax_anim.plot([], [], 'ro', markersize=8, label='Auto F1')
ax_anim.set_aspect('equal')
ax_anim.set_title("Simulación en Tiempo Real")
ax_anim.grid(True)
ax_anim.legend()

# 1. Configuración del tiempo visual (Framerate constante)
fps_visual = 30 # Cuadros por segundo para el video
dt_visual = 1.0 / fps_visual
# Array de tiempo desde t=0 hasta el tiempo final que tardó la vuelta
t_visual_array = np.arange(0, telemetria_t.max(), dt_visual)

def update(frame):
    # ¿Qué tiempo (en segundos) representa este frame de la animación?
    t_actual = t_visual_array[frame]
    
    # Buscamos en nuestra telemetría el índice donde ocurrió ese tiempo
    idx = np.searchsorted(telemetria_t, t_actual) 
    
    # Prevención de desbordes por si el último milisegundo se pasa de largo
    if idx >= len(telemetria_x): 
        idx = len(telemetria_x) - 1
        
    auto_dot.set_data([telemetria_x[idx]], [telemetria_y[idx]])
    return auto_dot,

# Creamos la animación. Durará exactamente lo que tardó la vuelta en la simulación
ani = FuncAnimation(fig_anim, update, frames=len(t_visual_array), interval=dt_visual*1000, blit=True)

# # Cerramos la figura base para que VSC no duplique el gráfico estático arriba del video
# plt.close(fig_anim) 
# HTML(ani.to_jshtml())
# Generamos el texto HTML de la animación
html_str = ani.to_jshtml()

# Calculamos el peso en Megabytes (1 caracter de texto normal = 1 byte)
peso_bytes = len(html_str.encode('utf-8'))
peso_mb = peso_bytes / (1024 * 1024)

print(f"El peso final de la animación HTML es: {peso_mb:.2f} MB")

# Mostramos la animación
from IPython.display import HTML
HTML(html_str)
# %%
