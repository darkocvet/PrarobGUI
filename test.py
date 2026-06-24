import numpy as np
import time

# Guard hardware SDK import so the math functions are always available
# (e.g. when imported from Windows GUI without dynamixel_sdk installed)
try:
    from dynamixel_sdk import *
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False


#  KONSTANTE - DYNAMIXEL REGISTRI
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

PROTOCOL_VERSION = 2.0
BAUDRATE         = 1000000
DEVICENAME       = '/dev/ttyUSB0'
MOTOR_IDS        = [1, 2, 3]

# Dynamixel XL430: 4096 koraka = 360°  →  11.3778 koraka/stupanj
KORACI_PO_STUPNJU = 4096.0 / 360.0

# Fiksne home pozicije (enkoder) - od ovih se uvijek odmiče
HOME = {1: 3072, 2: 0, 3: 762}


#  DIREKTNA KINEMATIKA
def izracunaj_DK(rotacijskiKut, kutPrvogZgloba, kutDrugogZgloba):
    """Ulaz: kutevi u radijanima. Izlaz: (x, y, z) u mm."""
    visina      = 95.0
    prviClanak  = 96.0
    drugiClanak = 180.0

    T_01 = np.array([
        [0, np.cos(rotacijskiKut), -np.sin(rotacijskiKut), 0],
        [0, np.sin(rotacijskiKut),  np.cos(rotacijskiKut), 0],
        [1, 0, 0, visina],
        [0, 0, 0, 1]
    ])
    T_12 = np.array([
        [np.cos(kutPrvogZgloba), -np.sin(kutPrvogZgloba), 0, prviClanak * np.cos(kutPrvogZgloba)],
        [np.sin(kutPrvogZgloba),  np.cos(kutPrvogZgloba), 0, prviClanak * np.sin(kutPrvogZgloba)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    T_2t = np.array([
        [-np.sin(kutDrugogZgloba), 0, np.cos(kutDrugogZgloba), drugiClanak * np.cos(kutDrugogZgloba)],
        [ np.cos(kutDrugogZgloba), 0, np.sin(kutDrugogZgloba), drugiClanak * np.sin(kutDrugogZgloba)],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    T = T_01 @ T_12 @ T_2t
    return T[0, 3], T[1, 3], T[2, 3]


#  INVERZNA KINEMATIKA
def izracunaj_IK(x, y, z=0):
    """Ulaz: (x, y, z) u mm. Izlaz: kutevi u radijanima."""
    d1 = 95.0
    a2 = 96.0
    a3 = 180.0

    rotacijskiKut = np.arctan2(y, x)
    R = np.sqrt(x**2 + y**2)

    A = 2 * d1 * a2
    B = -2 * R * a2
    C = a3**2 - a2**2 - d1**2 - R**2
    Nazivnik = np.sqrt(A**2 + B**2)

    if abs(C / Nazivnik) > 1:
        raise ValueError("Tražena točka je izvan dohvata ruke!")

    gamma = np.arctan2(B, A)
    kutPrvogZgloba = gamma + np.arccos(C / Nazivnik)

    skala_sin = (R - a2 * np.sin(kutPrvogZgloba)) / a3
    skala_cos = (-d1 - a2 * np.cos(kutPrvogZgloba)) / a3
    kutDrugogZgloba = np.arctan2(skala_sin, skala_cos) - kutPrvogZgloba
    kutDrugogZgloba = np.arctan2(np.sin(kutDrugogZgloba), np.cos(kutDrugogZgloba))

    return rotacijskiKut, kutPrvogZgloba, kutDrugogZgloba


#  PRETVORBA: KUT (stupnjevi) -> ENKODER POZICIJA
#
#  Logika: home pozicija = 0°, pa je ciljna enkoder vrijednost:
#      enkoder = HOME[id] + kut_deg * KORACI_PO_STUPNJU
#
#  Primjer motor 1, kut = +90°:  3049 + 90 * 11.378 = 3049 + 1024 = 4073
#  Primjer motor 1, kut = -90°:  3049 - 1024 = 2025

SMJER = {1: 1, 2: 1, 3: -1}
def kut_u_enkoder(motor_id, kut_deg):
    delta = int(round(kut_deg * KORACI_PO_STUPNJU * SMJER[motor_id]))
    if motor_id == 3:
        pozicija = HOME[motor_id] + delta  # bez % 4096
    else:
        pozicija = (HOME[motor_id] + delta) % 4096
    return pozicija


#  INICIJALIZACIJA MOTORA
def inicijaliziraj_motore():
    if not _HW_AVAILABLE:
        raise RuntimeError("dynamixel_sdk nije dostupan na ovom sustavu.")

    portHandler   = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        raise RuntimeError("Greška pri otvaranju porta! Provjerite DEVICENAME.")
    if not portHandler.setBaudRate(BAUDRATE):
        raise RuntimeError("Greška pri postavljanju brzine komunikacije!")

    print("U2D2 uspješno povezan.")

    for dxl_id in MOTOR_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

        if dxl_id == 3:
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 4)
        else:
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 3)

        result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)  # ← mora biti unutar for petlje

        if result != COMM_SUCCESS:
            print(f"  [UPOZORENJE] Kom. greška na ID {dxl_id}: {packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"  [UPOZORENJE] Greška motora ID {dxl_id}: {packetHandler.getRxPacketError(error)}")
        else:
            print(f"  Motor ID {dxl_id} spreman.")

    return portHandler, packetHandler


#  POMICANJE MOTORA
def pomakni_na_tocku(portHandler, packetHandler, q1_deg, q2_deg, q3_deg, pauza=3.0):
    kutevi = {1: q1_deg, 2: q2_deg, 3: q3_deg}

    print("\n  Enkoder ciljevi (home + delta):")
    for mid, kut in kutevi.items():
        enc = kut_u_enkoder(mid, kut)
        delta = enc - HOME[mid]
        print(f"    Motor ID {mid}: {HOME[mid]} + ({delta:+d}) = {enc}   [{kut:+.2f}°]")
        packetHandler.write4ByteTxRx(portHandler, mid, ADDR_GOAL_POSITION, enc)

    print(f"\n  Čekam {pauza}s da motori dođu na poziciju...")
    time.sleep(pauza)

    print("  Stvarne pozicije nakon pokreta:")
    for dxl_id in MOTOR_IDS:
        pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
        print(f"    Motor ID {dxl_id}: {pos}")



#  GAŠENJE MOTORA
def ugasi_motore(portHandler, packetHandler):
    if not _HW_AVAILABLE:
        raise RuntimeError("dynamixel_sdk nije dostupan na ovom sustavu.")
    print("\nGasim torque na svim motorima...")
    for dxl_id in MOTOR_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("Port zatvoren.")


#  GLAVNA FUNKCIJA
def main():
    print("=" * 55)
    print("  Robot - kontrola putem inverzne kinematike")
    print(f"  Home pozicije: {HOME}")
    print("=" * 55)

    # --- Unos koordinata ---
    print("\nUnesite ciljnu točku u mm:")
    try:
        x = float(input("  X = "))
        y = float(input("  Y = "))
        z_unos = input("  Z = (Enter za 0) ").strip()
        z = float(z_unos) if z_unos else 0.0
    except ValueError:
        print("Neispravan unos. Izlaz.")
        return

    # --- Inverzna kinematika ---
    print(f"\nCiljna točka: X={x:.1f} mm, Y={y:.1f} mm, Z={z:.1f} mm")
    try:
        q1_rad, q2_rad, q3_rad = izracunaj_IK(x, y, z)
    except ValueError as e:
        print(f"Greška IK: {e}")
        return

    q1_deg = np.rad2deg(q1_rad)
    q2_deg = np.rad2deg(q2_rad)
    q3_deg = np.rad2deg(q3_rad)

    print("\nIzračunani kutovi (relativno od home = 0°):")
    print(f"  Motor 1 - Rotacijski kut baze : {q1_deg:+.2f}°  →  enkoder {kut_u_enkoder(1, q1_deg)}")
    print(f"  Motor 2 - Kut prvog zgloba    : {q2_deg:+.2f}°  →  enkoder {kut_u_enkoder(2, q2_deg)}")
    print(f"  Motor 3 - Kut drugog zgloba   : {q3_deg:+.2f}°  →  enkoder {kut_u_enkoder(3, q3_deg)}")

    # Verifikacija DK
    x_v, y_v, z_v = izracunaj_DK(q1_rad, q2_rad, q3_rad)
    print(f"\nVerifikacija DK: X={x_v:.2f}, Y={y_v:.2f}, Z={z_v:.2f} mm")

    # --- Potvrda ---
    potvrda = input("\nPoslati motore na ovu poziciju? (da/ne): ").strip().lower()
    if potvrda not in ("da", "d", "yes", "y"):
        print("Odustajanje - motori nisu pomaknuti.")
        return

    # --- Pokret ---
    portHandler, packetHandler = inicijaliziraj_motore()
    try:
        pomakni_na_tocku(portHandler, packetHandler, q1_deg, q2_deg, q3_deg)
        print("\nPokret završen.")
    finally:
        ugasi_motore(portHandler, packetHandler)


def crtaj(x_start, y_start, x_end, y_end, broj_tocaka=20, pauza=0.3):
    x_start=150
    y_start=150
    x_end=150
    y_end=-150
    # Provjera svih točaka prije pokreta
    tocke = [(x_start + (i / broj_tocaka) * (x_end - x_start),
              y_start + (i / broj_tocaka) * (y_end - y_start))
             for i in range(broj_tocaka + 1)]

    for i, (x, y) in enumerate(tocke):
        try:
            izracunaj_IK(x, y, z=0)
        except ValueError:
            print(f"Točka {i+1} ({x:.1f}, {y:.1f}) je izvan dohvata! Prekid.")
            return

    # Inicijalizacija i pokret
    portHandler, packetHandler = inicijaliziraj_motore()
    try:
        print(f"Crtam liniju od ({x_start}, {y_start}) do ({x_end}, {y_end})...")
        for i, (x, y) in enumerate(tocke):
            q1_rad, q2_rad, q3_rad = izracunaj_IK(x, y, z=0)
            enc1 = kut_u_enkoder(1, np.rad2deg(q1_rad))
            enc2 = kut_u_enkoder(2, np.rad2deg(q2_rad))
            enc3 = kut_u_enkoder(3, np.rad2deg(q3_rad))
            packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, enc1)
            packetHandler.write4ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, enc2)
            packetHandler.write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, enc3)
            print(f"  {i+1}/{len(tocke)}: X={x:.1f} Y={y:.1f}  →  [{enc1}, {enc2}, {enc3}]")
            time.sleep(pauza)
        print("Gotovo.")
    finally:
        ugasi_motore(portHandler, packetHandler)

if __name__ == "__main__":
    crtaj(200,0,150,-150,20,0.3)