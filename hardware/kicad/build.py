"""ASTRAEUS flight computer - board build script (KiCad 7 pcbnew API).
Board coordinates in mm: x across the board (0..W), y along the rocket axis (0 = nose / antenna end .. L = aft end).
Usage: python3 build.py out.kicad_pcb  [--check]   (--check only reports courtyard overlaps)"""
import pcbnew, sys, math

OUT = sys.argv[1]
CHECK = "--check" in sys.argv
W, L = 30.0, 108.0
OX, OY = 100.0, 50.0
K = "/usr/share/kicad/footprints/"
E = "/tmp/claude-0/-home-claude/7795f454-d891-5f94-8c09-7ce707cfa121/scratchpad/eelib/astraeus.pretty"

def mm(v): return pcbnew.FromMM(v)
def P(x, y): return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))

board = pcbnew.NewBoard(OUT)
board.SetCopperLayerCount(4)
ds = board.GetDesignSettings()
ds.SetCopperLayerCount(4)
ds.m_TrackMinWidth = mm(0.15); ds.m_MinClearance = mm(0.15); ds.m_ViasMinSize = mm(0.5)
ds.m_MinThroughDrill = mm(0.2); ds.m_CopperEdgeClearance = mm(0.3); ds.m_HoleClearance = mm(0.25)
nc = ds.m_NetSettings.m_DefaultNetClass
nc.SetTrackWidth(mm(0.15)); nc.SetClearance(mm(0.15)); nc.SetViaDiameter(mm(0.6)); nc.SetViaDrill(mm(0.3))
board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)
board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_SIGNAL)
board.SetLayerName(pcbnew.In1_Cu, "In1.GND")
board.SetLayerName(pcbnew.In2_Cu, "In2.SIG")

nets = {}
def net(n):
    if n not in nets:
        ni = pcbnew.NETINFO_ITEM(board, n); board.Add(ni); nets[n] = ni
    return nets[n]

BOM = []      # (ref, value, footprint, lcsc, dnp)
placed = {}
def place(ref, lib, name, x, y, rot=0, pads=None, value="", lcsc="", dnp=False, show_ref=True, bottom=False):
    fp = pcbnew.FootprintLoad(lib, name)
    fp.SetReference(ref); fp.SetValue(value or name)
    fp.SetPosition(P(x, y))
    if bottom: fp.Flip(P(x, y), False)
    fp.SetOrientationDegrees(rot)
    pads = pads or {}
    for pad in fp.Pads():
        n = pads.get(pad.GetNumber())
        if n: pad.SetNet(net(n))
    if dnp: fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
    r = fp.Reference(); r.SetTextSize(pcbnew.VECTOR2I(mm(0.6), mm(0.6))); r.SetTextThickness(mm(0.1)); r.SetVisible(False)
    fp.Value().SetVisible(False)
    board.Add(fp); placed[ref] = fp
    if lcsc is not None: BOM.append((ref, value or name, name, lcsc, dnp))
    return fp

R0402 = (K + "Resistor_SMD.pretty", "R_0402_1005Metric")
C0402 = (K + "Capacitor_SMD.pretty", "C_0402_1005Metric")
C0805 = (K + "Capacitor_SMD.pretty", "C_0805_2012Metric")
C1206 = (K + "Capacitor_SMD.pretty", "C_1206_3216Metric")
R0603 = (K + "Resistor_SMD.pretty", "R_0603_1608Metric")
LED = (K + "LED_SMD.pretty", "LED_0603_1608Metric")
SOT23 = (K + "Package_TO_SOT_SMD.pretty", "SOT-23")
SOT236 = (K + "Package_TO_SOT_SMD.pretty", "SOT-23-6")
TSOT6 = (K + "Package_TO_SOT_SMD.pretty", "TSOT-23-6")
SOD123 = (K + "Diode_SMD.pretty", "D_SOD-123")
LC = {"100n": "C1525", "1u": "C52923", "10u25": "C15850", "22u": "C45783", "47u": "C96123", "10k": "C25744", "100k": "C25741",
      "5k1": "C25905", "1k": "C11702", "100R": "C25076", "33k": "C25779", "22k": "C25768", "2k2": "C25879", "10R": "C22859"}
def R(ref, val, x, y, a, b, rot=0, dnp=False): return place(ref, *R0402, x, y, rot, {"1": a, "2": b}, val, LC.get(val, ""), dnp)
def C(ref, val, x, y, a, b, rot=0, pkg=C0402):
    return place(ref, *pkg, x, y, rot, {"1": a, "2": b}, val.replace("25", "") + ("F" if val[-1] in "unp" else ""), LC.get(val, ""))

# ------------------------------------------------------------------ outline: 30 x 100 mm, 2 mm corner radius
def edge_line(a, b):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetStart(P(*a)); s.SetEnd(P(*b))
    s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); board.Add(s)
def edge_arc(c, start, end):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_ARC)
    mid = (c[0] + (start[0] + end[0] - 2 * c[0]) / math.sqrt(2) / 1.0 * 0.7071 / 0.7071 * 0.5 * math.sqrt(2) / 1.0, 0)
    ang = (math.atan2(start[1] - c[1], start[0] - c[0]) + math.atan2(end[1] - c[1], end[0] - c[0])) / 2
    if abs(math.atan2(start[1] - c[1], start[0] - c[0]) - math.atan2(end[1] - c[1], end[0] - c[0])) > math.pi: ang += math.pi
    rr = math.hypot(start[0] - c[0], start[1] - c[1]); mid = (c[0] + rr * math.cos(ang), c[1] + rr * math.sin(ang))
    s.SetArcGeometry(P(*start), P(*mid), P(*end)); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); board.Add(s)
Rc = 2.0
edge_line((Rc, 0), (W - Rc, 0)); edge_line((W, Rc), (W, L - Rc)); edge_line((W - Rc, L), (Rc, L)); edge_line((0, L - Rc), (0, Rc))
edge_arc((W - Rc, Rc), (W - Rc, 0), (W, Rc)); edge_arc((W - Rc, L - Rc), (W, L - Rc), (W - Rc, L))
edge_arc((Rc, L - Rc), (Rc, L), (0, L - Rc)); edge_arc((Rc, Rc), (0, Rc), (Rc, 0))

# ------------------------------------------------------------------ mounting: 6 x M2 (nylon screws near the antenna)
for i, (hx, hy) in enumerate([(2.6, 9.5), (W - 2.6, 9.5), (2.6, 47.0), (W - 2.6, 47.0), (2.6, 94.6), (W - 2.6, 94.6)]):
    place(f"H{i+1}", K + "MountingHole.pretty", "MountingHole_2.2mm_M2", hx, hy, lcsc=None)

# ------------------------------------------------------------------ U1 ESP32-S3-WROOM-1-N8 (antenna at the nose edge, keepout from footprint)
esp = {"1": "GND", "2": "+3V3", "3": "EN", "4": "VBAT_SENSE", "5": "VPYRO_SENSE", "6": "SERVO1", "7": "SERVO2", "8": "SERVO3",
       "9": "SERVO4", "10": "INT_BARO", "11": "CS_BARO", "12": "INT_IMU", "13": "USB_DN", "14": "USB_DP", "17": "INT_HG",
       "18": "CS_IMU", "19": "SPI_MOSI", "20": "SPI_SCK", "21": "SPI_MISO", "22": "CS_HG", "23": "BUZ", "24": "IO47", "25": "IO48",
       "27": "BOOT", "28": "SD_MOSI", "29": "SD_SCK", "30": "SD_MISO", "31": "SD_CS", "33": "PY1_GATE", "34": "PY2_GATE",
       "35": "LED_STAT", "36": "RXD0", "37": "TXD0", "38": "PY2_SENSE", "39": "PY1_SENSE", "40": "GND", "41": "GND"}
place("U1", K + "RF_Module.pretty", "ESP32-S3-WROOM-1", W / 2, 12.75, 0, esp, "ESP32-S3-WROOM-1-N8", "C2913198")
for _z in placed["U1"].Zones():                     # antenna keepout on ALL four copper layers (library footprint only lists 3)
    _ls = pcbnew.LSET(); [_ls.AddLayer(l) for l in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)]; _z.SetLayerSet(_ls)
C("C1", "10u25", 3.3, 13.8, "+3V3", "GND", 90, C0805)       # module 3V3 bulk
C("C2", "100n", 3.3, 16.6, "+3V3", "GND", 90)
R("R1", "10k", 3.3, 19.2, "+3V3", "EN", 90)                 # EN RC: 10k + 1uF
C("C3", "1u", 3.3, 21.4, "EN", "GND", 90)
R("R2", "10k", W - 3.3, 22.2, "+3V3", "BOOT", 90)           # BOOT pull-up

# ------------------------------------------------------------------ USB-C (left edge) + ESD, native USB on IO19/IO20
usb = {"A1": "GND", "B1": "GND", "A12": "GND", "B12": "GND", "A4": "VBUS", "B4": "VBUS", "A9": "VBUS", "B9": "VBUS",
       "A5": "CC1", "B5": "CC2", "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN", "S1": "GND"}
place("J1", K + "Connector_USB.pretty", "USB_C_Receptacle_HRO_TYPE-C-31-M-12", 4.17, 31.6, 270, usb, "TYPE-C-31-M-12", "C165948")
place("U7", *SOT236, 12.3, 31.5, 90, {"1": "USB_DP", "6": "USB_DP", "3": "USB_DN", "4": "USB_DN", "2": "GND", "5": "VBUS"}, "USBLC6-2SC6", "C2827654")
R("R3", "5k1", 10.6, 27.4, "CC1", "GND", 90)
R("R4", "5k1", 10.6, 35.6, "CC2", "GND", 90)
place("D2", *SOD123, 13.6, 37.7, 180, {"1": "VLOGIC", "2": "VBUS"}, "B5819W", "C8598")

# ------------------------------------------------------------------ buttons (right side, under the module) + LEDs
TS = (E, "SW-SMD_4P-L5.1-W5.1-P3.70-LS6.5-TL_H1.5")
place("SW1", *TS, W - 3.6, 30.0, 90, {"1": "BOOT", "4": "GND"}, "BOOT  TS-1187A", "C318884")
place("SW2", *TS, W - 10.3, 30.0, 90, {"1": "EN", "4": "GND"}, "RESET TS-1187A", "C318884")
place("D10", *LED, 17.6, 37.9, 180, {"1": "GND", "2": "LED_PWR"}, "PWR green", "C12624")
R("R10", "1k", 17.6, 36.3, "+3V3", "LED_PWR", 0)
place("D11", *LED, 21.4, 37.9, 180, {"1": "GND", "2": "LED_ST"}, "STATUS white", "C2290")
R("R11", "1k", 21.4, 36.3, "LED_STAT", "LED_ST", 0)
place("D12", *LED, 25.2, 37.9, 180, {"1": "GND", "2": "LED_AR"}, "ARMED red", "C2286")
R("R12", "2k2", 25.2, 36.3, "VPYRO", "LED_AR", 0)

# ------------------------------------------------------------------ sensors on SPI (IMU near board centre)
lsm = {"1": "SPI_MISO", "2": "GND", "3": "GND", "4": "INT_IMU", "5": "+3V3", "6": "GND", "7": "GND", "8": "+3V3",
       "12": "CS_IMU", "13": "SPI_SCK", "14": "SPI_MOSI"}
place("U2", E, "LGA-14_L3.0-W2.5-P0.50-TL", 11.5, 43.5, 0, lsm, "LSM6DSO32TR", "C1864162")
C("C10", "100n", 8.7, 42.0, "+3V3", "GND", 90)
C("C11", "100n", 8.7, 45.0, "+3V3", "GND", 90)
adx = {"1": "+3V3", "2": "GND", "3": "+3V3", "11": "GND", "4": "GND", "5": "GND", "6": "+3V3", "7": "CS_HG", "8": "INT_HG", "12": "SPI_MISO", "13": "SPI_MOSI", "14": "SPI_SCK"}
place("U3", E, "LGA-14_L5.0-W3.0-P0.80-BL", 19.5, 43.5, 0, adx, "ADXL375BCCZ-RL7", "C481898")
C("C12", "100n", 23.5, 41.6, "+3V3", "GND", 90)
C("C13", "1u", 23.5, 45.4, "+3V3", "GND", 90)
bmp = {"1": "+3V3", "2": "SPI_SCK", "3": "GND", "4": "SPI_MOSI", "5": "SPI_MISO", "6": "CS_BARO", "7": "INT_BARO", "8": "GND", "9": "GND", "10": "+3V3"}
place("U4", E, "SENSORM-SMD_BMP388", 3.6, 57.5, 0, bmp, "BMP388", "C779278")
C("C14", "100n", 3.6, 54.6, "+3V3", "GND", 0)
C("C15", "100n", 3.6, 60.4, "+3V3", "GND", 0)

# ------------------------------------------------------------------ microSD (right edge, card slot facing out), SPI mode
sd = {"1": "SD_D2", "2": "SD_CS", "3": "SD_MOSI", "4": "+3V3", "5": "SD_SCK", "6": "GND", "7": "SD_MISO", "8": "SD_D1", "11": "GND"}
place("J2", K + "Connector_Card.pretty", "microSD_HC_Hirose_DM3AT-SF-PEJM5", W - 13.78, 59.0, 90, sd, "DM3AT-SF-PEJM5", "C114218")
R("R20", "10k", 6.6, 52.6, "+3V3", "SD_CS", 90)
R("R21", "10k", 6.6, 54.7, "+3V3", "SD_MOSI", 90)
R("R22", "10k", 6.6, 61.6, "+3V3", "SD_MISO", 90)
R("R23", "10k", 6.6, 63.7, "+3V3", "SD_D1", 90)
R("R24", "10k", 6.6, 65.8, "+3V3", "SD_D2", 90)
C("C16", "10u25", 4.0, 65.0, "+3V3", "GND", 90, C0805)

# ------------------------------------------------------------------ power: reverse protection, diode-OR, two bucks
place("Q5", *SOT23, 15.0, 69.5, 0, {"1": "GND", "2": "VBAT", "3": "VBAT_IN"}, "AO3401A (reverse-battery)", "C15127")
place("D1", *SOD123, 15.0, 73.4, 180, {"1": "VLOGIC", "2": "VBAT_SW"}, "B5819W", "C8598")
R("R30", "100k", 12.2, 69.0, "VBAT_SW", "VBAT_SENSE", 90)    # VBAT_SENSE = VBAT_SW * 22/122
R("R31", "22k", 12.2, 71.2, "VBAT_SENSE", "GND", 90)
R("R32", "100k", 17.8, 69.0, "VPYRO", "VPYRO_SENSE", 90)    # ARM detect
R("R33", "22k", 17.8, 71.2, "VPYRO_SENSE", "GND", 90)
# 3.3 V logic buck (left): AP63203, 4.7 uH
place("U5", *TSOT6, 5.5, 70.5, 90, {"1": "+3V3", "2": "VLOGIC", "3": "VLOGIC", "4": "GND", "5": "SW3", "6": "BST3"}, "AP63203WU-7", "C780769")
C("C30", "10u25", 2.0, 70.5, "VLOGIC", "GND", 90, C0805)
C("C31", "100n", 8.2, 68.4, "BST3", "SW3", 0)
place("L1", E, "IND-SMD_L4.0-W4.0", 5.5, 76.0, 0, {"1": "SW3", "2": "+3V3"}, "4.7uH SWPA4020S4R7MT", "C48427")
C("C32", "22u", 2.6, 80.9, "+3V3", "GND", 90, C0805)
C("C33", "22u", 5.2, 80.9, "+3V3", "GND", 90, C0805)
C("C34", "22u", 7.8, 80.9, "+3V3", "GND", 90, C0805)
# 5 V servo / buzzer buck (right): AP63205, 6.8 uH 5x5
place("U6", *TSOT6, W - 5.5, 70.5, 90, {"1": "+5V", "2": "VBAT_SW", "3": "VBAT_SW", "4": "GND", "5": "SW5", "6": "BST5"}, "AP63205WU-7", "C2071056")
C("C40", "10u25", W - 2.0, 70.5, "VBAT_SW", "GND", 90, C0805)
C("C41", "100n", W - 8.9, 68.6, "BST5", "SW5", 0)
place("L2", E, "IND-SMD_L5.0-W5.0_SWPA5040S", W - 5.5, 76.5, 0, {"1": "SW5", "2": "+5V"}, "6.8uH SWPA5040S6R8MT", "C36411")
C("C42", "47u", W - 2.6, 81.6, "+5V", "GND", 90, C1206)
C("C43", "47u", W - 6.0, 81.6, "+5V", "GND", 90, C1206)
C("C44", "10u25", 15.0, 76.0, "VBAT_SW", "GND", 0, C0805)    # bulk on switched battery

# ------------------------------------------------------------------ buzzer (5 V, passive 2.7 kHz) + driver
place("BZ1", E, "BUZ-SMD_4P-L8.5-W8.5-P8.50-BR", 15.0, 82.5, 0, {"1": "BZ_P", "2": "BUZ_D"}, "MLT-8530", "C94599")
place("Q3", *SOT23, 15.0, 89.3, 0, {"1": "BUZ_G", "2": "GND", "3": "BUZ_D"}, "AO3400A", "C20917")
R("R41", "1k", 11.6, 89.3, "BUZ", "BUZ_G", 90)
R("R42", "100k", 18.4, 89.3, "BUZ_G", "GND", 90)
place("D3", *SOD123, 15.0, 92.4, 0, {"1": "BZ_P", "2": "BUZ_D"}, "1N4148W", "C81598")
place("R40", *R0603, 19.2, 93.1, 0, {"1": "+5V", "2": "BZ_P"}, "10R", LC["10R"])

# ------------------------------------------------------------------ pyro channels (aft corners) - low-side AO3400A, continuity sense
PH = (K + "Connector_JST.pretty", "JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal")
place("J3", *PH, 4.82, L - 5.45, 0, {"1": "VPYRO", "2": "PY1_D"}, "PYRO1 JST-PH", "C295747")
place("J4", *PH, W - 4.82, L - 5.45, 0, {"1": "VPYRO", "2": "PY2_D"}, "PYRO2 JST-PH", "C295747")
place("Q1", *SOT23, 7.4, 89.8, 270, {"1": "PY1_G", "2": "GND", "3": "PY1_D"}, "AO3400A", "C20917")
R("R50", "100R", 4.4, 88.7, "PY1_GATE", "PY1_G", 0)
R("R51", "100k", 4.4, 90.0, "PY1_G", "GND", 0)
R("R52", "100k", 4.4, 91.3, "PY1_D", "PY1_SENSE", 0)
R("R53", "33k", 1.9, 90.0, "PY1_SENSE", "GND", 90)
place("Q2", *SOT23, W - 7.4, 89.8, 90, {"1": "PY2_G", "2": "GND", "3": "PY2_D"}, "AO3400A", "C20917")
R("R54", "100R", W - 4.4, 88.7, "PY2_GATE", "PY2_G", 0)
R("R55", "100k", W - 4.4, 90.0, "PY2_G", "GND", 0)
R("R56", "100k", W - 4.4, 91.3, "PY2_D", "PY2_SENSE", 0)
R("R57", "33k", W - 1.9, 90.0, "PY2_SENSE", "GND", 90)

# ------------------------------------------------------------------ servo rail: 4 channels, right-angle 3x4 header, 100R series
def servo_rail(ref, x0, y0):
    fp = pcbnew.FOOTPRINT(board); fp.SetReference(ref); fp.SetValue("SERVO 3x4 right-angle 2.54"); fp.SetPosition(P(x0, y0))
    fp.SetFPID(pcbnew.LIB_ID("astraeus", "ServoRail_3x4_P2.54mm_Horizontal"))
    rows = [("S", 0.0), ("P", 2.54), ("G", 5.08)]
    for c in range(4):
        for tag, dy in rows:
            pd = pcbnew.PAD(fp); pd.SetNumber(f"{tag}{c+1}"); pd.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
            pd.SetShape(pcbnew.PAD_SHAPE_RECT if (tag == "S" and c == 0) else pcbnew.PAD_SHAPE_CIRCLE)
            pd.SetSize(pcbnew.VECTOR2I(mm(1.6), mm(1.6))); pd.SetDrillSize(pcbnew.VECTOR2I(mm(1.0), mm(1.0)))
            pd.SetLayerSet(pd.PTHMask()); pd.SetPosition(P(x0 + c * 2.54, y0 + dy)); pd.SetPos0(pcbnew.VECTOR2I(mm(c * 2.54), mm(dy)))
            n = {"S": f"SRV{c+1}", "P": "+5V", "G": "GND"}[tag]; pd.SetNet(net(n)); fp.Add(pd)
    for lay, (a, b, cc, d), w in [(pcbnew.F_CrtYd, (-1.3, -1.3, 8.92, 9.8), 0.05), (pcbnew.F_Fab, (-1.27, 6.3, 8.89, 8.84), 0.1)]:
        s = pcbnew.FP_SHAPE(fp); s.SetShape(pcbnew.SHAPE_T_RECT); s.SetStart(P(x0 + a, y0 + b)); s.SetEnd(P(x0 + cc, y0 + d))
        s.SetLocalCoord(); s.SetLayer(lay); s.SetWidth(mm(w)); fp.Add(s)
    fp.Reference().SetVisible(False); fp.Value().SetVisible(False)
    board.Add(fp); placed[ref] = fp; BOM.append((ref, "Pin header 3x4 right-angle 2.54 mm (hand-solder)", "ServoRail_3x4", "", False))
servo_rail("J5", W / 2 - 3.81, L - 9.4)
for i in range(4):
    R(f"R6{i}", "100R", W / 2 - 3.81 + i * 2.54, 95.0, f"SERVO{i+1}", f"SRV{i+1}", 90)

# ------------------------------------------------------------------ bottom side: battery / switch / arm solder pads, test pads
def bottom_pad(ref, label, x, y, w, h, n, value):
    fp = pcbnew.FOOTPRINT(board); fp.SetReference(ref); fp.SetValue(value); fp.SetPosition(P(x, y))
    fp.SetFPID(pcbnew.LIB_ID("astraeus", f"SolderPad_{w}x{h}mm"))
    pd = pcbnew.PAD(fp); pd.SetNumber("1"); pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD); pd.SetShape(pcbnew.PAD_SHAPE_ROUNDRECT)
    pd.SetRoundRectRadiusRatio(0.2); pd.SetSize(pcbnew.VECTOR2I(mm(w), mm(h)))
    ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); ls.AddLayer(pcbnew.F_Mask); pd.SetLayerSet(ls)
    pd.SetPosition(P(x, y)); pd.SetNet(net(n)); fp.Add(pd)
    fp.Reference().SetVisible(False); fp.Value().SetVisible(False)
    fp.Flip(P(x, y), False)
    board.Add(fp); placed[ref] = fp
for ref, lab, x, y, w, h, n in [("TP1", "BAT+", 4.2, 72.5, 3.6, 4.6, "VBAT_IN"), ("TP2", "BAT-", 4.2, 79.0, 3.6, 4.6, "GND"),
                                ("TP3", "SW", W - 4.2, 72.5, 3.0, 4.0, "VBAT"), ("TP4", "SW", W - 4.2, 78.5, 3.0, 4.0, "VBAT_SW"),
                                ("TP5", "ARM", 13.0, 76.5, 3.0, 4.0, "VBAT_SW"), ("TP6", "ARM", 17.0, 76.5, 3.0, 4.0, "VPYRO")]:
    bottom_pad(ref, lab, x, y, w, h, n, lab)
tps = [("TP10", "GND"), ("TP11", "+3V3"), ("TP12", "+5V"), ("TP13", "VBAT_SW"), ("TP14", "TXD0"), ("TP15", "RXD0"), ("TP16", "IO47"), ("TP17", "IO48")]
for i, (ref, n) in enumerate(tps):
    bottom_pad(ref, n, 6.5 + (i % 4) * 5.6, 40.5 + (i // 4) * 4.2, 1.5, 1.5, n, n)


# ------------------------------------------------------------------ inner planes: In1 = GND, In2 = +3V3 (antenna keepout comes from U1)
def plane(layer, n, inset=0.4):
    z = pcbnew.ZONE(board); z.SetLayer(layer); z.SetNet(net(n))
    ol = z.Outline(); ol.NewOutline()
    for x, y in [(inset, inset), (W - inset, inset), (W - inset, L - inset), (inset, L - inset)]:
        ol.Append(mm(OX + x), mm(OY + y))
    z.SetLocalClearance(mm(0.25)); z.SetMinThickness(mm(0.2)); z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3)); z.SetThermalReliefSpokeWidth(mm(0.4)); z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetAssignedPriority(0); board.Add(z)
plane(pcbnew.In1_Cu, "GND")
# In2 = signals + a +3V3 fill, poured after routing (finish.py)

# ------------------------------------------------------------------ silkscreen (white mask / black legend)
def silk(txt, x, y, h=0.8, rot=0, bottom=False, bold=False, just=None):
    h = max(h, 0.8)
    t = pcbnew.PCB_TEXT(board); t.SetText(txt); t.SetPosition(P(x, y))
    t.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(h), mm(h))); t.SetTextThickness(mm(max(0.15, h * (0.18 if bold else 0.15))))
    t.SetTextAngleDegrees(rot)
    if bottom: t.SetMirrored(True)
    if just == "L": t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT if bottom else pcbnew.GR_TEXT_H_ALIGN_LEFT)
    if just == "R": t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT if bottom else pcbnew.GR_TEXT_H_ALIGN_RIGHT)
    board.Add(t)
def star(cx, cy, r, layer, filled=True):
    pts = []
    for k in range(10):
        a = -math.pi / 2 + k * math.pi / 5; rr = r if k % 2 == 0 else r * 0.42
        pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_POLY)
    s.SetPolyPoints([P(*p) for p in pts]); s.SetLayer(layer); s.SetWidth(mm(0.1)); s.SetFilled(filled); board.Add(s)
# top
star(8.3, 48.75, 1.05, pcbnew.F_SilkS)
silk("ASTRAEUS", 15.6, 48.8, 1.5, bold=True)
silk("FLIGHT COMPUTER", 15.0, 50.35, 0.6)
silk("BARO", 3.6, 62.2, 0.6)
silk("BOOT", W - 3.6, 34.45, 0.7); silk("RESET", W - 10.3, 34.45, 0.7)
silk("PWR", 17.6, 39.4, 0.6); silk("STAT", 21.4, 39.4, 0.6); silk("ARM", 25.2, 39.4, 0.6)
silk("PYRO 1", 6.0, 96.9, 0.7); silk("PYRO 2", W - 6.0, 96.9, 0.7)
for i in range(4): silk(str(i + 1), W / 2 - 3.81 + i * 2.54, 96.7, 0.7)
silk("+", 2.6, 97.9, 0.8); silk("+", W - 2.6, 97.9, 0.8)
# bottom (mirrored)
B = True
LX = 16.4                                      # logo block sits in the via-free zone under the microSD socket
star(LX, 54.0, 1.35, pcbnew.B_SilkS)
silk("ASTRAEUS", LX, 57.4, 2.0, 0, B, bold=True)
def hline(x0, x1, y, layer=pcbnew.B_SilkS, w=0.15):
    s_ = pcbnew.PCB_SHAPE(board); s_.SetShape(pcbnew.SHAPE_T_SEGMENT); s_.SetStart(P(x0, y)); s_.SetEnd(P(x1, y)); s_.SetLayer(layer); s_.SetWidth(mm(w)); board.Add(s_)
hline(LX - 7.2, LX + 7.2, 59.1)
silk("FLIGHT COMPUTER", LX, 60.5, 0.85, 0, B)
silk("REV A  -  2026", LX, 62.4, 0.8, 0, B)
silk("DESIGN  L. CHAUVIN", LX, 64.3, 0.8, 0, B)
silk("SERVO", W / 2, L - 11.7, 0.8, 0, B)
# keep vias (all layers) and bottom tracks out of the logo
def rule(x0, y0, x1, y1, layers, vias, tracks):
    z = pcbnew.ZONE(board); z.SetIsRuleArea(True); z.SetDoNotAllowVias(vias); z.SetDoNotAllowTracks(tracks)
    z.SetDoNotAllowPads(False); z.SetDoNotAllowCopperPour(False); z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET(); [ls.AddLayer(l) for l in layers]; z.SetLayerSet(ls)
    ol = z.Outline(); ol.NewOutline()
    for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]: ol.Append(mm(OX + x), mm(OY + y))
    board.Add(z)
rule(LX - 6.2, 52.2, LX + 8.0, 65.2, (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu), True, False)
# bottom tracks may pass under the logo (only vias are kept out)
silk("BAT +", 4.2, 69.4, 0.8, 0, B); silk("BAT -", 4.2, 82.2, 0.8, 0, B); silk("2S 6-8.4V", 4.2, 75.8, 0.6, 0, B)
silk("POWER", W - 4.2, 69.6, 0.7, 0, B); silk("SWITCH", W - 4.2, 81.4, 0.7, 0, B)
silk("ARM", 15.0, 73.4, 0.8, 0, B); silk("SWITCH", 15.0, 79.6, 0.7, 0, B)
silk("ARM ONLY ON THE PAD", W / 2, 85.0, 0.7, 0, B)
for i, (ref, n) in enumerate(tps):
    silk(n.replace("+", ""), 6.5 + (i % 4) * 5.6, 38.9 + (i // 4) * 4.2 + (3.3 if i // 4 else 0), 0.55, 0, B)
silk("S", W / 2 - 5.6, L - 9.4, 0.7, 0, B); silk("+", W / 2 - 5.6, L - 6.86, 0.8, 0, B); silk("-", W / 2 - 5.6, L - 4.32, 0.8, 0, B)
silk("PYRO1  + -", 5.0, L - 2.3, 0.6, 0, B); silk("PYRO2  + -", W - 5.0, L - 2.3, 0.6, 0, B)
silk("USB-C", 2.0, 31.5, 0.7, 90, B)

# ------------------------------------------------------------------ plane fan-out: every SMD GND / +3V3 pad gets its own via to In1 / In2
NOVIA = {"U2", "U3", "U4", "J1", "J2"}            # no vias under the sensors (stress / tenting) or under the metal shells
keep = [z for z in board.Zones() if z.GetIsRuleArea()]
for fp in placed.values():
    for z in fp.Zones():
        if z.GetIsRuleArea(): keep.append(z)
allpads = [pd for fp in placed.values() for pd in fp.Pads()]
fan_vias = []; fan_trk = []
SILK = []
for d in board.Drawings():
    if d.GetClass() in ("PCB_TEXT", "PCB_SHAPE") and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
        bb = d.GetBoundingBox(); bb.Inflate(mm(0.4)); SILK.append(bb)
def ok_via(pos, netname, own):
    for bb in SILK:
        if bb.Contains(pos): return False
    if not (1.0 < pcbnew.ToMM(pos.x) - OX < W - 1.0 and 1.0 < pcbnew.ToMM(pos.y) - OY < L - 1.0): return False
    v = pcbnew.PCB_VIA(board); v.SetPosition(pos); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3))
    for z in keep:
        if z.GetDoNotAllowVias() and z.Outline().Collide(pos, mm(0.35)): return False
    for ref in NOVIA:
        if placed[ref].GetCourtyard(pcbnew.F_CrtYd).Collide(pos, mm(0.35)): return False
    for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
        vs = v.GetEffectiveShape(lay)
        for pd in allpads:
            if not pd.IsOnLayer(lay): continue
            cl = 0.12 if pd.GetNetname() == netname else 0.2
            if pd.GetEffectiveShape(lay).Collide(vs, mm(cl)): return False
            if pd.HasHole() and (pd.GetPosition() - pos).EuclideanNorm() < mm(0.3 + pcbnew.ToMM(pd.GetDrillSize().x) / 2 + 0.3): return False
    for q, n in fan_vias:
        d = pcbnew.ToMM((q - pos).EuclideanNorm())
        if d < (0.75 if n == netname else 0.9): return False
    for t in fan_trk:
        if t.GetNetname() != netname and t.GetEffectiveShape(t.GetLayer()).Collide(v.GetEffectiveShape(t.GetLayer()), mm(0.2)): return False
    return True
def ok_trk(a, b, lay, netname, own):
    t = pcbnew.PCB_TRACK(board); t.SetStart(a); t.SetEnd(b); t.SetWidth(mm(0.25)); t.SetLayer(lay)
    ts = t.GetEffectiveShape(lay)
    for pd in allpads:
        if pd.IsOnLayer(lay) and pd.GetNetname() != netname and pd.GetEffectiveShape(lay).Collide(ts, mm(0.18)): return False
    for q, n in fan_vias:
        if n != netname:
            v = pcbnew.PCB_VIA(board); v.SetPosition(q); v.SetWidth(mm(0.6))
            if v.GetEffectiveShape(lay).Collide(ts, mm(0.2)): return False
    for o in fan_trk:
        if o.GetLayer() == lay and o.GetNetname() != netname and o.GetEffectiveShape(lay).Collide(ts, mm(0.18)): return False
    return True
# reserved fan-outs placed first (spots chosen by hand where the generic search has no room)
FORCE = {("J2", "6"): (9.75, 61.72)}
for (ref, num), (fx, fy) in FORCE.items():
    pd = [p for p in placed[ref].Pads() if p.GetNumber() == num][0]
    q = P(fx, fy)
    v = pcbnew.PCB_VIA(board); v.SetPosition(q); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(pd.GetNet())
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetLocked(True); board.Add(v); fan_vias.append((q, pd.GetNetname()))
    t = pcbnew.PCB_TRACK(board); t.SetStart(pd.GetPosition()); t.SetEnd(q); t.SetWidth(mm(0.3)); t.SetLayer(pcbnew.F_Cu); t.SetNet(pd.GetNet())
    t.SetLocked(True); board.Add(t); fan_trk.append(t)
miss = []
for fp in placed.values():
    for pd in fp.Pads():
        n = pd.GetNetname()
        if n not in ("GND", "+3V3") or pd.GetAttribute() != pcbnew.PAD_ATTRIB_SMD: continue
        if fp.GetReference() == "U1" and pd.GetNumber() == "41": continue      # exposed pad already has its via array
        lay = pcbnew.B_Cu if pd.IsOnLayer(pcbnew.B_Cu) else pcbnew.F_Cu
        c = pd.GetPosition(); fc = fp.GetPosition()
        away = math.atan2(c.y - fc.y, c.x - fc.x) if (c - fc).EuclideanNorm() > mm(0.3) else 0
        bb = pd.GetBoundingBox(); half = pcbnew.ToMM(min(bb.GetWidth(), bb.GetHeight())) / 2
        cands = []
        for r in [half + 0.55 + 0.2 * k for k in range(12)]:
            for k in range(16):
                a = k * math.pi / 8
                pen = abs(math.atan2(math.sin(a - away), math.cos(a - away)))
                cands.append((r + 0.35 * pen, a, r))
        cands.sort()
        done = False
        for _, a, r in cands:
            q = pcbnew.VECTOR2I(int(c.x + mm(r) * math.cos(a)), int(c.y + mm(r) * math.sin(a)))
            if ok_via(q, n, pd) and ok_trk(c, q, lay, n, pd):
                v = pcbnew.PCB_VIA(board); v.SetPosition(q); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(pd.GetNet())
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetLocked(True); board.Add(v); fan_vias.append((q, n))
                t = pcbnew.PCB_TRACK(board); t.SetStart(c); t.SetEnd(q); t.SetWidth(mm(0.25)); t.SetLayer(lay); t.SetNet(pd.GetNet())
                t.SetLocked(True); board.Add(t); fan_trk.append(t); done = True; break
        if not done: miss.append(f"{fp.GetReference()}.{pd.GetNumber()}")
print("fan-out vias", len(fan_vias), "no room:", miss)

# ------------------------------------------------------------------ routing keepout along the board edge (no tracks hugging the tube wall)
def edge_keep(x0, y0, x1, y1):
    z = pcbnew.ZONE(board); z.SetIsRuleArea(True); z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(True)
    z.SetDoNotAllowPads(False); z.SetDoNotAllowCopperPour(False); z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET(); [ls.AddLayer(l) for l in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)]; z.SetLayerSet(ls)
    ol = z.Outline(); ol.NewOutline()
    for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]: ol.Append(mm(OX + x), mm(OY + y))
    board.Add(z)
E0 = 0.45
edge_keep(-0.1, -0.1, E0, L + 0.1); edge_keep(W - E0, -0.1, W + 0.1, L + 0.1); edge_keep(-0.1, L - E0, W + 0.1, L + 0.1)


import json; json.dump(BOM, open(OUT.replace('.kicad_pcb', '_bom.json'), 'w'))
board.Save(OUT)
print('saved', len(placed), 'parts', len(nets), 'nets')
import os; os._exit(0)

import json; json.dump(BOM, open(OUT.replace(".kicad_pcb", "_bom.json"), "w"))
