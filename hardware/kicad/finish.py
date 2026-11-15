"""Import Freerouting session into the 4-layer ASTRAEUS board, add outer GND pours + stitching, fill, DRC."""
import pcbnew, sys, re
PCB, SES, RPT = sys.argv[1], sys.argv[2], sys.argv[3]
W, L, OX, OY = 30.0, 108.0, 100.0, 50.0
mm = pcbnew.FromMM
b = pcbnew.LoadBoard(PCB)
for t in [t for t in b.GetTracks() if not t.IsLocked()]: b.Remove(t)
txt = open(SES).read()
toks = re.findall(r'\(|\)|"[^"]*"|[^\s()]+', txt)
def parse(i):
    out = []
    while i < len(toks):
        t = toks[i]
        if t == '(':
            sub, i = parse(i + 1); out.append(sub)
        elif t == ')': return out, i + 1
        else: out.append(t.strip('"')); i += 1
    return out, i
tree, _ = parse(0)
def find(node, key):
    for c in node:
        if isinstance(c, list) and c and c[0] == key: yield c
sess = tree[0]; routes = next(find(sess, "routes"))
res = 10000.0
def pt(x, y): return pcbnew.VECTOR2I(mm(OX * 0 + float(x) / res), mm(-float(y) / res))
LAY = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu, "In1.GND": pcbnew.In1_Cu, "In2.SIG": pcbnew.In2_Cu, "In2.PWR": pcbnew.In2_Cu}
nw = nv = 0
for n in find(next(find(routes, "network_out")), "net"):
    ni = b.FindNet(n[1])
    for w in find(n, "wire"):
        path = next(find(w, "path")); layer = LAY[path[1]]; width = mm(float(path[2]) / res); c = path[3:]
        for k in range(0, len(c) - 2, 2):
            tr = pcbnew.PCB_TRACK(b); tr.SetStart(pt(c[k], c[k + 1])); tr.SetEnd(pt(c[k + 2], c[k + 3]))
            tr.SetWidth(width); tr.SetLayer(layer); tr.SetNet(ni); b.Add(tr); nw += 1
    for v in find(n, "via"):
        d, dr = (0.8, 0.4) if "800" in v[1] else (0.6, 0.3)
        via = pcbnew.PCB_VIA(b); via.SetPosition(pt(v[2], v[3])); via.SetWidth(mm(d)); via.SetDrill(mm(dr))
        via.SetNet(ni); via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); b.Add(via); nv += 1
print("imported segments", nw, "vias", nv)

gnd = b.FindNet("GND")
for z in [b.GetArea(i) for i in range(b.GetAreaCount())]:
    if z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In2_Cu) and not z.GetIsRuleArea(): b.Remove(z)
for layer, nn in ((pcbnew.F_Cu, "GND"), (pcbnew.B_Cu, "GND"), (pcbnew.In2_Cu, "+3V3")):
    z = pcbnew.ZONE(b); z.SetLayer(layer); z.SetNet(b.FindNet(nn))
    ol = z.Outline(); ol.NewOutline()
    for x, y in [(0.4, 0.4), (W - 0.4, 0.4), (W - 0.4, L - 0.4), (0.4, L - 0.4)]: ol.Append(mm(OX + x), mm(OY + y))
    z.SetLocalClearance(mm(0.25)); z.SetMinThickness(mm(0.2)); z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3)); z.SetThermalReliefSpokeWidth(mm(0.35))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS); z.SetAssignedPriority(1); b.Add(z)

# stitching vias around the edge (GND fence) where free; keep out of the antenna end (y < 6.5)
SILK = []
for d in b.Drawings():
    if d.GetClass() in ("PCB_TEXT", "PCB_SHAPE") and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
        bb = d.GetBoundingBox(); bb.Inflate(mm(0.4)); SILK.append(bb)
RULES = [z for z in [b.GetArea(i) for i in range(b.GetAreaCount())] if z.GetIsRuleArea() and z.GetDoNotAllowVias()] + [z for fp in b.GetFootprints() for z in fp.Zones() if z.GetIsRuleArea()]
def free(x, y, r=0.75):
    p = pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))
    if any(bb.Contains(p) for bb in SILK): return False
    if any(z.Outline().Collide(p, mm(0.4)) for z in RULES): return False
    for t in b.GetTracks():
        if t.GetClass() == "PCB_VIA":
            if (t.GetPosition() - p).EuclideanNorm() < mm(r + 0.6): return False
        elif t.HitTest(p, mm(r + 0.15)): return False
    for fp in b.GetFootprints():
        if fp.GetCourtyard(pcbnew.F_CrtYd).Collide(p, mm(0.35)) or fp.GetCourtyard(pcbnew.B_CrtYd).Collide(p, mm(0.35)): return False
        for pad in fp.Pads():
            if pad.HitTest(p, mm(r + 0.2)): return False
    return True
ns = 0
pts = [(1.0, y) for y in [7 + 2.5 * i for i in range(41)]] + [(W - 1.0, y) for y in [7 + 2.5 * i for i in range(41)]]
pts += [(x, L - 1.0) for x in [3 + 3 * i for i in range(9)]]
pts += [(x, y) for x in [6.0, 10.0, 15.0, 20.0, 24.0] for y in [27 + 4 * k for k in range(18)]]
for x, y in pts:
    if 6.5 < y < L - 0.6 and free(x, y):
        v = pcbnew.PCB_VIA(b); v.SetPosition(pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3))
        v.SetNet(gnd); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); b.Add(v); ns += 1
print("stitching vias", ns)
filler = pcbnew.ZONE_FILLER(b); filler.Fill(b.Zones())
b.BuildConnectivity(); b.Save(PCB)
print("drc", pcbnew.WriteDRCReport(b, RPT, pcbnew.EDA_UNITS_MILLIMETRES, True))
print("unrouted:", b.GetConnectivity().GetUnconnectedCount(True))
import os; os._exit(0)
