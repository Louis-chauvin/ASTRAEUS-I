import json, sys
p = sys.argv[1]; j = json.load(open(p))
ns = j.setdefault("net_settings", {})
cls = ns.setdefault("classes", [])
base = dict(cls[0]) if cls else {}
base.update({"name": "Default", "track_width": 0.15, "clearance": 0.15, "via_diameter": 0.6, "via_drill": 0.3})
hi = dict(base); hi.update({"name": "PWR_HI", "track_width": 0.6, "clearance": 0.2, "via_diameter": 0.8, "via_drill": 0.4})
pw = dict(base); pw.update({"name": "PWR", "track_width": 0.4, "clearance": 0.15, "via_diameter": 0.6, "via_drill": 0.3})
usb = dict(base); usb.update({"name": "USB", "track_width": 0.25, "clearance": 0.15})
lo = dict(base); lo.update({"name": "PWR_LO", "track_width": 0.3, "clearance": 0.15})
ns["classes"] = [base, hi, pw, usb, lo]
pats = [("PWR_HI", n) for n in ["VBAT_IN", "VBAT", "VBAT_SW", "VPYRO", "PY1_D", "PY2_D", "+5V", "SW5"]]
pats += [("PWR", n) for n in ["+3V3", "SW3", "VLOGIC", "GND"]]
pats += [("PWR_LO", n) for n in ["VBUS", "BZ_P", "BUZ_D"]]
pats += [("USB", n) for n in ["USB_DP", "USB_DN"]]
ns["netclass_patterns"] = [{"netclass": c, "pattern": "^" + n.replace("+", "\\+") + "$"} for c, n in pats]
json.dump(j, open(p, "w"), indent=2)
print("netclasses set")
