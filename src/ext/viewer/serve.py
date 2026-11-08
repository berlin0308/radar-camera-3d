"""Serve the three reconstructions of one object in a browser, switchable.

    python -m src.ext.viewer.serve --port 8422

Radar, LiDAR and RGB are all written in the same object frame (x, y in the world's orientation about
the refined pedestal centre, z = height above the local ground), so the viewer only has to draw them:
nothing is fitted or moved here. Toggle a model with its row or the number keys, `solo` to see one on
its own, `side by side` to stand them in a row. Colour is the height through the same turbo map the
figures use, a flat colour per model (for telling them apart when overlaid), or, for DA3, the photo.

The clouds come from, in order:
    src.ext.obj_radar.export_ply --source grt    GRT occupancy, the radar model
    src.ext.obj_radar.export_ply --source cfar   the CFAR points, the baseline it is measured against
    src.ext.obj_lidar.export_ply                 the LiDAR reference
    src.ext.obj_rgb.run_da3 + align_to_lidar     DA3, after the similarity onto the LiDAR

--model NAME=PATH[=COLOUR] replaces the set, in the order given, for another object or another run.
Pass --host 0.0.0.0 to reach it from another machine on the network.
"""
import argparse, http.server, json, os, socket, socketserver, urllib.request, webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))

# three.js, pinned, fetched once into vendor/ and served from there, so the page needs no network
# after the first run. It is not committed: three.module.js alone is 56k lines and would bury the
# rest of a review.
THREE = "0.160.0"
VENDOR = {
    "three.module.js": f"https://unpkg.com/three@{THREE}/build/three.module.js",
    "OrbitControls.js": f"https://unpkg.com/three@{THREE}/examples/jsm/controls/OrbitControls.js",
    "PLYLoader.js": f"https://unpkg.com/three@{THREE}/examples/jsm/loaders/PLYLoader.js",
}


def fetch_vendor():
    d = os.path.join(HERE, "vendor")
    os.makedirs(d, exist_ok=True)
    for name, url in VENDOR.items():
        path = os.path.join(d, name)
        if os.path.isfile(path) and os.path.getsize(path) > 1000:
            continue
        print(f"  fetching {name} (three.js {THREE}) ...", end=" ", flush=True)
        try:
            with urllib.request.urlopen(url, timeout=30) as r, open(path, "wb") as f:
                f.write(r.read())
            print(f"{os.path.getsize(path) / 1e6:.2f} MB")
        except Exception as e:
            raise SystemExit(f"\ncould not fetch {url}: {e}\n"
                             f"Download the three files in VENDOR into {d} by hand and run again.")

# name, path, colour, note, whether the PLY's own colours are photographic ("photo" mode), and the
# scene cloud that belongs to it. The models are the 1.2 m cut-out about the pedestal, which is what
# every number is measured on; the scene is what surrounds it, behind one switch and fetched only when
# that switch is first turned on, because it is 40 MB that the comparison does not need.
DEFAULT = [
    ("radar · GRT", "out/scotty_radar.ply", "#E8534E",
     "occupancy p > 0.9, first return per column", False, None),
    ("radar · CFAR", "out/scotty_radar_cfar.ply", "#F2B134",
     "signal chain only, no elevation", False, "out/scotty_radar_scene.ply"),
    ("LiDAR", "out/scotty_lidar_rgb.ply", "#5B8DEF",
     "Ouster OS-0-128, the reference; 80% camera-coloured", True, "out/scotty_lidar_scene.ply"),
    ("RGB · DA3, 96 frames", "out/da3_posed_n96_r504/object.ply", "#3FBF7F",
     "calibrated poses, 96 frames, no fit", True, "out/scotty_da3_scene.ply"),
    ("RGB · DA3, 32 frames", "out/da3_scotty_posed/object.ply", "#2E9E63",
     "calibrated poses, 32 frames, no fit", True, None),
    ("RGB · DA3, pose-free", "out/da3_scotty/da3_aligned.ply", "#8FD3A8",
     "estimated its own cameras, then Sim3", True, None),
    ("3DGS · splatfacto", "out/scotty_gs.ply", "#B07BD8",
     "Gaussian centres, calibrated poses, 96 frames", True, "out/scotty_gs_scene.ply"),
]


class Handler(http.server.SimpleHTTPRequestHandler):
    manifest, files = b"", {}

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self.send_file(os.path.join(HERE, "index.html"), "text/html")
        if self.path == "/manifest.json":
            self.send_response(200); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(self.manifest))); self.end_headers()
            return self.wfile.write(self.manifest)
        if self.path.startswith("/vendor/"):
            name = os.path.basename(self.path)
            p = os.path.join(HERE, "vendor", name)
            if os.path.isfile(p):
                return self.send_file(p, "text/javascript")
        if self.path.startswith("/models/"):
            p = self.files.get(self.path[len("/models/"):])
            if p:
                return self.send_file(p, "application/octet-stream")
        self.send_error(404)

    def send_file(self, path, ctype):
        n = os.path.getsize(path)
        self.send_response(200); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(n)); self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with open(path, "rb") as f:
            while chunk := f.read(1 << 20):
                self.wfile.write(chunk)

    def log_message(self, *args):
        pass


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", default=[], metavar="NAME=PATH[=COLOUR]")
    ap.add_argument("--title", default="Scotty, 2026-11-03.17-01-12 · 30–165 s orbit")
    ap.add_argument("--subtitle", default="radar, LiDAR and RGB in one object frame — no fit between them")
    ap.add_argument("--pedestal-radius", type=float, default=0.39)
    ap.add_argument("--height-max", type=float, default=2.2, help="top of the turbo height colour map")
    ap.add_argument("--spacing", type=float, default=2.6, help="gap between models in the side-by-side layout (m)")
    ap.add_argument("--port", type=int, default=8422)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--open", action="store_true", help="open a browser on this machine")
    a = ap.parse_args()

    fetch_vendor()

    palette = ["#E8534E", "#F2B134", "#5B8DEF", "#3FBF7F", "#8FD3A8", "#B07BD8", "#4FC3C7"]
    spec = ([(m.split("=")[0], m.split("=")[1], (m.split("=") + [palette[i % len(palette)]])[2], "", True, None)
             for i, m in enumerate(a.model)] if a.model else DEFAULT)

    models, files = [], {}
    for i, (name, path, colour, note, photo, scene) in enumerate(spec):
        if not os.path.isfile(path):
            print(f"  ! missing, skipped: {path}  ({name})")
            continue
        key = f"m{i}.ply"
        files[key] = os.path.abspath(path)
        scene_url = None
        if scene and os.path.isfile(scene):
            files[f"s{i}.ply"] = os.path.abspath(scene)
            scene_url = f"models/s{i}.ply"
        elif scene:
            print(f"  ! scene missing: {scene}")
        models.append(dict(name=name, url=f"models/{key}", colour=colour, note=note,
                           has_photo_colour=photo, on=True, scene=scene_url))
        print(f"  {name:22s} {path}  ({os.path.getsize(path) / 1e6:.1f} MB)"
              + (f"  + scene {os.path.getsize(scene) / 1e6:.0f} MB" if scene_url else ""))
    if not models:
        raise SystemExit("no cloud found; run the export scripts first")

    Handler.files = files
    Handler.manifest = json.dumps(dict(title=a.title, subtitle=a.subtitle, models=models,
                                       pedestal_radius=a.pedestal_radius, height_max=a.height_max,
                                       spacing=a.spacing)).encode()

    with Server((a.host, a.port), Handler) as srv:
        url = f"http://{'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host}:{a.port}/"
        obj = sum(os.path.getsize(files[k]) for k in files if k.startswith("m")) / 1e6
        sc = sum(os.path.getsize(files[k]) for k in files if k.startswith("s")) / 1e6
        print(f"\n{len(models)} models, {obj:.0f} MB, plus {sc:.0f} MB of scene behind the switch -> {url}")
        if a.host == "0.0.0.0":
            print(f"from another machine: http://{socket.gethostname()}:{a.port}/")
        print("ctrl-c to stop")
        if a.open:
            webbrowser.open(url)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print("\nstopped")


if __name__ == "__main__":
    main()
