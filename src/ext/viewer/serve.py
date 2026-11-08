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

To put it on someone else's machine there are three ways, in order of how little they need from us:

  --bundle DIR   writes a self-contained folder (page, three.js, every cloud, a manifest with relative
                 URLs and a README). Zip it, hand it over, and they serve it with any static server --
                 `python3 -m http.server` is enough. Nothing of ours has to stay running, and they
                 need no account, no network and no access to nunga. It will not run from a file://
                 URL, because ES modules and fetch are blocked there; it has to be served.
  --host 0.0.0.0 serves it to the local network, so anyone on the same wifi can open
                 http://<this machine>:<port>/. Quickest, and it exposes the clouds to that network.
  an SSH tunnel  `ssh -N -L 8422:127.0.0.1:8422 <user>@<this machine>`, for someone who already has an
                 account here. The default bind stays on 127.0.0.1 for this.
"""
import argparse, http.server, json, os, shutil, socket, socketserver, urllib.request, webbrowser

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

# A scene is one capture and the clouds built from it. Each model is
# (name, path, colour, note, whether the PLY's own colours are photographic, its scene cloud, on at
# start). The models are the cut-out every number is measured on; the scene cloud is what surrounds
# it, behind one switch and fetched only when that switch is first used.
SCENES = [
    dict(key="scotty",
         title="Scotty · 2026-11-03.17-01-12 · 30–165 s",
         subtitle="one object from radar, LiDAR and RGB, in the object's own frame — no fit between them",
         pedestal_radius=0.39, height_max=2.2, spacing=2.6, grid=4.0,
         models=[
             ("radar · GRT", "out/scotty_radar.ply", "#E8534E",
              "occupancy p > 0.9, first return per column", False, "out/scotty_grt_scene.ply", False),
             ("radar · CFAR", "out/scotty_radar_cfar.ply", "#F2B134",
              "signal chain only, no elevation", False, "out/scotty_radar_scene.ply", False),
             ("LiDAR", "out/scotty_lidar_rgb.ply", "#5B8DEF",
              "Ouster OS-0-128, the reference; 80% camera-coloured", True, "out/scotty_lidar_scene.ply", True),
             ("RGB · DA3, 96 frames", "out/da3_posed_n96_r504/object.ply", "#3FBF7F",
              "calibrated poses, 96 frames, no fit", True, "out/scotty_da3_scene.ply", True),
             ("RGB · DA3, 32 frames", "out/da3_scotty_posed/object.ply", "#2E9E63",
              "calibrated poses, 32 frames, no fit", True, None, False),
             ("RGB · DA3, pose-free", "out/da3_scotty/da3_aligned.ply", "#8FD3A8",
              "estimated its own cameras, then Sim3", True, None, False),
             ("3DGS · Gaussian centres", "out/scotty_gs.ply", "#B07BD8",
              "the primitives themselves, 5.4k of them", True, "out/scotty_gs_scene.ply", False),
         ]),
    dict(key="cut",
         title="The Cut · 2026-11-03.17-07-11 · 192 m loop",
         subtitle="an open lawn instead of an object — world frame, KISS-ICP poses, nothing re-centred",
         pedestal_radius=0.0, height_max=2.5, spacing=70.0, grid=60.0,
         models=[
             ("LiDAR", "out/cut_lidar_scene.ply", "#5B8DEF",
              "the 0.3–2.5 m layer within 35 m of the path", False, None, True),
             ("radar · GRT", "out/cut_grt_scene.ply", "#E8534E",
              "occupancy p > 0.9, 414 scans of the loop", False, None, True),
             ("radar · CFAR", "out/cut_cfar_scene.ply", "#F2B134",
              "static-filtered, drawn flat at the radar's height", False, None, False),
         ]),
]


def bundle(out, scenes, files, manifest, with_scenes):
    """A folder anyone can serve with a static server, with nothing of ours left running."""
    fetch_vendor()
    os.makedirs(os.path.join(out, "models"), exist_ok=True)
    os.makedirs(os.path.join(out, "vendor"), exist_ok=True)
    shutil.copy(os.path.join(HERE, "index.html"), os.path.join(out, "index.html"))
    for name in VENDOR:
        shutil.copy(os.path.join(HERE, "vendor", name), os.path.join(out, "vendor", name))

    total = 0
    for sc in manifest["scenes"]:
      for m in sc["models"]:
        if not with_scenes:
            m["scene"] = None
        for url in (m["url"], m["scene"]):
            if not url:
                continue
            key = os.path.basename(url)
            shutil.copy(files[key], os.path.join(out, "models", key))
            total += os.path.getsize(files[key])
    json.dump(manifest, open(os.path.join(out, "manifest.json"), "w"))

    open(os.path.join(out, "README.txt"), "w").write(
        "Point clouds of one object, seen by radar, LiDAR and a camera.\n\n"
        "It has to be SERVED, not opened from the file system: ES modules and fetch do not work\n"
        "over file://. Any static server will do. From this folder:\n\n"
        "    python3 -m http.server 8422\n\n"
        "then open http://localhost:8422/ . Nothing is sent anywhere; three.js is included.\n")
    print(f"\nbundle -> {out}  ({total / 1e6:.0f} MB of clouds"
          + ("" if with_scenes else ", scene layers left out; --bundle-scenes adds them") + ")")
    print("  serve it with:  cd " + out + " && python3 -m http.server 8422")


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
    ap.add_argument("--grid", type=float, default=4.0, help="ground grid size (m)")
    ap.add_argument("--scene-key", dest="scene", action="append",
                    help="serve only these scenes, by key; repeatable")
    ap.add_argument("--port", type=int, default=8422)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--open", action="store_true", help="open a browser on this machine")
    ap.add_argument("--bundle", metavar="DIR", help="write a self-contained copy here instead of serving")
    ap.add_argument("--bundle-scenes", action="store_true", help="--bundle: include the scene layers too")
    a = ap.parse_args()

    fetch_vendor()

    palette = ["#E8534E", "#F2B134", "#5B8DEF", "#3FBF7F", "#8FD3A8", "#B07BD8", "#4FC3C7"]
    if a.model:
        scenes_in = [dict(key="custom", title=a.title, subtitle=a.subtitle,
                          pedestal_radius=a.pedestal_radius, height_max=a.height_max,
                          spacing=a.spacing, grid=a.grid,
                          models=[(m.split("=")[0], m.split("=")[1],
                                   (m.split("=") + [palette[i % len(palette)]])[2], "", True, None, True)
                                  for i, m in enumerate(a.model)])]
    else:
        scenes_in = [sc for sc in SCENES if not a.scene or sc["key"] in a.scene]
        if not scenes_in:
            raise SystemExit(f"no scene named {a.scene}; have " + ", ".join(sc["key"] for sc in SCENES))

    scenes, files = [], {}
    for si, sc in enumerate(scenes_in):
        models = []
        print(f"\n{sc['title']}")
        for mi, (name, path, colour, note, photo, scene, on) in enumerate(sc["models"]):
            if not os.path.isfile(path):
                print(f"  ! missing, skipped: {path}  ({name})")
                continue
            key = f"m{si}_{mi}.ply"
            files[key] = os.path.abspath(path)
            scene_url = None
            if scene and os.path.isfile(scene):
                files[f"s{si}_{mi}.ply"] = os.path.abspath(scene)
                scene_url = f"models/s{si}_{mi}.ply"
            elif scene:
                print(f"  ! scene missing: {scene}")
            models.append(dict(name=name, url=f"models/{key}", colour=colour, note=note,
                               has_photo_colour=photo, on=on, scene=scene_url))
            print(f"  {name:24s} {path}  ({os.path.getsize(path) / 1e6:.1f} MB)"
                  + (f"  + scene {os.path.getsize(scene) / 1e6:.0f} MB" if scene_url else "")
                  + ("" if on else "   [off at start]"))
        if models:
            scenes.append({**{k: v for k, v in sc.items() if k != "models"}, "models": models})
    if not scenes:
        raise SystemExit("no cloud found; run the export scripts first")

    manifest = dict(scenes=scenes)

    if a.bundle:
        return bundle(a.bundle, scenes, files, manifest, a.bundle_scenes)

    Handler.files = files
    Handler.manifest = json.dumps(manifest).encode()

    with Server((a.host, a.port), Handler) as srv:
        url = f"http://{'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host}:{a.port}/"
        obj = sum(os.path.getsize(files[k]) for k in files if k.startswith("m")) / 1e6
        bg = sum(os.path.getsize(files[k]) for k in files if k.startswith("s")) / 1e6
        n = sum(len(x["models"]) for x in scenes)
        print(f"\n{len(scenes)} scene(s), {n} models, {obj:.0f} MB, plus {bg:.0f} MB of background -> {url}")
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
