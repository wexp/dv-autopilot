# dv-autopilot

An autopilot engine for [Derail Valley](https://store.steampowered.com/app/588030/Derail_Valley/). It drives locomotives for you: road runs between yards, shunting moves inside a yard, coupling onto standing trains, spotting cuts inside track markers, and turntable rides — with block signalling between multiple autopilot trains so they queue instead of colliding.

This mod is an engine only — it has no UI of its own. It is designed as an addon for [dv-remote-dispatch](https://github.com/Alexand3r/dv-remote-dispatch), whose web console detects the mod and exposes the autopilot controls (destination, cruise speed, couple/uncouple, shove/pull arrival, via junction). Without the mod installed the console simply hides the autopilot panel.

## Features

- **Drive and Shunt modes** — Drive runs yard to yard at line speed with planned braking curves; Shunt works at yard pace with marker spotting and tie-down.
- **Routing** — Dijkstra over directed track states with reversal costs, occupancy awareness, traffic-priced detours and run-round planning.
- **Facing orders** — *shove* (wagons first) and *pull* (loco first) are hard orders derived from the physical consist orientation; a route that cannot honour them is refused out loud rather than arriving the wrong way round.
- **Block signalling** — exclusive track and junction claims, distance-based movement authority, dispatch-order queueing, buffer-block release. Multiple autopilot trains wait for each other like trains, not like ghosts.
- **Turntables** — rides the bridge when the consist's bogies fit, boards to a stop, winds the table so the correct end of the train meets the target stall (the facing order decides which), and rolls off.
- **Traction care** — wheelslip-managed throttle and dynamic brake with sanding, engine and traction-motor temperature protection, multiple-unit awareness (a loco without an MU cable counts as freight).

## Requirements

- Derail Valley (build 99.x)
- [Unity Mod Manager](https://www.nexusmods.com/site/mods/21)
- [dv-remote-dispatch](https://github.com/Alexand3r/dv-remote-dispatch) for the user interface

## Installation

Install through Unity Mod Manager like any other DV mod: point UMM at the game, then drop the release zip onto its Mods tab (or unpack into `Derail Valley/Mods/Autopilot`).

## Settings

The Unity Mod Manager settings panel holds one driving profile per locomotive type (unlisted and custom locos use the *All other locos* profile):

- **Throttle response (%/s)** — how quickly the autopilot winds the throttle handle open. Higher means livelier acceleration but more wheelslip risk on wet rail; wheelslip protection and the adhesion budget still have the last word.
- **Throttle to start (%)** — the smallest handle setting that actually gets that machine rolling from a stand; below its coarsest notch a loco does nothing at all.
- **Yard speed (km/h)** — top speed for shunting moves with that loco. Road runs use the cruise speed you dispatch with instead.

*Reset all profiles to defaults* restores the shipped values. *Write debug detail to the mod log* adds per-tick driving detail to `Player.log` for bug hunts.

## Credits

This project reuses and adapts code from two excellent Derail Valley mods:

- [dv-remote-dispatch](https://github.com/mspielberg/dv-remote-dispatch) by mspielberg — the mod this engine grew out of: world/track plumbing, job data handling and the updater scaffolding started there. The maintained fork with the autopilot console is [Alexand3r/dv-remote-dispatch](https://github.com/Alexand3r/dv-remote-dispatch).
- [dv-hud](https://github.com/koteq/dv-hud) by koteq — track following and speed/grade indexing ideas.

Thanks to both authors for publishing their work under permissive licences.

## License

[MIT](LICENCE.md)
