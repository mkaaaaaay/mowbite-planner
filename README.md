# MowBite Planner

A mowing planner for OpenMower that knows how big your mower is. It takes the place of OpenMower's own planner
(slic3r): OpenMower asks it for the route and mows it like always. The difference: it keeps the whole mower away from
edges and obstacles, not just its middle.

**Still experimental.** In use on three mowers so far, among them a YardForce NX60i and a John Deere Tango E5. It
mows, but expect rough edges and settings that still change.

## What you get

- Loops along the edge and around beds and trees, then straight lanes.
- Gentle turns: small loops instead of spinning on the spot, nicer for the lawn. Spinning on the spot only where
  nothing else fits, or everywhere if you want it.
- The whole mower is checked at every point and every turn, with a gap you choose. Where it doesn't fit, it goes
  another way or leaves that bit out.
- It picks the lane spacing and direction itself, and can turn the stripes a little after every mow.
- Optional: clean stripes (turns only at the edge), an extra round over bits it missed, and going back by itself for
  what a mow really left.
- Its own settings page with a preview of the plan, `http://<mower>:8090`. No app needed (the MowBite app can do it
  all too).

## How it works

When you record the map, you drive the **middle** of the mower along the edges. So each line is where the middle
was, and the rest of the mower was further out. The planner works out where the real edge is and keeps the whole
mower (a rectangle from your measurements, or its real outline) away from it, plus the gap you set.

Drew a line right on a wall or a bed's border instead? Set "Map lines are" to `hard`, then the mower stays inside it.

For all this it needs your mower's size. Without it, the route goes right up to the lines, like with OpenMower's
planner.

## Install

You need OpenMowerOS, with OpenMower on its `edge` version: the planner needs two changes that aren't in a release
yet. Everything is done in Dockge. More details: [`ros1/README.md`](ros1/README.md#setup).

1. **OpenMower on edge.** In OpenMower's `.env` set `VERSION="edge"` and restart OpenMower. It still uses its old
   planner for now.
2. **Add the planner.** Make a new stack in Dockge (e.g. `mowbite-planner`), paste
   [`ros1/compose.yaml`](ros1/compose.yaml) and deploy. The first start builds it on the mower, that takes a few
   minutes. Wait until its log says `MowBite Planner answers`.
3. **Switch OpenMower over.** Add `OM_PLANNER=external` to OpenMower's `.env` and restart OpenMower. Really in this
   order: otherwise OpenMower finds no planner and keeps restarting.
4. **Tell it your mower's size.** Open `http://<mower>:8090`, under "Your mower": width, length to the front and to
   the back (from the middle between the drive wheels), cutting width and where the blade sits. Is it rounded at the
   front or back? Then also how wide it is right there: square corners would bump into everything on paper.
5. **Look at the plan** on that page (or in the MowBite app) before the first mow, emergency stop at hand.

**Updating:** the [MowBite Updater](https://github.com/mkaaaaaay/mowbite-updater) (part of MowBite, under Settings,
Updates) shows when there's a new planner or OpenMower was updated, and builds it again with a tap on **Update**.
Without it: open the planner's stack in Dockge and press **Update**, also after an OpenMower update, and now and then
free the space old versions take: `docker image prune -f && docker builder prune -f`.

**Back to OpenMower's planner:** on the settings page set "Planner" to `slic3r`. For good: take `OM_PLANNER` out of
OpenMower's `.env`, restart OpenMower, then stop the planner's stack.

### Planning on a computer in the house (optional)

A big area can take a minute to plan on the mower. A computer in the house that runs anyway (home server, NAS, PC,
Raspberry Pi 5) is usually several times quicker. The mower still checks every plan itself before it drives it,
and plans on its own whenever the house computer isn't there or too slow.

You need Linux with Docker and Docker Compose, 64-bit: amd64 or arm64 (a Raspberry Pi with a 64-bit OS; 32-bit ARM
doesn't work). It has to reach the mower's MQTT broker on port 1883, OpenMowerOS has that open by default.

1. **Start it on that computer.** Copy [`house/compose.yaml`](house/compose.yaml) into an empty folder there, next
   to it a file `.env` with the line `MOWER_HOST=<the mower's address>`, and run `docker compose up -d --build` in that
   folder. The first start builds it, a few minutes. Does your broker want a login: add the lines
   `MOWER_MQTT_USER=...` and `MOWER_MQTT_PASSWORD=...` to `.env`. `HOUSE_WORKERS=...` there sets how many processes a
   plan uses (3, fewer on a small computer). It gets the planner code the mower runs from this repository by itself,
   also after every planner update.
2. **Turn it on.** Open `http://<mower>:8090`, under "Extras" switch on "Plan on a computer in the house". Says it
   "not in use" there, the reason is next to it: "no house computer" means the mower doesn't hear from it (address,
   port 1883, is it running?). "house computer out of date": it's still getting the mower's code, give it a minute;
   if it stays, run the command from step 1 again. It only plans for planner code from this repository, not a fork.
3. **Check it.** Plans the mower already has stay as they are, new ones (after a change to the map or the settings)
   come from the house computer: the preview on that page says "planned on the house computer" then. On the house
   computer `docker logs mowbite_house` has a line for every plan (`request ... planned in ... s`). Where the mower
   planned on its own after all, the planner's log on the mower says why (`planned here, not on the house computer:
   ...`); with MQTT at hand, `planned_on` and `remote_why` in `mowbite/planner/progress` say the same.

To stop it: `docker compose down` in that folder, and switch the setting off again.

## Good to know

- A new plan takes a few seconds up to a minute per area on the mower. Plans are made ahead, so you rarely wait.
- The gaps also have to cover GPS errors: about 5 cm with a good RTK fix. GPS jumps (close to walls or trees)
  aren't covered.
- Where obstacles are closer to the edge or to each other than the mower plus its gaps, it can't get through.
- Thin strips can stay in tight corners and along edges the mower can only reach head-on.

## For developers

How it plans, every setting and the RPCs over MQTT: [`ros1/README.md`](ros1/README.md) (with OpenMower) and
[`ros2/README.md`](ros2/README.md) (the planner itself). For OpenMower only `ros1/` is needed. `ros2/` holds the
planner (plain Python with shapely, used by both), a ROS 2 node for robots that already run ROS 2, and
`tools/plot_coverage.py` to draw a plan on a PC. `sim/` simulates OpenMower driving a plan (parts of OpenMower
ported, see there), for the ROS 1 node's `planner.simulate` and the measuring tools.

Tests (the planner only needs shapely, the tests pytest):

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install shapely pytest
/tmp/venv/bin/python -m pytest ros2/mowbite_planner/test ros1/mowbite_planner_ros1/test sim/test
```

## License

GPL-3.0, see [LICENSE](LICENSE).
