# Cesium

[Cesium for Omniverse](https://github.com/CesiumGS/cesium-omniverse) streams
real places into Isaac Sim as 3D tiles, including Google's photorealistic tiles.
`--scene campus` uses it to fly the drone over the Georgia Tech campus, 150 m
up, with the camera tipped 45 degrees down. A token must be set up by hand
first.

```bash
scripts/pace.sh bag campus --scene campus --steps 2100 \
    --resolution 1280x720 --image-hz 20
```

## A Cesium ion token

Cesium serves the tiles only to an account, and the account is free.

1. Sign up at <https://ion.cesium.com/signup>.
2. In Cesium ion, open **Access Tokens** and copy the default token.
3. In **Asset Depot**, add **Google Photorealistic 3D Tiles** to the account
   if it is not already under **My Assets**.
4. Save the token on ICE in a file that only its owner can read:

```bash
cat > ~/.cesium_token     # paste the token, press Enter, then Ctrl-D
chmod 600 ~/.cesium_token
```

The token is a secret. It stays in that file and is passed to Isaac Sim through
the environment, as `CESIUM_ION_TOKEN`. It is never written into this
repository, a script or a log.

To check the token from a login node:

```bash
curl -s -H "Authorization: Bearer $(cat ~/.cesium_token)" https://api.cesium.com/v1/me
```

A working token prints the account's id and its scopes, which must include
`assets:read`.

## How it is wired in

`scripts/pace.sh setup` downloads Cesium for Omniverse 0.30.0 into scratch and
installs `lxml`, which the extension imports and cannot install for itself here.
`scripts/pace.sh` passes the extension's folder as `ISAACDRONES_CESIUM` and the
token as `CESIUM_ION_TOKEN`. Away from ICE, set those two yourself and install
with `uv sync --extra sim --extra cesium`.

The scene differs from the others in two ways. It runs in Isaac Sim's stock app,
because Cesium imports GUI extensions that `src/isaacdrones/sim/isaacdrones.kit`
leaves out, so it takes one to six minutes to start where the others take
seconds. And Cesium is enabled as the app starts, because it registers USD
schemas, which is why the scene must be named with `--scene` and cannot be added
later.

Cesium loads the tiles that the camera sees. Before every frame,
`Simulation.step` hands it the camera's view and waits for those tiles, so a
recording does not show tiles arriving late, and two runs of one flight see the
same tiles.

The world's origin is at latitude 33.7756, longitude -84.3963 and 260 m above
the ellipsoid, which is about ground level in the middle of campus, with x east,
y north and z up. The tiles are photographs with the sunlight of their day in
them, so the scene adds even light and no sun. There is no sky, so the horizon
is blank. Google's tiles are asset 2275207 in Cesium ion. Google requires its
credit to be shown with the imagery, and its terms need reading before anyone
records data from the tiles or trains on it.
